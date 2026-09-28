// IITGN JointVelocityController (RT-safe + jerk/accel limited + stall guarded)
//
// What this version does to reduce Franka reflex + comm-constraint failures:
//  1) Jerk limiting -> continuous acceleration -> avoids velocity/accel discontinuity reflexes.
//  2) No output deadband-to-zero -> avoids hard steps at v_new.
//  3) If dt<=0: hold last command (do not integrate bogus dynamics).
//  4) Stall guard (dt too large): hold last command + reset accel history.
//  5) **NO ROS logging / clock / node access inside update()**.
//     All diagnostics are logged from a non-RT timer created in on_configure().
//  6) on_activate initializes last_sent_ from measured joint velocities to avoid takeover steps.
//     Also aligns desired_velocities_rt_ and RT buffer to that measured state.
//
// NOTE:
//  - communication_constraints_violation happens when the control packet is dropped
//    for 20 consecutive cycles. Code jitter in update() is a common cause.
//  - This implementation avoids anything in update() that can block (logging, time, params).

#include "iitgn_controllers/joint_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <string>
#include <chrono>
#include <functional>


namespace iitgn_controllers
{

// -----------------------
// Tunables (constants)
// -----------------------
static constexpr double EPS_VEL = 1e-4;   // [rad/s] input deadband (callback-side + watchdog target-side)
static constexpr double V_MAX   = 2.0;    // [rad/s] clamp
static constexpr double A_MAX   = 4.0;    // [rad/s^2] accel limit
static constexpr double J_MAX   = 150.0;  // [rad/s^3] jerk limit (typical range 50–200)

static constexpr double COMMAND_TIMEOUT_S = 0.5;  // [s] watchdog timeout
static constexpr double DECAY_FACTOR      = 0.95; // decay per update tick (target-side)
static constexpr double SMOOTH_ALPHA      = 0.10; // EMA on incoming commands (callback-side)

// ============================================================================
// Interface configs
// ============================================================================
controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::state_interface_configuration() const
{
  // We read measured joint velocities for smooth takeover at activation.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

// ============================================================================
// on_init()
// ============================================================================
CallbackReturn JointVelocityController::on_init()
{
  try {
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<bool>("gazebo", gazebo_);

    // Optional diagnostics params (safe to read outside update)
    auto_declare<double>("stall_dt_s", stall_dt_s_);
    auto_declare<double>("diag_period_s", diag_period_s_);
    auto_declare<double>("spike_dt_s", spike_dt_s_);

    
    desired_velocities_rt_.fill(0.0);
    smoothed_velocities_cb_.fill(0.0);
    rt_cmd_.writeFromNonRT(desired_velocities_rt_);

    new_cmd_.store(false, std::memory_order_relaxed);
    time_since_cmd_ = 0.0;
    saw_first_command_ = false;

    last_sent_.fill(0.0);
    last_acc_.fill(0.0);

    max_dt_ns_.store(0, std::memory_order_relaxed);
    dt_spike_count_.store(0, std::memory_order_relaxed);
    stall_count_.store(0, std::memory_order_relaxed);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure()
// ============================================================================
CallbackReturn
JointVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    auto node = get_node();

    arm_id_ = node->get_parameter("arm_id").as_string();
    gazebo_ = node->get_parameter("gazebo").as_bool();
    stall_dt_s_ = node->get_parameter("stall_dt_s").as_double();
    diag_period_s_ = node->get_parameter("diag_period_s").as_double();
    
    if (!node->get_parameter("spike_dt_s", spike_dt_s_)) {
      // keep the default from the header
    }

    // Subscriber (non-RT thread)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    commands_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands",
      qos,
      std::bind(&JointVelocityController::velocity_command_callback, this, std::placeholders::_1));

    // Diagnostics timer (non-RT): periodically report dt spikes / stalls.
    // This avoids any logging inside update().
    diag_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(std::max(0.2, diag_period_s_)),
      [this]() {
        const uint64_t max_dt_ns = max_dt_ns_.exchange(0, std::memory_order_relaxed);
        const uint64_t spikes    = dt_spike_count_.exchange(0, std::memory_order_relaxed);
        const uint64_t stalls    = stall_count_.exchange(0, std::memory_order_relaxed);

        if (max_dt_ns > 0 || spikes > 0 || stalls > 0) {
          const double max_dt = static_cast<double>(max_dt_ns) * 1e-9;
          RCLCPP_WARN(
            get_node()->get_logger(),
            "RT diag: max_dt=%.6f s, dt_spikes=%lu, stalls=%lu (stall_dt_s=%.4f)",
            max_dt, static_cast<unsigned long>(spikes), static_cast<unsigned long>(stalls), stall_dt_s_);
        }
      });

    RCLCPP_INFO(
      node->get_logger(),
      "JointVelocityController configured: arm_id='%s', gazebo=%s, stall_dt_s=%.4f, diag_period_s=%.2f",
      arm_id_.c_str(),
      gazebo_ ? "true" : "false",
      stall_dt_s_,
      diag_period_s_);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_configure: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate()
// ============================================================================
CallbackReturn
JointVelocityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  // Reset command buffers
  desired_velocities_rt_.fill(0.0);
  smoothed_velocities_cb_.fill(0.0);
  rt_cmd_.writeFromNonRT(desired_velocities_rt_);
  new_cmd_.store(false, std::memory_order_relaxed);
  saw_first_command_ = false;

  last_acc_.fill(0.0);

  // --- Sanity checks ---
  if (command_interfaces_.size() < num_joints || state_interfaces_.size() < num_joints) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Activate failed: command_if=%zu state_if=%zu (need %zu). Check arm_id / interface names.",
      command_interfaces_.size(), state_interfaces_.size(), num_joints);
    return CallbackReturn::ERROR;
  }

  // --- Smooth takeover ---
  // Start commanding exactly the currently measured joint velocities.
  // This prevents a discontinuity at controller activation.
  for (size_t i = 0; i < num_joints; ++i) {
    const double v_meas = state_interfaces_[i].get_value();
    last_sent_[i] = v_meas;
    // desired_velocities_rt_[i] = 0.0; // align internal target too
    desired_velocities_rt_[i] = v_meas;   // keep target aligned to measured velocity
    command_interfaces_[i].set_value(v_meas);
  }

  // Align RT buffer with the same “hold” command so the next update sees a consistent target.
  // rt_cmd_.writeFromNonRT(desired_velocities_rt_);  // zeros
  // new_cmd_.store(false, std::memory_order_release); // wait for actual command

  rt_cmd_.writeFromNonRT(desired_velocities_rt_);
  new_cmd_.store(true, std::memory_order_release); // make update consume this aligned target


  RCLCPP_INFO(node->get_logger(), "JointVelocityController activated (smooth takeover from measured velocity).");
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate()
// ============================================================================
CallbackReturn
JointVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop commanding (non-RT lifecycle transition; ok to send zeros)
  desired_velocities_rt_.fill(0.0);
  smoothed_velocities_cb_.fill(0.0);
  rt_cmd_.writeFromNonRT(desired_velocities_rt_);

  new_cmd_.store(false, std::memory_order_relaxed);

  last_sent_.fill(0.0);
  last_acc_.fill(0.0);

  for (size_t i = 0; i < std::min(command_interfaces_.size(), static_cast<size_t>(num_joints)); ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "JointVelocityController deactivated: all commands set to 0.0");
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// velocity_command_callback()  (ROS thread)
// ============================================================================
void JointVelocityController::velocity_command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::array<double, num_joints> cmd_out{};
  cmd_out.fill(0.0);

  // Empty message = explicit stop (still a "new command" event)
  if (!msg || msg->data.empty()) {
    smoothed_velocities_cb_.fill(0.0);
    rt_cmd_.writeFromNonRT(smoothed_velocities_cb_);
    new_cmd_.store(true, std::memory_order_release);
    return;
  }

  const size_t n = std::min(msg->data.size(), cmd_out.size());

  for (size_t i = 0; i < n; ++i) {
    const double raw = msg->data[i];

    // EMA on incoming commands (non-RT)
    double v = SMOOTH_ALPHA * raw + (1.0 - SMOOTH_ALPHA) * smoothed_velocities_cb_[i];

    // Small deadband at the *input/target* side is fine
    if (std::abs(v) < EPS_VEL) {
      v = 0.0;
    }

    smoothed_velocities_cb_[i] = v;
    cmd_out[i] = v;
  }

  // Unprovided joints -> 0
  for (size_t i = n; i < cmd_out.size(); ++i) {
    smoothed_velocities_cb_[i] = 0.0;
    cmd_out[i] = 0.0;
  }

  rt_cmd_.writeFromNonRT(cmd_out);
  new_cmd_.store(true, std::memory_order_release);

  if (!saw_first_command_) {
    saw_first_command_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Received first velocity command (%zu elements).", msg->data.size());
  }
}

// ============================================================================
// update()  (controller manager loop)  *** RT CRITICAL ***
//  - NO logging
//  - NO get_node(), NO clocks
//  - Only simple math + interface reads/writes
// ============================================================================
controller_interface::return_type
JointVelocityController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  const double dt = period.seconds();

  // ---- RT-safe dt tracking (no logging here) ----
  const uint64_t dt_ns = (dt > 0.0) ? static_cast<uint64_t>(dt * 1e9) : 0;
  uint64_t prev_max = max_dt_ns_.load(std::memory_order_relaxed);
  while (dt_ns > prev_max &&
         !max_dt_ns_.compare_exchange_weak(prev_max, dt_ns,
                                           std::memory_order_relaxed,
                                           std::memory_order_relaxed)) {
    // spin
  }

  // Count “meaningful” dt spikes (threshold chosen for visibility, not control)
  if (dt > spike_dt_s_) {
    dt_spike_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // ---- If dt is invalid, hold last command (do not integrate) ----
  if (dt <= 0.0) {
    for (size_t i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(last_sent_[i]);
    }
    return controller_interface::return_type::OK;
  }

  // ---- Stall guard: if loop stalls, DO NOT integrate a huge step ----
  // Key changes vs your current code:
  //  - DO NOT reset last_acc_ (avoids accel discontinuity after stall)
  //  - DO NOT reset time_since_cmd_ (don’t hide a real command gap)
  //  - Optionally decay the target (fine), but keep state continuity
  if (dt > stall_dt_s_) {
    stall_count_.fetch_add(1, std::memory_order_relaxed);

    // Hold last sent command (no jumps)
    for (size_t i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(last_sent_[i]);
    }

    // Optional: decay target so we don't chase stale commands after a stall
    for (auto& v : desired_velocities_rt_) {
      v *= DECAY_FACTOR;
      // no snapping here; let jerk limiter handle smoothness later
    }

    // Preserve watchdog timing (do not reset to 0)
    time_since_cmd_ += dt;

    return controller_interface::return_type::OK;
  }

  // 1) Pull latest command OR watchdog behavior (target-side)
  const bool got_new = new_cmd_.exchange(false, std::memory_order_acq_rel);

  if (got_new) {
    if (const auto* cmd = rt_cmd_.readFromRT()) {
      desired_velocities_rt_ = *cmd;
    }
    time_since_cmd_ = 0.0;
  } else {
    time_since_cmd_ += dt;

    if (time_since_cmd_ > COMMAND_TIMEOUT_S) {
      // If no commands arrive, decay target toward zero smoothly.
      for (auto& v : desired_velocities_rt_) {
        v *= DECAY_FACTOR;
        if (std::abs(v) < EPS_VEL) {
          v = 0.0;
        }
      }
    }
  }

  // 2) Clamp + jerk/accel limit + write commands
  for (size_t i = 0; i < num_joints; ++i) {
    const double v_target = std::clamp(desired_velocities_rt_[i], -V_MAX, V_MAX);
    const double v_prev   = last_sent_[i];

    // Desired acceleration to reach target in one step
    double a_des = (v_target - v_prev) / dt;

    // Accel clamp
    a_des = std::clamp(a_des, -A_MAX, A_MAX);

    // Jerk clamp (limit change in acceleration)
    double da = a_des - last_acc_[i];
    const double da_max = J_MAX * dt;
    da = std::clamp(da, -da_max, da_max);

    const double a_new = last_acc_[i] + da;

    // Integrate velocity + clamp actual output
    double v_new = v_prev + a_new * dt;
    v_new = std::clamp(v_new, -V_MAX, V_MAX);

    command_interfaces_[i].set_value(v_new);

    last_sent_[i] = v_new;
    last_acc_[i]  = a_new;
  }

  return controller_interface::return_type::OK;
}


}  // namespace iitgn_controllers

PLUGINLIB_EXPORT_CLASS(
  iitgn_controllers::JointVelocityController,
  controller_interface::ControllerInterface)
