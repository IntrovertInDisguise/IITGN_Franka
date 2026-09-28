#include <iitgn_controllers/impedance_controller.hpp>
#include <iitgn_controllers/robot_utils.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

namespace iitgn_controllers {

// ============================================================================
// Interface configs
// ============================================================================
controller_interface::InterfaceConfiguration
ImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request effort interfaces for all joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request position and velocity interfaces for all joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

// ============================================================================
// update()
// ============================================================================
controller_interface::return_type
ImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  // 1) Update current joint states
  updateJointStates();

  // 2) Update time bookkeeping
  const double dt = std::max(period.seconds(), 1e-6);
  elapsed_time_step_ = dt;
  elapsed_time_ += dt;

  // 3) Calculate desired trajectory based on impedance mode / external commands
  calculateDesiredTrajectory();

  // 4) Apply velocity filtering (alpha is a param)
  applyVelocityFilter(velocity_filter_alpha_);

  // 5) Calculate impedance control torques
  Vector7d tau_d = calculateImpedanceTorques();

  // 6) Apply safety limits and write
  for (int i = 0; i < num_joints; ++i) {
    tau_d(i) = std::clamp(tau_d(i), -max_torques[i], max_torques[i]);
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

// ============================================================================
// on_init()
// ============================================================================
CallbackReturn
ImpedanceController::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "fr3");

    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("m_gains", {});

    auto_declare<std::vector<double>>("target_position", {});

    auto_declare<std::string>("impedance_mode", "position_impedance");

    auto_declare<double>("motion_amplitude", 0.5);
    auto_declare<double>("motion_frequency", 0.2);
    auto_declare<double>("compliance_factor", 1.0);

    // Base topic family for: /impedance_command, /impedance_command_joint, /impedance_command_config
    auto_declare<std::string>("command_topic", "/impedance_command");

    // External smooth trajectory duration
    auto_declare<double>("trajectory_duration", 5.0);

    // Velocity filter alpha
    auto_declare<double>("velocity_filter_alpha", 0.1);

    // Embedded bridge: subscribe to existing Float64MultiArray velocity topic
    // Keep this equal to what your Python already publishes.
    auto_declare<std::string>("cmd_topic", "/NS_1/joint_velocity_controller/commands");

    RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure()
// ============================================================================
CallbackReturn
ImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    auto node = get_node();

    // Parameters
    arm_id_ = node->get_parameter("arm_id").as_string();

    auto k_gains = node->get_parameter("k_gains").as_double_array();
    auto d_gains = node->get_parameter("d_gains").as_double_array();
    auto m_gains = node->get_parameter("m_gains").as_double_array();

    target_position_ = node->get_parameter("target_position").as_double_array();

    const std::string mode_str = node->get_parameter("impedance_mode").as_string();

    motion_amplitude_ = node->get_parameter("motion_amplitude").as_double();
    motion_frequency_ = node->get_parameter("motion_frequency").as_double();
    compliance_factor_ = node->get_parameter("compliance_factor").as_double();

    command_topic_ = node->get_parameter("command_topic").as_string();
    trajectory_duration_ = node->get_parameter("trajectory_duration").as_double();

    velocity_filter_alpha_ = node->get_parameter("velocity_filter_alpha").as_double();
    velocity_filter_alpha_ = std::clamp(velocity_filter_alpha_, 0.0, 1.0);

    cmd_topic_ = node->get_parameter("cmd_topic").as_string();

    // Validate gains
    if (k_gains.size() != num_joints) {
      RCLCPP_ERROR(node->get_logger(), "k_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    if (d_gains.size() != num_joints) {
      RCLCPP_ERROR(node->get_logger(), "d_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    if (m_gains.size() != num_joints) {
      RCLCPP_ERROR(node->get_logger(), "m_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }

    for (int i = 0; i < num_joints; ++i) {
      k_gains_(i) = k_gains[i];
      d_gains_(i) = d_gains[i];
      m_gains_(i) = m_gains[i];
    }

    // Mode parsing
    if (mode_str == "position_impedance") {
      impedance_mode_ = POSITION_IMPEDANCE;
    } else if (mode_str == "trajectory_impedance") {
      impedance_mode_ = TRAJECTORY_IMPEDANCE;
    } else if (mode_str == "sinusoidal_impedance") {
      impedance_mode_ = SINUSOIDAL_IMPEDANCE;
    } else if (mode_str == "compliant_motion") {
      impedance_mode_ = COMPLIANT_MOTION;
    } else {
      RCLCPP_WARN(node->get_logger(), "Unknown impedance_mode='%s', using position_impedance", mode_str.c_str());
      impedance_mode_ = POSITION_IMPEDANCE;
    }

    // Init state
    q_.setZero();
    dq_.setZero();
    dq_filtered_.setZero();

    q_desired_.setZero();
    dq_desired_.setZero();
    ddq_desired_.setZero();

    q_target_.setZero();

    external_q_target_.setZero();
    external_dq_target_.setZero();
    external_ddq_target_.setZero();

    elapsed_time_ = 0.0;
    elapsed_time_step_ = 0.0;

    external_trajectory_time_ = 0.0;
    trajectory_start_q_.setZero();
    trajectory_start_dq_.setZero();

    new_command_received_ = false;
    external_command_active_ = false;
    direct_tracking_mode_ = false;
    velocity_only_streaming_ = false;

    // QoS (streaming)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    // External pose command
    command_subscriber_ =
      node->create_subscription<geometry_msgs::msg::PoseStamped>(
        command_topic_, qos,
        std::bind(&ImpedanceController::commandCallback, this, std::placeholders::_1));

    // External joint command (position/velocity)
    joint_command_subscriber_ =
      node->create_subscription<sensor_msgs::msg::JointState>(
        command_topic_ + "_joint", qos,
        std::bind(&ImpedanceController::jointCommandCallback, this, std::placeholders::_1));

    // Impedance config commands
    impedance_config_subscriber_ =
      node->create_subscription<std_msgs::msg::String>(
        command_topic_ + "_config", rclcpp::QoS(10),
        std::bind(&ImpedanceController::impedanceConfigCallback, this, std::placeholders::_1));

    // ------------------------------------------------------------------
    // Embedded bridge subscriber:
    // Float64MultiArray velocity commands (your existing Python topic)
    // ------------------------------------------------------------------
    vel_cmd_subscriber_ =
      node->create_subscription<std_msgs::msg::Float64MultiArray>(
        cmd_topic_, qos,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (!msg) {
            return;
          }
          if (msg->data.size() != static_cast<size_t>(num_joints)) {
            RCLCPP_WARN_THROTTLE(
              this->get_node()->get_logger(),
              *this->get_node()->get_clock(),
              2000,
              "cmd_topic '%s': expected %d velocities, got %zu",
              this->cmd_topic_.c_str(),
              num_joints,
              msg->data.size());
            return;
          }

          // Convert -> JointState (velocity-only)
          auto js = std::make_shared<sensor_msgs::msg::JointState>();
          js->velocity.resize(num_joints);
          for (int i = 0; i < num_joints; ++i) {
            js->velocity[i] = msg->data[i];
          }
          // leave position empty (velocity-only streaming)
          this->jointCommandCallback(js);
        });

    RCLCPP_INFO(node->get_logger(), "IITGN Impedance Controller configured");
    RCLCPP_INFO(node->get_logger(), "  arm_id: %s", arm_id_.c_str());
    RCLCPP_INFO(node->get_logger(), "  impedance_mode: %s", mode_str.c_str());
    RCLCPP_INFO(node->get_logger(), "  command_topic: %s", command_topic_.c_str());
    RCLCPP_INFO(node->get_logger(), "  embedded bridge cmd_topic: %s", cmd_topic_.c_str());
    RCLCPP_INFO(node->get_logger(), "  velocity_filter_alpha: %.3f", velocity_filter_alpha_);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate()
// ============================================================================
CallbackReturn
ImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  updateJointStates();

  initial_q_ = q_;

  if (target_position_.size() == static_cast<size_t>(num_joints)) {
    for (int i = 0; i < num_joints; ++i) {
      q_target_(i) = target_position_[i];
    }
  } else {
    q_target_ = initial_q_;
  }

  elapsed_time_ = 0.0;
  elapsed_time_step_ = 0.0;

  dq_filtered_.setZero();

  q_desired_ = q_target_;
  dq_desired_.setZero();
  ddq_desired_.setZero();

  external_command_active_ = false;
  new_command_received_ = false;
  direct_tracking_mode_ = false;
  velocity_only_streaming_ = false;

  external_trajectory_time_ = 0.0;
  trajectory_start_q_ = q_;
  trajectory_start_dq_.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller deactivated");
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// Joint state update
// ============================================================================
void
ImpedanceController::updateJointStates()
{
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

// ============================================================================
// Trajectory logic
// ============================================================================
void
ImpedanceController::calculateDesiredTrajectory()
{
  // Process new external command (resets timing for smooth trajectories)
  if (new_command_received_) {
    processExternalCommand();
    new_command_received_ = false;
  }

  // External command active?
  if (external_command_active_) {

    // -----------------------------
    // DIRECT TRACKING (streaming)
    // -----------------------------
    if (direct_tracking_mode_) {

      // Velocity-only streaming: hold position at current q_ to avoid stiffness fight
      if (velocity_only_streaming_) {
        q_desired_ = q_;                 // cancel K*(q_des - q)
        dq_desired_ = external_dq_target_;
        ddq_desired_.setZero();
        return;
      }

      // Full direct tracking (pos + vel + acc)
      q_desired_ = external_q_target_;
      dq_desired_ = external_dq_target_;
      ddq_desired_ = external_ddq_target_;
      return;
    }

    // -----------------------------
    // SMOOTH trajectory to external target (pos)
    // -----------------------------
    const double T = std::max(trajectory_duration_, 1e-3);
    const double t = std::min(external_trajectory_time_ / T, 1.0);

    const double s      = 3.0 * t * t - 2.0 * t * t * t;
    const double ds_dt  = (6.0 * t - 6.0 * t * t) / T;
    const double d2s_dt2 = (6.0 - 12.0 * t) / (T * T);

    const Vector7d pos_diff = external_q_target_ - trajectory_start_q_;

    q_desired_   = trajectory_start_q_ + s * pos_diff;
    dq_desired_  = trajectory_start_dq_ + ds_dt * pos_diff;
    ddq_desired_ = d2s_dt2 * pos_diff;

    external_trajectory_time_ += elapsed_time_step_;

    // stop condition
    if (t >= 1.0 && (external_q_target_ - q_).norm() < 0.01) {
      external_command_active_ = false;
      external_trajectory_time_ = 0.0;
      RCLCPP_INFO(get_node()->get_logger(), "External target reached (smooth mode)");
    }
    return;
  }

  // ------------------------------------------------------------------
  // No external command: run internal mode behavior
  // ------------------------------------------------------------------
  switch (impedance_mode_) {
    case POSITION_IMPEDANCE:
      q_desired_ = q_target_;
      dq_desired_.setZero();
      ddq_desired_.setZero();
      break;

    case TRAJECTORY_IMPEDANCE: {
      const double T = 5.0;
      const double t = std::min(elapsed_time_ / T, 1.0);

      const double s      = 3.0 * t * t - 2.0 * t * t * t;
      const double ds_dt  = (6.0 * t - 6.0 * t * t) / T;
      const double d2s_dt2 = (6.0 - 12.0 * t) / (T * T);

      const Vector7d pos_diff = q_target_ - initial_q_;

      q_desired_   = initial_q_ + s * pos_diff;
      dq_desired_  = ds_dt * pos_diff;
      ddq_desired_ = d2s_dt2 * pos_diff;
      break;
    }

    case SINUSOIDAL_IMPEDANCE: {
      const double omega = 2.0 * M_PI * motion_frequency_;
      const double phase = omega * elapsed_time_;

      const double delta_angle = motion_amplitude_ * (1.0 - std::cos(phase));
      const double delta_vel   = motion_amplitude_ * omega * std::sin(phase);
      const double delta_acc   = motion_amplitude_ * omega * omega * std::cos(phase);

      q_desired_ = initial_q_;
      dq_desired_.setZero();
      ddq_desired_.setZero();

      // joints 4 and 5 (0-indexed: 3,4)
      q_desired_(3) += delta_angle;
      q_desired_(4) += delta_angle;

      dq_desired_(3) += delta_vel;
      dq_desired_(4) += delta_vel;

      ddq_desired_(3) += delta_acc;
      ddq_desired_(4) += delta_acc;
      break;
    }

    case COMPLIANT_MOTION: {
      // A simple compliant behavior (your original idea)
      const Vector7d compliance_offset = compliance_factor_ * 0.1 * dq_filtered_;
      q_desired_ = initial_q_ + compliance_offset;
      dq_desired_ = compliance_factor_ * dq_filtered_;
      ddq_desired_.setZero();
      break;
    }
  }

  // joint limit check
  if (!validateJointLimits(q_desired_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Desired position exceeds joint limits");
  }
}

// ============================================================================
// Pose command callback
// ============================================================================
void
ImpedanceController::commandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/)
{
  // Placeholder: IK not implemented
  RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                       "Received pose command, but IK conversion is not implemented. Ignoring.");
}

// ============================================================================
// Joint command callback (supports pos-only, vel-only, or pos+vel)
// ============================================================================
void
ImpedanceController::jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool has_pos = (msg->position.size() == static_cast<size_t>(num_joints));
  const bool has_vel = (msg->velocity.size() == static_cast<size_t>(num_joints));

  // Allow velocity-only (bridge case)
  if (!has_pos && !has_vel) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Joint command invalid: need position[%d] and/or velocity[%d]. Got position[%zu], velocity[%zu].",
      num_joints, num_joints, msg->position.size(), msg->velocity.size());
    return;
  }

  // Update target position if provided
  if (has_pos) {
    for (int i = 0; i < num_joints; ++i) {
      external_q_target_(i) = msg->position[i];
    }
    if (!validateJointLimits(external_q_target_)) {
      RCLCPP_WARN(get_node()->get_logger(), "Joint position command rejected: exceeds limits");
      external_command_active_ = false;
      return;
    }
  }

  // Update target velocity if provided
  if (has_vel) {
    for (int i = 0; i < num_joints; ++i) {
      external_dq_target_(i) = msg->velocity[i];
    }
  } else {
    external_dq_target_.setZero();
  }

  // We don't use accelerations for velocity bridge
  external_ddq_target_.setZero();

  // Decide direct vs smooth:
  // - velocity-only streaming MUST be direct tracking
  // - otherwise: allow "direct_track" marker to enable direct
  velocity_only_streaming_ = (has_vel && !has_pos);

  if (velocity_only_streaming_) {
    direct_tracking_mode_ = true;
  } else {
    // keep your previous convention if you want: name[0] == "direct_track"
    if (!msg->name.empty() && msg->name[0] == "direct_track") {
      direct_tracking_mode_ = true;
    } else {
      direct_tracking_mode_ = false;
    }
  }

  external_command_active_ = true;
  new_command_received_ = true;
}

// ============================================================================
// Config callback
// ============================================================================
void
ImpedanceController::impedanceConfigCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const std::string command = msg->data;

  if (command == "stop") {
    external_command_active_ = false;
    direct_tracking_mode_ = false;
    velocity_only_streaming_ = false;

    q_target_ = q_;  // hold current position (internal hold)
    RCLCPP_INFO(get_node()->get_logger(), "Emergency stop activated");
    return;
  }

  if (command == "home") {
    external_q_target_ = initial_q_;
    external_dq_target_.setZero();
    external_ddq_target_.setZero();

    external_command_active_ = true;
    direct_tracking_mode_ = false;       // smooth
    velocity_only_streaming_ = false;
    new_command_received_ = true;

    RCLCPP_INFO(get_node()->get_logger(), "Homing command activated");
    return;
  }

  if (command == "direct_tracking_on") {
    direct_tracking_mode_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking enabled");
    return;
  }

  if (command == "direct_tracking_off") {
    direct_tracking_mode_ = false;
    velocity_only_streaming_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking disabled");
    return;
  }

  if (command.rfind("trajectory_duration:", 0) == 0) {
    try {
      const double new_duration = std::stod(command.substr(std::string("trajectory_duration:").size()));
      trajectory_duration_ = std::max(0.1, new_duration);
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory duration set to %.2f s", trajectory_duration_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid trajectory duration: %s", e.what());
    }
    return;
  }

  if (command.rfind("mode:", 0) == 0) {
    const std::string mode_str = command.substr(5);
    if (mode_str == "position_impedance") {
      impedance_mode_ = POSITION_IMPEDANCE;
    } else if (mode_str == "trajectory_impedance") {
      impedance_mode_ = TRAJECTORY_IMPEDANCE;
    } else if (mode_str == "sinusoidal_impedance") {
      impedance_mode_ = SINUSOIDAL_IMPEDANCE;
    } else if (mode_str == "compliant_motion") {
      impedance_mode_ = COMPLIANT_MOTION;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Impedance mode changed to '%s'", mode_str.c_str());
    return;
  }

  if (command.rfind("compliance:", 0) == 0) {
    try {
      const double new_c = std::stod(command.substr(std::string("compliance:").size()));
      compliance_factor_ = std::clamp(new_c, 0.0, 2.0);
      RCLCPP_INFO(get_node()->get_logger(), "Compliance factor set to %.2f", compliance_factor_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid compliance value: %s", e.what());
    }
    return;
  }

  RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                       "Unknown impedance config command: '%s'", command.c_str());
}

// ============================================================================
// External command processing
// ============================================================================
void
ImpedanceController::processExternalCommand()
{
  if (!external_command_active_) {
    return;
  }

  // For smooth trajectories, reset the external timer + store start state.
  // For direct tracking, this doesn't hurt, but also isn't needed.
  external_trajectory_time_ = 0.0;
  trajectory_start_q_ = q_;
  trajectory_start_dq_ = dq_filtered_;
}

// ============================================================================
// Control law
// ============================================================================
Vector7d
ImpedanceController::calculateImpedanceTorques()
{
  const Vector7d position_error = q_desired_ - q_;
  const Vector7d velocity_error = dq_desired_ - dq_filtered_;

  // NOTE: m_gains_ is currently unused as "true inertia"; kept for future extension.
  // If you want feedforward accel: tau += m_gains_.cwiseProduct(ddq_desired_);
  Vector7d tau_d =
    d_gains_.cwiseProduct(velocity_error) +
    k_gains_.cwiseProduct(position_error);

  return tau_d;
}

void
ImpedanceController::applyVelocityFilter(double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  dq_filtered_ = (1.0 - alpha) * dq_filtered_ + alpha * dq_;
}

bool
ImpedanceController::validateJointLimits(const Vector7d& q) const
{
  for (int i = 0; i < num_joints; ++i) {
    if (q(i) < joint_limits_lower[i] || q(i) > joint_limits_upper[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace iitgn_controllers

PLUGINLIB_EXPORT_CLASS(iitgn_controllers::ImpedanceController,
                       controller_interface::ControllerInterface)
