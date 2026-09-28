#include <iitgn_controllers/cartesian_impedance_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace iitgn_controllers {

// ============================================================================
// Interface configuration
// ============================================================================

controller_interface::InterfaceConfiguration
SpringController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
SpringController::state_interface_configuration() const
{
  // Fixed order — offsets must match updateJointStates():
  //   [0  .. 15] : FrankaCartesianPoseInterface (O_T_EE, 16 values)
  //   [16 .. 22] : joint positions  (7)
  //   [23 .. 29] : joint velocities (7)
  //   [30+]      : FrankaRobotModel (Jacobian, Coriolis, ...)
  //   [last]     : robot_time
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& n : franka_cartesian_pose_->get_state_interface_names()) { config.names.push_back(n); }
  for (int i = 1; i <= num_joints; ++i) { config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position"); }
  for (int i = 1; i <= num_joints; ++i) { config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity"); }
  for (const auto& n : franka_robot_model_->get_state_interface_names())    { config.names.push_back(n); }
  config.names.push_back(arm_id_ + "/robot_time");

  return config;
}

// ============================================================================
// update() — 1 kHz real-time loop
// ============================================================================

controller_interface::return_type
SpringController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  const double dt    = std::max(period.seconds(), 1e-6);
  elapsed_time_step_ = dt;
  elapsed_time_     += dt;

  updateJointStates();

  // Read current filter alpha under params lock
  double vel_filter_alpha;
  double gain;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    vel_filter_alpha = velocity_filter_alpha_;
    gain             = filter_gain_;
  }
  applyVelocityFilter(vel_filter_alpha);

  // One-time init: latch EE pose on first call after activation
  if (initialization_flag_) {
    std::tie(initial_orientation_, initial_position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();

    position_d_target_            = initial_position_;
    orientation_d_target_         = initial_orientation_;
    position_d_                   = initial_position_;
    orientation_d_                = initial_orientation_;
    trajectory_start_position_    = initial_position_;
    trajectory_start_orientation_ = initial_orientation_;
    external_position_target_     = initial_position_;
    external_orientation_target_  = initial_orientation_;
    initial_q_                    = q_;
    initial_robot_time_           = state_interfaces_.back().get_value();
    last_cmd_time_                = time;
    initialization_flag_          = false;
  } else {
    robot_time_ = state_interfaces_.back().get_value();
  }

  calculateDesiredCartesianTrajectory(time);

  // Smooth filter: slide desired pose toward target
  position_d_    = gain * position_d_target_ + (1.0 - gain) * position_d_;
  orientation_d_ = orientation_d_.slerp(gain, orientation_d_target_);

  Vector7d tau_d = calculateCartesianImpedanceTorques();

  double delta_tau_max;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    delta_tau_max = k_delta_tau_max_;
  }
  tau_d    = saturateTorqueRate(tau_d, tau_J_d_, delta_tau_max);
  tau_J_d_ = tau_d;

  for (int i = 0; i < num_joints; ++i) {
    tau_d(i) = std::clamp(tau_d(i), -max_torques[i], max_torques[i]);
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

// ============================================================================
// on_init()
// ============================================================================

CallbackReturn SpringController::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("cartesian_stiffness", std::vector<double>{});
    auto_declare<std::vector<double>>("cartesian_damping", std::vector<double>{});
    auto_declare<double>("nullspace_stiffness", 0.5);
    auto_declare<std::vector<double>>("q_d_nullspace", std::vector<double>{});
    auto_declare<double>("filter_gain", 0.1);
    auto_declare<double>("velocity_filter_alpha", 0.3);
    auto_declare<double>("delta_tau_max", 1.0);
    auto_declare<double>("cmd_timeout_sec", 0.025);
    auto_declare<std::string>("impedance_mode", "position_impedance");
    auto_declare<double>("compliance_factor", 0.5);
    auto_declare<double>("trajectory_duration", 5.0);
    auto_declare<std::string>("command_topic", "/spring_command");
    auto_declare<std::string>("cmd_topic", "/NS_1/joint_velocity_controller/commands");

    RCLCPP_INFO(get_node()->get_logger(), "SpringController initialized.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure()
// ============================================================================

CallbackReturn SpringController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    auto node = get_node();

    // ---- Read structural parameters (not runtime-changeable) ----------------
    arm_id_        = node->get_parameter("arm_id").as_string();
    command_topic_ = node->get_parameter("command_topic").as_string();
    cmd_topic_     = node->get_parameter("cmd_topic").as_string();

    // ---- Read all tunable parameters under params_mutex_ --------------------
    {
      std::lock_guard<std::mutex> lock(params_mutex_);

      filter_gain_           = node->get_parameter("filter_gain").as_double();
      velocity_filter_alpha_ = std::clamp(node->get_parameter("velocity_filter_alpha").as_double(), 0.0, 1.0);
      k_delta_tau_max_       = node->get_parameter("delta_tau_max").as_double();
      cmd_timeout_sec_       = node->get_parameter("cmd_timeout_sec").as_double();
      nullspace_stiffness_   = node->get_parameter("nullspace_stiffness").as_double();
      compliance_factor_     = node->get_parameter("compliance_factor").as_double();
      trajectory_duration_   = node->get_parameter("trajectory_duration").as_double();

      const std::string mode_str = node->get_parameter("impedance_mode").as_string();
      if      (mode_str == "position_impedance")   impedance_mode_ = POSITION_IMPEDANCE;
      else if (mode_str == "trajectory_impedance") impedance_mode_ = TRAJECTORY_IMPEDANCE;
      else if (mode_str == "compliant_motion")     impedance_mode_ = COMPLIANT_MOTION;
      else {
        RCLCPP_WARN(node->get_logger(), "Unknown mode '%s', using position_impedance", mode_str.c_str());
        impedance_mode_ = POSITION_IMPEDANCE;
      }

      const auto k_cart = node->get_parameter("cartesian_stiffness").as_double_array();
      if (k_cart.size() != 6) {
        RCLCPP_FATAL(node->get_logger(), "cartesian_stiffness needs 6 elements, got %zu", k_cart.size());
        return CallbackReturn::FAILURE;
      }
      cartesian_stiffness_.setZero();
      for (int i = 0; i < 6; ++i) cartesian_stiffness_(i, i) = k_cart[i];

      const auto d_cart = node->get_parameter("cartesian_damping").as_double_array();
      if (d_cart.size() != 6) {
        RCLCPP_FATAL(node->get_logger(), "cartesian_damping needs 6 elements, got %zu", d_cart.size());
        return CallbackReturn::FAILURE;
      }
      cartesian_damping_.setZero();
      for (int i = 0; i < 6; ++i) cartesian_damping_(i, i) = d_cart[i];

      // cartesian_stiffness and cartesian_damping are fully independent.
      // Set any values you want in the YAML — no formula is enforced.
      // Typical starting point: damping = 2*sqrt(stiffness) for critical damping,
      // but you are free to tune both independently for your use case.

      const auto q_ns = node->get_parameter("q_d_nullspace").as_double_array();
      if (q_ns.size() != static_cast<size_t>(num_joints)) {
        RCLCPP_FATAL(node->get_logger(), "q_d_nullspace needs %d elements, got %zu", num_joints, q_ns.size());
        return CallbackReturn::FAILURE;
      }
      for (int i = 0; i < num_joints; ++i) q_d_nullspace_(i) = q_ns[i];

      RCLCPP_INFO(node->get_logger(), "=== SpringController configured ===");
      RCLCPP_INFO(node->get_logger(), "  arm_id           : %s", arm_id_.c_str());
      RCLCPP_INFO(node->get_logger(), "  cmd_topic        : %s", cmd_topic_.c_str());
      RCLCPP_INFO(node->get_logger(), "  filter_gain      : %.4f", filter_gain_);
      RCLCPP_INFO(node->get_logger(), "  vel_filter_alpha : %.3f", velocity_filter_alpha_);
      RCLCPP_INFO(node->get_logger(), "  delta_tau_max    : %.2f Nm", k_delta_tau_max_);
      RCLCPP_INFO(node->get_logger(), "  cmd_timeout      : %.3f s", cmd_timeout_sec_);
      RCLCPP_INFO(node->get_logger(), "  nullspace_k      : %.3f", nullspace_stiffness_);
      RCLCPP_INFO(node->get_logger(),
                  "  stiffness [tx,ty,tz,rx,ry,rz]: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                  k_cart[0], k_cart[1], k_cart[2], k_cart[3], k_cart[4], k_cart[5]);
      RCLCPP_INFO(node->get_logger(),
                  "  damping   [tx,ty,tz,rx,ry,rz]: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  d_cart[0], d_cart[1], d_cart[2], d_cart[3], d_cart[4], d_cart[5]);
    }

    // ---- Franka semantic components -----------------------------------------
    franka_cartesian_pose_ =
        std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
            franka_semantic_components::FrankaCartesianPoseInterface("", false));

    franka_robot_model_ =
        std::make_unique<franka_semantic_components::FrankaRobotModel>(
            franka_semantic_components::FrankaRobotModel(
                arm_id_ + "/" + k_robot_model_interface_name,
                arm_id_ + "/" + k_robot_state_interface_name));

    // ---- Subscribers --------------------------------------------------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    command_subscriber_ =
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            command_topic_, qos,
            std::bind(&SpringController::commandCallback, this, std::placeholders::_1));

    joint_command_subscriber_ =
        node->create_subscription<sensor_msgs::msg::JointState>(
            command_topic_ + "_joint", qos,
            std::bind(&SpringController::jointCommandCallback, this, std::placeholders::_1));

    impedance_config_subscriber_ =
        node->create_subscription<std_msgs::msg::String>(
            command_topic_ + "_config", rclcpp::QoS(10),
            std::bind(&SpringController::impedanceConfigCallback, this, std::placeholders::_1));

    vel_cmd_subscriber_ =
        node->create_subscription<std_msgs::msg::Float64MultiArray>(
            cmd_topic_, qos,
            std::bind(&SpringController::cartesianVelCmdCallback, this, std::placeholders::_1));

    // ---- Register runtime parameter callback --------------------------------
    // This allows: ros2 param set /NS_1/spring_controller cartesian_stiffness "[200,200,200,20,20,20]"
    // The callback fires immediately and updates the gains without any restart.
    parameter_callback_handle_ = node->add_on_set_parameters_callback(
        std::bind(&SpringController::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(),
                "Runtime parameter updates enabled. Use: "
                "ros2 param set /NS_1/spring_controller <param_name> <value>");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_configure exception: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// parametersCallback() — called by ROS2 on every ros2 param set
// Updates all tunable gains immediately without restart.
// NOT called for arm_id, command_topic, cmd_topic — those are structural.
// ============================================================================

rcl_interfaces::msg::SetParametersResult
SpringController::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(params_mutex_);

  for (const auto& param : parameters) {
    const std::string& name = param.get_name();

    if (name == "cartesian_stiffness") {
      const auto vals = param.as_double_array();
      if (vals.size() != 6) {
        result.successful = false;
        result.reason = "cartesian_stiffness needs exactly 6 values";
        continue;
      }
      for (int i = 0; i < 6; ++i) cartesian_stiffness_(i, i) = vals[i];
      RCLCPP_INFO(get_node()->get_logger(),
                  "[RUNTIME] stiffness updated: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                  vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);

    } else if (name == "cartesian_damping") {
      const auto vals = param.as_double_array();
      if (vals.size() != 6) {
        result.successful = false;
        result.reason = "cartesian_damping needs exactly 6 values";
        continue;
      }
      for (int i = 0; i < 6; ++i) cartesian_damping_(i, i) = vals[i];
      RCLCPP_INFO(get_node()->get_logger(),
                  "[RUNTIME] damping updated: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);

    } else if (name == "nullspace_stiffness") {
      nullspace_stiffness_ = param.as_double();
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] nullspace_stiffness = %.3f", nullspace_stiffness_);

    } else if (name == "q_d_nullspace") {
      const auto vals = param.as_double_array();
      if (vals.size() != static_cast<size_t>(num_joints)) {
        result.successful = false;
        result.reason = "q_d_nullspace needs 7 values";
        continue;
      }
      for (int i = 0; i < num_joints; ++i) q_d_nullspace_(i) = vals[i];
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] q_d_nullspace updated");

    } else if (name == "filter_gain") {
      filter_gain_ = std::clamp(param.as_double(), 0.001, 1.0);
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] filter_gain = %.4f", filter_gain_);

    } else if (name == "velocity_filter_alpha") {
      velocity_filter_alpha_ = std::clamp(param.as_double(), 0.0, 1.0);
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] velocity_filter_alpha = %.3f", velocity_filter_alpha_);

    } else if (name == "delta_tau_max") {
      k_delta_tau_max_ = std::max(0.1, param.as_double());
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] delta_tau_max = %.2f Nm", k_delta_tau_max_);

    } else if (name == "cmd_timeout_sec") {
      cmd_timeout_sec_ = std::max(0.005, param.as_double());
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] cmd_timeout_sec = %.3f s", cmd_timeout_sec_);

    } else if (name == "compliance_factor") {
      compliance_factor_ = std::clamp(param.as_double(), 0.0, 2.0);
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] compliance_factor = %.2f", compliance_factor_);

    } else if (name == "trajectory_duration") {
      trajectory_duration_ = std::max(0.1, param.as_double());
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] trajectory_duration = %.2f s", trajectory_duration_);

    } else if (name == "impedance_mode") {
      const std::string m = param.as_string();
      if (m == "position_impedance")        impedance_mode_ = POSITION_IMPEDANCE;
      else if (m == "trajectory_impedance") impedance_mode_ = TRAJECTORY_IMPEDANCE;
      else if (m == "compliant_motion")     impedance_mode_ = COMPLIANT_MOTION;
      else {
        result.successful = false;
        result.reason = "Unknown mode: " + m;
        continue;
      }
      RCLCPP_INFO(get_node()->get_logger(), "[RUNTIME] impedance_mode = %s", m.c_str());

    } else if (name == "arm_id" || name == "command_topic" || name == "cmd_topic") {
      result.successful = false;
      result.reason = name + " cannot be changed at runtime (requires controller restart)";
      RCLCPP_WARN(get_node()->get_logger(), "%s", result.reason.c_str());
    }
    // Unknown params are silently allowed (may belong to other subsystems)
  }
  return result;
}

// ============================================================================
// on_activate()
// ============================================================================

CallbackReturn SpringController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  initialization_flag_      = true;
  elapsed_time_             = 0.0;
  elapsed_time_step_        = 0.0;
  tau_J_d_.setZero();
  dq_filtered_.setZero();
  dq_cmd_.setZero();
  new_command_received_     = false;
  external_command_active_  = false;
  velocity_only_streaming_  = false;
  external_trajectory_time_ = 0.0;
  last_cmd_time_            = get_node()->get_clock()->now();

  RCLCPP_INFO(get_node()->get_logger(), "SpringController activated.");
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_deactivate()
// ============================================================================

CallbackReturn SpringController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_cartesian_pose_->release_interfaces();
  franka_robot_model_->release_interfaces();
  RCLCPP_INFO(get_node()->get_logger(), "SpringController deactivated.");
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// updateJointStates()
// ============================================================================

void SpringController::updateJointStates()
{
  constexpr int kPoseOffset     = 16;
  constexpr int kVelocityOffset = 23;
  for (int i = 0; i < num_joints; ++i) {
    q_(i)  = state_interfaces_.at(kPoseOffset     + i).get_value();
    dq_(i) = state_interfaces_.at(kVelocityOffset + i).get_value();
  }
}

// ============================================================================
// calculateDesiredCartesianTrajectory()
// ============================================================================

void SpringController::calculateDesiredCartesianTrajectory(const rclcpp::Time& time)
{
  if (new_command_received_) {
    processExternalCommand();
    new_command_received_ = false;
  }

  if (external_command_active_) {

    if (velocity_only_streaming_) {
      // ------------------------------------------------------------------
      // Check for stop condition:
      //   (a) timeout  — publisher has stopped sending messages
      //   (b) zero cmd — Python explicitly sent zeros before stopping
      //
      // In BOTH cases: snap position_d_ and position_d_target_ to the
      // actual current EE position. This makes cartesian_error = 0
      // immediately, eliminating all residual spring force and drift.
      // ------------------------------------------------------------------

      const double cmd_age = (time - last_cmd_time_).seconds();

      double timeout;
      {
        std::lock_guard<std::mutex> lock(params_mutex_);
        timeout = cmd_timeout_sec_;
      }

      bool is_zero;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        is_zero = (dq_cmd_.norm() < 1e-6);
      }

      const bool should_stop = (cmd_age > timeout) || is_zero;

      if (should_stop) {
        // Snap desired pose to actual current EE position.
        // cartesian_error = 0 immediately → no spring force → clean stop.
        auto [cur_ori, cur_pos] =
            franka_cartesian_pose_->getCurrentOrientationAndTranslation();

        position_d_target_    = cur_pos;
        orientation_d_target_ = cur_ori;
        position_d_           = cur_pos;
        orientation_d_        = cur_ori;

        // Zero the velocity command but KEEP streaming mode active.
        // This holds the current position instead of falling back to
        // POSITION_IMPEDANCE which would pull the robot to initial_position_.
        // Behaviour matches joint_velocity_controller: robot stops at current pose.
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          dq_cmd_.setZero();
          // external_command_active_ and velocity_only_streaming_ stay true.
          // Next cycle: is_zero=true → snap again → stable hold at current pose.
        }
        return;
      }

      // ------------------------------------------------------------------
      // Normal streaming: integrate dq_cmd every 1kHz cycle.
      // 5 × (v × 1ms) == 1 × (v × 5ms) — same displacement as 200Hz,
      // but continuous rather than stepwise → smooth motion.
      // ------------------------------------------------------------------
      std::array<double, 42> jacobian_array =
          franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
      Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

      Vector6d cart_vel;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cart_vel = J * dq_cmd_;
      }

      position_d_target_ += cart_vel.head(3) * elapsed_time_step_;

      const Eigen::Vector3d omega = cart_vel.tail(3);
      const double angle = omega.norm() * elapsed_time_step_;
      if (angle > 1e-10) {
        orientation_d_target_ =
            Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized())) *
            orientation_d_target_;
        orientation_d_target_.normalize();
      }
      return;
    }

    // ---- Smooth trajectory mode ------------------------------------------
    const double T = std::max(trajectory_duration_, 1e-3);
    const double t = std::min(external_trajectory_time_ / T, 1.0);
    const double s = 3.0 * t * t - 2.0 * t * t * t;

    position_d_target_ =
        trajectory_start_position_ +
        s * (external_position_target_ - trajectory_start_position_);
    orientation_d_target_ =
        trajectory_start_orientation_.slerp(s, external_orientation_target_);

    external_trajectory_time_ += elapsed_time_step_;

    if (t >= 1.0 && (external_position_target_ - position_d_).norm() < 0.005) {
      external_command_active_  = false;
      external_trajectory_time_ = 0.0;
      RCLCPP_INFO(get_node()->get_logger(), "Cartesian target reached.");
    }
    return;
  }

  // ---- Internal mode -------------------------------------------------------
  double traj_dur;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    traj_dur = trajectory_duration_;
  }

  switch (impedance_mode_) {
    case POSITION_IMPEDANCE:
      position_d_target_    = initial_position_;
      orientation_d_target_ = initial_orientation_;
      break;

    case TRAJECTORY_IMPEDANCE: {
      const double T = std::max(traj_dur, 1e-3);
      const double t = std::min(elapsed_time_ / T, 1.0);
      const double s = 3.0 * t * t - 2.0 * t * t * t;
      position_d_target_    = initial_position_ + s * (external_position_target_ - initial_position_);
      orientation_d_target_ = initial_orientation_.slerp(s, external_orientation_target_);
      break;
    }

    case COMPLIANT_MOTION:
      position_d_target_    = position_d_;
      orientation_d_target_ = orientation_d_;
      break;
  }
}

// ============================================================================
// calculateCartesianImpedanceTorques()
// F = K*Δx - D*ẋ,  τ = Jᵀ*F + τ_coriolis + τ_nullspace
// ============================================================================

SpringController::Vector7d
SpringController::calculateCartesianImpedanceTorques()
{
  auto [current_orientation, current_position] =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();

  // Position error
  const Eigen::Vector3d position_error = position_d_ - current_position;

  // Orientation error — shortest path
  Eigen::Quaterniond q_d = orientation_d_;
  const Eigen::Quaterniond q_c = current_orientation;
  if (q_c.coeffs().dot(q_d.coeffs()) < 0.0) q_d.coeffs() = -q_d.coeffs();
  const Eigen::Vector3d orient_err = (q_d * q_c.inverse()).vec();

  Vector6d cartesian_error;
  cartesian_error.head(3) = position_error;
  cartesian_error.tail(3) = orient_err;

  // Robot model quantities
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<Vector7d> coriolis(coriolis_array.data());

  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());

  const Vector6d cartesian_velocity = J * dq_filtered_;

  // Read gains under params lock
  Matrix6d K, D;
  double ns_k, comp;
  Vector7d q_ns;
  ControllerMode mode;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    K    = cartesian_stiffness_;
    D    = cartesian_damping_;
    ns_k = nullspace_stiffness_;
    comp = compliance_factor_;
    q_ns = q_d_nullspace_;
    mode = impedance_mode_;
  }

  // K and D are fully independent — set any values in YAML.
  // In compliant mode, both are scaled to keep their ratio intact.
  const double k_scale = (mode == COMPLIANT_MOTION) ? comp : 1.0;
  const double d_scale = (mode == COMPLIANT_MOTION) ? std::sqrt(comp) : 1.0;

  const Vector6d F_impedance =
      (k_scale * K) * cartesian_error - (d_scale * D) * cartesian_velocity;

  const Vector7d tau_task = J.transpose() * F_impedance;

  // Nullspace: keeps elbow near preferred config without disturbing EE
  const Eigen::MatrixXd JJt   = J * J.transpose();
  const Eigen::MatrixXd J_inv = J.transpose() * JJt.inverse();
  const Vector7d tau_nullspace =
      (Eigen::MatrixXd::Identity(7, 7) - J.transpose() * J_inv.transpose()) *
      (ns_k * (q_ns - q_) - (2.0 * std::sqrt(ns_k)) * dq_filtered_);

  return tau_task + coriolis + tau_nullspace;
}

// ============================================================================
// processExternalCommand()
// ============================================================================

void SpringController::processExternalCommand()
{
  if (!external_command_active_) return;
  trajectory_start_position_    = position_d_;
  trajectory_start_orientation_ = orientation_d_;
  external_trajectory_time_     = 0.0;
}

// ============================================================================
// commandCallback() — PoseStamped direct Cartesian target
// ============================================================================

void SpringController::commandCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) return;

  Eigen::Vector3d new_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond new_ori(msg->pose.orientation.w, msg->pose.orientation.x,
                             msg->pose.orientation.y, msg->pose.orientation.z);
  new_ori.normalize();

  if (!validateCartesianLimits(new_pos)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Pose rejected: [%.3f,%.3f,%.3f] exceeds %.3f m workspace limit.",
                new_pos.x(), new_pos.y(), new_pos.z(), k_workspace_limit_);
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  external_position_target_    = new_pos;
  external_orientation_target_ = new_ori;
  external_command_active_     = true;
  velocity_only_streaming_     = false;
  new_command_received_        = true;
}

// ============================================================================
// jointCommandCallback() — JointState velocity-only
// ============================================================================

void SpringController::jointCommandCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) return;
  if (msg->velocity.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Joint cmd needs %d velocities. Ignoring.", num_joints);
    return;
  }
  constexpr double kCmdAlpha = 0.5;
  std::lock_guard<std::mutex> lock(command_mutex_);
  for (int i = 0; i < num_joints; ++i) {
    dq_cmd_(i) = (1.0 - kCmdAlpha) * dq_cmd_(i) + kCmdAlpha * msg->velocity[i];
  }
  external_command_active_ = true;
  velocity_only_streaming_ = true;
  new_command_received_    = true;
  last_cmd_time_           = get_node()->get_clock()->now();
}

// ============================================================================
// cartesianVelCmdCallback() — Float64MultiArray
// Primary path for Python pick-and-place scripts.
// Same topic / message type as joint_velocity_controller.
// ============================================================================

void SpringController::cartesianVelCmdCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg || msg->data.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Expected %d velocities, got %zu. Ignoring.",
                         num_joints, msg ? msg->data.size() : 0UL);
    return;
  }
  // Smooth commanded velocity to reduce jerkiness at waypoint transitions.
  // Exponential filter: dq_cmd_ = (1-alpha)*dq_cmd_ + alpha*new_cmd
  // alpha=0.5 → ~3 message cycles (15ms at 200Hz) to reach new velocity.
  // This prevents abrupt direction changes from spiking the spring force.
  constexpr double kCmdAlpha = 0.5;
  std::lock_guard<std::mutex> lock(command_mutex_);
  for (int i = 0; i < num_joints; ++i) {
    dq_cmd_(i) = (1.0 - kCmdAlpha) * dq_cmd_(i) + kCmdAlpha * msg->data[i];
  }
  external_command_active_ = true;
  velocity_only_streaming_ = true;
  new_command_received_    = true;
  last_cmd_time_           = get_node()->get_clock()->now();
}

// ============================================================================
// impedanceConfigCallback() — runtime reconfiguration via String topic
// Commands: stop | home | mode:<n> | compliance:<val> | trajectory_duration:<val>
// ============================================================================

void SpringController::impedanceConfigCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) return;
  const std::string cmd = msg->data;

  if (cmd == "stop") {
    // Snap to current EE pose and hold it.
    // Keep streaming mode active with zero velocity so the robot holds
    // exactly where it is — does NOT move back to initial_position_.
    auto [cur_ori, cur_pos] =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    position_d_target_    = cur_pos;
    orientation_d_target_ = cur_ori;
    position_d_           = cur_pos;
    orientation_d_        = cur_ori;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      dq_cmd_.setZero();
      external_command_active_ = true;
      velocity_only_streaming_ = true;
    }
    RCLCPP_INFO(get_node()->get_logger(), "spring_controller: STOP — holding current EE pose.");
    return;
  }

  if (cmd == "home") {
    std::lock_guard<std::mutex> lock(command_mutex_);
    external_position_target_    = initial_position_;
    external_orientation_target_ = initial_orientation_;
    external_command_active_     = true;
    velocity_only_streaming_     = false;
    new_command_received_        = true;
    RCLCPP_INFO(get_node()->get_logger(), "spring_controller: homing to initial EE pose.");
    return;
  }

  if (cmd.rfind("mode:", 0) == 0) {
    const std::string m = cmd.substr(5);
    std::lock_guard<std::mutex> lock(params_mutex_);
    if (m == "position_impedance")        impedance_mode_ = POSITION_IMPEDANCE;
    else if (m == "trajectory_impedance") impedance_mode_ = TRAJECTORY_IMPEDANCE;
    else if (m == "compliant_motion")     impedance_mode_ = COMPLIANT_MOTION;
    else { RCLCPP_WARN(get_node()->get_logger(), "Unknown mode: %s", m.c_str()); return; }
    RCLCPP_INFO(get_node()->get_logger(), "spring_controller: mode -> %s", m.c_str());
    return;
  }

  if (cmd.rfind("compliance:", 0) == 0) {
    try {
      std::lock_guard<std::mutex> lock(params_mutex_);
      compliance_factor_ = std::clamp(std::stod(cmd.substr(11)), 0.0, 2.0);
      RCLCPP_INFO(get_node()->get_logger(), "spring_controller: compliance = %.2f", compliance_factor_);
    } catch (...) {}
    return;
  }

  if (cmd.rfind("trajectory_duration:", 0) == 0) {
    try {
      std::lock_guard<std::mutex> lock(params_mutex_);
      trajectory_duration_ = std::max(0.1, std::stod(cmd.substr(20)));
      RCLCPP_INFO(get_node()->get_logger(), "spring_controller: traj_dur = %.2f s", trajectory_duration_);
    } catch (...) {}
    return;
  }

  RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                       "spring_controller: unknown command '%s'", cmd.c_str());
}

// ============================================================================
// Helpers
// ============================================================================

void SpringController::applyVelocityFilter(double alpha)
{
  alpha        = std::clamp(alpha, 0.0, 1.0);
  dq_filtered_ = (1.0 - alpha) * dq_filtered_ + alpha * dq_;
}

bool SpringController::validateCartesianLimits(const Eigen::Vector3d& position) const
{
  return position.norm() <= k_workspace_limit_;
}

SpringController::Vector7d
SpringController::saturateTorqueRate(const Vector7d& tau_d_calculated,
                                     const Vector7d& tau_J_d,
                                     double delta_tau_max)
{
  Vector7d out;
  for (int i = 0; i < num_joints; ++i) {
    const double diff = tau_d_calculated(i) - tau_J_d(i);
    out(i) = tau_J_d(i) + std::max(std::min(diff, delta_tau_max), -delta_tau_max);
  }
  return out;
}

}  // namespace iitgn_controllers

PLUGINLIB_EXPORT_CLASS(iitgn_controllers::SpringController,
                       controller_interface::ControllerInterface)