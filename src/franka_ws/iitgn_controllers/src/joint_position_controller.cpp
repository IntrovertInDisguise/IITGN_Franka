#include <iitgn_controllers/joint_position_controller.hpp>
#include <iitgn_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace iitgn_controllers {

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request effort interfaces for all joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Request position and velocity interfaces for all joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  
  // Store time step for trajectory calculations
  elapsed_time_step_ = period.seconds();
  
  // 1. Update current joint states
  updateJointStates();
  
  // 2. Update time
  elapsed_time_ += period.seconds();
  
  // 3. Calculate desired trajectory
  calculateDesiredTrajectory();
  
  // 4. Apply velocity filtering
  applyVelocityFilter();
  
  // 5. Calculate control torques
  Vector7d tau_d = calculateControlTorques();
  
  // 6. Apply safety limits and send commands
  for (int i = 0; i < num_joints; ++i) {
    // Clamp torques to maximum limits
    tau_d(i) = std::clamp(tau_d(i), -max_torques[i], max_torques[i]);
    
    // Send commands to robot
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  return controller_interface::return_type::OK;
}

CallbackReturn JointPositionController::on_init() {
  try {
    // Declare parameters
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("kp_gains", {});
    auto_declare<std::vector<double>>("kd_gains", {});
    auto_declare<std::vector<double>>("ki_gains", {});
    auto_declare<std::vector<double>>("target_position", {});
    auto_declare<std::string>("control_mode", "position_control");
    auto_declare<double>("motion_amplitude", 0.5);
    auto_declare<double>("motion_frequency", 0.2);
    auto_declare<std::string>("command_topic", "/joint_position_command");
    auto_declare<double>("trajectory_duration", 5.0);
    
    RCLCPP_INFO(get_node()->get_logger(), "IITGN Joint Position Controller initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  try {
    // Get parameters
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    auto kp_gains = get_node()->get_parameter("kp_gains").as_double_array();
    auto kd_gains = get_node()->get_parameter("kd_gains").as_double_array();
    auto ki_gains = get_node()->get_parameter("ki_gains").as_double_array();
    target_position_ = get_node()->get_parameter("target_position").as_double_array();
    std::string mode_str = get_node()->get_parameter("control_mode").as_string();
    motion_amplitude_ = get_node()->get_parameter("motion_amplitude").as_double();
    motion_frequency_ = get_node()->get_parameter("motion_frequency").as_double();
    command_topic_ = get_node()->get_parameter("command_topic").as_string();
    trajectory_duration_ = get_node()->get_parameter("trajectory_duration").as_double();
    
    // Validate gains
    if (kp_gains.size() != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), "kp_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    if (kd_gains.size() != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), "kd_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    if (ki_gains.size() != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), "ki_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    
    // Set gains
    for (int i = 0; i < num_joints; ++i) {
      kp_gains_(i) = kp_gains[i];
      kd_gains_(i) = kd_gains[i];
      ki_gains_(i) = ki_gains[i];
    }
    
    // Set control mode
    if (mode_str == "position_control") {
      control_mode_ = POSITION_CONTROL;
    } else if (mode_str == "trajectory_tracking") {
      control_mode_ = TRAJECTORY_TRACKING;
    } else if (mode_str == "sinusoidal_motion") {
      control_mode_ = SINUSOIDAL_MOTION;
    } else if (mode_str == "hold_position") {
      control_mode_ = HOLD_POSITION;
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Unknown control mode, using position_control");
      control_mode_ = POSITION_CONTROL;
    }
    
    // Initialize state variables
    q_.setZero();
    dq_.setZero();
    dq_filtered_.setZero();
    q_desired_.setZero();
    dq_desired_.setZero();
    ddq_desired_.setZero();
    q_target_.setZero();
    position_error_integral_.setZero();
    
    // Initialize external command variables
    external_q_target_.setZero();
    external_dq_target_.setZero();
    external_ddq_target_.setZero();
    trajectory_start_q_.setZero();
    trajectory_start_dq_.setZero();
    
    // Initialize command flags
    new_command_received_ = false;
    external_command_active_ = false;
    direct_tracking_mode_ = false;
    
    // Create subscribers
    joint_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        command_topic_, 10, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointCommandCallback(msg);
        });
    
    pose_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        command_topic_ + "_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->poseCommandCallback(msg);
        });
    
    config_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
        command_topic_ + "_config", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->configCallback(msg);
        });
    
    RCLCPP_INFO(get_node()->get_logger(), "IITGN Joint Position Controller configured successfully");
    RCLCPP_INFO(get_node()->get_logger(), "Control mode: %s", mode_str.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Command topic: %s", command_topic_.c_str());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Read initial joint states
  updateJointStates();
  
  // Store initial position
  q_initial_ = q_;
  
  // Set target position
  if (target_position_.size() == num_joints) {
    for (int i = 0; i < num_joints; ++i) {
      q_target_(i) = target_position_[i];
    }
  } else {
    q_target_ = q_initial_;  // Default to current position
  }
  
  // Reset time and filters
  elapsed_time_ = 0.0;
  external_trajectory_time_ = 0.0;
  dq_filtered_.setZero();
  dq_desired_.setZero();
  ddq_desired_.setZero();
  position_error_integral_.setZero();
  
  // Reset command flags
  new_command_received_ = false;
  external_command_active_ = false;
  
  RCLCPP_INFO(get_node()->get_logger(), "IITGN Joint Position Controller activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "IITGN Joint Position Controller deactivated");
  return CallbackReturn::SUCCESS;
}

void JointPositionController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointPositionController::calculateDesiredTrajectory() {
  // Check if we have a new external command
  if (new_command_received_) {
    processExternalCommand();
    new_command_received_ = false;
  }
  
  // If external command is active
  if (external_command_active_) {
    
    // DIRECT TRACKING MODE - for real-time control
    if (direct_tracking_mode_) {
      q_desired_ = external_q_target_;
      dq_desired_ = external_dq_target_;
      ddq_desired_ = external_ddq_target_;
      return;
    }
    
    // SMOOTH TRAJECTORY MODE - for point-to-point movements
    else {
      double t_normalized = std::min(external_trajectory_time_ / trajectory_duration_, 1.0);
      generateSmoothTrajectory(t_normalized, trajectory_start_q_, external_q_target_,
                             q_desired_, dq_desired_, ddq_desired_);
      
      // Update trajectory time
      external_trajectory_time_ += elapsed_time_step_;
      
      // Check if we've reached the target
      Vector7d position_error = external_q_target_ - q_;
      if (position_error.norm() < 0.01 && t_normalized >= 1.0) {
        external_command_active_ = false;
        external_trajectory_time_ = 0.0;
        RCLCPP_INFO(get_node()->get_logger(), "External command target reached");
      }
      return;
    }
  }
  
  // Built-in trajectory modes
  switch (control_mode_) {
    case POSITION_CONTROL:
      // Hold target position
      q_desired_ = q_target_;
      dq_desired_.setZero();
      ddq_desired_.setZero();
      break;
      
    case TRAJECTORY_TRACKING:
      // Smooth trajectory to target position
      {
        double t_normalized = std::min(elapsed_time_ / trajectory_duration_, 1.0);
        generateSmoothTrajectory(t_normalized, q_initial_, q_target_,
                               q_desired_, dq_desired_, ddq_desired_);
      }
      break;
      
    case SINUSOIDAL_MOTION:
      // Sinusoidal motion on specified joints
      {
        double omega = 2 * M_PI * motion_frequency_;
        double phase = omega * elapsed_time_;
        double delta_angle = motion_amplitude_ * std::sin(phase);
        double delta_velocity = motion_amplitude_ * omega * std::cos(phase);
        double delta_acceleration = -motion_amplitude_ * omega * omega * std::sin(phase);
        
        q_desired_ = q_initial_;
        dq_desired_.setZero();
        ddq_desired_.setZero();
        
        // Apply to joint 4 (index 3)
        q_desired_(3) += delta_angle;
        dq_desired_(3) += delta_velocity;
        ddq_desired_(3) += delta_acceleration;
      }
      break;
      
    case HOLD_POSITION:
      // Hold current position
      q_desired_ = q_;
      dq_desired_.setZero();
      ddq_desired_.setZero();
      break;
  }
  
  // Validate joint limits
  if (!validateJointLimits(q_desired_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Desired position exceeds joint limits");
  }
}

void JointPositionController::processExternalCommand() {
  if (external_command_active_) {
    // Reset trajectory timer for smooth motion
    external_trajectory_time_ = 0.0;
    
    // Store current position as starting point for trajectory
    trajectory_start_q_ = q_;
    trajectory_start_dq_ = dq_filtered_;
    
    RCLCPP_INFO(get_node()->get_logger(), "Processing external command - generating trajectory");
  }
}

Vector7d JointPositionController::calculateControlTorques() {
  // Position error
  Vector7d position_error = q_desired_ - q_;
  
  // Velocity error
  Vector7d velocity_error = dq_desired_ - dq_filtered_;
  
  // Integral of position error (with windup protection)
  position_error_integral_ += position_error * elapsed_time_step_;
  
  // Clamp integral term to prevent windup
  for (int i = 0; i < num_joints; ++i) {
    position_error_integral_(i) = std::clamp(position_error_integral_(i), -1.0, 1.0);
  }
  
  // PID control law: Kp*e + Kd*de/dt + Ki*∫e dt
  Vector7d tau_d = kp_gains_.cwiseProduct(position_error) +
                   kd_gains_.cwiseProduct(velocity_error) +
                   ki_gains_.cwiseProduct(position_error_integral_);
  
  return tau_d;
}

void JointPositionController::applyVelocityFilter(double alpha) {
  dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq_;
}

bool JointPositionController::validateJointLimits(const Vector7d& q) const {
  for (int i = 0; i < num_joints; ++i) {
    if (q(i) < joint_limits_lower[i] || q(i) > joint_limits_upper[i]) {
      return false;
    }
  }
  return true;
}

void JointPositionController::generateSmoothTrajectory(double t_normalized, 
                                                      const Vector7d& start_pos, 
                                                      const Vector7d& end_pos,
                                                      Vector7d& pos, 
                                                      Vector7d& vel, 
                                                      Vector7d& acc) {
  // Cubic polynomial trajectory (3rd order)
  double s = 3 * t_normalized * t_normalized - 2 * t_normalized * t_normalized * t_normalized;
  double ds_dt = (6 * t_normalized - 6 * t_normalized * t_normalized) / trajectory_duration_;
  double d2s_dt2 = (6 - 12 * t_normalized) / (trajectory_duration_ * trajectory_duration_);
  
  Vector7d position_diff = end_pos - start_pos;
  
  pos = start_pos + s * position_diff;
  vel = ds_dt * position_diff;
  acc = d2s_dt2 * position_diff;
}

void JointPositionController::jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCLCPP_INFO(get_node()->get_logger(), "Received joint command");
  
  if (msg->position.size() != num_joints) {
    RCLCPP_WARN(get_node()->get_logger(), "Joint command size mismatch: expected %d, got %zu", 
                num_joints, msg->position.size());
    return;
  }
  
  // Store the joint command
  for (int i = 0; i < num_joints; ++i) {
    external_q_target_(i) = msg->position[i];
  }
  
  // Set velocities if provided
  if (msg->velocity.size() == num_joints) {
    for (int i = 0; i < num_joints; ++i) {
      external_dq_target_(i) = msg->velocity[i];
    }
  } else {
    external_dq_target_.setZero();
  }
  
  // Set accelerations if provided (using effort field)
  if (msg->effort.size() == num_joints) {
    for (int i = 0; i < num_joints; ++i) {
      external_ddq_target_(i) = msg->effort[i];
    }
  } else {
    external_ddq_target_.setZero();
  }
  
  // Check for direct tracking mode
  if (!msg->name.empty() && msg->name[0] == "direct_track") {
    direct_tracking_mode_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking mode enabled");
  } else {
    direct_tracking_mode_ = false;
  }
  
  // Validate joint limits
  if (validateJointLimits(external_q_target_)) {
    external_command_active_ = true;
    new_command_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Joint command accepted");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Joint command rejected: exceeds limits");
  }
}

void JointPositionController::poseCommandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(get_node()->get_logger(), "Received pose command - inverse kinematics not implemented");
  
  // TODO: Implement inverse kinematics to convert pose to joint positions
  // This would require a kinematics solver like MoveIt or custom IK
  // For now, just log that the command was received
}

void JointPositionController::configCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(get_node()->get_logger(), "Received config command: %s", msg->data.c_str());
  
  std::string command = msg->data;
  
  if (command == "stop") {
    external_command_active_ = false;
    direct_tracking_mode_ = false;
    q_target_ = q_;  // Hold current position
    position_error_integral_.setZero();  // Reset integral term
    RCLCPP_INFO(get_node()->get_logger(), "Emergency stop activated");
  }
  else if (command == "home") {
    external_q_target_ = q_initial_;
    external_dq_target_.setZero();
    external_ddq_target_.setZero();
    external_command_active_ = true;
    direct_tracking_mode_ = false;
    new_command_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Homing command activated");
  }
  else if (command == "direct_tracking_on") {
    direct_tracking_mode_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking mode enabled");
  }
  else if (command == "direct_tracking_off") {
    direct_tracking_mode_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking mode disabled");
  }
  else if (command.find("trajectory_duration:") == 0) {
    try {
      double new_duration = std::stod(command.substr(19));
      trajectory_duration_ = std::max(0.1, new_duration);
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory duration changed to: %.2f seconds", trajectory_duration_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid trajectory duration: %s", e.what());
    }
  }
  else if (command.find("mode:") == 0) {
    std::string mode_str = command.substr(5);
    if (mode_str == "position_control") {
      control_mode_ = POSITION_CONTROL;
    } else if (mode_str == "trajectory_tracking") {
      control_mode_ = TRAJECTORY_TRACKING;
    } else if (mode_str == "sinusoidal_motion") {
      control_mode_ = SINUSOIDAL_MOTION;
    } else if (mode_str == "hold_position") {
      control_mode_ = HOLD_POSITION;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Control mode changed to: %s", mode_str.c_str());
  }
  else if (command == "reset_integral") {
    position_error_integral_.setZero();
    RCLCPP_INFO(get_node()->get_logger(), "Integral term reset");
  }
}

}  // namespace iitgn_controllers

// Register the controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(iitgn_controllers::JointPositionController,
                       controller_interface::ControllerInterface)