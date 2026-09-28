#pragma once

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace iitgn_controllers {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum ControlMode {
  POSITION_CONTROL,
  TRAJECTORY_TRACKING,
  SINUSOIDAL_MOTION,
  HOLD_POSITION
};

class JointPositionController : public controller_interface::ControllerInterface {
public:
  JointPositionController() = default;
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  // Constants
  static constexpr int num_joints = 7;
  
  // Joint limits (FR3 robot typical limits)
  const std::array<double, num_joints> joint_limits_lower = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
  };
  const std::array<double, num_joints> joint_limits_upper = {
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973
  };
  
  // Maximum torques (Nm)
  const std::array<double, num_joints> max_torques = {
    87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0
  };

  // Parameters
  std::string arm_id_;
  std::string command_topic_;
  double trajectory_duration_;
  double motion_amplitude_;
  double motion_frequency_;
  std::vector<double> target_position_;
  
  // Control gains
  Vector7d kp_gains_;    // Proportional gains
  Vector7d kd_gains_;    // Derivative gains
  Vector7d ki_gains_;    // Integral gains
  
  // State variables
  Vector7d q_;                    // Current joint positions
  Vector7d dq_;                   // Current joint velocities
  Vector7d dq_filtered_;          // Filtered joint velocities
  Vector7d q_desired_;            // Desired joint positions
  Vector7d dq_desired_;           // Desired joint velocities
  Vector7d ddq_desired_;          // Desired joint accelerations
  Vector7d q_target_;             // Target joint positions
  Vector7d q_initial_;            // Initial joint positions
  Vector7d position_error_integral_; // Integral of position error
  
  // External command handling
  Vector7d external_q_target_;
  Vector7d external_dq_target_;
  Vector7d external_ddq_target_;
  Vector7d trajectory_start_q_;
  Vector7d trajectory_start_dq_;
  
  // Control flags
  bool new_command_received_;
  bool external_command_active_;
  bool direct_tracking_mode_;
  ControlMode control_mode_;
  
  // Timing
  double elapsed_time_;
  double external_trajectory_time_;
  double elapsed_time_step_;
  
  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_subscriber_;
  
  // Methods
  void updateJointStates();
  void calculateDesiredTrajectory();
  void processExternalCommand();
  Vector7d calculateControlTorques();
  void applyVelocityFilter(double alpha = 0.1);
  bool validateJointLimits(const Vector7d& q) const;
  
  // Callback methods
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void poseCommandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void configCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // Trajectory generation
  void generateSmoothTrajectory(double t_normalized, const Vector7d& start_pos, 
                               const Vector7d& end_pos, Vector7d& pos, 
                               Vector7d& vel, Vector7d& acc);
};

}  // namespace iitgn_controllers