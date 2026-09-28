#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <eigen3/Eigen/Dense>

namespace iitgn_controllers {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum ImpedanceMode {
  POSITION_IMPEDANCE,
  TRAJECTORY_IMPEDANCE,
  SINUSOIDAL_IMPEDANCE,
  COMPLIANT_MOTION
};

class ImpedanceController : public controller_interface::ControllerInterface {
public:
  ImpedanceController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  // --------------------------------------------------------------------------
  // Constants
  // --------------------------------------------------------------------------
  static constexpr int num_joints = 7;

  // --------------------------------------------------------------------------
  // Robot params
  // --------------------------------------------------------------------------
  std::string arm_id_;
  std::string robot_description_;

  // --------------------------------------------------------------------------
  // Gains
  // --------------------------------------------------------------------------
  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d m_gains_;

  // --------------------------------------------------------------------------
  // State (measured)
  // --------------------------------------------------------------------------
  Vector7d q_;           // joint position
  Vector7d dq_;          // joint velocity
  Vector7d dq_filtered_; // filtered joint velocity
  Vector7d initial_q_;   // initial q at activation

  // --------------------------------------------------------------------------
  // Desired (internal / controller output)
  // --------------------------------------------------------------------------
  Vector7d q_desired_;
  Vector7d dq_desired_;
  Vector7d ddq_desired_;
  Vector7d q_target_;    // internal target (e.g., home / hold)

  // --------------------------------------------------------------------------
  // External command (from topics)
  // --------------------------------------------------------------------------
  Vector7d external_q_target_;
  Vector7d external_dq_target_;
  Vector7d external_ddq_target_;
  Vector7d trajectory_start_q_;
  Vector7d trajectory_start_dq_;

  geometry_msgs::msg::PoseStamped commanded_pose_;

  bool new_command_received_{false};
  bool external_command_active_{false};
  bool direct_tracking_mode_{false};

  // Embedded bridge streaming flags
  bool velocity_only_streaming_{false};

  // --------------------------------------------------------------------------
  // Mode + timing
  // --------------------------------------------------------------------------
  ImpedanceMode impedance_mode_{POSITION_IMPEDANCE};

  double motion_amplitude_{0.5};
  double motion_frequency_{0.2};
  double compliance_factor_{1.0};

  double elapsed_time_{0.0};
  double elapsed_time_step_{0.0};

  double trajectory_duration_{5.0};
  double external_trajectory_time_{0.0};

  // --------------------------------------------------------------------------
  // Parameters / topics
  // --------------------------------------------------------------------------
  std::vector<double> target_position_;
  std::string command_topic_{"/impedance_command"};

  // Embedded bridge: existing velocity controller topic (Float64MultiArray)
  std::string cmd_topic_{"/NS_1/joint_velocity_controller/commands"};

  // Velocity filter alpha
  double velocity_filter_alpha_{0.1};

  // --------------------------------------------------------------------------
  // Subscribers
  // --------------------------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr impedance_config_subscriber_;

  // Embedded bridge subscriber
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_cmd_subscriber_;

  // --------------------------------------------------------------------------
  // Safety limits (FR3 / Panda-like)
  // --------------------------------------------------------------------------
  static constexpr std::array<double, num_joints> max_torques = {
      87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

  static constexpr std::array<double, num_joints> joint_limits_lower = {
      -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};

  static constexpr std::array<double, num_joints> joint_limits_upper = {
      2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

  // --------------------------------------------------------------------------
  // Methods
  // --------------------------------------------------------------------------
  void updateJointStates();
  void calculateDesiredTrajectory();
  Vector7d calculateImpedanceTorques();
  void applyVelocityFilter(double alpha = 0.1);
  bool validateJointLimits(const Vector7d& q) const;

  // Callbacks
  void commandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void impedanceConfigCallback(const std_msgs::msg::String::SharedPtr msg);

  void processExternalCommand();
};

}  // namespace iitgn_controllers
