#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

#include <eigen3/Eigen/Dense>

namespace iitgn_controllers {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * SpringController — Cartesian Impedance Controller for Franka FR3.
 *
 * KEY FEATURES:
 *
 * 1. Runtime parameter updates — no rebuild, no restart needed.
 *    Use:  ros2 param set /NS_1/spring_controller cartesian_stiffness "[200,200,200,20,20,20]"
 *    All tunable gains take effect immediately via ROS2 parameter callback.
 *
 * 2. Clean stop behaviour — no residual drift after zero velocity commands.
 *    When zero velocity is received or timeout occurs, both position_d_ and
 *    position_d_target_ are snapped to the actual current EE position.
 *    This makes cartesian_error = 0 instantly → no residual spring force.
 *
 * 3. Smooth continuous integration — dq_cmd integrated every 1kHz cycle.
 *    5 × (v × 1ms) == 1 × (v × 5ms), so total displacement is identical
 *    to 200Hz integration but motion is continuous instead of step-wise.
 */
class SpringController : public controller_interface::ControllerInterface {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  enum ControllerMode {
    POSITION_IMPEDANCE,
    TRAJECTORY_IMPEDANCE,
    COMPLIANT_MOTION,
  };

  SpringController() = default;

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
  // ---- Constants -----------------------------------------------------------
  static constexpr int num_joints = 7;

  static constexpr std::array<double, num_joints> max_torques = {
      87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

  static constexpr double k_workspace_limit_{0.855};

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  // ---- Franka semantic components ------------------------------------------
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel>             franka_robot_model_;

  // ---- Robot config (structural — not runtime-changeable) ------------------
  std::string arm_id_;
  std::string command_topic_;
  std::string cmd_topic_;

  // ---- Impedance gains (runtime-tunable via ros2 param set) ----------------
  // Protected by params_mutex_ during concurrent read (RT) / write (callback).
  Matrix6d cartesian_stiffness_{Matrix6d::Zero()};
  Matrix6d cartesian_damping_{Matrix6d::Zero()};
  double   nullspace_stiffness_{0.5};
  Vector7d q_d_nullspace_{Vector7d::Zero()};

  // ---- Other tunable parameters (runtime-tunable) --------------------------
  double         k_delta_tau_max_{1.0};
  double         cmd_timeout_sec_{0.025};
  double         compliance_factor_{0.5};
  double         trajectory_duration_{5.0};
  double         filter_gain_{0.3};
  double         velocity_filter_alpha_{0.3};
  ControllerMode impedance_mode_{POSITION_IMPEDANCE};

  // ---- Parameter update mechanism ------------------------------------------
  // Protects all tunable gain members above during concurrent access between
  // the RT update() thread and the parameter callback service thread.
  // Critical section is ~1 microsecond (copying ~100 bytes), so impact on
  // the 1kHz RT loop is negligible.
  std::mutex params_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Called by ROS2 whenever ros2 param set is used. Updates gains immediately.
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  // ---- Joint-space state ---------------------------------------------------
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d initial_q_;
  Vector7d tau_J_d_{Vector7d::Zero()};

  // ---- Desired Cartesian pose ----------------------------------------------
  // position_d_ / orientation_d_:
  //   Filtered desired pose driving the impedance law each cycle.
  //   When zero velocity or timeout detected, BOTH this AND position_d_target_
  //   are snapped to actual current EE position → cartesian_error = 0 → clean stop.
  //
  // position_d_target_ / orientation_d_target_:
  //   Raw integrated target. Moves continuously as dq_cmd is integrated.
  Eigen::Vector3d    position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d    position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  Eigen::Vector3d    initial_position_;
  Eigen::Quaterniond initial_orientation_;
  Eigen::Vector3d    trajectory_start_position_;
  Eigen::Quaterniond trajectory_start_orientation_;
  Eigen::Vector3d    external_position_target_;
  Eigen::Quaterniond external_orientation_target_;

  // ---- Velocity command (written by callback, read by RT loop) -------------
  Vector7d     dq_cmd_{Vector7d::Zero()};
  rclcpp::Time last_cmd_time_;

  // ---- Command state -------------------------------------------------------
  bool   new_command_received_{false};
  bool   external_command_active_{false};
  bool   velocity_only_streaming_{false};
  double external_trajectory_time_{0.0};

  // ---- Timing --------------------------------------------------------------
  double elapsed_time_{0.0};
  double elapsed_time_step_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};

  // ---- Init flag -----------------------------------------------------------
  bool initialization_flag_{true};

  // ---- Subscribers ---------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr       command_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr          joint_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                 impedance_config_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr      vel_cmd_subscriber_;

  // Protects dq_cmd_, last_cmd_time_, and command state flags across threads.
  std::mutex command_mutex_;

  // ---- Methods -------------------------------------------------------------
  void     updateJointStates();
  void     calculateDesiredCartesianTrajectory(const rclcpp::Time& time);
  Vector7d calculateCartesianImpedanceTorques();
  void     processExternalCommand();
  void     applyVelocityFilter(double alpha);
  bool     validateCartesianLimits(const Eigen::Vector3d& position) const;
  Vector7d saturateTorqueRate(const Vector7d& tau_d_calculated, const Vector7d& tau_J_d, double delta_tau_max);

  void commandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void cartesianVelCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void impedanceConfigCallback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace iitgn_controllers