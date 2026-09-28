#pragma once

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <atomic>
#include <mutex>

#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace iitgn_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GripperTopicController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  void gripperCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  void publishGripperStatus(const std::string& status);
  void assignMoveGoalOptionsCallbacks();
  void assignGraspGoalOptionsCallbacks();
  bool openGripper();
  bool closeGripper();
  bool stopGripper();
  bool graspAtForce(double force_n);
  void forceFeedbackCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void forceRegulationTick();
  void smartGrasp(double target_force_n);
  void jawWidthCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void smartGraspMonitorTick();

  std::string namespace_;
  std::string gripper_command_topic_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   gripper_command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      gripper_status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  force_feedback_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     force_status_publisher_;
  rclcpp::TimerBase::SharedPtr                             force_regulation_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jaw_width_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     jaw_width_publisher_;
  rclcpp::TimerBase::SharedPtr                             smart_grasp_timer_;

  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_grasp_action_client_;
  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr  gripper_move_action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr            gripper_stop_client_;

  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions  move_goal_options_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_goal_options_;

  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_speed_;
  double gripper_force_;
  double epsilon_inner_;
  double epsilon_outer_;

  double force_target_{10.0};
  double force_deadband_{1.0};
  double force_p_gain_{0.3};
  double force_step_max_{2.0};
  double force_min_{1.0};
  double force_max_{70.0};

  static constexpr double kContactThreshold = 1.0;   // N — Phase 1→2 switchover
  static constexpr double kSlowCloseSpeed   = 0.02;  // m/s — Phase 1 closing speed

  std::atomic<double> measured_force_{0.0};
  std::atomic<double> commanded_force_{10.0};
  std::atomic<bool>   force_control_active_{false};
  std::atomic<bool>   is_grasped_{false};
  std::atomic<double> jaw_width_{0.0};
  std::atomic<bool>   smart_grasp_phase1_{false};
  double              smart_grasp_target_force_{10.0};
  std::mutex          grasp_mutex_;
};

}  // namespace iitgn_controllers