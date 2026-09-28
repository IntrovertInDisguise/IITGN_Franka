#pragma once

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace iitgn_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Topic-based gripper controller that accepts commands via ROS topics
 * 
 * This controller subscribes to a topic for gripper commands ("open", "close", "stop")
 * and publishes gripper status updates. It provides configurable parameters for
 * gripper width, speed, force, and epsilon values.
 */
class GripperTopicController : public controller_interface::ControllerInterface {
 public:
  /**
   * @brief Get the command interface configuration
   * @return Interface configuration (NONE for this controller)
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Get the state interface configuration
   * @return Interface configuration (NONE for this controller)
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Initialize the controller
   * @return SUCCESS if initialization was successful, ERROR otherwise
   */
  CallbackReturn on_init() override;

  /**
   * @brief Configure the controller
   * @param previous_state The previous lifecycle state
   * @return SUCCESS if configuration was successful, ERROR otherwise
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Activate the controller
   * @param previous_state The previous lifecycle state
   * @return SUCCESS if activation was successful, ERROR otherwise
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Deactivate the controller
   * @param previous_state The previous lifecycle state
   * @return SUCCESS if deactivation was successful, ERROR otherwise
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Update the controller
   * @param time Current time
   * @param period Time period since last update
   * @return OK if update was successful
   */
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  /**
   * @brief Callback function for gripper commands
   * @param msg String message containing the command ("open", "close", "stop")
   */
  void gripperCommandCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Publish gripper status message
   * @param status Status string to publish
   */
  void publishGripperStatus(const std::string& status);

  /**
   * @brief Assign callbacks for move goal options
   */
  void assignMoveGoalOptionsCallbacks();

  /**
   * @brief Assign callbacks for grasp goal options
   */
  void assignGraspGoalOptionsCallbacks();

  /**
   * @brief Open the gripper
   * @return true if command was sent successfully, false otherwise
   */
  bool openGripper();

  /**
   * @brief Close the gripper
   * @return true if command was sent successfully, false otherwise
   */
  bool closeGripper();

  /**
   * @brief Stop the gripper
   * @return true if command was sent successfully, false otherwise
   */
  bool stopGripper();

  // ROS communication members
  std::string namespace_;
  std::string gripper_command_topic_;
  
  // Publishers and subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_command_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_status_publisher_;

  // Action clients
  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_grasp_action_client_;
  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr gripper_move_action_client_;

  // Service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_stop_client_;

  // Goal options
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_goal_options_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_goal_options_;

  // Configurable parameters
  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_speed_;
  double gripper_force_;
  double epsilon_inner_;
  double epsilon_outer_;
};

}  // namespace iitgn_controllers