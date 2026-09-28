#include <iitgn_controllers/cartesian_velocity_controller.hpp>
#include <iitgn_controllers/default_robot_behavior_utils.hpp>

#include <Eigen/Eigen>
#include <cmath>

namespace iitgn_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type CartesianVelocityController::update(
    const rclcpp::Time&,  // unused
    const rclcpp::Duration&) {  // unused

  // Directly use the last received velocity without any timeout or ramp-down
  Eigen::Vector3d cartesian_linear_velocity(
      latest_twist_.linear.x,
      latest_twist_.linear.y,
      latest_twist_.linear.z);

  Eigen::Vector3d cartesian_angular_velocity(
      latest_twist_.angular.x,
      latest_twist_.angular.y,
      latest_twist_.angular.z);

  if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity,
                                             cartesian_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianVelocityController::on_init() {
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {

  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(k_elbow_activated_);

  // Set default collision behavior for Franka
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = iitgn_controllers::DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(std::chrono::milliseconds(1000));

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  // Subscribe to external velocity commands
  velocity_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "cartesian_velocity_command", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_twist_ = *msg;
      });

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  latest_twist_ = geometry_msgs::msg::Twist();  // Zero velocity at startup
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace iitgn_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(iitgn_controllers::CartesianVelocityController,
                       controller_interface::ControllerInterface)
