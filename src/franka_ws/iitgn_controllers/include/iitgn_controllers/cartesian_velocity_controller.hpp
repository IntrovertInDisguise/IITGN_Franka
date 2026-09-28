#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace iitgn_controllers {

class CartesianVelocityController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;

  // Subscription for external velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  geometry_msgs::msg::Twist latest_twist_;

  const bool k_elbow_activated_{false};
};

}  // namespace iitgn_controllers
