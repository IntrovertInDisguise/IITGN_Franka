#pragma once

#include <franka_msgs/srv/set_full_collision_behavior.hpp>

namespace iitgn_controllers::DefaultRobotBehavior {

/**
 * @brief Provides a default collision behavior configuration request for Franka robot.
 *
 * These threshold values define nominal and acceleration torque/force limits for safety.
 * You can customize these values if needed for your specific controller.
 */
inline franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr
getDefaultCollisionBehaviorRequest() {
  auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();

  request->lower_torque_thresholds_nominal = {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};
  request->upper_torque_thresholds_nominal = {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};
  request->lower_torque_thresholds_acceleration = {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};
  request->upper_torque_thresholds_acceleration = {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};
  request->lower_force_thresholds_nominal = {30.0, 30.0, 30.0, 25.0, 25.0, 25.0};
  request->upper_force_thresholds_nominal = {40.0, 40.0, 40.0, 35.0, 35.0, 35.0};
  request->lower_force_thresholds_acceleration = {30.0, 30.0, 30.0, 25.0, 25.0, 25.0};
  request->upper_force_thresholds_acceleration = {40.0, 40.0, 40.0, 35.0, 35.0, 35.0};

  return request;
}

}  // namespace iitgn_controllers::DefaultRobotBehavior
