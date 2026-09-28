// wrist_torque_publisher.cpp
//
// Wrist torque → jaw force publisher for FR3 gripper force feedback.
//
// Subscribes to franka_robot_state_broadcaster/robot_state
// (already published by your stack — no second FCI connection needed).
// Extracts tau_ext_hat_filtered[6], converts to jaw force,
// and re-publishes to gripper_topic_controller/force_feedback.
//
// Usage:
//   ros2 run iitgn_controllers wrist_torque_publisher 
//       --ros-args -p moment_arm:=0.058 
//                  -r __ns:=/fr3
//
// Topics (resolved under /fr3 namespace):
//   Subscribed : franka_robot_state_broadcaster/robot_state
//                  (franka_msgs/msg/FrankaRobotState)
//   Published  : gripper_topic_controller/force_feedback
//                  (std_msgs/Float64)

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>

class WristTorquePublisher : public rclcpp::Node {
 public:
  WristTorquePublisher() : Node("wrist_torque_publisher") {
    declare_parameter<double>     ("moment_arm",   0.058);
    declare_parameter<std::string>("input_topic",
        "franka_robot_state_broadcaster/robot_state");
    declare_parameter<std::string>("output_topic",
        "gripper_topic_controller/force_feedback");

    moment_arm_       = get_parameter("moment_arm").as_double();
    std::string in_t  = get_parameter("input_topic").as_string();
    std::string out_t = get_parameter("output_topic").as_string();

    pub_ = create_publisher<std_msgs::msg::Float64>(out_t, 10);

    sub_ = create_subscription<franka_msgs::msg::FrankaRobotState>(
        in_t, 10,
        [this](const franka_msgs::msg::FrankaRobotState::SharedPtr msg) {
          // tau_ext_hat_filtered[6] = joint 7 external torque (Nm)
          if (msg->tau_ext_hat_filtered.effort.size() < 7) return;
          double tau7 = std::abs(msg->tau_ext_hat_filtered.effort[6]);
          double jaw_force = tau7 / moment_arm_;

          std_msgs::msg::Float64 out;
          out.data = jaw_force;
          pub_->publish(out);

          RCLCPP_DEBUG(get_logger(),
                       "tau7=%.3f Nm -> jaw_force=%.2f N", tau7, jaw_force);
        });

    RCLCPP_INFO(get_logger(),
                "WristTorquePublisher ready\n"
                "  subscribing : %s\n"
                "  publishing  : %s\n"
                "  moment_arm  : %.4f m",
                in_t.c_str(), out_t.c_str(), moment_arm_);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                 pub_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr  sub_;
  double moment_arm_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WristTorquePublisher>());
  rclcpp::shutdown();
  return 0;
}