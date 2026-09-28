#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fmt/format.h"
#include "iitgn_controllers/default_robot_behavior_utils.hpp"
#include "iitgn_controllers/gripper_topic_controller.hpp"
#include "std_msgs/msg/string.hpp"

#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define RESET "\033[0m"

namespace iitgn_controllers {

controller_interface::InterfaceConfiguration
GripperTopicController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
GripperTopicController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

CallbackReturn GripperTopicController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("gripper_command_topic", "~/gripper_command");
    auto_declare<double>("gripper_open_width", 0.08);
    auto_declare<double>("gripper_close_width", 0.04);
    auto_declare<double>("gripper_speed", 0.1);
    auto_declare<double>("gripper_force", 10.0);
    auto_declare<double>("epsilon_inner", 0.03);
    auto_declare<double>("epsilon_outer", 0.03);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperTopicController::on_configure(const rclcpp_lifecycle::State&) {
  namespace_ = get_node()->get_namespace();
  
  // Get parameters
  gripper_command_topic_ = get_node()->get_parameter("gripper_command_topic").as_string();
  gripper_open_width_ = get_node()->get_parameter("gripper_open_width").as_double();
  gripper_close_width_ = get_node()->get_parameter("gripper_close_width").as_double();
  gripper_speed_ = get_node()->get_parameter("gripper_speed").as_double();
  gripper_force_ = get_node()->get_parameter("gripper_force").as_double();
  epsilon_inner_ = get_node()->get_parameter("epsilon_inner").as_double();
  epsilon_outer_ = get_node()->get_parameter("epsilon_outer").as_double();

  // Create action clients
  gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      get_node(), fmt::format("{}/franka_gripper/grasp", namespace_));

  gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      get_node(), fmt::format("{}/franka_gripper/move", namespace_));

  gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
      fmt::format("{}/franka_gripper/stop", namespace_));

  // Create subscriber for gripper commands
  gripper_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
      gripper_command_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        this->gripperCommandCallback(msg);
      });

  // Create publisher for gripper status
  gripper_status_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
      "~/gripper_status", 10);

  assignMoveGoalOptionsCallbacks();
  assignGraspGoalOptionsCallbacks();
  
  return nullptr != gripper_grasp_action_client_ && 
         nullptr != gripper_move_action_client_ &&
         nullptr != gripper_stop_client_ &&
         nullptr != gripper_command_subscriber_ &&
         nullptr != gripper_status_publisher_
             ? CallbackReturn::SUCCESS
             : CallbackReturn::ERROR;
}

CallbackReturn GripperTopicController::on_activate(const rclcpp_lifecycle::State&) {
  if (!gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Move Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  if (!gripper_grasp_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Grasp Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_node()->get_logger(), 
              "Gripper Topic Controller activated. Send 'open' or 'close' commands to topic: %s", 
              gripper_command_topic_.c_str());
  
  // Publish initial status
  publishGripperStatus("ready");
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperTopicController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (gripper_stop_client_->service_is_ready()) {
    std_srvs::srv::Trigger::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = gripper_stop_client_->async_send_request(request);
    if (result.get() && result.get()->success) {
      RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperTopicController::update(const rclcpp::Time&,
                                                                const rclcpp::Duration&) {
  return controller_interface::return_type::OK;
}

void GripperTopicController::gripperCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
  std::string command = msg->data;
  
  // Convert to lowercase for case-insensitive comparison
  std::transform(command.begin(), command.end(), command.begin(), ::tolower);
  
  RCLCPP_INFO(get_node()->get_logger(), "Received gripper command: %s", command.c_str());
  
  if (command == "open") {
    openGripper();
  } else if (command == "close") {
    closeGripper();
  } else if (command == "stop") {
    stopGripper();
  } else {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Unknown gripper command: %s. Valid commands are: 'open', 'close', 'stop'", 
                command.c_str());
    publishGripperStatus("unknown_command");
  }
}

void GripperTopicController::publishGripperStatus(const std::string& status) {
  std_msgs::msg::String status_msg;
  status_msg.data = status;
  gripper_status_publisher_->publish(status_msg);
}

void GripperTopicController::assignMoveGoalOptionsCallbacks() {
  move_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       RED "Move Goal (i.e. open gripper) NOT accepted." RESET);
          publishGripperStatus("open_failed");
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Move Goal accepted");
          publishGripperStatus("opening");
        }
      };

  move_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal current_width [%f].",
                    feedback->current_width);
      };

  move_goal_options_.result_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
        if (rclcpp_action::ResultCode::SUCCEEDED == result.code) {
          publishGripperStatus("opened");
        } else {
          publishGripperStatus("open_failed");
        }
      };
}

void GripperTopicController::assignGraspGoalOptionsCallbacks() {
  grasp_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Grasp Goal NOT accepted." RESET);
          publishGripperStatus("close_failed");
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal accepted.");
          publishGripperStatus("closing");
        }
      };

  grasp_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&,
             const std::shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal current_width: %f",
                    feedback->current_width);
      };

  grasp_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
        if (rclcpp_action::ResultCode::SUCCEEDED == result.code) {
          publishGripperStatus("closed");
        } else {
          publishGripperStatus("close_failed");
        }
      };
}

bool GripperTopicController::openGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Opening the gripper - Submitting a Move Goal");

  // Define open gripper goal
  franka_msgs::action::Move::Goal move_goal;
  move_goal.width = gripper_open_width_;
  move_goal.speed = gripper_speed_;

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
      move_goal_handle =
          gripper_move_action_client_->async_send_goal(move_goal, move_goal_options_);
  bool ret = move_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submitted a Move Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Move Goal" RESET);
    publishGripperStatus("open_failed");
  }
  return ret;
}

bool GripperTopicController::closeGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

  // Define grasp goal with configurable parameters
  franka_msgs::action::Grasp::Goal grasp_goal;
  grasp_goal.width = gripper_close_width_;
  grasp_goal.speed = gripper_speed_;
  grasp_goal.force = gripper_force_;
  grasp_goal.epsilon.inner = epsilon_inner_;
  grasp_goal.epsilon.outer = epsilon_outer_;

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>>
      grasp_goal_handle =
          gripper_grasp_action_client_->async_send_goal(grasp_goal, grasp_goal_options_);

  bool ret = grasp_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submitted a Grasp Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Grasp Goal" RESET);
    publishGripperStatus("close_failed");
  }
  return ret;
}

bool GripperTopicController::stopGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Stopping the gripper");
  
  if (gripper_stop_client_->service_is_ready()) {
    std_srvs::srv::Trigger::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = gripper_stop_client_->async_send_request(request);
    if (result.get() && result.get()->success) {
      RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
      publishGripperStatus("stopped");
      return true;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
      publishGripperStatus("stop_failed");
      return false;
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
    publishGripperStatus("stop_failed");
    return false;
  }
}

}  // namespace iitgn_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(iitgn_controllers::GripperTopicController,
                       controller_interface::ControllerInterface)