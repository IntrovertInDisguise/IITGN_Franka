// gripper_topic_controller.cpp
// Extended with: force regulation, smart grasp (velocity→force), jaw width output.

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "fmt/format.h"
#include "iitgn_controllers/default_robot_behavior_utils.hpp"
#include "iitgn_controllers/gripper_topic_controller.hpp"
#include "std_msgs/msg/string.hpp"

#define RED    "\033[1;31m"
#define GREEN  "\033[1;32m"
#define YELLOW "\033[1;33m"
#define RESET  "\033[0m"

namespace iitgn_controllers {

// ─── Interface configuration ──────────────────────────────────────────────────

controller_interface::InterfaceConfiguration
GripperTopicController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE};
}
controller_interface::InterfaceConfiguration
GripperTopicController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE};
}

// ─── on_init ─────────────────────────────────────────────────────────────────

CallbackReturn GripperTopicController::on_init() {
  try {
    auto_declare<std::string>("arm_id",                "fr3");
    auto_declare<std::string>("gripper_command_topic", "~/gripper_command");
    auto_declare<double>("gripper_open_width",  0.08);
    auto_declare<double>("gripper_close_width", 0.04);
    auto_declare<double>("gripper_speed",       0.1);
    auto_declare<double>("gripper_force",       10.0);
    auto_declare<double>("epsilon_inner",       0.03);
    auto_declare<double>("epsilon_outer",       0.03);
    auto_declare<double>("force_deadband",      1.0);
    auto_declare<double>("force_p_gain",        0.3);
    auto_declare<double>("force_step_max",      2.0);
    auto_declare<double>("force_min",           1.0);
    auto_declare<double>("force_max",           70.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception in on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ─── on_configure ────────────────────────────────────────────────────────────

CallbackReturn GripperTopicController::on_configure(const rclcpp_lifecycle::State&) {
  namespace_ = get_node()->get_namespace();

  gripper_command_topic_ = get_node()->get_parameter("gripper_command_topic").as_string();
  gripper_open_width_    = get_node()->get_parameter("gripper_open_width").as_double();
  gripper_close_width_   = get_node()->get_parameter("gripper_close_width").as_double();
  gripper_speed_         = get_node()->get_parameter("gripper_speed").as_double();
  gripper_force_         = get_node()->get_parameter("gripper_force").as_double();
  epsilon_inner_         = get_node()->get_parameter("epsilon_inner").as_double();
  epsilon_outer_         = get_node()->get_parameter("epsilon_outer").as_double();
  force_deadband_        = get_node()->get_parameter("force_deadband").as_double();
  force_p_gain_          = get_node()->get_parameter("force_p_gain").as_double();
  force_step_max_        = get_node()->get_parameter("force_step_max").as_double();
  force_min_             = get_node()->get_parameter("force_min").as_double();
  force_max_             = get_node()->get_parameter("force_max").as_double();
  commanded_force_.store(gripper_force_);

  // Action / service clients
  gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      get_node(), fmt::format("{}/franka_gripper/grasp", namespace_));
  gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      get_node(), fmt::format("{}/franka_gripper/move", namespace_));
  gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
      fmt::format("{}/franka_gripper/stop", namespace_));

  // Command subscriber + status publisher (original)
  gripper_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
      gripper_command_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { gripperCommandCallback(msg); });
  gripper_status_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
      "~/gripper_status", 10);

  // Force feedback subscriber + status publisher + 10 Hz regulation timer
  force_feedback_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "~/force_feedback", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) { forceFeedbackCallback(msg); });
  force_status_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/force_status", 10);
  force_regulation_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { forceRegulationTick(); });

  // Jaw width subscriber: reads /franka_gripper/joint_states
  // Topic is absolute (not namespaced under controller) since it's from franka_gripper node
  jaw_width_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      fmt::format("{}/franka_gripper/joint_states", namespace_), 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jawWidthCallback(msg); });
  jaw_width_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/jaw_width", 10);

  // Smart grasp monitor timer (10 Hz, only acts when smart_grasp_phase1_ == true)
  smart_grasp_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { smartGraspMonitorTick(); });

  assignMoveGoalOptionsCallbacks();
  assignGraspGoalOptionsCallbacks();

  return (gripper_grasp_action_client_ && gripper_move_action_client_ &&
          gripper_stop_client_ && gripper_command_subscriber_ &&
          gripper_status_publisher_ && force_feedback_subscriber_ &&
          force_status_publisher_ && jaw_width_subscriber_ && jaw_width_publisher_)
             ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

// ─── on_activate ─────────────────────────────────────────────────────────────

CallbackReturn GripperTopicController::on_activate(const rclcpp_lifecycle::State&) {
  if (!gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Move action server not available.");
    return CallbackReturn::ERROR;
  }
  if (!gripper_grasp_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Grasp action server not available.");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(),
              "GripperTopicController active.\n"
              "  Commands: open | close | stop | grasp_force:<N> |\n"
              "            force_control_on | force_control_off | smart_grasp:<N>\n"
              "  Topics  : ~/force_feedback (in) | ~/force_status (out) | ~/jaw_width (out)");
  publishGripperStatus("ready");
  return CallbackReturn::SUCCESS;
}

// ─── on_deactivate ───────────────────────────────────────────────────────────

controller_interface::CallbackReturn GripperTopicController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  force_control_active_.store(false);
  smart_grasp_phase1_.store(false);
  if (gripper_stop_client_->service_is_ready()) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto res  = gripper_stop_client_->async_send_request(req);
    if (res.get() && res.get()->success)
      RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped.");
    else
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
  }
  return CallbackReturn::SUCCESS;
}

// ─── update ──────────────────────────────────────────────────────────────────

controller_interface::return_type GripperTopicController::update(const rclcpp::Time&,
                                                                   const rclcpp::Duration&) {
  return controller_interface::return_type::OK;
}

// ─── gripperCommandCallback ───────────────────────────────────────────────────

void GripperTopicController::gripperCommandCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  std::string command = msg->data;
  std::transform(command.begin(), command.end(), command.begin(), ::tolower);
  RCLCPP_INFO(get_node()->get_logger(), "Command received: %s", command.c_str());

  if (command == "open") {
    smart_grasp_phase1_.store(false);
    force_control_active_.store(false);
    openGripper();
  } else if (command == "close") {
    closeGripper();
  } else if (command == "stop") {
    smart_grasp_phase1_.store(false);
    force_control_active_.store(false);
    stopGripper();
  } else if (command.rfind("grasp_force:", 0) == 0) {
    try { graspAtForce(std::stod(command.substr(12))); }
    catch (...) {
      RCLCPP_ERROR(get_node()->get_logger(), "Bad grasp_force value: %s", command.c_str());
      publishGripperStatus("bad_command");
    }
  } else if (command == "force_control_on") {
    force_control_active_.store(true);
    RCLCPP_INFO(get_node()->get_logger(), "Force regulation ON — target: %.1f N", force_target_);
    publishGripperStatus("force_control_on");
  } else if (command == "force_control_off") {
    force_control_active_.store(false);
    RCLCPP_INFO(get_node()->get_logger(), "Force regulation OFF.");
    publishGripperStatus("force_control_off");
  } else if (command.rfind("smart_grasp:", 0) == 0) {
    try { smartGrasp(std::stod(command.substr(12))); }
    catch (...) {
      RCLCPP_ERROR(get_node()->get_logger(), "Bad smart_grasp value: %s", command.c_str());
      publishGripperStatus("bad_command");
    }
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Unknown command: '%s'. Valid: open | close | stop | "
                "grasp_force:<N> | force_control_on | force_control_off | smart_grasp:<N>",
                command.c_str());
    publishGripperStatus("unknown_command");
  }
}

// ─── publishGripperStatus ─────────────────────────────────────────────────────

void GripperTopicController::publishGripperStatus(const std::string& status) {
  std_msgs::msg::String msg;
  msg.data = status;
  gripper_status_publisher_->publish(msg);
}

// ─── assignMoveGoalOptionsCallbacks ──────────────────────────────────────────

void GripperTopicController::assignMoveGoalOptionsCallbacks() {
  move_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Move Goal NOT accepted." RESET);
          publishGripperStatus("open_failed");
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Move Goal accepted.");
          publishGripperStatus("opening");
        }
      };
  move_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
        // Publish jaw width during move as well
        std_msgs::msg::Float64 w;
        w.data = jaw_width_.load();
        jaw_width_publisher_->publish(w);
        RCLCPP_DEBUG(get_node()->get_logger(), "Move feedback width: %.4f m",
                     feedback->current_width);
      };
  move_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        bool ok = rclcpp_action::ResultCode::SUCCEEDED == result.code;
        RCLCPP_INFO(get_node()->get_logger(), "Move result: %s", ok ? "SUCCESS" : "FAIL");
        is_grasped_.store(false);
        publishGripperStatus(ok ? "opened" : "open_failed");
      };
}

// ─── assignGraspGoalOptionsCallbacks ─────────────────────────────────────────

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
        RCLCPP_DEBUG(get_node()->get_logger(), "Grasp feedback width: %.4f m",
                     feedback->current_width);
      };
  grasp_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult& result) {
        bool ok = rclcpp_action::ResultCode::SUCCEEDED == result.code;
        RCLCPP_INFO(get_node()->get_logger(), "Grasp result: %s  jaw_width=%.4f m",
                    ok ? GREEN "SUCCESS" RESET : RED "FAIL" RESET, jaw_width_.load());
        is_grasped_.store(ok);
        publishGripperStatus(ok ? "closed" : "close_failed");
      };
}

// ─── openGripper ─────────────────────────────────────────────────────────────

bool GripperTopicController::openGripper() {
  franka_msgs::action::Move::Goal goal;
  goal.width = gripper_open_width_;
  goal.speed = gripper_speed_;
  auto handle = gripper_move_action_client_->async_send_goal(goal, move_goal_options_);
  if (!handle.valid()) { publishGripperStatus("open_failed"); return false; }
  return true;
}

// ─── closeGripper ────────────────────────────────────────────────────────────

bool GripperTopicController::closeGripper() {
  franka_msgs::action::Grasp::Goal goal;
  goal.width         = gripper_close_width_;
  goal.speed         = gripper_speed_;
  goal.force         = gripper_force_;
  goal.epsilon.inner = epsilon_inner_;
  goal.epsilon.outer = epsilon_outer_;
  auto handle = gripper_grasp_action_client_->async_send_goal(goal, grasp_goal_options_);
  if (!handle.valid()) { publishGripperStatus("close_failed"); return false; }
  return true;
}

// ─── stopGripper ─────────────────────────────────────────────────────────────

bool GripperTopicController::stopGripper() {
  if (!gripper_stop_client_->service_is_ready()) {
    publishGripperStatus("stop_failed"); return false;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res  = gripper_stop_client_->async_send_request(req);
  bool ok   = res.get() && res.get()->success;
  publishGripperStatus(ok ? "stopped" : "stop_failed");
  return ok;
}

// ─── graspAtForce ────────────────────────────────────────────────────────────

bool GripperTopicController::graspAtForce(double force_n) {
  force_n = std::clamp(force_n, force_min_, force_max_);
  force_target_ = force_n;
  commanded_force_.store(force_n);

  // Use current jaw width as the target width so epsilon check passes
  double current_width = jaw_width_.load();
  // Add small epsilon so the gripper doesn't think it's already at target
  double target_width  = std::max(0.0, current_width - 0.002);

  RCLCPP_INFO(get_node()->get_logger(),
              "graspAtForce: %.1f N  target_width=%.4f m", force_n, target_width);

  franka_msgs::action::Grasp::Goal goal;
  goal.width         = target_width;
  goal.speed         = gripper_speed_;
  goal.force         = force_n;
  goal.epsilon.inner = epsilon_inner_;
  goal.epsilon.outer = epsilon_outer_;

  std::lock_guard<std::mutex> lock(grasp_mutex_);
  auto handle = gripper_grasp_action_client_->async_send_goal(goal, grasp_goal_options_);
  if (!handle.valid()) { publishGripperStatus("close_failed"); return false; }
  return true;
}

// ─── forceFeedbackCallback ───────────────────────────────────────────────────

void GripperTopicController::forceFeedbackCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  measured_force_.store(std::abs(msg->data));
}

// ─── jawWidthCallback ────────────────────────────────────────────────────────
// /franka_gripper/joint_states: position[0] + position[1] = total jaw opening (m)

void GripperTopicController::jawWidthCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < 2) return;
  double width = msg->position[0] + msg->position[1];
  jaw_width_.store(width);

  // Always publish jaw width so users can monitor it
  std_msgs::msg::Float64 out;
  out.data = width;
  jaw_width_publisher_->publish(out);
}

// ─── forceRegulationTick  (10 Hz) ────────────────────────────────────────────

void GripperTopicController::forceRegulationTick() {
  if (!force_control_active_.load()) return;

  double measured  = measured_force_.load();
  double commanded = commanded_force_.load();
  double error     = force_target_ - measured;

  std_msgs::msg::Float64 status;
  status.data = measured;
  force_status_publisher_->publish(status);

  // Only regulate if object confirmed grasped
  if (!is_grasped_.load()) return;

  if (std::abs(error) <= force_deadband_) return;

  double delta     = std::clamp(error * force_p_gain_, -force_step_max_, force_step_max_);
  double new_force = std::clamp(commanded + delta, force_min_, force_max_);
  commanded_force_.store(new_force);

  RCLCPP_DEBUG(get_node()->get_logger(),
               "ForceReg: measured=%.2f N  target=%.2f N  cmd=%.2f N  width=%.4f m",
               measured, force_target_, new_force, jaw_width_.load());

  double current_width = jaw_width_.load();
  double target_width  = std::max(0.0, current_width - 0.002);

  franka_msgs::action::Grasp::Goal goal;
  goal.width         = target_width;
  goal.speed         = gripper_speed_;
  goal.force         = new_force;
  goal.epsilon.inner = epsilon_inner_;
  goal.epsilon.outer = epsilon_outer_;

  std::lock_guard<std::mutex> lock(grasp_mutex_);
  gripper_grasp_action_client_->async_send_goal(goal, grasp_goal_options_);
}

// ─── smartGrasp ──────────────────────────────────────────────────────────────
// Phase 1: Move to 0 at kSlowCloseSpeed. Monitor force at 10 Hz.
// Phase 2 (triggered by smartGraspMonitorTick): Grasp at contact width + force control.

void GripperTopicController::smartGrasp(double target_force_n) {
  target_force_n = std::clamp(target_force_n, force_min_, force_max_);
  smart_grasp_target_force_ = target_force_n;
  force_target_             = target_force_n;
  force_control_active_.store(false);  // disable regulation during Phase 1
  is_grasped_.store(false);

  RCLCPP_INFO(get_node()->get_logger(),
              "SmartGrasp: Phase 1 — closing at %.3f m/s until force > %.1f N",
              kSlowCloseSpeed, kContactThreshold);
  publishGripperStatus("smart_grasp_phase1");

  // Issue Move to width=0 at slow speed — gripper physically stops at contact
  franka_msgs::action::Move::Goal move_goal;
  move_goal.width = 0.0;
  move_goal.speed = kSlowCloseSpeed;

  smart_grasp_phase1_.store(true);  // arm the monitor timer
  gripper_move_action_client_->async_send_goal(move_goal, move_goal_options_);
}

// ─── smartGraspMonitorTick  (10 Hz) ──────────────────────────────────────────
// Runs during Phase 1. When force crosses kContactThreshold, cancels the Move
// and issues a Grasp at the current width with the target force.

void GripperTopicController::smartGraspMonitorTick() {
  if (!smart_grasp_phase1_.load()) return;

  double force = measured_force_.load();
  double width = jaw_width_.load();

  RCLCPP_DEBUG(get_node()->get_logger(),
               "SmartGrasp Phase1: force=%.3f N  width=%.4f m", force, width);

  if (force < kContactThreshold) return;

  // Contact detected — switch to Phase 2
  smart_grasp_phase1_.store(false);

  RCLCPP_INFO(get_node()->get_logger(),
              "SmartGrasp: CONTACT at width=%.4f m (force=%.2f N) — Phase 2: regulate to %.1f N",
              width, force, smart_grasp_target_force_);
  publishGripperStatus("smart_grasp_phase2");

  // Issue Grasp at current contact width with target force
  // Use current width ± epsilon so the gripper holds position and applies force
  double contact_width = std::max(0.0, width - 0.001);

  franka_msgs::action::Grasp::Goal grasp_goal;
  grasp_goal.width         = contact_width;
  grasp_goal.speed         = gripper_speed_;
  grasp_goal.force         = smart_grasp_target_force_;
  grasp_goal.epsilon.inner = 0.05;   // generous epsilon — we don't care about exact width
  grasp_goal.epsilon.outer = 0.05;

  commanded_force_.store(smart_grasp_target_force_);

  {
    std::lock_guard<std::mutex> lock(grasp_mutex_);
    gripper_grasp_action_client_->async_send_goal(grasp_goal, grasp_goal_options_);
  }

  // Enable force regulation loop — it will maintain force from here
  force_control_active_.store(true);
}

}  // namespace iitgn_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(iitgn_controllers::GripperTopicController,
                       controller_interface::ControllerInterface)