#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

// Thread-safe RT buffer (callback thread -> update loop)
#include <realtime_tools/realtime_buffer.hpp>

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace iitgn_controllers {

class JointVelocityController : public controller_interface::ControllerInterface {
public:
  JointVelocityController() = default;
  ~JointVelocityController() override = default;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  void velocity_command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // -----------------------
  // Configuration / params
  // -----------------------
  static constexpr size_t num_joints = 7;

  // Parameters (set in on_init / on_configure)
  std::string arm_id_{"fr3"};
  bool gazebo_{false};

  // Diagnostics + stall handling (read from params; never read in update())
  double stall_dt_s_{0.0015};     // [s] guard threshold for "stall" dt (typical 0.002–0.01)
  double diag_period_s_{1.0};    // [s] non-RT diagnostics print period
  double spike_dt_s_{0.0012};   // [s] dt spike threshold for diag counter


  // -----------------------
  // Runtime state (RT path)
  // -----------------------
  // Latest target command used by update() (read from rt_cmd_)
  std::array<double, num_joints> desired_velocities_rt_{{0, 0, 0, 0, 0, 0, 0}};

  // Last command actually SENT to hardware (for accel/jerk limiting)
  std::array<double, num_joints> last_sent_{{0, 0, 0, 0, 0, 0, 0}};

  // Last acceleration SENT (for jerk limiting => continuous acceleration)
  std::array<double, num_joints> last_acc_{{0, 0, 0, 0, 0, 0, 0}};

  // (Optional) smoothing state in callback thread (EMA on incoming messages)
  std::array<double, num_joints> smoothed_velocities_cb_{{0, 0, 0, 0, 0, 0, 0}};

  // RT buffer for command handoff (Non-RT callback -> RT update)
  realtime_tools::RealtimeBuffer<std::array<double, num_joints>> rt_cmd_;

  // -----------------------
  // Watchdog state
  // -----------------------
  // Flag set by callback when a new command arrives. update() consumes it.
  std::atomic<bool> new_cmd_{false};

  // Accumulated time since last command (advanced only in update()).
  // No ROS clock access needed; uses period.seconds().
  double time_since_cmd_{0.0};

  // For one-time info logging in callback thread
  bool saw_first_command_{false};

  // -----------------------
  // RT diagnostics (written in update, read/logged in non-RT timer)
  // -----------------------
  std::atomic<uint64_t> max_dt_ns_{0};
  std::atomic<uint64_t> dt_spike_count_{0};
  std::atomic<uint64_t> stall_count_{0};

  // -----------------------
  // ROS interfaces (non-RT)
  // -----------------------
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commands_sub_;

  // Non-RT timer to report dt spikes/stalls without touching ROS in update()
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace iitgn_controllers
