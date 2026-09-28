// dummy_loadcell_publisher.cpp
//
// MODE B — Load cell dummy publisher
//
// Reads newline-delimited force values (in Newtons) from stdin
// and publishes them to ~/force_feedback (std_msgs/Float64).
//
// Pipe your actual load cell serial driver output here, e.g.:
//   python3 my_loadcell_driver.py | \
//       ros2 run iitgn_controllers dummy_loadcell_publisher \
//           --ros-args -r __ns:=/fr3
//
// OR for quick bench testing without real hardware, use the
// interactive mode — just type a number and press Enter:
//   ros2 run iitgn_controllers dummy_loadcell_publisher \
//       --ros-args -r __ns:=/fr3
//
// The GripperTopicController subscribes to ~/force_feedback,
// so this node feeds the force regulation loop directly.
//
// Build: add to iitgn_controllers/CMakeLists.txt (see snippet at bottom)

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class DummyLoadcellPublisher : public rclcpp::Node {
 public:
  DummyLoadcellPublisher() : Node("dummy_loadcell_publisher") {
    declare_parameter<std::string>("output_topic",
                                   "gripper_topic_controller/force_feedback");
    declare_parameter<double>("publish_rate_hz", 50.0);

    std::string topic = get_parameter("output_topic").as_string();
    double rate_hz    = get_parameter("publish_rate_hz").as_double();
    period_ms_        = static_cast<int>(1000.0 / rate_hz);

    pub_ = create_publisher<std_msgs::msg::Float64>(topic, 10);

    RCLCPP_INFO(get_logger(),
                "DummyLoadcellPublisher\n"
                "  publishing : %s  @ %.0f Hz\n"
                "  stdin mode : type force (N) + Enter, or pipe driver output",
                topic.c_str(), rate_hz);

    // Publish thread at fixed rate
    publish_thread_ = std::thread([this]() { publishLoop(); });

    // Stdin reader thread
    stdin_thread_ = std::thread([this]() { stdinLoop(); });
  }

  ~DummyLoadcellPublisher() override {
    running_ = false;
    if (publish_thread_.joinable()) publish_thread_.join();
    if (stdin_thread_.joinable())   stdin_thread_.join();
  }

 private:
  void publishLoop() {
    while (running_ && rclcpp::ok()) {
      std_msgs::msg::Float64 msg;
      msg.data = latest_force_.load();
      pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
    }
  }

  void stdinLoop() {
    std::string line;
    while (running_ && rclcpp::ok() && std::getline(std::cin, line)) {
      if (line.empty()) continue;
      try {
        double force = std::stod(line);
        latest_force_.store(std::abs(force));
        RCLCPP_INFO(get_logger(), "Load cell input: %.2f N", std::abs(force));
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Could not parse force from: '%s'", line.c_str());
      }
    }
    running_ = false;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  std::thread        publish_thread_;
  std::thread        stdin_thread_;
  std::atomic<bool>  running_{true};
  std::atomic<double> latest_force_{0.0};
  int                period_ms_{20};  // 50 Hz default
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyLoadcellPublisher>());
  rclcpp::shutdown();
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// CMakeLists.txt snippet (add inside iitgn_controllers/CMakeLists.txt):
//
//   add_executable(dummy_loadcell_publisher
//       src/dummy_loadcell_publisher.cpp)
//   ament_target_dependencies(dummy_loadcell_publisher
//       rclcpp std_msgs)
//   install(TARGETS dummy_loadcell_publisher
//       DESTINATION lib/${PROJECT_NAME})
// ─────────────────────────────────────────────────────────────────────────────
