cmake_minimum_required(VERSION 3.5)
project(iitgn_controllers)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)
# Add library
add_library(${PROJECT_NAME} SHARED
  src/joint_velocity_controller.cpp
  src/joint_position_controller.cpp
  src/cartesian_velocity_controller.cpp
  src/impedance_controller.cpp
  src/gripper_topic_controller.cpp
  src/move_to_start_controller.cpp
  src/motion_generator.cpp
  src/cartesian_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)
# ------------------------
# Gripper
# ------------------------
add_executable(wrist_torque_publisher
  src/wrist_torque_publisher.cpp)
ament_target_dependencies(wrist_torque_publisher
  rclcpp
  std_msgs
  franka_msgs
)
install(TARGETS wrist_torque_publisher
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(loadcell_publisher
  src/loadcell_publisher.cpp)
ament_target_dependencies(loadcell_publisher
  rclcpp
  std_msgs
)
install(TARGETS loadcell_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# ------------------------


ament_target_dependencies(${PROJECT_NAME}
  realtime_tools
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  franka_msgs
  realtime_tools
  franka_semantic_components
  Eigen3
  rclcpp_action
  std_msgs
  std_srvs
)
# Export plugin description file
pluginlib_export_plugin_description_file(controller_interface iitgn_controllers.xml)
# -----------------------
# Install targets + files
# -----------------------
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
# IMPORTANT: install launch + config so `ros2 launch iitgn_controllers ...` can find them
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# Optional: Add basic linters
if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  set(CPP_DIRS src include)
  ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRS})
  ament_cppcheck(${CPP_DIRS})
  ament_lint_cmake(CMakeLists.txt)
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  franka_msgs
  franka_semantic_components
  Eigen3
  realtime_tools
  rclcpp_action
  std_msgs
  std_srvs
)
ament_package()