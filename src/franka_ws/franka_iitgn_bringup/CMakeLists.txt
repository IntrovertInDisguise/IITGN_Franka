cmake_minimum_required(VERSION 3.8)
project(franka_iitgn_bringup)

find_package(ament_cmake REQUIRED)

# Install launch + config so ros2 launch can find them from the install space
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
