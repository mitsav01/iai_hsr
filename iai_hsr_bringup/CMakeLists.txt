cmake_minimum_required(VERSION 3.5)
project(iai_hsr_bringup)

# Find ament and dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()