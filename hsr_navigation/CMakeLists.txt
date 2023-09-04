cmake_minimum_required(VERSION 2.8.3)
project(hsr_navigation)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS src/global_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
