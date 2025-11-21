cmake_minimum_required(VERSION 3.0.2)
project(onelane_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/onelane_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
