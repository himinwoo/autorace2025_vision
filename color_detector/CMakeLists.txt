cmake_minimum_required(VERSION 3.0.2)
project(color_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  coss_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport coss_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Color detector node (통합 버전)
add_executable(color_detector_node src/color_detector.cpp)
target_link_libraries(color_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)