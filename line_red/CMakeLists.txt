cmake_minimum_required(VERSION 3.0.2)
project(red_detector)

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

add_executable(red_detector_node src/red.cpp)

target_link_libraries(red_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
