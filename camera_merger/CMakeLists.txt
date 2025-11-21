cmake_minimum_required(VERSION 3.0.2)
project(camera_merger)

## C++14 standard
add_compile_options(-std=c++14)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  coss_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    coss_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(camera_merger_node 
  src/camera_merger_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(camera_merger_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS camera_merger_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
