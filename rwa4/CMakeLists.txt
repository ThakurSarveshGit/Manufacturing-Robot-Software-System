cmake_minimum_required(VERSION 2.8.3)
project(rwa4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_ros_planning_interface
  osrf_gear
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rwa4
 CATKIN_DEPENDS geometry_msgs moveit_ros_planning_interface osrf_gear roscpp sensor_msgs std_msgs std_srvs trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  launch
  config
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mainNode src/main_node.cpp src/AriacSensorManager.cpp src/RobotController.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(mainNode ${mainNode_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(mainNode
  ${catkin_LIBRARIES}
)