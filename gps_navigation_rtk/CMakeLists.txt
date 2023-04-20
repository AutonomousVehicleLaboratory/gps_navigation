cmake_minimum_required(VERSION 3.0.2)
project(gps_navigation_rtk)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  planner_msgs
  image_transport
  cv_bridge
  gps_navigation
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gps_navigation_rtk
 CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs planner_msgs gps_navigation
 DEPENDS system_lib 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/gps_navigation_rtk_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
