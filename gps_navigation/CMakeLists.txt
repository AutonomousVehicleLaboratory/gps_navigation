cmake_minimum_required(VERSION 3.0.2)
project(gps_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  tf
)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps_navigation
  CATKIN_DEPENDS 
  DEPENDS TinyXML
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# set(CMAKE_CXX_FLAGS "-O3 -Wall ${CMAKE_CXX_FLAGS}")
set(GPS_SRC
  src/gps_navigation.cpp
  src/graph.cpp
  src/utils.cpp
  src/render_bev.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${GPS_SRC}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)