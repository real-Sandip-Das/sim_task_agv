cmake_minimum_required(VERSION 3.10)
project(waypoints_server)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  geometry_msgs
  visualization_msgs
  cartographer_ros_msgs
  move_base_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    visualization_msgs
    cartographer_ros_msgs
    move_base_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
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

## Declare cpp executables
# Waypoints saver node
add_executable(${PROJECT_NAME}_saver
  src/${PROJECT_NAME}_saver.cpp
  src/WaypointsServerSaver.cpp
)
target_compile_features(${PROJECT_NAME}_saver INTERFACE cxx_std_11)

add_dependencies(${PROJECT_NAME}_saver
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_saver
  ${catkin_LIBRARIES}
)

# Waypoints loader and sender node
add_executable(${PROJECT_NAME}_loader_sender
  src/${PROJECT_NAME}_loader_sender.cpp
  src/WaypointsServerLoaderSender.cpp
)
target_compile_features(${PROJECT_NAME}_loader_sender INTERFACE cxx_std_11)

add_dependencies(${PROJECT_NAME}_loader_sender
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_loader_sender
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_saver ${PROJECT_NAME}_loader_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

##########################
## Static code analysis ##
##########################

roslint_cpp()