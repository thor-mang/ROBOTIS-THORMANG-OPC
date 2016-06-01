cmake_minimum_required(VERSION 2.8.3)
project(thormang3_motion_script_player)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  robotis_controller_msgs
  thormang3_action_module_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(thormang3_motion_script_player
   src/motion_script_player.cpp   
)

## Specify libraries to link a library or executable target against
target_link_libraries(thormang3_motion_script_player
   yaml-cpp
   ${catkin_LIBRARIES}
)

