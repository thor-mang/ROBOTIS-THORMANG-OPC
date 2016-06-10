cmake_minimum_required(VERSION 2.8.3)
project(thormang3_action_script_player)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  robotis_controller_msgs
  thormang3_action_module_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(thormang3_action_script_player
   src/action_script_player.cpp   
)

target_link_libraries(thormang3_action_script_player
   yaml-cpp
   ${catkin_LIBRARIES}
)

