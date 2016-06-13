cmake_minimum_required(VERSION 2.8.3)
project(thormang3_foot_step_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  message_generation
  robotis_controller_msgs
  thormang3_walking_module_msgs
)

add_message_files(
   FILES
   FootStepCommand.msg
   Step2D.msg
   Step2DArray.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs 
)

catkin_package(
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(thormang3_foot_step_generator_node 
   src/robotis_foot_step_generator.cpp 
   src/message_callback.cpp
   src/main.cpp
)

target_link_libraries(thormang3_foot_step_generator_node
   yaml-cpp
   ${catkin_LIBRARIES}
)

add_dependencies(thormang3_foot_step_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

