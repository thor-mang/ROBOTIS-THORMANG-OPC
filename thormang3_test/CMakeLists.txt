cmake_minimum_required(VERSION 2.8.3)
project(thormang3_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  thormang3_manipulation_module_msgs
  thormang3_foot_step_generator
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(thormang3_test
  src/main.cpp
  src/thormang3_test.cpp
)

add_dependencies(thormang3_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_test
  ${catkin_LIBRARIES}
)
