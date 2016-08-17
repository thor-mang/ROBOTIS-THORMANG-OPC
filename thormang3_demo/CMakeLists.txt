################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.0)
project(thormang3_demo)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  qt_build
  roscpp
  roslib
  cmake_modules
  eigen_conversions
  thormang3_foot_step_generator
  robotis_controller_msgs
  thormang3_walking_module_msgs
  thormang3_manipulation_module_msgs
  thormang3_feet_ft_module_msgs
  humanoid_nav_msgs
  visualization_msgs
  interactive_markers
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
 INCLUDE_DIRS include
)

################################################################################
# Qt Environment
################################################################################
rosbuild_prepare_qt4(QtCore QtGui)

################################################################################
# QT Sections and Sources
################################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/thormang3_demo/*.hpp)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(thormang3_demo ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

add_dependencies(thormang3_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_demo
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  yaml-cpp
)

################################################################################
# Install
################################################################################
install(TARGETS thormang3_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ui/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
