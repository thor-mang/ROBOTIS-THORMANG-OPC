cmake_minimum_required(VERSION 2.8.3)
project(thormang3_rviz)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  qt_build
  roscpp
  rviz
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

#############
##    QT   ##
#############

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(MOC_FILES
  include/${PROJECT_NAME}/rviz_pose_panel.h
  include/${PROJECT_NAME}/ros_pose_panel_node.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/rviz_pose_panel.cpp
  src/ros_pose_panel_node.cpp
  ${MOC_FILES}
)

## Declare a cpp library
add_library(rviz_thor3_pose_panel
  ${SOURCE_FILES}
  ${QT_FORMS_HPP}
)

target_link_libraries(rviz_thor3_pose_panel
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARY}
)

#############
## Install ##
#############

install(TARGETS rviz_thor3_pose_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
