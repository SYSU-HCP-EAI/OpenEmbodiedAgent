cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_visualization)

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

find_package(Boost REQUIRED thread date_time system filesystem)
find_package(catkin REQUIRED COMPONENTS
  class_loader
  geometric_shapes
  interactive_markers
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_warehouse
  object_recognition_msgs
  pluginlib
  rosconsole
  roscpp
  rospy
  rviz
  tf2_eigen
  roscpp
  rosconsole
  object_recognition_msgs
)
moveit_build_options()

# TODO: Remove when Kinetic support is dropped
if(rviz_VERSION VERSION_LESS 1.13.1) # Does rviz support TF2?
  add_definitions(-DRVIZ_TF1)
endif()

find_package(Eigen3 REQUIRED)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    moveit_motion_planning_rviz_plugin_core
    moveit_planning_scene_rviz_plugin_core
    moveit_robot_state_rviz_plugin_core
    moveit_rviz_plugin_render_tools
    moveit_trajectory_rviz_plugin_core
  INCLUDE_DIRS
    motion_planning_rviz_plugin/include
    planning_scene_rviz_plugin/include
    robot_state_rviz_plugin/include
    rviz_plugin_render_tools/include
    trajectory_rviz_plugin/include
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    object_recognition_msgs
    roscpp
    rviz
  DEPENDS
    EIGEN3
)

catkin_install_python(PROGRAMS scripts/moveit_joy.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_python_setup()

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    planning_scene_rviz_plugin/include
                    motion_planning_rviz_plugin/include
                    trajectory_rviz_plugin/include)

include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR})

add_subdirectory(rviz_plugin_render_tools)
add_subdirectory(robot_state_rviz_plugin)
add_subdirectory(planning_scene_rviz_plugin)
add_subdirectory(motion_planning_rviz_plugin)
add_subdirectory(trajectory_rviz_plugin)

install(FILES
  motion_planning_rviz_plugin_description.xml
  trajectory_rviz_plugin_description.xml
  planning_scene_rviz_plugin_description.xml
  robot_state_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/moveit_joy.test)
endif()
