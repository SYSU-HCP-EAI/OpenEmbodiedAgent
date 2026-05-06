cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_planning_interface)

find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_ros_planning
  moveit_ros_warehouse
  moveit_ros_manipulation
  moveit_ros_move_group
  geometry_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  roscpp
  actionlib
  rospy
  rosconsole
)
moveit_build_options()

find_package(PythonInterp REQUIRED)
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)

find_package(Boost REQUIRED)
if(Boost_VERSION LESS 106700)
  set(BOOST_PYTHON_COMPONENT python)
else()
  set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
endif()

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  ${BOOST_PYTHON_COMPONENT}
  system
  thread
)
find_package(Eigen3 REQUIRED)
find_package(eigenpy REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  move_group_interface/include
)

catkin_python_setup()

catkin_package(
  LIBRARIES
    moveit_common_planning_interface_objects
    moveit_planning_scene_interface
    moveit_move_group_interface
    moveit_py_bindings_tools
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    moveit_msgs
    moveit_ros_planning
    moveit_ros_warehouse
    moveit_ros_manipulation
    moveit_ros_move_group
    roscpp
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS})

add_subdirectory(py_bindings_tools)
add_subdirectory(common_planning_interface_objects)
add_subdirectory(planning_scene_interface)
add_subdirectory(move_group_interface)
add_subdirectory(robot_interface)
add_subdirectory(test)
