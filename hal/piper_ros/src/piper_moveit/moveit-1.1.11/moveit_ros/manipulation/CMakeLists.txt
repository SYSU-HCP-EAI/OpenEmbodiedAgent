cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_manipulation)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_move_group
  dynamic_reconfigure
  roscpp
  rosconsole
  tf2_eigen
  pluginlib
  actionlib
)
moveit_build_options()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system filesystem date_time program_options)

generate_dynamic_reconfigure_options("pick_place/cfg/PickPlaceDynamicReconfigure.cfg")

catkin_package(
  INCLUDE_DIRS
    pick_place/include
    move_group_pick_place_capability/include
  LIBRARIES
    moveit_pick_place_planner
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    moveit_core
    moveit_msgs
    moveit_ros_planning
    roscpp
  DEPENDS
    EIGEN3
)

include_directories(pick_place/include)
include_directories(move_group_pick_place_capability/include)

include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(pick_place)
add_subdirectory(move_group_pick_place_capability)

install(FILES
  pick_place_capability_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
