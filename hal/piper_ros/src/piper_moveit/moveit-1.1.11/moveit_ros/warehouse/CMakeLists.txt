cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_warehouse)

find_package(Boost REQUIRED thread system filesystem regex date_time program_options)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  roscpp
  rosconsole
  warehouse_ros
  tf2_eigen
  tf2_ros
)
moveit_build_options()

catkin_package(
  LIBRARIES
    moveit_warehouse
  INCLUDE_DIRS
    warehouse/include
  CATKIN_DEPENDS
    moveit_ros_planning
    warehouse_ros
    roscpp
)

include_directories(warehouse/include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_subdirectory(warehouse)
