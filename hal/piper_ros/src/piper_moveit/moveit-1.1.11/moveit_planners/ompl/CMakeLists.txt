cmake_minimum_required(VERSION 3.1.3)
project(moveit_planners_ompl)

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  rosconsole
  pluginlib
  tf2
  dynamic_reconfigure
)
moveit_build_options()

find_package(ompl REQUIRED)

generate_dynamic_reconfigure_options("ompl_interface/cfg/OMPLDynamicReconfigure.cfg")

catkin_package(
  LIBRARIES
    moveit_ompl_interface
  INCLUDE_DIRS
    ompl_interface/include
  CATKIN_DEPENDS
    dynamic_reconfigure
    moveit_core
    roscpp
  DEPENDS
    OMPL
)

include_directories(ompl_interface/include)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

add_subdirectory(ompl_interface)

install(FILES ompl_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
