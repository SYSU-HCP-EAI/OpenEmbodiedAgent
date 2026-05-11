cmake_minimum_required(VERSION 3.1.3)
project(moveit_kinematics)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2
  tf2_kdl
)
moveit_build_options()

set(THIS_PACKAGE_INCLUDE_DIRS
    kdl_kinematics_plugin/include
    lma_kinematics_plugin/include
    srv_kinematics_plugin/include
    cached_ik_kinematics_plugin/include
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    )

add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(lma_kinematics_plugin)
add_subdirectory(srv_kinematics_plugin)
add_subdirectory(ikfast_kinematics_plugin)
add_subdirectory(cached_ik_kinematics_plugin)
add_subdirectory(test)

install(
  FILES
    kdl_kinematics_plugin_description.xml
    lma_kinematics_plugin_description.xml
    srv_kinematics_plugin_description.xml
    cached_ik_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
