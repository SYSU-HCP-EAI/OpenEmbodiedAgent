cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_planning)

find_package(Boost REQUIRED system filesystem date_time program_options thread chrono)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  pluginlib
  actionlib
  roscpp
  rosconsole
  dynamic_reconfigure
  message_filters
  srdfdom
  urdf
  tf2
  tf2_geometry_msgs
  tf2_eigen
  tf2_msgs
  tf2_ros
)
moveit_build_options()

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  "planning_scene_monitor/cfg/PlanningSceneMonitorDynamicReconfigure.cfg"
  "trajectory_execution_manager/cfg/TrajectoryExecutionDynamicReconfigure.cfg"
  "plan_execution/cfg/PlanExecutionDynamicReconfigure.cfg"
  "plan_execution/cfg/SenseForPlanDynamicReconfigure.cfg")

set(THIS_PACKAGE_INCLUDE_DIRS
    rdf_loader/include
    kinematics_plugin_loader/include
    robot_model_loader/include
    constraint_sampler_manager_loader/include
    planning_pipeline/include
    planning_scene_monitor/include
    trajectory_execution_manager/include
    plan_execution/include
    collision_plugin_loader/include
    moveit_cpp/include
)

catkin_package(
  LIBRARIES
    moveit_rdf_loader
    moveit_kinematics_plugin_loader
    moveit_robot_model_loader
    moveit_constraint_sampler_manager_loader
    moveit_planning_pipeline
    moveit_trajectory_execution_manager
    moveit_plan_execution
    moveit_planning_scene_monitor
    moveit_collision_plugin_loader
    moveit_cpp
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    moveit_core
    moveit_ros_occupancy_map_monitor
    moveit_msgs
    pluginlib
    roscpp
    tf2_msgs
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(rdf_loader)
add_subdirectory(collision_plugin_loader)
add_subdirectory(kinematics_plugin_loader)
add_subdirectory(robot_model_loader)
add_subdirectory(constraint_sampler_manager_loader)
add_subdirectory(planning_pipeline)
add_subdirectory(planning_request_adapter_plugins)
add_subdirectory(planning_scene_monitor)
add_subdirectory(planning_components_tools)
add_subdirectory(trajectory_execution_manager)
add_subdirectory(plan_execution)
add_subdirectory(moveit_cpp)

install(
  FILES
    planning_request_adapters_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
