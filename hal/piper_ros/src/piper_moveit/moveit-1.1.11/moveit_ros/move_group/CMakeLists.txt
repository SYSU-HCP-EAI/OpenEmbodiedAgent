cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_move_group)

find_package(Boost REQUIRED system filesystem date_time program_options thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  actionlib
  roscpp
  pluginlib
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
moveit_build_options()

catkin_package(
  LIBRARIES
    moveit_move_group_capabilities_base
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    moveit_core
    moveit_ros_planning
    roscpp
    std_srvs
    tf2_geometry_msgs
)

include_directories(include)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

add_library(moveit_move_group_capabilities_base
  src/move_group_context.cpp
  src/move_group_capability.cpp
  )
set_target_properties(moveit_move_group_capabilities_base PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(moveit_move_group_capabilities_base ${catkin_EXPORTED_TARGETS}) # wait until all *_msgs packages are finished being built

add_executable(move_group src/move_group.cpp)

add_executable(list_move_group_capabilities src/list_capabilities.cpp)

add_library(moveit_move_group_default_capabilities
  src/default_capabilities/move_action_capability.cpp
  src/default_capabilities/plan_service_capability.cpp
  src/default_capabilities/execute_trajectory_action_capability.cpp
  src/default_capabilities/query_planners_service_capability.cpp
  src/default_capabilities/kinematics_service_capability.cpp
  src/default_capabilities/state_validation_service_capability.cpp
  src/default_capabilities/cartesian_path_service_capability.cpp
  src/default_capabilities/get_planning_scene_service_capability.cpp
  src/default_capabilities/apply_planning_scene_service_capability.cpp
  src/default_capabilities/clear_octomap_service_capability.cpp
  src/default_capabilities/tf_publisher_capability.cpp
  )
set_target_properties(moveit_move_group_default_capabilities PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(moveit_move_group_default_capabilities ${catkin_EXPORTED_TARGETS})

target_link_libraries(moveit_move_group_capabilities_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(move_group moveit_move_group_capabilities_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(moveit_move_group_default_capabilities moveit_move_group_capabilities_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(list_move_group_capabilities ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS move_group list_move_group_capabilities
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS moveit_move_group_capabilities_base moveit_move_group_default_capabilities
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(PROGRAMS
  scripts/load_map scripts/save_map DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  default_capabilities_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # this test is flaky
  # add_rostest(test/test_cancel_before_plan_execution.test)
  add_rostest(test/test_check_state_validity_in_empty_scene.test)
endif()
