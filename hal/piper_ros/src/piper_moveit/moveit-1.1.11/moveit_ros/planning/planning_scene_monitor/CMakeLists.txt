set(MOVEIT_LIB_NAME moveit_planning_scene_monitor)

add_library(${MOVEIT_LIB_NAME}
  src/planning_scene_monitor.cpp
  src/current_state_monitor.cpp
  src/trajectory_monitor.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model_loader
  moveit_collision_plugin_loader
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}) # don't build until necessary msgs are available

add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(demo_scene ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
