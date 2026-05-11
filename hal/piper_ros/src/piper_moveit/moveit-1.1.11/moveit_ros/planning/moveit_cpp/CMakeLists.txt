set(MOVEIT_LIB_NAME moveit_cpp)

add_library(${MOVEIT_LIB_NAME}
  src/moveit_cpp.cpp
  src/planning_component.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_planning_scene_monitor
  moveit_planning_pipeline
  moveit_trajectory_execution_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
  find_package(moveit_resources_panda_moveit_config REQUIRED)
  find_package(rostest REQUIRED)

  add_rostest_gtest(moveit_cpp_test test/moveit_cpp_test.test test/moveit_cpp_test.cpp)
  target_link_libraries(moveit_cpp_test moveit_cpp ${catkin_LIBRARIES})
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
