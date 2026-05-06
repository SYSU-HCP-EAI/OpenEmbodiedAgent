set(MOVEIT_LIB_NAME moveit_ompl_interface)

add_library(${MOVEIT_LIB_NAME}
  src/ompl_interface.cpp
  src/planning_context_manager.cpp
  src/model_based_planning_context.cpp
  src/parameterization/model_based_state_space.cpp
  src/parameterization/model_based_state_space_factory.cpp
  src/parameterization/joint_space/joint_model_state_space.cpp
  src/parameterization/joint_space/joint_model_state_space_factory.cpp
  src/parameterization/work_space/pose_model_state_space.cpp
  src/parameterization/work_space/pose_model_state_space_factory.cpp
  src/detail/threadsafe_state_storage.cpp
  src/detail/state_validity_checker.cpp
  src/detail/projection_evaluators.cpp
  src/detail/goal_union.cpp
  src/detail/constraints_library.cpp
  src/detail/constrained_sampler.cpp
  src/detail/constrained_goal_sampler.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

find_package(OpenMP REQUIRED)

target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

add_executable(moveit_generate_state_database scripts/generate_state_database.cpp)
target_link_libraries(moveit_generate_state_database ${MOVEIT_LIB_NAME})
set_target_properties(moveit_generate_state_database PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_generate_state_database PROPERTIES OUTPUT_NAME "generate_state_database")

add_library(moveit_ompl_planner_plugin src/ompl_planner_manager.cpp)
set_target_properties(moveit_ompl_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(moveit_ompl_planner_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS}) # don't build until necessary msgs are available

install(TARGETS ${MOVEIT_LIB_NAME} moveit_ompl_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS moveit_generate_state_database
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_state_space test/test_state_space.cpp)
  target_link_libraries(test_state_space ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES})
  set_target_properties(test_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

  find_package(rostest REQUIRED)
  find_package(tf2_eigen REQUIRED)

  add_rostest_gtest(test_planning_context_manager
    test/test_planning_context_manager.test
    test/test_planning_context_manager.cpp)
  target_link_libraries(test_planning_context_manager ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${tf2_eigen_LIBRARIES})

  catkin_add_gtest(test_state_validity_checker test/test_state_validity_checker.cpp)
  target_link_libraries(test_state_validity_checker ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES})
  set_target_properties(test_state_validity_checker PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
endif()
