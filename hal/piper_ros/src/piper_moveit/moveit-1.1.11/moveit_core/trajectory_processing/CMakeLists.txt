set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME}
  src/iterative_time_parameterization.cpp
  src/iterative_spline_parameterization.cpp
  src/ruckig_traj_smoothing.cpp
  src/trajectory_tools.cpp
  src/time_optimal_trajectory_generation.cpp
  src/limit_cartesian_speed.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state moveit_robot_trajectory ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${ruckig_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_time_parameterization test/test_time_parameterization.cpp)
  target_link_libraries(test_time_parameterization moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
  catkin_add_gtest(test_time_optimal_trajectory_generation test/test_time_optimal_trajectory_generation.cpp)
  target_link_libraries(test_time_optimal_trajectory_generation moveit_test_utils ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${MOVEIT_LIB_NAME})
  catkin_add_gtest(test_limit_cartesian_speed test/test_limit_cartesian_speed.cpp)
  target_link_libraries(test_limit_cartesian_speed moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})

  # These tests are flaky: https://github.com/ros-planning/moveit/actions/runs/3747836136/jobs/6366564878
  #catkin_add_gtest(test_ruckig_traj_smoothing test/test_ruckig_traj_smoothing.cpp)
  #target_link_libraries(test_ruckig_traj_smoothing ${MOVEIT_LIB_NAME} moveit_test_utils)
endif()
