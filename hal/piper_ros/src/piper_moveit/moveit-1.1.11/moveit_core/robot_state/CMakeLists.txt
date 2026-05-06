set(MOVEIT_LIB_NAME moveit_robot_state)

add_library(${MOVEIT_LIB_NAME}
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
  src/cartesian_interpolator.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model moveit_kinematics_base moveit_transforms ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# Unit tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_robot_state test/robot_state_test.cpp)
  target_link_libraries(test_robot_state ${MOVEIT_LIB_NAME} moveit_test_utils)

  # As an executable, this benchmark is not run as a test by default
  add_executable(robot_state_benchmark test/robot_state_benchmark.cpp)
  target_link_libraries(robot_state_benchmark ${MOVEIT_LIB_NAME} moveit_test_utils ${GTEST_LIBRARIES})

  add_rostest_gtest(test_cartesian_interpolator
    test/test_cartesian_interpolator.test
    test/test_cartesian_interpolator.cpp)
  target_link_libraries(test_cartesian_interpolator ${MOVEIT_LIB_NAME} moveit_test_utils)

  catkin_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_link_libraries(test_robot_state_complex ${MOVEIT_LIB_NAME} moveit_test_utils)

  catkin_add_gtest(test_aabb test/test_aabb.cpp)
  target_link_libraries(test_aabb ${MOVEIT_LIB_NAME} moveit_test_utils)
endif()
