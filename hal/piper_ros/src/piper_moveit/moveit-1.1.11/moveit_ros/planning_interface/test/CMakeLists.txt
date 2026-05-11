if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(test_cleanup test_cleanup.cpp)
  target_link_libraries(test_cleanup moveit_move_group_interface)

  add_rostest_gtest(move_group_interface_cpp_test move_group_interface_cpp_test.test move_group_interface_cpp_test.cpp)
  target_link_libraries(move_group_interface_cpp_test moveit_move_group_interface ${catkin_LIBRARIES})

  add_rostest_gtest(move_group_pick_place_test move_group_pick_place_test.test move_group_pick_place_test.cpp)
  target_link_libraries(move_group_pick_place_test moveit_move_group_interface ${catkin_LIBRARIES})

  add_rostest_gtest(subframes_test subframes_test.test subframes_test.cpp)
  target_link_libraries(subframes_test moveit_move_group_interface ${catkin_LIBRARIES})

  add_rostest(python_move_group.test)
  add_rostest(python_move_group_ns.test)
  add_rostest(robot_state_update.test)
  add_rostest(dual_arm_robot_state_update.test)
  add_rostest(cleanup.test)

  SET(HELPER_LIB moveit_planning_interface_test_serialize_msg_cpp_helper)
  add_library(${HELPER_LIB} serialize_msg_python_cpp_helpers.cpp)
  set_target_properties(${HELPER_LIB} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
  target_link_libraries(${HELPER_LIB} ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_py_bindings_tools)
  set_target_properties(${HELPER_LIB} PROPERTIES OUTPUT_NAME "_${HELPER_LIB}" PREFIX "")
  set_target_properties(${HELPER_LIB} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
  if(WIN32)
    set_target_properties(${HELPER_LIB} PROPERTIES SUFFIX .pyd)
  endif(WIN32)

  catkin_add_nosetests(serialize_msg.py)
endif()
