set(MOVEIT_LIB_NAME moveit_collision_detection_bullet)

add_library(${MOVEIT_LIB_NAME}
  src/bullet_integration/bullet_utils.cpp
  src/bullet_integration/bullet_discrete_bvh_manager.cpp
  src/bullet_integration/bullet_cast_bvh_manager.cpp
  src/collision_env_bullet.cpp
  src/bullet_integration/bullet_bvh_manager.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection
    ${catkin_LIBRARIES} ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES}
    ${BULLET_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(collision_detector_bullet_plugin src/collision_detector_bullet_plugin_loader.cpp)
set_target_properties(collision_detector_bullet_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(collision_detector_bullet_plugin ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} moveit_planning_scene)

install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_bullet_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES ../collision_detector_bullet_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_bullet_collision_detection test/test_bullet_collision_detection_pr2.cpp)
  target_link_libraries(test_bullet_collision_detection moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
    target_compile_options(test_bullet_collision_detection PRIVATE -Wno-deprecated-declarations)
  endif()

  catkin_add_gtest(test_bullet_collision_detection_panda test/test_bullet_collision_detection_panda.cpp)
  target_link_libraries(test_bullet_collision_detection_panda moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
    target_compile_options(test_bullet_collision_detection_panda PRIVATE -Wno-deprecated-declarations)
  endif()

  catkin_add_gtest(test_bullet_continuous_collision_checking test/test_bullet_continuous_collision_checking.cpp)
  target_link_libraries(test_bullet_continuous_collision_checking moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
endif()
