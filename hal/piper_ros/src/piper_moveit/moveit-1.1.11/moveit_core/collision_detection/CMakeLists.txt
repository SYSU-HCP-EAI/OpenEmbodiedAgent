set(MOVEIT_LIB_NAME moveit_collision_detection)

add_library(${MOVEIT_LIB_NAME}
  src/allvalid/collision_env_allvalid.cpp
  src/collision_common.cpp
  src/collision_matrix.cpp
  src/collision_octomap_filter.cpp
  src/collision_tools.cpp
  src/world.cpp
  src/world_diff.cpp
  src/collision_env.cpp
  src/collision_plugin_cache.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

# unit tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_world test/test_world.cpp)
  target_link_libraries(test_world ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_world_diff test/test_world_diff.cpp)
  target_link_libraries(test_world_diff ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_all_valid test/test_all_valid.cpp)
  target_link_libraries(test_all_valid ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
endif()


install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
