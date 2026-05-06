set(MOVEIT_LIB_NAME moveit_collision_distance_field)

add_library(${MOVEIT_LIB_NAME}
  src/collision_distance_field_types.cpp
  src/collision_common_distance_field.cpp
  src/collision_env_distance_field.cpp
  src/collision_env_hybrid.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_planning_scene moveit_distance_field
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_collision_distance_field test/test_collision_distance_field.cpp)
  target_link_libraries(test_collision_distance_field ${MOVEIT_LIB_NAME} moveit_test_utils)
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
