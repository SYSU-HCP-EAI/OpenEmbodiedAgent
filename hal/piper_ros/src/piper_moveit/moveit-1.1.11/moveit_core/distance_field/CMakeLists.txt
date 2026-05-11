set(MOVEIT_LIB_NAME moveit_distance_field)

add_library(${MOVEIT_LIB_NAME}
  src/distance_field.cpp
  src/find_internal_points.cpp
  src/propagation_distance_field.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
  target_link_libraries(test_voxel_grid ${MOVEIT_LIB_NAME})

  catkin_add_gtest(test_distance_field test/test_distance_field.cpp)
  target_link_libraries(test_distance_field ${MOVEIT_LIB_NAME})
endif()
