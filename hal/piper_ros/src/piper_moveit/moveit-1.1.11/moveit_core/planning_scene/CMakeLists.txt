set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} src/planning_scene.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME} EXPORT_FILE_NAME ${CATKIN_DEVEL_PREFIX}/include/moveit/${MOVEIT_LIB_NAME}_export.h)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE $<BUILD_INTERFACE:${CATKIN_DEVEL_PREFIX}/include>)  # for this library
list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)  # for use by other libraries in moveit_core

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
  moveit_exceptions
  moveit_transforms
  moveit_collision_detection_fcl
  moveit_kinematic_constraints
  moveit_robot_trajectory
  moveit_trajectory_processing
  ${LIBOCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES ${CATKIN_DEVEL_PREFIX}/include/moveit/${MOVEIT_LIB_NAME}_export.h
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/moveit)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_planning_scene test/test_planning_scene.cpp)
  target_link_libraries(test_planning_scene ${MOVEIT_LIB_NAME} moveit_test_utils)

  catkin_add_gtest(test_collision_objects test/test_collision_objects.cpp)
  target_link_libraries(test_collision_objects ${MOVEIT_LIB_NAME} moveit_test_utils)

  catkin_add_gtest(test_multi_threaded test/test_multi_threaded.cpp)
  target_link_libraries(test_multi_threaded ${MOVEIT_LIB_NAME} moveit_test_utils)
endif()
