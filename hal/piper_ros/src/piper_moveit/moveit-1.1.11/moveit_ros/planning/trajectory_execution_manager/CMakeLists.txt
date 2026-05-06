set(MOVEIT_LIB_NAME moveit_trajectory_execution_manager)

add_library(${MOVEIT_LIB_NAME} src/trajectory_execution_manager.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_planning_scene_monitor moveit_robot_model_loader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}) # don't build until necessary msgs are available

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(moveit_resources_panda_moveit_config REQUIRED)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_execution_manager
                      test/test_execution_manager.test
                      test/test_execution_manager.cpp)
    target_link_libraries(test_execution_manager moveit_cpp ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})


## This needs further cleanup before it can run
# add_library(test_controller_manager_plugin test/test_moveit_controller_manager_plugin.cpp)
# set_target_properties(test_controller_manager_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# target_link_libraries(test_controller_manager_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

endif(CATKIN_ENABLE_TESTING)
