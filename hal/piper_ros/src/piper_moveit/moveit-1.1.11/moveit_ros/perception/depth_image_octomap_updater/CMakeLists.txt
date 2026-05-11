set(MOVEIT_LIB_NAME moveit_depth_image_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core src/depth_image_octomap_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_lazy_free_space_updater moveit_mesh_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME}_core ${sensor_msgs_EXPORTED_TARGETS})

add_library(${MOVEIT_LIB_NAME} src/updater_plugin.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
