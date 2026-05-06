set(MOVEIT_LIB_NAME moveit_trajectory_rviz_plugin)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
  include/moveit/trajectory_rviz_plugin/trajectory_display.h
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Trajectory Display
add_library(${MOVEIT_LIB_NAME}_core
  src/trajectory_display.cpp
  ${HEADERS}
)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core
  moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES}
)

# Plugin Initializer
add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
