cmake_minimum_required(VERSION 3.1.3)
project(moveit_chomp_optimizer_adapter)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  chomp_motion_planner
  pluginlib
)
moveit_build_options()

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/chomp_optimizer_adapter.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES chomp_optimizer_adapter_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
