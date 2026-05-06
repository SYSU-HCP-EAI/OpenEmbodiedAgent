cmake_minimum_required(VERSION 3.1.3)
project(moveit_fake_controller_manager)

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
)
moveit_build_options()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    roscpp
)

add_library(${PROJECT_NAME}
   src/moveit_fake_controller_manager.cpp
   src/moveit_fake_controllers.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES moveit_fake_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
