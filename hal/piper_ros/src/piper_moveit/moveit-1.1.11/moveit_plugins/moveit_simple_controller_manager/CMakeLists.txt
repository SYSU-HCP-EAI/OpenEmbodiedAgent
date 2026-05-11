cmake_minimum_required(VERSION 3.1.3)
project(moveit_simple_controller_manager)

find_package(Boost REQUIRED thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
)
moveit_build_options()

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    control_msgs
    moveit_core
    roscpp
)

add_library(${PROJECT_NAME}
  src/moveit_simple_controller_manager.cpp
  src/follow_joint_trajectory_controller_handle.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES moveit_simple_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
install(DIRECTORY include/${PROJECT_NAME}/
       DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
