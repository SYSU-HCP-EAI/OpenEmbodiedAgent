cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_control_interface)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_manager_msgs
  moveit_core
  moveit_simple_controller_manager
  pluginlib
  trajectory_msgs
)
moveit_build_options()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_plugin
    ${PROJECT_NAME}_trajectory_plugin
  CATKIN_DEPENDS
    actionlib
    controller_manager_msgs
    moveit_core
    trajectory_msgs
  DEPENDS Boost
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugin
  src/controller_manager_plugin.cpp
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME}_plugin
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}_trajectory_plugin
  src/joint_trajectory_controller_plugin.cpp
)
set_target_properties(${PROJECT_NAME}_trajectory_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME}_trajectory_plugin
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_plugin ${PROJECT_NAME}_trajectory_plugin
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES moveit_core_plugins.xml moveit_ros_control_interface_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_canopen_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
