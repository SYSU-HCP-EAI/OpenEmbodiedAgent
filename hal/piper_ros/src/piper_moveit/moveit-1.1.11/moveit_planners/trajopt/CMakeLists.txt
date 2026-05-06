cmake_minimum_required(VERSION 3.1.3)
project(moveit_planners_trajopt)

find_package(catkin REQUIRED
    COMPONENTS
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      roscpp
      rosparam_shortcuts
)
find_package(trajopt REQUIRED)

moveit_build_options()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
      ${PROJECT_NAME}
    INCLUDE_DIRS
    CATKIN_DEPENDS
      roscpp
      pluginlib
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      rosparam_shortcuts
)

# The following include_directory should have include folder of the new planner.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/trajopt_interface.cpp
  src/problem_description.cpp
  src/kinematic_terms.cpp
  src/trajopt_planning_context.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  trajopt::trajopt
)

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES
  VERSION
  "${${PROJECT_NAME}_VERSION}"
)

# TrajOpt planning plugin
add_library(moveit_trajopt_planner_plugin src/trajopt_planner_manager.cpp)
set_target_properties(moveit_trajopt_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_trajopt_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    moveit_trajopt_planner_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/trajopt_interface/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES trajopt_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(trajectory_test test/trajectory_test.test test/trajectory_test.cpp)
  target_link_libraries(
    trajectory_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )

endif()
