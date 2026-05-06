cmake_minimum_required(VERSION 3.1.3)
project(moveit_planners_chomp)

# find catkin in isolation so that CATKIN_ENABLE_TESTING is defined
find_package(catkin REQUIRED)
if (CATKIN_ENABLE_TESTING)
  set(CHOMP_TEST_DEPS moveit_ros_planning_interface)
else()
  set(CHOMP_TEST_DEPS)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  pluginlib
  chomp_motion_planner
  ${CHOMP_TEST_DEPS}
)
moveit_build_options()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp moveit_core pluginlib
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/chomp_interface.cpp src/chomp_planning_context.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(chomp_planner_plugin src/chomp_plugin.cpp)
set_target_properties(chomp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(chomp_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES chomp_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/chomp_interface/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} chomp_planner_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # additional test dependencies
  find_package(rostest REQUIRED)
  add_rostest_gtest(chomp_moveit_test
    test/chomp_moveit.test
    test/chomp_moveit_test.cpp)
  target_link_libraries(chomp_moveit_test
    ${catkin_LIBRARIES}
    ${rostest_LIBRARIES})
endif()
