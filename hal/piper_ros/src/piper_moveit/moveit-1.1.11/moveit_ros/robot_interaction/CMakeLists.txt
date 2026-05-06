cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_robot_interaction)

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  interactive_markers
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  roscpp
)
moveit_build_options()

set(MOVEIT_LIB_NAME moveit_robot_interaction)

catkin_package(
  LIBRARIES
    ${MOVEIT_LIB_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    interactive_markers
    moveit_ros_planning
    roscpp
    tf2_geometry_msgs
)

include_directories(include)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

add_library(${MOVEIT_LIB_NAME}
  src/interactive_marker_helpers.cpp
  src/kinematic_options.cpp
  src/kinematic_options_map.cpp
  src/locked_robot_state.cpp
  src/interaction_handler.cpp
  src/robot_interaction.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(locked_robot_state_test test/locked_robot_state_test.cpp)
  target_link_libraries(locked_robot_state_test
    ${MOVEIT_LIB_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
