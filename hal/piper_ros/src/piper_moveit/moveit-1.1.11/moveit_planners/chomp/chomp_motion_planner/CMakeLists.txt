cmake_minimum_required(VERSION 3.1.3)
project(chomp_motion_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
)
moveit_build_options()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/chomp_cost.cpp
  src/chomp_parameters.cpp
  src/chomp_trajectory.cpp
  src/chomp_optimizer.cpp
  src/chomp_planner.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
