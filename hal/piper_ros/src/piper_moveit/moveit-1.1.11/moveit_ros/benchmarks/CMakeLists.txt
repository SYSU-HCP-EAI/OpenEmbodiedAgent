cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_benchmarks)

set(MOVEIT_LIB_NAME moveit_ros_benchmarks)

find_package(Boost REQUIRED filesystem)

find_package(catkin REQUIRED COMPONENTS
  tf2_eigen
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  pluginlib
)
moveit_build_options()

catkin_package(
  LIBRARIES ${MOVEIT_LIB_NAME}
  CATKIN_DEPENDS
    tf2_eigen
    moveit_ros_planning
    moveit_ros_warehouse
    roscpp
  DEPENDS
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${MOVEIT_LIB_NAME} src/BenchmarkOptions.cpp
                               src/BenchmarkExecutor.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(moveit_combine_predefined_poses_benchmark src/simple_benchmarks/CombinePredefinedPosesBenchmark.cpp)
target_link_libraries(moveit_combine_predefined_poses_benchmark ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS
    moveit_run_benchmark
    moveit_combine_predefined_poses_benchmark
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

catkin_install_python(
  PROGRAMS scripts/moveit_benchmark_statistics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY examples/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)
