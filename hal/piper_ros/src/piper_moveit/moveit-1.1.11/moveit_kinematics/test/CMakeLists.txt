if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(xmlrpcpp REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

  include_directories(${moveit_ros_planning_INCLUDE_DIRS})
  include_directories(SYSTEM ${xmlrpcpp_INCLUDE_DIRS})

  catkin_add_executable_with_gtest(test_kinematics_plugin test_kinematics_plugin.cpp)
  target_link_libraries(test_kinematics_plugin ${catkin_LIBRARIES} ${moveit_ros_planning_LIBRARIES} ${xmlrpcpp_LIBRARIES})
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(test_kinematics_plugin PRIVATE -Wno-deprecated-declarations)
  endif()

  set(DEPS DEPENDENCIES test_kinematics_plugin)
  set(ARGS ARGS ik_plugin:=kdl_kinematics_plugin/KDLKinematicsPlugin)
  add_rostest(fanuc-kdl.test ${DEPS} ${ARGS})
  add_rostest(panda-kdl.test ${DEPS} ${ARGS})

  # LMA solver seems to have less precision
  set(ARGS ARGS ik_plugin:=lma_kinematics_plugin/LMAKinematicsPlugin tolerance:=0.0005)
  add_rostest(fanuc-kdl.test ${DEPS} ${ARGS})
  add_rostest(panda-kdl.test ${DEPS} ${ARGS})

  # Run ikfast tests only if the corresponding packages were built
  find_package(fanuc_ikfast_plugin QUIET)
  if (fanuc_ikfast_plugin_FOUND)
    add_rostest(fanuc-ikfast.test ${DEPS})
  endif()

  find_package(panda_ikfast_plugin QUIET)
  if (panda_ikfast_plugin_FOUND)
    add_rostest(panda-ikfast.test ${DEPS})
  endif()

  # Benchmarking program for cached_ik_kinematics
  add_executable(benchmark_ik benchmark_ik.cpp)
  target_link_libraries(benchmark_ik
      ${catkin_LIBRARIES} ${moveit_ros_planning_LIBRARIES}
      ${Boost_PROGRAM_OPTIONS_LIBRARY})
  install(TARGETS benchmark_ik
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
