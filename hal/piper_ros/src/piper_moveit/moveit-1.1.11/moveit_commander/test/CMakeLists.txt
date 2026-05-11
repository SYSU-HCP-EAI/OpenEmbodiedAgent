if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(python_moveit_commander.test)
  add_rostest(python_moveit_commander_ns.test)
  add_rostest(python_time_parameterization.test)
endif()
