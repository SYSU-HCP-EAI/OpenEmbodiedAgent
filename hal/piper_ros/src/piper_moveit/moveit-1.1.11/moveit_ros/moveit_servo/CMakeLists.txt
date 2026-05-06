cmake_minimum_required(VERSION 3.1.3)
project(moveit_servo)

set(SERVO_LIB_NAME moveit_servo_cpp_api)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)
moveit_build_options()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    pose_tracking
    ${SERVO_LIB_NAME}
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    std_srvs
    tf2_eigen
    trajectory_msgs
  DEPENDS
    EIGEN3
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

#########################################################
## Library to process realtime twist or joint commands ##
#########################################################

# This library provides an interface for sending realtime twist or joint commands to a robot
add_library(${SERVO_LIB_NAME}
  # These files are used to produce differential motion
  src/collision_check.cpp
  src/servo_calcs.cpp
  src/servo.cpp
  src/low_pass_filter.cpp
)
set_target_properties(${SERVO_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(${SERVO_LIB_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${SERVO_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

# An example of streaming realtime Cartesian and joint commands
add_executable(cpp_interface_example
  src/cpp_interface_example/cpp_interface_example.cpp
)
add_dependencies(cpp_interface_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpp_interface_example
  ${catkin_LIBRARIES}
  ${SERVO_LIB_NAME}
)

# An example of pose tracking
add_executable(pose_tracking_example
  src/cpp_interface_example/pose_tracking_example.cpp
)
add_dependencies(pose_tracking_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_tracking_example
  ${catkin_LIBRARIES}
  ${SERVO_LIB_NAME}
  pose_tracking
)

########################################
## Library for servoing toward a pose ##
########################################

add_library(pose_tracking
  src/pose_tracking.cpp
)
add_dependencies(pose_tracking ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_tracking
  ${catkin_LIBRARIES}
  ${SERVO_LIB_NAME}
)

############################
## ROS message-based node ##
############################

add_executable(servo_server
  src/servo_server.cpp
)
add_dependencies(servo_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_server
  ${SERVO_LIB_NAME}
  ${catkin_LIBRARIES}
)

################################################
## An example of converting joystick commands ##
################################################

add_executable(spacenav_to_twist
  src/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES})

##################
## INSTALLATION ##
##################

install(
  TARGETS
    ${SERVO_LIB_NAME}
    pose_tracking
    pose_tracking_example
    servo_server
    spacenav_to_twist
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## TESTING ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # basic functionality
  add_rostest_gtest(basic_servo_tests
    test/basic_servo_tests.test
    test/basic_servo_tests.cpp
  )
  target_link_libraries(basic_servo_tests
    ${SERVO_LIB_NAME}
    ${catkin_LIBRARIES}
  )

  # servo_cpp_interface
  add_rostest_gtest(servo_cpp_interface_test
    test/servo_cpp_interface_test.test
    test/servo_cpp_interface_test.cpp
  )
  target_link_libraries(servo_cpp_interface_test
    ${SERVO_LIB_NAME}
    ${catkin_LIBRARIES}
  )

  # pose_tracking
  add_rostest_gtest(pose_tracking_test
    test/pose_tracking_test.test
    test/pose_tracking_test.cpp
  )
  target_link_libraries(pose_tracking_test
    pose_tracking
    ${catkin_LIBRARIES}
  )
endif()
