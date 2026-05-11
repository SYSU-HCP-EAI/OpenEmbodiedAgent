cmake_minimum_required(VERSION 3.0.2)
project(piper_mujoco)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/piper_no_gripper_mujoco_ros.py
  scripts/piper_mujoco_ros.py
  scripts/piper_mujoco_pid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
