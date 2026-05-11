#!/usr/bin/env python

import rospy
from moveit_ros_planning_interface._moveit_move_group_interface import (
    MoveGroupInterface,
)

group = MoveGroupInterface("manipulator", "robot_description", rospy.get_namespace())
