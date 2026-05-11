#!/usr/bin/env python

# This is a simple ROS node used only to configure the param server
# This is used in conjunction with ROS_HOME & roslaunch
# to set the default path for database storage

import roslib

roslib.load_manifest("moveit_warehouse")
import rospy
import os

if __name__ == "__main__":
    rospy.init_node("moveit_warehouse")
    path_base = os.getcwd() + "/moveit_warehouse/"
    rospy.set_param("~database_path_base", path_base)
    rospy.set_param("~default_database", path_base + "default")
