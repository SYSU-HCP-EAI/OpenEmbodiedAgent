#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Masaki Murooka (JSK Robotics Lab, The University of Tokyo)

import unittest
import numpy as np
import rospy
import rostest
import os

from moveit_commander import RobotCommander


class PythonTimeParameterizationTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        self.commander = RobotCommander("robot_description")
        self.group = self.commander.get_group(self.PLANNING_GROUP)

    @classmethod
    def tearDown(self):
        pass

    def plan(self):
        start_pose = self.group.get_current_pose().pose
        goal_pose = self.group.get_current_pose().pose
        goal_pose.position.z -= 0.1
        (plan, fraction) = self.group.compute_cartesian_path(
            [start_pose, goal_pose], 0.005, 0.0
        )
        self.assertEqual(fraction, 1.0, "Cartesian path plan failed")
        return plan

    def time_parameterization(self, plan, algorithm):
        ref_state = self.commander.get_current_state()
        retimed_plan = self.group.retime_trajectory(
            ref_state,
            plan,
            velocity_scaling_factor=0.1,
            acceleration_scaling_factor=0.1,
            algorithm=algorithm,
        )
        return retimed_plan

    def test_plan_and_time_parameterization(self):
        plan = self.plan()
        retimed_plan = self.time_parameterization(
            plan, "iterative_time_parameterization"
        )
        self.assertTrue(
            len(retimed_plan.joint_trajectory.points) > 0, "Retimed plan is invalid"
        )
        retimed_plan = self.time_parameterization(
            plan, "iterative_spline_parameterization"
        )
        self.assertTrue(
            len(retimed_plan.joint_trajectory.points) > 0, "Retimed plan is invalid"
        )
        retimed_plan = self.time_parameterization(
            plan, "time_optimal_trajectory_generation"
        )
        self.assertTrue(
            len(retimed_plan.joint_trajectory.points) > 0, "Retimed plan is invalid"
        )
        retimed_plan = self.time_parameterization(plan, "")
        self.assertTrue(
            len(retimed_plan.joint_trajectory.points) == 0, "Invalid retime algorithm"
        )


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_python_time_parameterization"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonTimeParameterizationTest)

    # suppress cleanup segfault
    os._exit(0)
