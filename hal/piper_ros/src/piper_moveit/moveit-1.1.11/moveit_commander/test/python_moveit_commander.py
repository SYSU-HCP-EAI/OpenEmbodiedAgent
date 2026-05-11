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
# Author: William Baker

import unittest

import genpy
import numpy as np
import rospy
import rostest
import os

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    MoveItCommanderException,
)


class PythonMoveitCommanderTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"
    JOINT_NAMES = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]

    @classmethod
    def setUpClass(self):
        self.commander = RobotCommander("robot_description")
        self.group = self.commander.get_group(self.PLANNING_GROUP)

    @classmethod
    def tearDown(self):
        pass

    def test_enforce_bounds_empty_state(self):
        in_msg = RobotState()
        with self.assertRaises(genpy.DeserializationError):
            self.group.enforce_bounds(in_msg)

    def test_enforce_bounds(self):
        in_msg = RobotState()
        in_msg.joint_state.header.frame_id = "base_link"
        in_msg.joint_state.name = self.JOINT_NAMES
        in_msg.joint_state.position = [0] * 6
        in_msg.joint_state.position[0] = 1000

        out_msg = self.group.enforce_bounds(in_msg)

        self.assertEqual(in_msg.joint_state.position[0], 1000)
        self.assertLess(out_msg.joint_state.position[0], 1000)

    def test_get_current_state(self):
        expected_state = RobotState()
        expected_state.joint_state.header.frame_id = "base_link"
        expected_state.multi_dof_joint_state.header.frame_id = "base_link"
        expected_state.joint_state.name = self.JOINT_NAMES
        expected_state.joint_state.position = [0] * 6
        self.assertEqual(self.group.get_current_state(), expected_state)

    def check_target_setting(self, expect, *args):
        if len(args) == 0:
            args = [expect]
        self.group.set_joint_value_target(*args)
        res = self.group.get_joint_value_target()
        self.assertTrue(
            np.all(np.asarray(res) == np.asarray(expect)),
            "Setting failed for %s, values: %s" % (type(args[0]), res),
        )

    def test_target_setting(self):
        n = self.group.get_variable_count()
        self.check_target_setting([0.1] * n)
        self.check_target_setting((0.2,) * n)
        self.check_target_setting(np.zeros(n))
        self.check_target_setting(
            [0.3] * n, {name: 0.3 for name in self.group.get_active_joints()}
        )
        self.check_target_setting([0.5] + [0.3] * (n - 1), "joint_1", 0.5)

        js_target = JointState(name=self.JOINT_NAMES, position=[0.1] * n)
        self.check_target_setting([0.1] * n, js_target)
        # name and position should have the same size, or raise exception
        with self.assertRaises(MoveItCommanderException):
            js_target.position = []
            self.check_target_setting(None, js_target)

    def plan(self, target):
        self.group.set_joint_value_target(target)
        return self.group.plan()

    def test_validation(self):
        current = np.asarray(self.group.get_current_joint_values())

        success1, plan1, time1, err1 = self.plan(current + 0.2)
        success2, plan2, time2, err2 = self.plan(current + 0.2)
        self.assertTrue(success1)
        self.assertTrue(success2)

        # first plan should execute
        self.assertTrue(self.group.execute(plan1))

        # second plan should be invalid now (due to modified start point) and rejected
        self.assertFalse(self.group.execute(plan2))

        # newly planned trajectory should execute again
        success3, plan3, time3, err3 = self.plan(current)
        self.assertTrue(success3)
        self.assertTrue(self.group.execute(plan3))

    def test_gogogo(self):
        current_joints = np.asarray(self.group.get_current_joint_values())

        self.group.set_joint_value_target(current_joints)
        self.assertTrue(self.group.go(True))

        self.assertTrue(self.group.go(current_joints))
        self.assertTrue(self.group.go(list(current_joints)))
        self.assertTrue(self.group.go(tuple(current_joints)))
        self.assertTrue(
            self.group.go(JointState(name=self.JOINT_NAMES, position=current_joints))
        )

        self.group.remember_joint_values("current")
        self.assertTrue(self.group.go("current"))

        current_pose = self.group.get_current_pose()
        self.assertTrue(self.group.go(current_pose))

    def test_planning_scene_interface(self):
        planning_scene = PlanningSceneInterface()


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_python_moveit_commander"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonMoveitCommanderTest)

    # suppress cleanup segfault
    os._exit(0)
