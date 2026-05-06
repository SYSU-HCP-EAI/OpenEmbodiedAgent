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
#
# This test is used to ensure planning with a RobotCommander is
# possbile if the robot's move_group node is in a different namespace

import unittest
import numpy as np
import rospy
import rostest
import os

from moveit_commander import RobotCommander


class PythonMoveitCommanderNsTest(unittest.TestCase):
    PLANNING_GROUP = "manipulator"
    PLANNING_NS = "test_ns/"

    @classmethod
    def setUpClass(self):
        self.commander = RobotCommander(
            "%srobot_description" % self.PLANNING_NS, self.PLANNING_NS
        )
        self.group = self.commander.get_group(self.PLANNING_GROUP)

    @classmethod
    def tearDown(self):
        pass

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


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_python_moveit_commander_ns"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonMoveitCommanderNsTest)

    # suppress cleanup segfault
    os._exit(0)
