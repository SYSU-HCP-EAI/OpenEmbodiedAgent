#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Open Source Robotics Foundation
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
#  * Neither the name of Open Source Robotics Foundation. nor the names of its
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
# Author: Peter Mitrano
#
# This test is used to ensure planning with a RobotCommander is
# possible if the robot's move_group node is in a different namespace

import unittest
import rospy
import rostest
import os

from moveit_commander import PlanningSceneInterface


class PythonMoveitCommanderRosNamespaceTest(unittest.TestCase):
    def test_namespace(self):
        self.scene = PlanningSceneInterface(synchronous=False)
        expected_resolved_co_name = "/test_ros_namespace/collision_object"
        expected_resolved_aco_name = "/test_ros_namespace/attached_collision_object"
        self.assertEqual(self.scene._pub_co.resolved_name, expected_resolved_co_name)
        self.assertEqual(self.scene._pub_aco.resolved_name, expected_resolved_aco_name)

    def test_namespace_synchronous(self):
        self.scene = PlanningSceneInterface(synchronous=True)
        expected_resolved_apply_diff_name = "/test_ros_namespace/apply_planning_scene"
        self.assertEqual(
            self.scene._apply_planning_scene_diff.resolved_name,
            expected_resolved_apply_diff_name,
        )


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_python_moveit_commander_ros_namespace"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonMoveitCommanderRosNamespaceTest)

    # suppress cleanup segfault
    os._exit(0)
