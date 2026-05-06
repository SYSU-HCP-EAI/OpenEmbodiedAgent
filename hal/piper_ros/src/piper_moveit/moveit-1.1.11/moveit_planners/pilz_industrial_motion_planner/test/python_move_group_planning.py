#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Cristian C. Beltran-Hernandez
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
#  * Neither the name of Cristian C. Beltran-Hernandez nor the names of its
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
# Author: Cristian C. Beltran-Hernandez
#
# This test is used to ensure planning with an attached object
# correctly validates collision states between the attached objects
# and the environment

import unittest
import rospy
import rostest

from geometry_msgs.msg import PoseStamped, Point
from moveit_msgs.msg import MoveItErrorCodes
from moveit_commander import MoveGroupCommander, PlanningSceneInterface


class PythonMoveGroupPlanningTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        PLANNING_GROUP = "panda_arm"
        self.group = MoveGroupCommander(PLANNING_GROUP)
        self.planning_scene_interface = PlanningSceneInterface(synchronous=True)
        self.tcp_link = self.group.get_end_effector_link()

        # Plan a motion where the attached object 'box2' collides with the obstacle 'box1'
        target_pose = self.group.get_current_pose(self.tcp_link)
        target_pose.pose.position.y += 0.1
        self.group.set_pose_target(target_pose)

    @classmethod
    def tearDown(self):
        pass

    @classmethod
    def addObjects(self, coll_pos=(0.4, 0.1, 0.25), attached_pos=(0.0, 0.0, 0.15)):
        # Add obstacle to the world
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position = Point(*coll_pos)
        ps.pose.orientation.w = 1.0
        self.planning_scene_interface.add_box(
            name="box1", pose=ps, size=(0.1, 0.1, 0.5)
        )

        # Attach object to robot's TCP
        ps = PoseStamped()
        ps.header.frame_id = self.tcp_link
        ps.pose.position = Point(*attached_pos)
        ps.pose.orientation.w = 1.0
        self.planning_scene_interface.attach_box(
            link=self.tcp_link,
            name="box2",
            pose=ps,
            size=(0.1, 0.1, 0.1),
            touch_links=["panda_rightfinger", "panda_leftfinger"],
        )

    def plan(self, planner, pipeline="pilz_industrial_motion_planner"):
        self.group.set_planning_pipeline_id(pipeline)
        self.group.set_planner_id(planner)
        success, plan, time, error_code = self.group.plan()
        return error_code.val

    def testLINGoalStateCollision(self):
        self.addObjects()
        result = self.plan("LIN")
        self.assertEqual(result, MoveItErrorCodes.INVALID_MOTION_PLAN)

    def testPTPGoalStateCollision(self):
        self.addObjects()
        result = self.plan("PTP")
        self.assertEqual(result, MoveItErrorCodes.INVALID_MOTION_PLAN)

    def testPTPStartStateCollision(self):
        self.addObjects(coll_pos=(0.0, 0.0, 0.25))
        result = self.plan("PTP")
        self.assertEqual(result, MoveItErrorCodes.INVALID_MOTION_PLAN)

    def testPTPStartStateCollisionAttached(self):
        self.addObjects(attached_pos=(-0.3, 0.3, 0.0))
        result = self.plan("PTP")
        self.assertEqual(result, MoveItErrorCodes.INVALID_MOTION_PLAN)


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_python_move_group"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, PythonMoveGroupPlanningTest)
