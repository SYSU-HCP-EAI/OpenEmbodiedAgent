#!/usr/bin/env python

import unittest
import rospy
import rostest
import os

import moveit_commander

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory


class RobotStateUpdateTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.dual_arm_group = moveit_commander.MoveGroupCommander("dual_arm")
        self.panda_1_group = moveit_commander.MoveGroupCommander("panda_1")
        self.panda_2_group = moveit_commander.MoveGroupCommander("panda_2")

    @classmethod
    def tearDown(self):
        pass

    def plan(self, target):
        self.dual_arm_group.set_joint_value_target(target)
        return self.dual_arm_group.plan()

    def test(self):
        panda_1_pose = self.panda_1_group.get_current_pose("panda_1_link8")
        panda_1_pose.pose.position.z -= 0.05
        panda_1_pose.pose.position.x += 0.05

        panda_2_pose = self.panda_2_group.get_current_pose("panda_2_link8")
        panda_2_pose.pose.position.z -= 0.05
        panda_2_pose.pose.position.x -= 0.05
        self.dual_arm_group.set_start_state_to_current_state()
        self.dual_arm_group.set_pose_target(
            panda_1_pose, end_effector_link="panda_1_link8"
        )
        self.dual_arm_group.set_pose_target(
            panda_2_pose, end_effector_link="panda_2_link8"
        )

        result = self.dual_arm_group.plan()
        if isinstance(result, RobotTrajectory):
            self.assertTrue(result.joint_trajectory.joint_names)
        else:
            success, plan, planning_time, error = result
            self.assertTrue(error.val == MoveItErrorCodes.SUCCESS)


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_robot_state_update"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, RobotStateUpdateTest)

    # suppress cleanup segfault in ROS < Kinetic
    os._exit(0)
