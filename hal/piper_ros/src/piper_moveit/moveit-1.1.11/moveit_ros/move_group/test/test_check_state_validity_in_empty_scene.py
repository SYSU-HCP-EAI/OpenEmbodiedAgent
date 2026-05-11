#!/usr/bin/env python

import sys
import rospy
import unittest
from actionlib import SimpleActionClient
import moveit_commander
from moveit_msgs.msg import Constraints
from moveit_msgs.srv import GetStateValidity


class TestCheckStateValidityInEmptyScene(unittest.TestCase):
    def setUp(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot_commander = moveit_commander.RobotCommander()
        self.group_name = self.robot_commander.get_group_names()[0]
        self.group_commander = moveit_commander.MoveGroupCommander(self.group_name)

        self._check_state_validity = rospy.ServiceProxy(
            "/check_state_validity", GetStateValidity
        )

    def test_check_collision_free_state_validity_in_empty_scene(self):
        current_robot_state = self.robot_commander.get_current_state()

        validity_report = self._check_state_validity(
            current_robot_state, self.group_name, Constraints()
        )
        self.assertListEqual(
            validity_report.contacts,
            [],
            "In the default_robot_state, the robot should not collide with itself",
        )

    def test_check_colliding_state_validity_in_empty_scene(self):
        current_robot_state = self.robot_commander.get_current_state()

        # force a colliding state with the Fanuc M-10iA
        current_robot_state.joint_state.position = list(
            current_robot_state.joint_state.position
        )
        current_robot_state.joint_state.position[
            current_robot_state.joint_state.name.index("joint_3")
        ] = -2

        validity_report = self._check_state_validity(
            current_robot_state, self.group_name, Constraints()
        )

        self.assertNotEqual(
            len(validity_report.contacts),
            0,
            "When the robot collides with itself, it should have some contacts (with itself)",
        )


if __name__ == "__main__":
    import rostest

    rospy.init_node("check_state_validity_in_empty_scene")
    rostest.rosrun(
        "moveit_ros_move_group",
        "test_check_state_validity_in_empty_scene",
        TestCheckStateValidityInEmptyScene,
    )
