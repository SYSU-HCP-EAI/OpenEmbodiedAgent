#!/usr/bin/env python

import sys
import rospy
import unittest
from actionlib import SimpleActionClient
import moveit_commander
from moveit_msgs.msg import (
    MoveItErrorCodes,
    MoveGroupGoal,
    Constraints,
    JointConstraint,
    MoveGroupAction,
)


class TestMoveActionCancelDrop(unittest.TestCase):
    def setUp(self):
        # create a action client of move group
        self._move_client = SimpleActionClient("move_group", MoveGroupAction)
        self._move_client.wait_for_server()

        moveit_commander.roscpp_initialize(sys.argv)
        group_name = moveit_commander.RobotCommander().get_group_names()[0]
        group = moveit_commander.MoveGroupCommander(group_name)

        # prepare a joint goal
        self._goal = MoveGroupGoal()
        self._goal.request.group_name = group_name
        self._goal.request.max_velocity_scaling_factor = 0.1
        self._goal.request.max_acceleration_scaling_factor = 0.1
        self._goal.request.start_state.is_diff = True

        goal_constraint = Constraints()
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        joint_names = group.get_active_joints()
        for i in range(len(joint_names)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_names[i]
            joint_constraint.position = joint_values[i]
            joint_constraint.weight = 1.0
            goal_constraint.joint_constraints.append(joint_constraint)

        self._goal.request.goal_constraints.append(goal_constraint)
        self._goal.planning_options.planning_scene_diff.robot_state.is_diff = True

    def test_cancel_drop_plan_execution(self):
        # send the goal
        self._move_client.send_goal(self._goal)

        # cancel the goal
        self._move_client.cancel_goal()

        # wait for result
        self._move_client.wait_for_result()

        # polling the result, since result can come after the state is Done
        result = None
        while result is None:
            result = self._move_client.get_result()
            rospy.sleep(0.1)

        # check the error code in result
        # error code is 0 if the server ends with RECALLED status
        self.assertTrue(
            result.error_code.val == MoveItErrorCodes.PREEMPTED
            or result.error_code.val == 0
        )

    def test_cancel_drop_plan_only(self):
        # set the plan only flag
        self._goal.planning_options.plan_only = True

        # send the goal
        self._move_client.send_goal(self._goal)

        # cancel the goal
        self._move_client.cancel_goal()

        # wait for result
        self._move_client.wait_for_result()

        # polling the result, since result can come after the state is Done
        result = None
        while result is None:
            result = self._move_client.get_result()
            rospy.sleep(0.1)

        # check the error code in result
        # error code is 0 if the server ends with RECALLED status
        self.assertTrue(
            result.error_code.val == MoveItErrorCodes.PREEMPTED
            or result.error_code.val == 0
        )

    def test_cancel_resend(self):
        # send the goal
        self._move_client.send_goal(self._goal)

        # cancel the goal
        self._move_client.cancel_goal()

        # send the goal again
        self._move_client.send_goal(self._goal)

        # wait for result
        self._move_client.wait_for_result()

        # polling the result, since result can come after the state is Done
        result = None
        while result is None:
            result = self._move_client.get_result()
            rospy.sleep(0.1)

        # check the error code in result
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)


if __name__ == "__main__":
    import rostest

    rospy.init_node("cancel_before_plan_execution")
    rostest.rosrun(
        "moveit_ros_move_group",
        "test_cancel_before_plan_execution",
        TestMoveActionCancelDrop,
    )
