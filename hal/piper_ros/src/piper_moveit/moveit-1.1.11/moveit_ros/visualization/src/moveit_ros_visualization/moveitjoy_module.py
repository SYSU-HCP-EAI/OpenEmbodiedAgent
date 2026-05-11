# ********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, JSK, The University of Tokyo.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the JSK, The University of Tokyo nor the names of its
#     nor the names of its contributors may be
#     used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ********************************************************************/

#   Author: Ryohei Ueda, Dave Coleman
#   Desc:   Interface between PS3/XBox controller and MoveIt Motion Planning Rviz Plugin

from __future__ import print_function

import xml.dom.minidom
from operator import add
import sys
import threading
from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface

import rospy
import roslib
import numpy
import time
import tf
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit


def signedSquare(val):
    if val > 0:
        sign = 1
    else:
        sign = -1
    return val * val * sign


# classes to use joystick of xbox, ps3(wired) and ps3(wireless).


class JoyStatus:
    def __init__(self):
        self.center = False
        self.select = False
        self.start = False
        self.L3 = False
        self.R3 = False
        self.square = False
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.triangle = False
        self.cross = False
        self.circle = False
        self.L1 = False
        self.R1 = False
        self.L2 = False
        self.R2 = False
        self.left_analog_x = 0.0
        self.left_analog_y = 0.0
        self.right_analog_x = 0.0
        self.right_analog_y = 0.0


class XBoxStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        self.center = msg.buttons[8] == 1
        self.select = msg.buttons[6] == 1
        self.start = msg.buttons[7] == 1
        self.L3 = msg.buttons[9] == 1
        self.R3 = msg.buttons[10] == 1
        self.square = msg.buttons[2] == 1
        self.circle = msg.buttons[1] == 1
        self.up = msg.axes[7] > 0.1
        self.down = msg.axes[7] < -0.1
        self.left = msg.axes[6] > 0.1
        self.right = msg.axes[6] < -0.1
        self.triangle = msg.buttons[3] == 1
        self.cross = msg.buttons[0] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.axes[2] < -0.5
        self.R2 = msg.axes[5] < -0.5
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg


class PS3DualShockStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.cross = msg.buttons[0] == 1
        self.circle = msg.buttons[1] == 1
        self.triangle = msg.buttons[2] == 1
        self.square = msg.buttons[3] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.center = msg.buttons[10] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]

        self.orig_msg = msg


class PS3Status(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.center = msg.buttons[16] == 1
        self.select = msg.buttons[0] == 1
        self.start = msg.buttons[3] == 1
        self.L3 = msg.buttons[1] == 1
        self.R3 = msg.buttons[2] == 1
        self.square = msg.axes[15] < 0
        self.up = msg.axes[4] < 0
        self.down = msg.axes[6] < 0
        self.left = msg.axes[7] < 0
        self.right = msg.axes[5] < 0
        self.triangle = msg.axes[12] < 0
        self.cross = msg.axes[14] < 0
        self.circle = msg.axes[13] < 0
        self.L1 = msg.axes[10] < 0
        self.R1 = msg.axes[11] < 0
        self.L2 = msg.axes[8] < 0
        self.R2 = msg.axes[9] < 0
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg


class PS3WiredStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msgs/Joy
        self.center = msg.buttons[16] == 1
        self.select = msg.buttons[0] == 1
        self.start = msg.buttons[3] == 1
        self.L3 = msg.buttons[1] == 1
        self.R3 = msg.buttons[2] == 1
        self.square = msg.buttons[15] == 1
        self.up = msg.buttons[4] == 1
        self.down = msg.buttons[6] == 1
        self.left = msg.buttons[7] == 1
        self.right = msg.buttons[5] == 1
        self.triangle = msg.buttons[12] == 1
        self.cross = msg.buttons[14] == 1
        self.circle = msg.buttons[13] == 1
        self.L1 = msg.buttons[10] == 1
        self.R1 = msg.buttons[11] == 1
        self.L2 = msg.buttons[8] == 1
        self.R2 = msg.buttons[9] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[2]
        self.right_analog_y = msg.axes[3]
        self.orig_msg = msg


class PS4Status(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msg/Joy
        self.center = msg.buttons[12] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.L3 = msg.buttons[10] == 1
        self.R3 = msg.buttons[11] == 1
        self.square = msg.buttons[0] == 1
        self.up = msg.axes[10] < 0
        self.down = msg.axes[10] > 0
        self.left = msg.axes[9] < 0
        self.right = msg.axes[9] > 0
        self.triangle = msg.buttons[3] == 1
        self.cross = msg.buttons[1] == 1
        self.circle = msg.buttons[2] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[5]
        self.right_analog_y = msg.axes[2]
        self.orig_msg = msg


class PS4WiredStatus(JoyStatus):
    def __init__(self, msg):
        JoyStatus.__init__(self)
        # creating from sensor_msg/Joy
        self.center = msg.buttons[10] == 1
        self.select = msg.buttons[8] == 1
        self.start = msg.buttons[9] == 1
        self.L3 = msg.buttons[11] == 1
        self.R3 = msg.buttons[12] == 1
        self.square = msg.buttons[3] == 1
        self.up = msg.axes[7] < 0
        self.down = msg.axes[7] > 0
        self.left = msg.axes[6] < 0
        self.right = msg.axes[6] > 0
        self.triangle = msg.buttons[2] == 1
        self.cross = msg.buttons[0] == 1
        self.circle = msg.buttons[1] == 1
        self.L1 = msg.buttons[4] == 1
        self.R1 = msg.buttons[5] == 1
        self.L2 = msg.buttons[6] == 1
        self.R2 = msg.buttons[7] == 1
        self.left_analog_x = msg.axes[0]
        self.left_analog_y = msg.axes[1]
        self.right_analog_x = msg.axes[3]
        self.right_analog_y = msg.axes[4]
        self.orig_msg = msg


class StatusHistory:
    def __init__(self, max_length=10):
        self.max_length = max_length
        self.buffer = []

    def add(self, status):
        self.buffer.append(status)
        if len(self.buffer) > self.max_length:
            self.buffer = self.buffer[1 : self.max_length + 1]

    def all(self, proc):
        for status in self.buffer:
            if not proc(status):
                return False
        return True

    def latest(self):
        if len(self.buffer) > 0:
            return self.buffer[-1]
        else:
            return None

    def length(self):
        return len(self.buffer)

    def new(self, status, attr):
        if len(self.buffer) == 0:
            return getattr(status, attr)
        else:
            return getattr(status, attr) and not getattr(self.latest(), attr)


class MoveitJoy:
    def parseSRDF(self):
        ri = RobotInterface("/robot_description")
        planning_groups = {}
        for g in ri.get_group_names():
            self.planning_groups_tips[g] = ri.get_group_joint_tips(g)
            if len(self.planning_groups_tips[g]) > 0:
                planning_groups[g] = [
                    "/rviz/moveit/move_marker/goal_" + l
                    for l in self.planning_groups_tips[g]
                ]
        for name in planning_groups.keys():
            print(name, planning_groups[name])
        self.planning_groups = planning_groups
        self.planning_groups_keys = list(
            planning_groups.keys()
        )  # we'd like to store the 'order'
        self.frame_id = ri.get_planning_frame()

    def __init__(self):
        self.initial_poses = {}
        self.planning_groups_tips = {}
        self.tf_listener = tf.TransformListener()
        self.marker_lock = threading.Lock()
        self.prev_time = rospy.Time.now()
        self.counter = 0
        self.history = StatusHistory(max_length=10)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.current_planning_group_index = 0
        self.current_eef_index = 0
        self.initialize_poses = False
        self.initialized = False
        self.parseSRDF()
        self.plan_group_pub = rospy.Publisher(
            "/rviz/moveit/select_planning_group", String, queue_size=5
        )
        self.updatePlanningGroup(0)
        self.updatePoseTopic(0, False)
        self.joy_pose_pub = rospy.Publisher("/joy_pose", PoseStamped, queue_size=1)
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
        self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty, queue_size=5)
        self.update_start_state_pub = rospy.Publisher(
            "/rviz/moveit/update_start_state", Empty, queue_size=5
        )
        self.update_goal_state_pub = rospy.Publisher(
            "/rviz/moveit/update_goal_state", Empty, queue_size=5
        )
        self.interactive_marker_sub = rospy.Subscriber(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full",
            InteractiveMarkerInit,
            self.markerCB,
            queue_size=1,
        )
        self.sub = rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=1)

    def updatePlanningGroup(self, next_index):
        if next_index >= len(self.planning_groups_keys):
            self.current_planning_group_index = 0
        elif next_index < 0:
            self.current_planning_group_index = len(self.planning_groups_keys) - 1
        else:
            self.current_planning_group_index = next_index
        next_planning_group = None
        try:
            next_planning_group = self.planning_groups_keys[
                self.current_planning_group_index
            ]
        except IndexError:
            msg = "Check if you started movegroups. Exiting."
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)
        rospy.loginfo("Changed planning group to " + next_planning_group)
        self.plan_group_pub.publish(next_planning_group)

    def updatePoseTopic(self, next_index, wait=True):
        planning_group = self.planning_groups_keys[self.current_planning_group_index]
        topics = self.planning_groups[planning_group]
        if next_index >= len(topics):
            self.current_eef_index = 0
        elif next_index < 0:
            self.current_eef_index = len(topics) - 1
        else:
            self.current_eef_index = next_index
        next_topic = topics[self.current_eef_index]

        rospy.loginfo(
            "Changed controlled end effector to "
            + self.planning_groups_tips[planning_group][self.current_eef_index]
        )
        self.pose_pub = rospy.Publisher(next_topic, PoseStamped, queue_size=5)
        if wait:
            self.waitForInitialPose(next_topic)
        self.current_pose_topic = next_topic

    def markerCB(self, msg):
        try:
            self.marker_lock.acquire()
            if not self.initialize_poses:
                return
            self.initial_poses = {}
            for marker in msg.markers:
                if marker.name.startswith("EE:goal_"):
                    # resolve tf
                    if marker.header.frame_id != self.frame_id:
                        ps = PoseStamped(header=marker.header, pose=marker.pose)
                        try:
                            transformed_pose = self.tf_listener.transformPose(
                                self.frame_id, ps
                            )
                            self.initial_poses[marker.name[3:]] = transformed_pose.pose
                        except (
                            tf.LookupException,
                            tf.ConnectivityException,
                            tf.ExtrapolationException,
                            e,
                        ):
                            rospy.logerr("tf error when resolving tf: %s" % e)
                    else:
                        self.initial_poses[
                            marker.name[3:]
                        ] = marker.pose  # tf should be resolved
        finally:
            self.marker_lock.release()

    def waitForInitialPose(self, next_topic, timeout=None):
        counter = 0
        while not rospy.is_shutdown():
            counter = counter + 1
            if timeout and counter >= timeout:
                return False
            try:
                self.marker_lock.acquire()
                self.initialize_poses = True
                topic_suffix = next_topic.split("/")[-1]
                if topic_suffix in self.initial_poses:
                    self.pre_pose = PoseStamped(pose=self.initial_poses[topic_suffix])
                    self.initialize_poses = False
                    return True
                else:
                    rospy.logdebug(self.initial_poses.keys())
                    rospy.loginfo(
                        "Waiting for pose topic of '%s' to be initialized", topic_suffix
                    )
                    rospy.sleep(1)
            finally:
                self.marker_lock.release()

    def joyCB(self, msg):
        axes_amount = len(msg.axes)
        buttons_amount = len(msg.buttons)
        if axes_amount == 27 and buttons_amount == 19:
            status = PS3WiredStatus(msg)
        elif axes_amount == 8 and buttons_amount == 11:
            status = XBoxStatus(msg)
        elif axes_amount == 20 and buttons_amount == 17:
            status = PS3Status(msg)
        elif axes_amount == 14 and buttons_amount == 14:
            status = PS4Status(msg)
        elif axes_amount == 8 and buttons_amount == 13:
            status = PS4WiredStatus(msg)
        elif axes_amount == 6 and buttons_amount == 17:
            status = PS3DualShockStatus(msg)
        else:
            raise Exception(
                "Unknown joystick, axes: {}, buttons: {}".format(
                    axes_amount, buttons_amount
                )
            )
        self.run(status)
        self.history.add(status)

    def computePoseFromJoy(self, pre_pose, status):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)
        # move in local
        dist = (
            status.left_analog_y * status.left_analog_y
            + status.left_analog_x * status.left_analog_x
        )
        scale = 200.0
        x_diff = signedSquare(status.left_analog_y) / scale
        y_diff = signedSquare(status.left_analog_x) / scale
        # z
        if status.L2:
            z_diff = 0.005
        elif status.R2:
            z_diff = -0.005
        else:
            z_diff = 0.0
        if self.history.all(lambda s: s.L2) or self.history.all(lambda s: s.R2):
            z_scale = 4.0
        else:
            z_scale = 2.0
        local_move = numpy.array((x_diff, y_diff, z_diff * z_scale, 1.0))
        q = numpy.array(
            (
                pre_pose.pose.orientation.x,
                pre_pose.pose.orientation.y,
                pre_pose.pose.orientation.z,
                pre_pose.pose.orientation.w,
            )
        )
        xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q), local_move)
        new_pose.pose.position.x = pre_pose.pose.position.x + xyz_move[0]
        new_pose.pose.position.y = pre_pose.pose.position.y + xyz_move[1]
        new_pose.pose.position.z = pre_pose.pose.position.z + xyz_move[2]
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        DTHETA = 0.005
        if status.L1:
            if self.history.all(lambda s: s.L1):
                yaw = yaw + DTHETA * 2
            else:
                yaw = yaw + DTHETA
        elif status.R1:
            if self.history.all(lambda s: s.R1):
                yaw = yaw - DTHETA * 2
            else:
                yaw = yaw - DTHETA
        if status.up:
            if self.history.all(lambda s: s.up):
                pitch = pitch + DTHETA * 2
            else:
                pitch = pitch + DTHETA
        elif status.down:
            if self.history.all(lambda s: s.down):
                pitch = pitch - DTHETA * 2
            else:
                pitch = pitch - DTHETA
        if status.right:
            if self.history.all(lambda s: s.right):
                roll = roll + DTHETA * 2
            else:
                roll = roll + DTHETA
        elif status.left:
            if self.history.all(lambda s: s.left):
                roll = roll - DTHETA * 2
            else:
                roll = roll - DTHETA
        diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        new_q = tf.transformations.quaternion_multiply(q, diff_q)
        new_pose.pose.orientation.x = new_q[0]
        new_pose.pose.orientation.y = new_q[1]
        new_pose.pose.orientation.z = new_q[2]
        new_pose.pose.orientation.w = new_q[3]
        return new_pose

    def run(self, status):
        if not self.initialized:
            # when not initialized, we will force to change planning_group
            while True:
                self.updatePlanningGroup(self.current_planning_group_index)
                planning_group = self.planning_groups_keys[
                    self.current_planning_group_index
                ]
                topics = self.planning_groups[planning_group]
                next_topic = topics[self.current_eef_index]
                if not self.waitForInitialPose(next_topic, timeout=3):
                    rospy.logwarn(
                        "Unable to initialize planning group "
                        + planning_group
                        + ". Trying different group."
                    )
                    rospy.logwarn(
                        "Is 'Allow External Comm.' enabled in Rviz? Is the 'Query Goal State' robot enabled?"
                    )
                else:
                    rospy.loginfo("Initialized planning group")
                    self.initialized = True
                    self.updatePoseTopic(self.current_eef_index)
                    return
                # Try to initialize with different planning group
                self.current_planning_group_index += 1
                if self.current_planning_group_index >= len(self.planning_groups_keys):
                    self.current_planning_group_index = 0  # reset loop
        if self.history.new(status, "select"):  # increment planning group
            self.updatePlanningGroup(self.current_planning_group_index + 1)
            self.current_eef_index = 0  # force to reset
            self.updatePoseTopic(self.current_eef_index)
            return
        elif self.history.new(status, "start"):  # decrement planning group
            self.updatePlanningGroup(self.current_planning_group_index - 1)
            self.current_eef_index = 0  # force to reset
            self.updatePoseTopic(self.current_eef_index)
            return
        elif self.history.new(status, "triangle"):
            self.updatePoseTopic(self.current_eef_index + 1)
            return
        elif self.history.new(status, "cross"):
            self.updatePoseTopic(self.current_eef_index - 1)
            return
        elif self.history.new(status, "square"):  # plan
            rospy.loginfo("Plan")
            self.plan_pub.publish(Empty())
            return
        elif self.history.new(status, "circle"):  # execute
            rospy.loginfo("Execute")
            self.execute_pub.publish(Empty())
            return
        self.marker_lock.acquire()
        pre_pose = self.pre_pose
        new_pose = self.computePoseFromJoy(pre_pose, status)
        now = rospy.Time.from_sec(time.time())
        # placement.time_from_start = now - self.prev_time
        if (now - self.prev_time).to_sec() > 1 / 30.0:
            # rospy.loginfo(new_pose)
            self.pose_pub.publish(new_pose)
            self.joy_pose_pub.publish(new_pose)
            self.prev_time = now
        # sync start state to the real robot state
        self.counter = self.counter + 1
        if self.counter % 10:
            self.update_start_state_pub.publish(Empty())
        self.pre_pose = new_pose
        self.marker_lock.release()
        # update self.initial_poses
        self.marker_lock.acquire()
        self.initial_poses[self.current_pose_topic.split("/")[-1]] = new_pose.pose
        self.marker_lock.release()
