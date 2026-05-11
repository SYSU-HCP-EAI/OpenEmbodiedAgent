#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Houston Mechatronics, Inc.
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
# Author: Siddharth Srivatsa

import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Constraints
from geometry_msgs.msg import Pose

if __name__ == "__main__":

    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_py_demo", anonymous=True)

    robot = RobotCommander()
    rospy.sleep(1)

    a = robot.right_arm
    a.set_start_state(RobotState())

    print("current pose:")
    print(a.get_current_pose())
    c = Constraints()

    waypoints = []
    waypoints.append(a.get_current_pose().pose)

    # Move forward
    wpose = Pose()
    wpose.position.x = wpose.position.x + 0.1
    waypoints.append(wpose)

    # Move down
    wpose.position.z -= 0.10
    waypoints.append(wpose)

    # Move to the side
    wpose.position.y += 0.05
    waypoints.append(wpose)

    plan, fraction = a.compute_cartesian_path(waypoints, 0.01, 0.0, path_constraints=c)
    print("Plan success percent: ", fraction)
