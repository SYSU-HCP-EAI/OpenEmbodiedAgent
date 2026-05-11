#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2018 Pilz GmbH & Co. KG
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
#   * Neither the name of Pilz GmbH & Co. KG nor the names of its
#     contributors may be used to endorse or promote products derived
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

from geometry_msgs.msg import Point
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
import math
import rospy

__REQUIRED_API_VERSION__ = "1"

robot_configs = {}
robot_configs["prbt"] = {
    "initJointPose": [0.0, 0.0, math.radians(-25), 0.0, -1.57, 0],
    "L": 0.2,
    "M": 0.1,
    "planning_group": "manipulator",
    "target_link": "prbt_tcp",
    "reference_frame": "prbt_base",
    "default_or": from_euler(0, math.radians(180), math.radians(90)),
    "P1_position": Point(0.3, 0.0, 0.5),
    "P1_orientation": from_euler(0, math.radians(180), math.radians(135)),
}

robot_configs["panda"] = {
    "initJointPose": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "L": 0.2,
    "M": 0.1,
    "planning_group": "panda_arm",
    "target_link": "panda_link8",
    "reference_frame": "panda_link0",
    "default_or": Quaternion(0.924, -0.382, 0.000, 0.000),
    "P1_position": Point(0.2, 0.0, 0.8),
}


def start_program(robot_name):
    print("Executing " + __file__)

    test_sequence(**robot_configs[robot_name])


def test_sequence(
    initJointPose,
    L,
    M,
    planning_group,
    target_link,
    reference_frame,
    default_or,
    P1_position,
    P1_orientation,
):
    r = Robot(__REQUIRED_API_VERSION__)

    r.move(Ptp(goal=initJointPose, planning_group=planning_group))

    P1 = Pose(position=P1_position, orientation=P1_orientation)

    P2 = Pose(
        position=Point(P1.position.x + L, P1.position.y + L, P1.position.z - M),
        orientation=default_or,
    )
    P3 = Pose(
        position=Point(P1.position.x, P1.position.y + 2 * L, P1.position.z - 2 * M),
        orientation=default_or,
    )
    P4 = Pose(
        position=Point(P1.position.x, P1.position.y + L, P1.position.z - M),
        orientation=default_or,
    )

    ptp1 = Ptp(
        goal=P1,
        acc_scale=0.3,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    ptp2 = Ptp(
        goal=P2,
        acc_scale=0.3,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    ptp3 = Ptp(
        goal=P3,
        acc_scale=0.3,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    ptp4 = Ptp(
        goal=P4,
        acc_scale=0.3,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    lin1 = Lin(
        goal=P1,
        acc_scale=0.1,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    lin2 = Lin(
        goal=P2,
        acc_scale=0.1,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    lin3 = Lin(
        goal=P3,
        acc_scale=0.1,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    lin4 = Lin(
        goal=P4,
        acc_scale=0.1,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )

    r.move(ptp1)  # PTP_12 test
    r.move(ptp2)  # PTP_23
    r.move(ptp3)  # PTP_34
    r.move(ptp4)  # PTP_41
    r.move(ptp1)

    r.move(lin1)  # LIN_12
    r.move(lin2)  # LIN_23
    r.move(lin3)  # LIN_34
    r.move(lin4)  # LIN_41
    r.move(lin1)

    circ3_interim_2 = Circ(
        goal=P3,
        interim=P2.position,
        acc_scale=0.2,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )
    circ1_center_2 = Circ(
        goal=P1,
        center=P2.position,
        acc_scale=0.2,
        planning_group=planning_group,
        target_link=target_link,
        reference_frame=reference_frame,
    )

    for radius in [0, 0.1]:
        r.move(Ptp(goal=initJointPose, planning_group=planning_group))

        seq = Sequence()
        seq.append(ptp1, blend_radius=radius)
        seq.append(circ3_interim_2, blend_radius=radius)
        seq.append(ptp2, blend_radius=radius)
        seq.append(lin3, blend_radius=radius)
        seq.append(circ1_center_2, blend_radius=radius)
        seq.append(lin2, blend_radius=radius)
        seq.append(ptp3)

        r.move(seq)


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node("robot_program_node")

    import sys

    robots = list(robot_configs.keys())

    if len(sys.argv) < 2:
        print(
            "Please specify the robot you want to use."
            + ", ".join('"{0}"'.format(r) for r in robots)
        )
        sys.exit()

    if sys.argv[1] not in robots:
        print(
            "Robot "
            + sys.argv[1]
            + " not available. Use one of "
            + ", ".join('"{0}"'.format(r) for r in robots)
        )
        sys.exit()

    start_program(sys.argv[1])
