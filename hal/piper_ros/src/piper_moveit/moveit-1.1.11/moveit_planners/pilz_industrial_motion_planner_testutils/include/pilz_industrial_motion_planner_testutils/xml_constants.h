/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Pilz GmbH & Co. KG
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef XML_CONSTANTS_H
#define XML_CONSTANTS_H

#include <string>

namespace pilz_industrial_motion_planner_testutils
{
const std::string EMPTY_STR{};

const std::string XML_ATTR_STR{ "<xmlattr>" };
const std::string JOINT_STR{ "joints" };
const std::string POSE_STR{ "pos" };
const std::string XYZ_QUAT_STR{ "xyzQuat" };
const std::string XYZ_EULER_STR{ "xyzEuler" };
const std::string SEED_STR{ "seed" };

const std::string PTP_STR{ "ptp" };
const std::string LIN_STR{ "lin" };
const std::string CIRC_STR{ "circ" };
const std::string BLEND_STR{ "blend" };
const std::string GRIPPER_STR{ "gripper" };

const std::string PLANNING_GROUP_STR{ "planningGroup" };
const std::string TARGET_LINK_STR{ "targetLink" };
const std::string START_POS_STR{ "startPos" };
const std::string END_POS_STR{ "endPos" };
const std::string INTERMEDIATE_POS_STR{ "intermediatePos" };
const std::string CENTER_POS_STR{ "centerPos" };
const std::string VEL_STR{ "vel" };
const std::string ACC_STR{ "acc" };

const std::string POSES_PATH_STR{ "testdata.poses" };
const std::string PTPS_PATH_STR{ "testdata." + PTP_STR + "s" };
const std::string LINS_PATH_STR{ "testdata." + LIN_STR + "s" };
const std::string CIRCS_PATH_STR{ "testdata." + CIRC_STR + "s" };
const std::string SEQUENCE_PATH_STR{ "testdata.sequences" };
const std::string GRIPPERS_PATH_STR{ "testdata." + GRIPPER_STR + "s" };

const std::string NAME_PATH_STR{ XML_ATTR_STR + ".name" };
const std::string CMD_TYPE_PATH_STR{ XML_ATTR_STR + ".type" };
const std::string BLEND_RADIUS_PATH_STR{ XML_ATTR_STR + ".blend_radius" };
const std::string LINK_NAME_PATH_STR{ XML_ATTR_STR + ".link_name" };
const std::string GROUP_NAME_PATH_STR{ XML_ATTR_STR + ".group_name" };
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // XML_CONSTANTS_H
