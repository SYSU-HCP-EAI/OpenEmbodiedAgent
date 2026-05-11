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

#include "pilz_industrial_motion_planner_testutils/robotconfiguration.h"

#include <stdexcept>

namespace pilz_industrial_motion_planner_testutils
{
RobotConfiguration::RobotConfiguration() : RobotStateMsgConvertible(), GoalConstraintMsgConvertible()
{
}

RobotConfiguration::RobotConfiguration(const std::string& group_name)
  : RobotStateMsgConvertible(), GoalConstraintMsgConvertible(), group_name_(group_name)
{
}

RobotConfiguration::RobotConfiguration(const std::string& group_name,
                                       const moveit::core::RobotModelConstPtr& robot_model)
  : RobotStateMsgConvertible(), GoalConstraintMsgConvertible(), group_name_(group_name), robot_model_(robot_model)
{
  if (robot_model && (!robot_model_->hasJointModelGroup(group_name_)))
  {
    std::string msg{ "Specified robot model does not contain specified group \"" };
    msg.append(group_name).append("\"");
    throw std::invalid_argument(msg);
  }
}

}  // namespace pilz_industrial_motion_planner_testutils
