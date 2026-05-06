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

#include "pilz_industrial_motion_planner_testutils/jointconfiguration.h"

#include <moveit/kinematic_constraints/utils.h>

namespace pilz_industrial_motion_planner_testutils
{
JointConfiguration::JointConfiguration() : RobotConfiguration()
{
}

JointConfiguration::JointConfiguration(const std::string& group_name, const std::vector<double>& config,
                                       CreateJointNameFunc&& create_joint_name_func)
  : RobotConfiguration(group_name), joints_(config), create_joint_name_func_(create_joint_name_func)
{
}

JointConfiguration::JointConfiguration(const std::string& group_name, const std::vector<double>& config,
                                       const moveit::core::RobotModelConstPtr& robot_model)
  : RobotConfiguration(group_name, robot_model), joints_(config)
{
}

moveit_msgs::Constraints JointConfiguration::toGoalConstraintsWithoutModel() const
{
  if (!create_joint_name_func_)
  {
    throw JointConfigurationException("Create-Joint-Name function not set");
  }

  moveit_msgs::Constraints gc;

  for (size_t i = 0; i < joints_.size(); ++i)
  {
    moveit_msgs::JointConstraint jc;
    jc.joint_name = create_joint_name_func_(i);
    jc.position = joints_.at(i);
    gc.joint_constraints.push_back(jc);
  }

  return gc;
}

moveit_msgs::Constraints JointConfiguration::toGoalConstraintsWithModel() const
{
  if (!robot_model_)
  {
    throw JointConfigurationException("No robot model set");
  }

  robot_state::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(group_name_, joints_);

  return kinematic_constraints::constructGoalConstraints(state, state.getRobotModel()->getJointModelGroup(group_name_));
}

moveit_msgs::RobotState JointConfiguration::toMoveitMsgsRobotStateWithoutModel() const
{
  if (!create_joint_name_func_)
  {
    throw JointConfigurationException("Create-Joint-Name function not set");
  }

  moveit_msgs::RobotState robot_state;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    robot_state.joint_state.name.emplace_back(create_joint_name_func_(i));
    robot_state.joint_state.position.push_back(joints_.at(i));
  }
  return robot_state;
}

robot_state::RobotState JointConfiguration::toRobotState() const
{
  if (!robot_model_)
  {
    throw JointConfigurationException("No robot model set");
  }

  robot_state::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.setJointGroupPositions(group_name_, joints_);
  return robot_state;
}

moveit_msgs::RobotState JointConfiguration::toMoveitMsgsRobotStateWithModel() const
{
  robot_state::RobotState start_state(toRobotState());
  moveit_msgs::RobotState rob_state_msg;
  moveit::core::robotStateToRobotStateMsg(start_state, rob_state_msg, false);
  return rob_state_msg;
}

sensor_msgs::JointState JointConfiguration::toSensorMsg() const
{
  if (!create_joint_name_func_)
  {
    throw JointConfigurationException("Create-Joint-Name function not set");
  }

  sensor_msgs::JointState state;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    state.name.emplace_back(create_joint_name_func_(i));
    state.position.push_back(joints_.at(i));
  }
  return state;
}

std::ostream& operator<<(std::ostream& os, const JointConfiguration& obj)
{
  const size_t n{ obj.size() };
  os << "JointConfiguration: [";
  for (size_t i = 0; i < n; ++i)
  {
    os << obj.getJoint(i);
    if (i != n - 1)
    {
      os << ", ";
    }
  }
  os << "]";

  return os;
}

}  // namespace pilz_industrial_motion_planner_testutils
