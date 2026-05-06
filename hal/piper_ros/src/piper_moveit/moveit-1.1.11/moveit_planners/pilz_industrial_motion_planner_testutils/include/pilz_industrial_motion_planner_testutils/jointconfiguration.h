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

#ifndef JOINTCONFIGURATION_H
#define JOINTCONFIGURATION_H

#include <string>
#include <utility>
#include <vector>
#include <functional>
#include <stdexcept>

#include <sensor_msgs/JointState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "robotconfiguration.h"

namespace pilz_industrial_motion_planner_testutils
{
class JointConfigurationException : public std::runtime_error
{
public:
  JointConfigurationException(const std::string& error_desc) : std::runtime_error(error_desc)
  {
  }
};

using CreateJointNameFunc = std::function<std::string(const size_t&)>;

/**
 * @brief Class to define a robot configuration in space with the help
 * of joint values.
 */
class JointConfiguration : public RobotConfiguration
{
public:
  JointConfiguration();

  JointConfiguration(const std::string& group_name, const std::vector<double>& config,
                     CreateJointNameFunc&& create_joint_name_func);

  JointConfiguration(const std::string& group_name, const std::vector<double>& config,
                     const moveit::core::RobotModelConstPtr& robot_model);

public:
  void setJoint(const size_t index, const double value);
  double getJoint(const size_t index) const;
  const std::vector<double> getJoints() const;

  size_t size() const;

  moveit_msgs::Constraints toGoalConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

  sensor_msgs::JointState toSensorMsg() const;

  robot_state::RobotState toRobotState() const;

  void setCreateJointNameFunc(CreateJointNameFunc create_joint_name_func);

private:
  moveit_msgs::RobotState toMoveitMsgsRobotStateWithoutModel() const;
  moveit_msgs::RobotState toMoveitMsgsRobotStateWithModel() const;

  moveit_msgs::Constraints toGoalConstraintsWithoutModel() const;
  moveit_msgs::Constraints toGoalConstraintsWithModel() const;

private:
  //! Joint positions
  std::vector<double> joints_;

  CreateJointNameFunc create_joint_name_func_;
};

std::ostream& operator<<(std::ostream& /*os*/, const JointConfiguration& /*obj*/);

inline moveit_msgs::Constraints JointConfiguration::toGoalConstraints() const
{
  return robot_model_ ? toGoalConstraintsWithModel() : toGoalConstraintsWithoutModel();
}

inline moveit_msgs::RobotState JointConfiguration::toMoveitMsgsRobotState() const
{
  return robot_model_ ? toMoveitMsgsRobotStateWithModel() : toMoveitMsgsRobotStateWithoutModel();
}

inline void JointConfiguration::setJoint(const size_t index, const double value)
{
  joints_.at(index) = value;
}

inline double JointConfiguration::getJoint(const size_t index) const
{
  return joints_.at(index);
}

inline const std::vector<double> JointConfiguration::getJoints() const
{
  return joints_;
}

inline size_t JointConfiguration::size() const
{
  return joints_.size();
}

inline void JointConfiguration::setCreateJointNameFunc(CreateJointNameFunc create_joint_name_func)
{
  create_joint_name_func_ = std::move(create_joint_name_func);
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // JOINTCONFIGURATION_H
