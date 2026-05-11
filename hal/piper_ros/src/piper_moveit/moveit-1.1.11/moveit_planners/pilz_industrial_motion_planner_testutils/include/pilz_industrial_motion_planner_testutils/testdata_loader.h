/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef TESTDATA_LOADER_H
#define TESTDATA_LOADER_H

#include <string>
#include <utility>

#include <moveit/robot_model/robot_model.h>

#include "jointconfiguration.h"
#include "cartesianconfiguration.h"
#include "command_types_typedef.h"
#include "sequence.h"
#include "gripper.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Abstract base class describing the interface to access
 * test data like robot poses and robot commands.
 */
class TestdataLoader
{
public:
  TestdataLoader() = default;

  TestdataLoader(moveit::core::RobotModelConstPtr robot_model) : robot_model_(std::move(robot_model))
  {
  }

  virtual ~TestdataLoader() = default;

public:
  void setRobotModel(moveit::core::RobotModelConstPtr robot_model);

  virtual JointConfiguration getJoints(const std::string& pos_name, const std::string& group_name) const = 0;

  virtual CartesianConfiguration getPose(const std::string& pos_name, const std::string& group_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual PtpJoint getPtpJoint(const std::string& cmd_name) const = 0;
  virtual PtpCart getPtpCart(const std::string& cmd_name) const = 0;
  virtual PtpJointCart getPtpJointCart(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual LinJoint getLinJoint(const std::string& cmd_name) const = 0;
  virtual LinCart getLinCart(const std::string& cmd_name) const = 0;
  virtual LinJointCart getLinJointCart(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual CircCenterCart getCircCartCenterCart(const std::string& cmd_name) const = 0;
  virtual CircJointCenterCart getCircJointCenterCart(const std::string& cmd_name) const = 0;
  virtual CircInterimCart getCircCartInterimCart(const std::string& cmd_name) const = 0;
  virtual CircJointInterimCart getCircJointInterimCart(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual Sequence getSequence(const std::string& cmd_name) const = 0;

  /**
   * @brief Returns the command with the specified name from the test data.
   */
  virtual Gripper getGripper(const std::string& cmd_name) const = 0;

protected:
  moveit::core::RobotModelConstPtr robot_model_;
};

inline void TestdataLoader::setRobotModel(moveit::core::RobotModelConstPtr robot_model)
{
  robot_model_ = std::move(robot_model);
}

using TestdataLoaderUPtr = std::unique_ptr<TestdataLoader>;
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // TESTDATA_LOADER_H
