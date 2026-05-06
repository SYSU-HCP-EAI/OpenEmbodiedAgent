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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_industrial_motion_planner/joint_limits_extension.h"
#include "pilz_industrial_motion_planner/joint_limits_interface_extension.h"

#include "pilz_industrial_motion_planner/joint_limits_validator.h"

bool pilz_industrial_motion_planner::JointLimitsValidator::validateAllPositionLimitsEqual(
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  return validateWithEqualFunc(&pilz_industrial_motion_planner::JointLimitsValidator::positionEqual, joint_limits);
}

bool pilz_industrial_motion_planner::JointLimitsValidator::validateAllVelocityLimitsEqual(
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  return validateWithEqualFunc(&pilz_industrial_motion_planner::JointLimitsValidator::velocityEqual, joint_limits);
}

bool pilz_industrial_motion_planner::JointLimitsValidator::validateAllAccelerationLimitsEqual(
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  return validateWithEqualFunc(&pilz_industrial_motion_planner::JointLimitsValidator::accelerationEqual, joint_limits);
}

bool pilz_industrial_motion_planner::JointLimitsValidator::validateAllDecelerationLimitsEqual(
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  return validateWithEqualFunc(&pilz_industrial_motion_planner::JointLimitsValidator::decelerationEqual, joint_limits);
}

bool pilz_industrial_motion_planner::JointLimitsValidator::validateWithEqualFunc(
    bool (*eq_func)(const JointLimit&, const JointLimit&),
    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  // If there are no joint_limits it is considered equal
  if (joint_limits.empty())
  {
    return true;
  }

  JointLimit reference = joint_limits.begin()->second;

  for (auto it = std::next(joint_limits.begin()); it != joint_limits.end(); ++it)
  {
    if (!eq_func(reference, it->second))
    {
      return false;
    }
  }

  return true;
}

bool pilz_industrial_motion_planner::JointLimitsValidator::positionEqual(const JointLimit& lhs, const JointLimit& rhs)
{
  // Return false if .has_velocity_limits differs
  if (lhs.has_position_limits != rhs.has_position_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if (lhs.has_position_limits && ((lhs.max_position != rhs.max_position) || (lhs.min_position != rhs.min_position)))
  {
    return false;
  }
  return true;
}

bool pilz_industrial_motion_planner::JointLimitsValidator::velocityEqual(const JointLimit& lhs, const JointLimit& rhs)
{
  // Return false if .has_velocity_limits differs
  if (lhs.has_velocity_limits != rhs.has_velocity_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if (lhs.has_velocity_limits && (lhs.max_velocity != rhs.max_velocity))
  {
    return false;
  }

  return true;
}

bool pilz_industrial_motion_planner::JointLimitsValidator::accelerationEqual(const JointLimit& lhs,
                                                                             const JointLimit& rhs)
{
  // Return false if .has_acceleration_limits differs
  if (lhs.has_acceleration_limits != rhs.has_acceleration_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if (lhs.has_acceleration_limits && (lhs.max_acceleration != rhs.max_acceleration))
  {
    return false;
  }

  return true;
}

bool pilz_industrial_motion_planner::JointLimitsValidator::decelerationEqual(const JointLimit& lhs,
                                                                             const JointLimit& rhs)
{
  // Return false if .has_acceleration_limits differs
  if (lhs.has_deceleration_limits != rhs.has_deceleration_limits)
  {
    return false;
  }

  // If velocity limits are the same make sure they are equal
  if (lhs.has_deceleration_limits && (lhs.max_deceleration != rhs.max_deceleration))
  {
    return false;
  }

  return true;
}
