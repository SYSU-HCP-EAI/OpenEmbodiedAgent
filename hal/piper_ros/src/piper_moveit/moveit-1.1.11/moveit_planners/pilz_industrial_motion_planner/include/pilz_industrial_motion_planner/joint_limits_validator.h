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

#pragma once

#include "pilz_industrial_motion_planner/joint_limits_container.h"
#include "pilz_industrial_motion_planner/joint_limits_extension.h"

namespace pilz_industrial_motion_planner
{
/**
 * @brief Validates the equality of all limits inside a container
 */
class JointLimitsValidator
{
public:
  /**
   * @brief Validates that the position limits of all limits are equal
   * @param joint_limits the joint limits
   * @return true if all are equal
   * @note always returns true if has_position_limits=false for all limits, or
   * if the size of joint_limits is 0 or 1
   */
  static bool validateAllPositionLimitsEqual(const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

  /**
   * @brief Validates that the velocity of all limits is equal
   * @param joint_limits the joint limits
   * @return true if all are equal
   * @note always returns true if has_velocity_limits=false for all limits, or
   * if the size of joint_limits is 0 or 1
   */
  static bool validateAllVelocityLimitsEqual(const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

  /**
   * @brief Validates that the acceleration of all limits is equal
   * @param joint_limits the joint limits
   * @return true if all are equal
   * @note always returns true if has_acceleration_limits=false for all limits,
   * or if size of joint_limits is 0 or 1
   */
  static bool
  validateAllAccelerationLimitsEqual(const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

  /**
   * @brief Validates that the deceleration of all limits is equal
   * @param joint_limits the joint limits
   * @return true if all are equal
   * @note always returns true if has_acceleration_limits=false for all limits,
   * or if size of joint_limits is 0 or 1
   */
  static bool
  validateAllDecelerationLimitsEqual(const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

private:
  static bool validateWithEqualFunc(bool (*eq_func)(const JointLimit&, const JointLimit&),
                                    const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

  static bool positionEqual(const JointLimit& lhs, const JointLimit& rhs);

  static bool velocityEqual(const JointLimit& lhs, const JointLimit& rhs);

  static bool accelerationEqual(const JointLimit& lhs, const JointLimit& rhs);

  static bool decelerationEqual(const JointLimit& lhs, const JointLimit& rhs);
};

/**
 * @class ValidationException
 * @brief A base class for all validations exceptions inheriting from
 * std::runtime_exception
 */
class ValidationException : public std::runtime_error
{
public:
  ValidationException(const std::string& error_desc) : std::runtime_error(error_desc)
  {
  }
};

/**
 * @class ValidationJointMissingException
 * @brief Thrown the limits for a joint are defined in the urdf but not on the
 * parameter server (loaded from yaml)
 *
 */
class ValidationJointMissingException : public ValidationException
{
public:
  ValidationJointMissingException(const std::string& error_desc) : ValidationException(error_desc)
  {
  }
};

/**
 * @class ValidationDifferentLimitsException
 * @brief Thrown when the limits differ
 *
 */
class ValidationDifferentLimitsException : public ValidationException
{
public:
  ValidationDifferentLimitsException(const std::string& error_desc) : ValidationException(error_desc)
  {
  }
};

/**
 * @class ValidationBoundsViolationException
 * @brief Thrown when the limits from the param server are weaker than the ones
 * obtained from the urdf
 *
 */
class ValidationBoundsViolationException : public ValidationException
{
public:
  ValidationBoundsViolationException(const std::string& error_desc) : ValidationException(error_desc)
  {
  }
};

}  // namespace pilz_industrial_motion_planner
