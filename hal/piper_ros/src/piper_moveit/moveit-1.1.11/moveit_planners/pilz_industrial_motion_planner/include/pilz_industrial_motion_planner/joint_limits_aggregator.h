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

#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/robot_model/joint_model.h>

#include <map>

namespace pilz_industrial_motion_planner
{
/**
 * @brief  Unifies the joint limits from the given joint models with joint
 * limits from the parameter server.
 *
 * Does not support MultiDOF joints.
 */
class JointLimitsAggregator
{
public:
  /**
   * @brief  Aggregates(combines) the joint limits from joint model and
   * parameter server.
   * The rules for the combination are:
   *   1. Position and velocity limits on the parameter server must be stricter
   * or equal if they are defined.
   *   2. Limits on the parameter server where the corresponding
   *      has_<position|velocity|acceleration|deceleration>_limits are „false“
   * are considered undefined(see point 1).
   *   3. Not all joints have to be limited by the parameter server. Selective
   * limitation is possible.
   *   4. If max_deceleration is unset, it will be set to: max_deceleration = -
   * max_acceleration.
   * @note The acceleration/deceleration can only be set via the parameter
   * server since they are not supported
   * in the urdf so far.
   * @param nh Node handle in whose namespace the joint limit parameters are
   * expected.
   * @param joint_models The joint models
   * @return Container containing the limits
   */
  static JointLimitsContainer getAggregatedLimits(const ros::NodeHandle& nh,
                                                  const std::vector<const moveit::core::JointModel*>& joint_models);

protected:
  /**
   * @brief Update the position limits with the ones from the joint_model.
   *
   * If the joint model has no position limit, the value is unchanged.
   *
   * @param joint_model The joint model
   * @param joint_limit The joint_limit to be filled with new values.
   */
  static void updatePositionLimitFromJointModel(const moveit::core::JointModel* joint_model, JointLimit& joint_limit);

  /**
   * @brief Update the velocity limit with the one from the joint_model.
   *
   * If the joint model has no velocity limit, the value is unchanged.
   *
   * @param joint_model The joint model
   * @param joint_limit The joint_limit to be filled with new values.
   */
  static void updateVelocityLimitFromJointModel(const moveit::core::JointModel* joint_model, JointLimit& joint_limit);

  /**
   * @brief Checks if the position limits from the given joint_limit are
   * stricter than the limits of the joint_model.
   * Throws AggregationBoundsViolationException on violation
   * @param joint_model The joint_model
   * @param joint_limit The joint_limit
   */
  static void checkPositionBoundsThrowing(const moveit::core::JointModel* joint_model, const JointLimit& joint_limit);

  /**
   * @brief Checks if the velocity limit from the given joint_limit are stricter
   * than the limit of the joint_model.
   * Throws AggregationBoundsViolationException on violation
   * @param joint_model The joint_model
   * @param joint_limit The joint_limit
   */
  static void checkVelocityBoundsThrowing(const moveit::core::JointModel* joint_model, const JointLimit& joint_limit);
};

/**
 * @class AggregationException
 * @brief A base class for all aggregation exceptions inheriting from
 * std::runtime_exception
 */
class AggregationException : public std::runtime_error
{
public:
  AggregationException(const std::string& error_desc) : std::runtime_error(error_desc)
  {
  }
};

/**
 * @class AggregationJointMissingException
 * @brief Thrown the limits from the parameter server are weaker(forbidden) than
 * the ones defined in the urdf
 *
 */
class AggregationBoundsViolationException : public AggregationException
{
public:
  AggregationBoundsViolationException(const std::string& error_desc) : AggregationException(error_desc)
  {
  }
};

}  // namespace pilz_industrial_motion_planner
