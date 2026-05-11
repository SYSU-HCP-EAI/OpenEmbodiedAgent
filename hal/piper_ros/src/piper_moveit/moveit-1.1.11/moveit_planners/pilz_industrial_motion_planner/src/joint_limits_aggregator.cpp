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

#include "pilz_industrial_motion_planner/joint_limits_aggregator.h"
#include "pilz_industrial_motion_planner/joint_limits_interface_extension.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <vector>

pilz_industrial_motion_planner::JointLimitsContainer
pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
    const ros::NodeHandle& nh, const std::vector<const moveit::core::JointModel*>& joint_models)
{
  JointLimitsContainer container;

  ROS_INFO_STREAM("Reading limits from namespace " << nh.getNamespace());

  // Iterate over all joint models and generate the map
  for (auto joint_model : joint_models)
  {
    JointLimit joint_limit;

    // If there is something defined for the joint on the parameter server
    if (joint_limits_interface::getJointLimits(joint_model->getName(), nh, joint_limit))
    {
      if (joint_limit.has_position_limits)
      {
        checkPositionBoundsThrowing(joint_model, joint_limit);
      }
      else
      {
        updatePositionLimitFromJointModel(joint_model, joint_limit);
      }

      if (joint_limit.has_velocity_limits)
      {
        checkVelocityBoundsThrowing(joint_model, joint_limit);
      }
      else
      {
        updateVelocityLimitFromJointModel(joint_model, joint_limit);
      }
    }
    else
    {
      // If there is nothing defined for this joint on the parameter server just
      // update the values by the values of
      // the urdf

      updatePositionLimitFromJointModel(joint_model, joint_limit);
      updateVelocityLimitFromJointModel(joint_model, joint_limit);
    }

    // Update max_deceleration if no max_acceleration has been set
    if (joint_limit.has_acceleration_limits && !joint_limit.has_deceleration_limits)
    {
      joint_limit.max_deceleration = -joint_limit.max_acceleration;
      joint_limit.has_deceleration_limits = true;
    }

    // Insert the joint limit into the map
    container.addLimit(joint_model->getName(), joint_limit);
  }

  return container;
}

void pilz_industrial_motion_planner::JointLimitsAggregator::updatePositionLimitFromJointModel(
    const moveit::core::JointModel* joint_model, JointLimit& joint_limit)
{
  switch (joint_model->getVariableBounds().size())
  {
    // LCOV_EXCL_START
    case 0:
      ROS_WARN_STREAM("no bounds set for joint " << joint_model->getName());
      break;
    // LCOV_EXCL_STOP
    case 1:
      joint_limit.has_position_limits = joint_model->getVariableBounds()[0].position_bounded_;
      joint_limit.min_position = joint_model->getVariableBounds()[0].min_position_;
      joint_limit.max_position = joint_model->getVariableBounds()[0].max_position_;
      break;
    // LCOV_EXCL_START
    default:
      ROS_WARN_STREAM("Multi-DOF-Joint '" << joint_model->getName() << "' not supported.");
      joint_limit.has_position_limits = true;
      joint_limit.min_position = 0;
      joint_limit.max_position = 0;
      break;
      // LCOV_EXCL_STOP
  }

  ROS_DEBUG_STREAM("Limit(" << joint_model->getName() << " min:" << joint_limit.min_position
                            << " max:" << joint_limit.max_position);
}

void pilz_industrial_motion_planner::JointLimitsAggregator::updateVelocityLimitFromJointModel(
    const moveit::core::JointModel* joint_model, JointLimit& joint_limit)
{
  switch (joint_model->getVariableBounds().size())
  {
    // LCOV_EXCL_START
    case 0:
      ROS_WARN_STREAM("no bounds set for joint " << joint_model->getName());
      break;
    // LCOV_EXCL_STOP
    case 1:
      joint_limit.has_velocity_limits = joint_model->getVariableBounds()[0].velocity_bounded_;
      joint_limit.max_velocity = joint_model->getVariableBounds()[0].max_velocity_;
      break;
    // LCOV_EXCL_START
    default:
      ROS_WARN_STREAM("Multi-DOF-Joint '" << joint_model->getName() << "' not supported.");
      joint_limit.has_velocity_limits = true;
      joint_limit.max_velocity = 0;
      break;
      // LCOV_EXCL_STOP
  }
}

void pilz_industrial_motion_planner::JointLimitsAggregator::checkPositionBoundsThrowing(
    const moveit::core::JointModel* joint_model, const JointLimit& joint_limit)
{
  // Check min position
  if (!joint_model->satisfiesPositionBounds(&joint_limit.min_position))
  {
    throw AggregationBoundsViolationException("min_position of " + joint_model->getName() +
                                              " violates min limit from URDF");
  }

  // Check max position
  if (!joint_model->satisfiesPositionBounds(&joint_limit.max_position))
  {
    throw AggregationBoundsViolationException("max_position of " + joint_model->getName() +
                                              " violates max limit from URDF");
  }
}

void pilz_industrial_motion_planner::JointLimitsAggregator::checkVelocityBoundsThrowing(
    const moveit::core::JointModel* joint_model, const JointLimit& joint_limit)
{
  // Check min position
  if (!joint_model->satisfiesVelocityBounds(&joint_limit.max_velocity))
  {
    throw AggregationBoundsViolationException("max_velocity of " + joint_model->getName() +
                                              " violates velocity limit from URDF");
  }
}
