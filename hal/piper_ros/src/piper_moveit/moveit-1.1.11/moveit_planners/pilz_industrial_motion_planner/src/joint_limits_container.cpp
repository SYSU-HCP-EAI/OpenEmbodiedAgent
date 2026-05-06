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

#include "pilz_industrial_motion_planner/joint_limits_container.h"

#include "ros/ros.h"
#include <stdexcept>

namespace pilz_industrial_motion_planner
{
bool JointLimitsContainer::addLimit(const std::string& joint_name, JointLimit joint_limit)
{
  if (joint_limit.has_deceleration_limits && joint_limit.max_deceleration >= 0)
  {
    ROS_ERROR_STREAM("joint_limit.max_deceleration MUST be negative!");
    return false;
  }
  const auto& insertion_result{ container_.insert(std::pair<std::string, JointLimit>(joint_name, joint_limit)) };
  if (!insertion_result.second)
  {
    ROS_ERROR_STREAM("joint_limit for joint " << joint_name << " already contained.");
    return false;
  }
  return true;
}

bool JointLimitsContainer::hasLimit(const std::string& joint_name) const
{
  return container_.find(joint_name) != container_.end();
}

size_t JointLimitsContainer::getCount() const
{
  return container_.size();
}

bool JointLimitsContainer::empty() const
{
  return container_.empty();
}

JointLimit JointLimitsContainer::getCommonLimit() const
{
  JointLimit common_limit;
  for (const auto& limit : container_)
  {
    updateCommonLimit(limit.second, common_limit);
  }
  return common_limit;
}

JointLimit JointLimitsContainer::getCommonLimit(const std::vector<std::string>& joint_names) const
{
  JointLimit common_limit;
  for (const auto& joint_name : joint_names)
  {
    updateCommonLimit(container_.at(joint_name), common_limit);
  }
  return common_limit;
}

JointLimit JointLimitsContainer::getLimit(const std::string& joint_name) const
{
  return container_.at(joint_name);
}

std::map<std::string, JointLimit>::const_iterator JointLimitsContainer::begin() const
{
  return container_.begin();
}

std::map<std::string, JointLimit>::const_iterator JointLimitsContainer::end() const
{
  return container_.end();
}

bool JointLimitsContainer::verifyVelocityLimit(const std::string& joint_name, const double& joint_velocity) const
{
  return (!(hasLimit(joint_name) && getLimit(joint_name).has_velocity_limits &&
            fabs(joint_velocity) > getLimit(joint_name).max_velocity));
}

bool JointLimitsContainer::verifyPositionLimit(const std::string& joint_name, const double& joint_position) const
{
  return (!(hasLimit(joint_name) && getLimit(joint_name).has_position_limits &&
            (joint_position < getLimit(joint_name).min_position || joint_position > getLimit(joint_name).max_position)));
}

void JointLimitsContainer::updateCommonLimit(const JointLimit& joint_limit, JointLimit& common_limit)
{
  // check position limits
  if (joint_limit.has_position_limits)
  {
    double min_position = joint_limit.min_position;
    double max_position = joint_limit.max_position;

    common_limit.min_position =
        (!common_limit.has_position_limits) ? min_position : std::max(common_limit.min_position, min_position);
    common_limit.max_position =
        (!common_limit.has_position_limits) ? max_position : std::min(common_limit.max_position, max_position);
    common_limit.has_position_limits = true;
  }

  // check velocity limits
  if (joint_limit.has_velocity_limits)
  {
    double max_velocity = joint_limit.max_velocity;
    common_limit.max_velocity =
        (!common_limit.has_velocity_limits) ? max_velocity : std::min(common_limit.max_velocity, max_velocity);
    common_limit.has_velocity_limits = true;
  }

  // check acceleration limits
  if (joint_limit.has_acceleration_limits)
  {
    double max_acc = joint_limit.max_acceleration;
    common_limit.max_acceleration =
        (!common_limit.has_acceleration_limits) ? max_acc : std::min(common_limit.max_acceleration, max_acc);
    common_limit.has_acceleration_limits = true;
  }

  // check deceleration limits
  if (joint_limit.has_deceleration_limits)
  {
    double max_dec = joint_limit.max_deceleration;
    common_limit.max_deceleration =
        (!common_limit.has_deceleration_limits) ? max_dec : std::max(common_limit.max_deceleration, max_dec);
    common_limit.has_deceleration_limits = true;
  }
}

}  // namespace pilz_industrial_motion_planner
