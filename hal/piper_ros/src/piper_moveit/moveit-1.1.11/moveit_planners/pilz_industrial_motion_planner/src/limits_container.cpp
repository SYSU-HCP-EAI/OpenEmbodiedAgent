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

#include "pilz_industrial_motion_planner/limits_container.h"

pilz_industrial_motion_planner::LimitsContainer::LimitsContainer()
  : has_joint_limits_(false), has_cartesian_limits_(false)
{
}

bool pilz_industrial_motion_planner::LimitsContainer::hasJointLimits() const
{
  return has_joint_limits_;
}

void pilz_industrial_motion_planner::LimitsContainer::setJointLimits(
    pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  has_joint_limits_ = true;
  joint_limits_ = joint_limits;
}

const pilz_industrial_motion_planner::JointLimitsContainer&
pilz_industrial_motion_planner::LimitsContainer::getJointLimitContainer() const
{
  return joint_limits_;
}

bool pilz_industrial_motion_planner::LimitsContainer::hasFullCartesianLimits() const
{
  return (has_cartesian_limits_ && cartesian_limit_.hasMaxTranslationalVelocity() &&
          cartesian_limit_.hasMaxTranslationalAcceleration() && cartesian_limit_.hasMaxTranslationalDeceleration() &&
          cartesian_limit_.hasMaxRotationalVelocity());
}

void pilz_industrial_motion_planner::LimitsContainer::setCartesianLimits(
    pilz_industrial_motion_planner::CartesianLimit& cartesian_limit)
{
  has_cartesian_limits_ = true;
  cartesian_limit_ = cartesian_limit;
}

const pilz_industrial_motion_planner::CartesianLimit&
pilz_industrial_motion_planner::LimitsContainer::getCartesianLimits() const
{
  return cartesian_limit_;
}
