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

#include "ros/ros.h"

#include "pilz_industrial_motion_planner/cartesian_limits_aggregator.h"

static const std::string PARAM_CARTESIAN_LIMITS_NS = "cartesian_limits";

static const std::string PARAM_MAX_TRANS_VEL = "max_trans_vel";
static const std::string PARAM_MAX_TRANS_ACC = "max_trans_acc";
static const std::string PARAM_MAX_TRANS_DEC = "max_trans_dec";
static const std::string PARAM_MAX_ROT_VEL = "max_rot_vel";
static const std::string PARAM_MAX_ROT_ACC = "max_rot_acc";
static const std::string PARAM_MAX_ROT_DEC = "max_rot_dec";

pilz_industrial_motion_planner::CartesianLimit
pilz_industrial_motion_planner::CartesianLimitsAggregator::getAggregatedLimits(const ros::NodeHandle& nh)
{
  std::string param_prefix = PARAM_CARTESIAN_LIMITS_NS + "/";

  pilz_industrial_motion_planner::CartesianLimit cartesian_limit;

  // translational velocity
  double max_trans_vel;
  if (nh.getParam(param_prefix + PARAM_MAX_TRANS_VEL, max_trans_vel))
  {
    cartesian_limit.setMaxTranslationalVelocity(max_trans_vel);
  }

  // translational acceleration
  double max_trans_acc;
  if (nh.getParam(param_prefix + PARAM_MAX_TRANS_ACC, max_trans_acc))
  {
    cartesian_limit.setMaxTranslationalAcceleration(max_trans_acc);
  }

  // translational deceleration
  double max_trans_dec;
  if (nh.getParam(param_prefix + PARAM_MAX_TRANS_DEC, max_trans_dec))
  {
    cartesian_limit.setMaxTranslationalDeceleration(max_trans_dec);
  }

  // rotational velocity
  double max_rot_vel;
  if (nh.getParam(param_prefix + PARAM_MAX_ROT_VEL, max_rot_vel))
  {
    cartesian_limit.setMaxRotationalVelocity(max_rot_vel);
  }

  // rotational acceleration + deceleration deprecated
  // LCOV_EXCL_START
  if (nh.hasParam(param_prefix + PARAM_MAX_ROT_ACC) || nh.hasParam(param_prefix + PARAM_MAX_ROT_DEC))
  {
    ROS_WARN_STREAM("Ignoring cartesian limits parameters for rotational "
                    "acceleration / deceleration;"
                    << "these parameters are deprecated and are automatically "
                       "calculated from"
                    << "translational to rotational ratio.");
  }
  // LCOV_EXCL_STOP

  return cartesian_limit;
}
