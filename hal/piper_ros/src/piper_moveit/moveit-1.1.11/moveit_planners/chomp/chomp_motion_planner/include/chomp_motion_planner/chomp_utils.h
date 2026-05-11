/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Mrinal Kalakrishnan */

#pragma once

#include <iostream>
#include <Eigen/Core>
#include <moveit/planning_scene/planning_scene.h>

namespace chomp
{
static const int DIFF_RULE_LENGTH = 7;

// the differentiation rules (centered at the center)
static const double DIFF_RULES[3][DIFF_RULE_LENGTH] = {
  { 0, 0, -2 / 6.0, -3 / 6.0, 6 / 6.0, -1 / 6.0, 0 },                       // velocity
  { 0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0 },         // acceleration
  { 0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0 }  // jerk
};

static inline void robotStateToArray(const moveit::core::RobotState& state, const std::string& planning_group_name,
                                     Eigen::MatrixXd::RowXpr joint_array)
{
  const moveit::core::JointModelGroup* group = state.getJointModelGroup(planning_group_name);
  size_t joint_index = 0;
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
    joint_array[joint_index++] = state.getVariablePosition(jm->getFirstVariableIndex());
}

// copied from geometry/angles/angles.h
static inline double normalizeAnglePositive(double angle)
{
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

static inline double normalizeAngle(double angle)
{
  double a = normalizeAnglePositive(angle);
  if (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

static inline double shortestAngularDistance(double start, double end)
{
  double res = normalizeAnglePositive(normalizeAnglePositive(end) - normalizeAnglePositive(start));
  if (res > M_PI)
  {
    res = -(2.0 * M_PI - res);
  }
  return normalizeAngle(res);
}

}  // namespace chomp
