/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/robot_model/prismatic_joint_model.h>
#include <limits>

namespace moveit
{
namespace core
{
PrismaticJointModel::PrismaticJointModel(const std::string& name) : JointModel(name), axis_(0.0, 0.0, 0.0)
{
  type_ = PRISMATIC;
  variable_names_.push_back(name_);
  variable_bounds_.resize(1);
  variable_bounds_[0].position_bounded_ = true;
  variable_bounds_[0].min_position_ = -std::numeric_limits<double>::max();
  variable_bounds_[0].max_position_ = std::numeric_limits<double>::max();
  variable_index_map_[name_] = 0;
  computeVariableBoundsMsg();
}

unsigned int PrismaticJointModel::getStateSpaceDimension() const
{
  return 1;
}

double PrismaticJointModel::getMaximumExtent(const Bounds& other_bounds) const
{
  return variable_bounds_[0].max_position_ - other_bounds[0].min_position_;
}

void PrismaticJointModel::getVariableDefaultPositions(double* values, const Bounds& bounds) const
{
  // if zero is a valid value
  if (bounds[0].min_position_ <= 0.0 && bounds[0].max_position_ >= 0.0)
    values[0] = 0.0;
  else
    values[0] = (bounds[0].min_position_ + bounds[0].max_position_) / 2.0;
}

bool PrismaticJointModel::satisfiesPositionBounds(const double* values, const Bounds& bounds, double margin) const
{
  return !(values[0] < bounds[0].min_position_ - margin || values[0] > bounds[0].max_position_ + margin);
}

void PrismaticJointModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                                     const Bounds& bounds) const
{
  values[0] = rng.uniformReal(bounds[0].min_position_, bounds[0].max_position_);
}

void PrismaticJointModel::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                           const Bounds& bounds, const double* seed,
                                                           const double distance) const
{
  values[0] = rng.uniformReal(std::max(bounds[0].min_position_, seed[0] - distance),
                              std::min(bounds[0].max_position_, seed[0] + distance));
}

bool PrismaticJointModel::enforcePositionBounds(double* values, const Bounds& bounds) const
{
  if (values[0] < bounds[0].min_position_)
  {
    values[0] = bounds[0].min_position_;
    return true;
  }
  else if (values[0] > bounds[0].max_position_)
  {
    values[0] = bounds[0].max_position_;
    return true;
  }
  return false;
}

double PrismaticJointModel::distance(const double* values1, const double* values2) const
{
  return fabs(values1[0] - values2[0]);
}

void PrismaticJointModel::interpolate(const double* from, const double* to, const double t, double* state) const
{
  state[0] = from[0] + (to[0] - from[0]) * t;
}

void PrismaticJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const
{
  double* d = transf.data();
  d[0] = 1.0;
  d[1] = 0.0;
  d[2] = 0.0;
  d[3] = 0.0;

  d[4] = 0.0;
  d[5] = 1.0;
  d[6] = 0.0;
  d[7] = 0.0;

  d[8] = 0.0;
  d[9] = 0.0;
  d[10] = 1.0;
  d[11] = 0.0;

  d[12] = axis_.x() * joint_values[0];
  d[13] = axis_.y() * joint_values[0];
  d[14] = axis_.z() * joint_values[0];
  d[15] = 1.0;

  //  transf.setIdentity();
  //  transf.translation() = Eigen::Vector3d(axis_ * joint_values[0]);
}

void PrismaticJointModel::computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const
{
  joint_values[0] = transf.translation().dot(axis_);
}

}  // end of namespace core
}  // end of namespace moveit
