/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2008-2013, Willow Garage, Inc.
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

#include <moveit/robot_model/floating_joint_model.h>
#include <geometric_shapes/check_isometry.h>
#include <boost/math/constants/constants.hpp>
#include <ros/console.h>
#include <limits>
#include <cmath>

namespace moveit
{
namespace core
{
namespace
{
constexpr size_t STATE_SPACE_DIMENSION = 7;

}  // namespace

FloatingJointModel::FloatingJointModel(const std::string& name) : JointModel(name), angular_distance_weight_(1.0)
{
  type_ = FLOATING;
  local_variable_names_.push_back("trans_x");
  local_variable_names_.push_back("trans_y");
  local_variable_names_.push_back("trans_z");
  local_variable_names_.push_back("rot_x");
  local_variable_names_.push_back("rot_y");
  local_variable_names_.push_back("rot_z");
  local_variable_names_.push_back("rot_w");
  for (size_t i = 0; i < STATE_SPACE_DIMENSION; ++i)
  {
    variable_names_.push_back(name_ + "/" + local_variable_names_[i]);
    variable_index_map_[variable_names_.back()] = i;
  }

  variable_bounds_.resize(7);

  variable_bounds_[0].position_bounded_ = true;
  variable_bounds_[1].position_bounded_ = true;
  variable_bounds_[2].position_bounded_ = true;
  variable_bounds_[3].position_bounded_ = true;
  variable_bounds_[4].position_bounded_ = true;
  variable_bounds_[5].position_bounded_ = true;
  variable_bounds_[6].position_bounded_ = true;

  variable_bounds_[0].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[0].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[1].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[1].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[2].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[2].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[3].min_position_ = -1.0;
  variable_bounds_[3].max_position_ = 1.0;
  variable_bounds_[4].min_position_ = -1.0;
  variable_bounds_[4].max_position_ = 1.0;
  variable_bounds_[5].min_position_ = -1.0;
  variable_bounds_[5].max_position_ = 1.0;
  variable_bounds_[6].min_position_ = -1.0;
  variable_bounds_[6].max_position_ = 1.0;

  computeVariableBoundsMsg();
}

double FloatingJointModel::getMaximumExtent(const Bounds& other_bounds) const
{
  double dx = other_bounds[0].max_position_ - other_bounds[0].min_position_;
  double dy = other_bounds[1].max_position_ - other_bounds[1].min_position_;
  double dz = other_bounds[2].max_position_ - other_bounds[2].min_position_;
  return sqrt(dx * dx + dy * dy + dz * dz) + boost::math::constants::pi<double>() * 0.5 * angular_distance_weight_;
}

double FloatingJointModel::distance(const double* values1, const double* values2) const
{
  return distanceTranslation(values1, values2) + angular_distance_weight_ * distanceRotation(values1, values2);
}

double FloatingJointModel::distanceTranslation(const double* values1, const double* values2) const
{
  double dx = values1[0] - values2[0];
  double dy = values1[1] - values2[1];
  double dz = values1[2] - values2[2];
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double FloatingJointModel::distanceRotation(const double* values1, const double* values2) const
{
  double dq =
      fabs(values1[3] * values2[3] + values1[4] * values2[4] + values1[5] * values2[5] + values1[6] * values2[6]);
  if (dq + std::numeric_limits<double>::epsilon() >= 1.0)
    return 0.0;
  else
    return acos(dq);
}

void FloatingJointModel::interpolate(const double* from, const double* to, const double t, double* state) const
{
  // interpolate position
  state[0] = from[0] + (to[0] - from[0]) * t;
  state[1] = from[1] + (to[1] - from[1]) * t;
  state[2] = from[2] + (to[2] - from[2]) * t;

  double dq = fabs(from[3] * to[3] + from[4] * to[4] + from[5] * to[5] + from[6] * to[6]);
  double theta = (dq + std::numeric_limits<double>::epsilon() >= 1.0) ? 0.0 : acos(dq);
  if (theta > std::numeric_limits<double>::epsilon())
  {
    double d = 1.0 / sin(theta);
    double s0 = sin((1.0 - t) * theta);
    double s1 = sin(t * theta);
    if (dq < 0)  // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
      s1 = -s1;
    state[3] = (from[3] * s0 + to[3] * s1) * d;
    state[4] = (from[4] * s0 + to[4] * s1) * d;
    state[5] = (from[5] * s0 + to[5] * s1) * d;
    state[6] = (from[6] * s0 + to[6] * s1) * d;
  }
  else
  {
    state[3] = from[3];
    state[4] = from[4];
    state[5] = from[5];
    state[6] = from[6];
  }
}

bool FloatingJointModel::satisfiesPositionBounds(const double* values, const Bounds& bounds, double margin) const
{
  if (values[0] < bounds[0].min_position_ - margin || values[0] > bounds[0].max_position_ + margin)
    return false;
  if (values[1] < bounds[1].min_position_ - margin || values[1] > bounds[1].max_position_ + margin)
    return false;
  if (values[2] < bounds[2].min_position_ - margin || values[2] > bounds[2].max_position_ + margin)
    return false;
  double norm_sqr = values[3] * values[3] + values[4] * values[4] + values[5] * values[5] + values[6] * values[6];
  return fabs(norm_sqr - 1.0) <= std::numeric_limits<float>::epsilon() * 10.0;
}

bool FloatingJointModel::normalizeRotation(double* values) const
{
  // normalize the quaternion if we need to
  double norm_sqr = values[3] * values[3] + values[4] * values[4] + values[5] * values[5] + values[6] * values[6];
  if (fabs(norm_sqr - 1.0) > std::numeric_limits<double>::epsilon() * 100.0)
  {
    double norm = sqrt(norm_sqr);
    if (norm < std::numeric_limits<double>::epsilon() * 100.0)
    {
      ROS_WARN_NAMED("robot_model", "Quaternion is zero in RobotState representation. Setting to identity");
      values[3] = 0.0;
      values[4] = 0.0;
      values[5] = 0.0;
      values[6] = 1.0;
    }
    else
    {
      values[3] /= norm;
      values[4] /= norm;
      values[5] /= norm;
      values[6] /= norm;
    }
    return true;
  }
  else
    return false;
}

unsigned int FloatingJointModel::getStateSpaceDimension() const
{
  return STATE_SPACE_DIMENSION;
}

bool FloatingJointModel::enforcePositionBounds(double* values, const Bounds& bounds) const
{
  bool result = normalizeRotation(values);
  for (unsigned int i = 0; i < 3; ++i)
  {
    if (values[i] < bounds[i].min_position_)
    {
      values[i] = bounds[i].min_position_;
      result = true;
    }
    else if (values[i] > bounds[i].max_position_)
    {
      values[i] = bounds[i].max_position_;
      result = true;
    }
  }
  return result;
}

void FloatingJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const
{
  transf = Eigen::Isometry3d(
      Eigen::Translation3d(joint_values[0], joint_values[1], joint_values[2]) *
      Eigen::Quaterniond(joint_values[6], joint_values[3], joint_values[4], joint_values[5]).normalized());
}

void FloatingJointModel::computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const
{
  joint_values[0] = transf.translation().x();
  joint_values[1] = transf.translation().y();
  joint_values[2] = transf.translation().z();
  ASSERT_ISOMETRY(transf)  // unsanitized input, could contain non-isometry
  Eigen::Quaterniond q(transf.linear());
  joint_values[3] = q.x();
  joint_values[4] = q.y();
  joint_values[5] = q.z();
  joint_values[6] = q.w();
}

void FloatingJointModel::getVariableDefaultPositions(double* values, const Bounds& bounds) const
{
  for (unsigned int i = 0; i < 3; ++i)
  {
    // if zero is a valid value
    if (bounds[i].min_position_ <= 0.0 && bounds[i].max_position_ >= 0.0)
      values[i] = 0.0;
    else
      values[i] = (bounds[i].min_position_ + bounds[i].max_position_) / 2.0;
  }

  values[3] = 0.0;
  values[4] = 0.0;
  values[5] = 0.0;
  values[6] = 1.0;
}

void FloatingJointModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                                    const Bounds& bounds) const
{
  if (bounds[0].max_position_ >= std::numeric_limits<double>::infinity() ||
      bounds[0].min_position_ <= -std::numeric_limits<double>::infinity())
    values[0] = 0.0;
  else
    values[0] = rng.uniformReal(bounds[0].min_position_, bounds[0].max_position_);
  if (bounds[1].max_position_ >= std::numeric_limits<double>::infinity() ||
      bounds[1].min_position_ <= -std::numeric_limits<double>::infinity())
    values[1] = 0.0;
  else
    values[1] = rng.uniformReal(bounds[1].min_position_, bounds[1].max_position_);
  if (bounds[2].max_position_ >= std::numeric_limits<double>::infinity() ||
      bounds[2].min_position_ <= -std::numeric_limits<double>::infinity())
    values[2] = 0.0;
  else
    values[2] = rng.uniformReal(bounds[2].min_position_, bounds[2].max_position_);

  double q[4];
  rng.quaternion(q);
  values[3] = q[0];
  values[4] = q[1];
  values[5] = q[2];
  values[6] = q[3];
}

void FloatingJointModel::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                          const Bounds& bounds, const double* seed,
                                                          const double distance) const
{
  if (bounds[0].max_position_ >= std::numeric_limits<double>::infinity() ||
      bounds[0].min_position_ <= -std::numeric_limits<double>::infinity())
    values[0] = 0.0;
  else
    values[0] = rng.uniformReal(std::max(bounds[0].min_position_, seed[0] - distance),
                                std::min(bounds[0].max_position_, seed[0] + distance));
  if (bounds[1].max_position_ >= std::numeric_limits<double>::infinity() ||
      bounds[1].min_position_ <= -std::numeric_limits<double>::infinity())
    values[1] = 0.0;
  else
    values[1] = rng.uniformReal(std::max(bounds[1].min_position_, seed[1] - distance),
                                std::min(bounds[1].max_position_, seed[1] + distance));
  if (bounds[2].max_position_ >= std::numeric_limits<double>::infinity() ||
      bounds[2].min_position_ <= -std::numeric_limits<double>::infinity())
    values[2] = 0.0;
  else
    values[2] = rng.uniformReal(std::max(bounds[2].min_position_, seed[2] - distance),
                                std::min(bounds[2].max_position_, seed[2] + distance));

  double da = angular_distance_weight_ * distance;
  if (da >= .25 * boost::math::constants::pi<double>())
  {
    double q[4];
    rng.quaternion(q);
    values[3] = q[0];
    values[4] = q[1];
    values[5] = q[2];
    values[6] = q[3];
  }
  else
  {
    // taken from OMPL
    // sample angle & axis
    double ax = rng.gaussian01();
    double ay = rng.gaussian01();
    double az = rng.gaussian01();
    double angle = 2.0 * pow(rng.uniform01(), 1.0 / 3.0) * da;
    // convert to quaternion
    double q[4];
    double norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-6)
    {
      q[0] = q[1] = q[2] = 0.0;
      q[3] = 1.0;
    }
    else
    {
      double s = sin(angle / 2.0);
      q[0] = s * ax / norm;
      q[1] = s * ay / norm;
      q[2] = s * az / norm;
      q[3] = cos(angle / 2.0);
    }
    // multiply quaternions: near * q
    values[3] = seed[6] * q[0] + seed[3] * q[3] + seed[4] * q[2] - seed[5] * q[1];
    values[4] = seed[6] * q[1] + seed[4] * q[3] + seed[5] * q[0] - seed[3] * q[2];
    values[5] = seed[6] * q[2] + seed[5] * q[3] + seed[3] * q[1] - seed[4] * q[0];
    values[6] = seed[6] * q[3] - seed[3] * q[0] - seed[4] * q[1] - seed[5] * q[2];
  }
}

}  // end of namespace core
}  // end of namespace moveit
