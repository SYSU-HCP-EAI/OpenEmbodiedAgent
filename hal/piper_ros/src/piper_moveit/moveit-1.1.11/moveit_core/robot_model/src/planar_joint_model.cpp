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

#include <moveit/robot_model/planar_joint_model.h>
#include <geometric_shapes/check_isometry.h>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <cmath>

namespace moveit
{
namespace core
{
PlanarJointModel::PlanarJointModel(const std::string& name) : JointModel(name), angular_distance_weight_(1.0)
{
  type_ = PLANAR;

  local_variable_names_.push_back("x");
  local_variable_names_.push_back("y");
  local_variable_names_.push_back("theta");
  for (int i = 0; i < 3; ++i)
  {
    variable_names_.push_back(name_ + "/" + local_variable_names_[i]);
    variable_index_map_[variable_names_.back()] = i;
  }

  variable_bounds_.resize(3);
  variable_bounds_[0].position_bounded_ = true;
  variable_bounds_[1].position_bounded_ = true;
  variable_bounds_[2].position_bounded_ = false;

  variable_bounds_[0].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[0].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[1].min_position_ = -std::numeric_limits<double>::infinity();
  variable_bounds_[1].max_position_ = std::numeric_limits<double>::infinity();
  variable_bounds_[2].min_position_ = -boost::math::constants::pi<double>();
  variable_bounds_[2].max_position_ = boost::math::constants::pi<double>();

  computeVariableBoundsMsg();
}

unsigned int PlanarJointModel::getStateSpaceDimension() const
{
  return 3;
}

double PlanarJointModel::getMaximumExtent(const Bounds& other_bounds) const
{
  double dx = other_bounds[0].max_position_ - other_bounds[0].min_position_;
  double dy = other_bounds[1].max_position_ - other_bounds[1].min_position_;
  return sqrt(dx * dx + dy * dy) + boost::math::constants::pi<double>() * angular_distance_weight_;
}

void PlanarJointModel::getVariableDefaultPositions(double* values, const Bounds& bounds) const
{
  for (unsigned int i = 0; i < 2; ++i)
  {
    // if zero is a valid value
    if (bounds[i].min_position_ <= 0.0 && bounds[i].max_position_ >= 0.0)
      values[i] = 0.0;
    else
      values[i] = (bounds[i].min_position_ + bounds[i].max_position_) / 2.0;
  }
  values[2] = 0.0;
}

void PlanarJointModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
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
  values[2] = rng.uniformReal(bounds[2].min_position_, bounds[2].max_position_);
}

void PlanarJointModel::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
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

  double da = angular_distance_weight_ * distance;
  // limit the sampling range to 2pi to work correctly even if the distance is very large
  if (da > boost::math::constants::pi<double>())
    da = boost::math::constants::pi<double>();
  values[2] = rng.uniformReal(seed[2] - da, seed[2] + da);
  normalizeRotation(values);
}

void PlanarJointModel::interpolate(const double* from, const double* to, const double t, double* state) const
{
  // interpolate position
  state[0] = from[0] + (to[0] - from[0]) * t;
  state[1] = from[1] + (to[1] - from[1]) * t;

  // interpolate angle
  double diff = to[2] - from[2];
  if (fabs(diff) <= boost::math::constants::pi<double>())
    state[2] = from[2] + diff * t;
  else
  {
    if (diff > 0.0)
      diff = 2.0 * boost::math::constants::pi<double>() - diff;
    else
      diff = -2.0 * boost::math::constants::pi<double>() - diff;
    state[2] = from[2] - diff * t;
    // input states are within bounds, so the following check is sufficient
    if (state[2] > boost::math::constants::pi<double>())
      state[2] -= 2.0 * boost::math::constants::pi<double>();
    else if (state[2] < -boost::math::constants::pi<double>())
      state[2] += 2.0 * boost::math::constants::pi<double>();
  }
}

double PlanarJointModel::distance(const double* values1, const double* values2) const
{
  double dx = values1[0] - values2[0];
  double dy = values1[1] - values2[1];

  double d = fabs(values1[2] - values2[2]);
  d = (d > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - d : d;
  return sqrt(dx * dx + dy * dy) + angular_distance_weight_ * d;
}

bool PlanarJointModel::satisfiesPositionBounds(const double* values, const Bounds& bounds, double margin) const
{
  for (unsigned int i = 0; i < 3; ++i)
    if (values[i] < bounds[i].min_position_ - margin || values[i] > bounds[i].max_position_ + margin)
      return false;
  return true;
}

bool PlanarJointModel::normalizeRotation(double* values) const
{
  double& v = values[2];
  if (v >= -boost::math::constants::pi<double>() && v <= boost::math::constants::pi<double>())
    return false;
  v = fmod(v, 2.0 * boost::math::constants::pi<double>());
  if (v < -boost::math::constants::pi<double>())
    v += 2.0 * boost::math::constants::pi<double>();
  else if (v > boost::math::constants::pi<double>())
    v -= 2.0 * boost::math::constants::pi<double>();
  return true;
}

bool PlanarJointModel::enforcePositionBounds(double* values, const Bounds& bounds) const
{
  bool result = normalizeRotation(values);
  for (unsigned int i = 0; i < 2; ++i)
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

void PlanarJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const
{
  transf = Eigen::Isometry3d(Eigen::Translation3d(joint_values[0], joint_values[1], 0.0) *
                             Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitZ()));
}

void PlanarJointModel::computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const
{
  joint_values[0] = transf.translation().x();
  joint_values[1] = transf.translation().y();

  ASSERT_ISOMETRY(transf)  // unsanitized input, could contain a non-isometry
  Eigen::Quaterniond q(transf.linear());
  // taken from Bullet
  double s_squared = 1.0 - (q.w() * q.w());
  if (s_squared < 10.0 * std::numeric_limits<double>::epsilon())
    joint_values[2] = 0.0;
  else
  {
    double s = 1.0 / sqrt(s_squared);
    joint_values[2] = (acos(q.w()) * 2.0f) * (q.z() * s);
  }
}

}  // end of namespace core
}  // end of namespace moveit
