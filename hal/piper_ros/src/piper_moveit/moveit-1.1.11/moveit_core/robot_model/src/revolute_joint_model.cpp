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

#include <moveit/robot_model/revolute_joint_model.h>
#include <geometric_shapes/check_isometry.h>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <cmath>

namespace moveit
{
namespace core
{
RevoluteJointModel::RevoluteJointModel(const std::string& name)
  : JointModel(name), axis_(0.0, 0.0, 0.0), continuous_(false), x2_(0.0), y2_(0.0), z2_(0.0), xy_(0.0), xz_(0.0), yz_(0.0)
{
  type_ = REVOLUTE;
  variable_names_.push_back(name_);
  variable_bounds_.resize(1);
  variable_bounds_[0].position_bounded_ = true;
  variable_bounds_[0].min_position_ = -boost::math::constants::pi<double>();
  variable_bounds_[0].max_position_ = boost::math::constants::pi<double>();
  variable_index_map_[name_] = 0;
  computeVariableBoundsMsg();
}

unsigned int RevoluteJointModel::getStateSpaceDimension() const
{
  return 1;
}

void RevoluteJointModel::setAxis(const Eigen::Vector3d& axis)
{
  axis_ = axis.normalized();
  x2_ = axis_.x() * axis_.x();
  y2_ = axis_.y() * axis_.y();
  z2_ = axis_.z() * axis_.z();
  xy_ = axis_.x() * axis_.y();
  xz_ = axis_.x() * axis_.z();
  yz_ = axis_.y() * axis_.z();
}

void RevoluteJointModel::setContinuous(bool flag)
{
  continuous_ = flag;
  if (flag)
  {
    variable_bounds_[0].position_bounded_ = false;
    variable_bounds_[0].min_position_ = -boost::math::constants::pi<double>();
    variable_bounds_[0].max_position_ = boost::math::constants::pi<double>();
  }
  else
    variable_bounds_[0].position_bounded_ = true;
  computeVariableBoundsMsg();
}

double RevoluteJointModel::getMaximumExtent(const Bounds& /*other_bounds*/) const
{
  return variable_bounds_[0].max_position_ - variable_bounds_[0].min_position_;
}

void RevoluteJointModel::getVariableDefaultPositions(double* values, const Bounds& bounds) const
{
  // if zero is a valid value
  if (bounds[0].min_position_ <= 0.0 && bounds[0].max_position_ >= 0.0)
    values[0] = 0.0;
  else
    values[0] = (bounds[0].min_position_ + bounds[0].max_position_) / 2.0;
}

void RevoluteJointModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                                    const Bounds& bounds) const
{
  values[0] = rng.uniformReal(bounds[0].min_position_, bounds[0].max_position_);
}

void RevoluteJointModel::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                          const Bounds& bounds, const double* seed,
                                                          const double distance) const
{
  if (continuous_)
  {
    values[0] = rng.uniformReal(seed[0] - distance, seed[0] + distance);
    enforcePositionBounds(values, bounds);
  }
  else
    values[0] = rng.uniformReal(std::max(bounds[0].min_position_, seed[0] - distance),
                                std::min(bounds[0].max_position_, seed[0] + distance));
}

void RevoluteJointModel::interpolate(const double* from, const double* to, const double t, double* state) const
{
  if (continuous_)
  {
    double diff = to[0] - from[0];
    if (fabs(diff) <= boost::math::constants::pi<double>())
      state[0] = from[0] + diff * t;
    else
    {
      if (diff > 0.0)
        diff = 2.0 * boost::math::constants::pi<double>() - diff;
      else
        diff = -2.0 * boost::math::constants::pi<double>() - diff;
      state[0] = from[0] - diff * t;
      // input states are within bounds, so the following check is sufficient
      if (state[0] > boost::math::constants::pi<double>())
        state[0] -= 2.0 * boost::math::constants::pi<double>();
      else if (state[0] < -boost::math::constants::pi<double>())
        state[0] += 2.0 * boost::math::constants::pi<double>();
    }
  }
  else
    state[0] = from[0] + (to[0] - from[0]) * t;
}

double RevoluteJointModel::distance(const double* values1, const double* values2) const
{
  if (continuous_)
  {
    double d = fmod(fabs(values1[0] - values2[0]), 2.0 * boost::math::constants::pi<double>());
    return (d > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - d : d;
  }
  else
    return fabs(values1[0] - values2[0]);
}

bool RevoluteJointModel::satisfiesPositionBounds(const double* values, const Bounds& bounds, double margin) const
{
  if (continuous_)
    return true;
  else
    return !(values[0] < bounds[0].min_position_ - margin || values[0] > bounds[0].max_position_ + margin);
}

bool RevoluteJointModel::harmonizePosition(double* values, const JointModel::Bounds& other_bounds) const
{
  bool modified = false;
  if (*values < other_bounds[0].min_position_)
  {
    while (*values + 2 * M_PI <= other_bounds[0].max_position_)
    {
      *values += 2 * M_PI;
      modified = true;
    }
  }
  else if (*values > other_bounds[0].max_position_)
  {
    while (*values - 2 * M_PI >= other_bounds[0].min_position_)
    {
      *values -= 2 * M_PI;
      modified = true;
    }
  }
  return modified;
}

bool RevoluteJointModel::enforcePositionBounds(double* values, const Bounds& bounds) const
{
  if (continuous_)
  {
    double& v = values[0];
    if (v <= -boost::math::constants::pi<double>() || v > boost::math::constants::pi<double>())
    {
      v = fmod(v, 2.0 * boost::math::constants::pi<double>());
      if (v <= -boost::math::constants::pi<double>())
        v += 2.0 * boost::math::constants::pi<double>();
      else if (v > boost::math::constants::pi<double>())
        v -= 2.0 * boost::math::constants::pi<double>();
      return true;
    }
  }
  else
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
  }
  return false;
}

void RevoluteJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const
{
  const double c = cos(joint_values[0]);
  const double s = sin(joint_values[0]);
  const double t = 1.0 - c;
  const double txy = t * xy_;
  const double txz = t * xz_;
  const double tyz = t * yz_;

  const double zs = axis_.z() * s;
  const double ys = axis_.y() * s;
  const double xs = axis_.x() * s;

  // column major
  double* d = transf.data();

  d[0] = t * x2_ + c;
  d[1] = txy + zs;
  d[2] = txz - ys;
  d[3] = 0.0;

  d[4] = txy - zs;
  d[5] = t * y2_ + c;
  d[6] = tyz + xs;
  d[7] = 0.0;

  d[8] = txz + ys;
  d[9] = tyz - xs;
  d[10] = t * z2_ + c;
  d[11] = 0.0;

  d[12] = 0.0;
  d[13] = 0.0;
  d[14] = 0.0;
  d[15] = 1.0;

  //  transf = Eigen::Isometry3d(Eigen::AngleAxisd(joint_values[0], axis_));
}

void RevoluteJointModel::computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const
{
  ASSERT_ISOMETRY(transf)  // unsanitized input, could contain a non-isometry
  Eigen::Quaterniond q(transf.linear());
  q.normalize();
  size_t max_idx;
  axis_.array().abs().maxCoeff(&max_idx);
  joint_values[0] = 2. * atan2(q.vec()[max_idx] / axis_[max_idx], q.w());
}

}  // end of namespace core
}  // end of namespace moveit
