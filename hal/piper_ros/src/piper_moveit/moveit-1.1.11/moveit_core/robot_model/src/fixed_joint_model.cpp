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

#include <moveit/robot_model/fixed_joint_model.h>

namespace moveit
{
namespace core
{
FixedJointModel::FixedJointModel(const std::string& name) : JointModel(name)
{
  type_ = FIXED;
}

unsigned int FixedJointModel::getStateSpaceDimension() const
{
  return 0;
}

void FixedJointModel::getVariableDefaultPositions(double* /*values*/, const Bounds& /*bounds*/) const
{
}

void FixedJointModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& /*rng*/, double* /*values*/,
                                                 const Bounds& /*bounds*/) const
{
}

void FixedJointModel::getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& /*rng*/,
                                                       double* /*values*/, const Bounds& /*bounds*/,
                                                       const double* /*seed*/, const double /*distance*/) const
{
}

bool FixedJointModel::enforcePositionBounds(double* /*values*/, const Bounds& /*bounds*/) const
{
  return false;
}

bool FixedJointModel::satisfiesPositionBounds(const double* /*values*/, const Bounds& /*bounds*/,
                                              double /*margin*/) const
{
  return true;
}

double FixedJointModel::distance(const double* /*values1*/, const double* /*values2*/) const
{
  return 0.0;
}

double FixedJointModel::getMaximumExtent(const Bounds& /*other_bounds*/) const
{
  return 0.0;
}

void FixedJointModel::interpolate(const double* /*from*/, const double* /*to*/, const double /*t*/,
                                  double* /*state*/) const
{
}

void FixedJointModel::computeTransform(const double* /* joint_values */, Eigen::Isometry3d& transf) const
{
  transf.setIdentity();
}

void FixedJointModel::computeVariablePositions(const Eigen::Isometry3d& /* transform */,
                                               double* /* joint_values */) const
{
}

}  // end of namespace core
}  // end of namespace moveit
