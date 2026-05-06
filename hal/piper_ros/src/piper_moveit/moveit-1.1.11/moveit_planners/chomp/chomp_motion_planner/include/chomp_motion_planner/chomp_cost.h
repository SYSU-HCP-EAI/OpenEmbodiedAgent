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

#include <eigen3/Eigen/Core>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <vector>

namespace chomp
{
/**
 * \brief Represents the smoothness cost for CHOMP, for a single joint
 */
class ChompCost
{
public:
  ChompCost(const ChompTrajectory& trajectory, int joint_number, const std::vector<double>& derivative_costs,
            double ridge_factor = 0.0);
  virtual ~ChompCost();

  template <typename Derived>
  void getDerivative(const Eigen::MatrixXd::ColXpr& joint_trajectory, Eigen::MatrixBase<Derived>& derivative) const;

  const Eigen::MatrixXd& getQuadraticCostInverse() const;

  const Eigen::MatrixXd& getQuadraticCost() const;

  double getCost(const Eigen::MatrixXd::ColXpr& joint_trajectory) const;

  double getMaxQuadCostInvValue() const;

  void scale(double scale);

private:
  Eigen::MatrixXd quad_cost_full_;
  Eigen::MatrixXd quad_cost_;
  // Eigen::VectorXd linear_cost_;
  Eigen::MatrixXd quad_cost_inv_;

  Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;
};

template <typename Derived>
void ChompCost::getDerivative(const Eigen::MatrixXd::ColXpr& joint_trajectory,
                              Eigen::MatrixBase<Derived>& derivative) const
{
  derivative = (quad_cost_full_ * (2.0 * joint_trajectory));
}

inline const Eigen::MatrixXd& ChompCost::getQuadraticCostInverse() const
{
  return quad_cost_inv_;
}

inline const Eigen::MatrixXd& ChompCost::getQuadraticCost() const
{
  return quad_cost_;
}

inline double ChompCost::getCost(const Eigen::MatrixXd::ColXpr& joint_trajectory) const
{
  return joint_trajectory.dot(quad_cost_full_ * joint_trajectory);
}

}  // namespace chomp
