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

#include <chomp_motion_planner/chomp_cost.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <eigen3/Eigen/LU>

using namespace Eigen;
using namespace std;

namespace chomp
{
ChompCost::ChompCost(const ChompTrajectory& trajectory, int /* joint_number */,
                     const std::vector<double>& derivative_costs, double ridge_factor)
{
  int num_vars_all = trajectory.getNumPoints();
  int num_vars_free = num_vars_all - 2 * (DIFF_RULE_LENGTH - 1);
  MatrixXd diff_matrix = MatrixXd::Zero(num_vars_all, num_vars_all);
  quad_cost_full_ = MatrixXd::Zero(num_vars_all, num_vars_all);

  // construct the quad cost for all variables, as a sum of squared differentiation matrices
  double multiplier = 1.0;
  for (unsigned int i = 0; i < derivative_costs.size(); i++)
  {
    multiplier *= trajectory.getDiscretization();
    diff_matrix = getDiffMatrix(num_vars_all, &DIFF_RULES[i][0]);
    quad_cost_full_ += (derivative_costs[i] * multiplier) * (diff_matrix.transpose() * diff_matrix);
  }
  quad_cost_full_ += MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;

  // extract the quad cost just for the free variables:
  quad_cost_ = quad_cost_full_.block(DIFF_RULE_LENGTH - 1, DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);

  // invert the matrix:
  quad_cost_inv_ = quad_cost_.inverse();

  // cout << quad_cost_inv_ << endl;
}

Eigen::MatrixXd ChompCost::getDiffMatrix(int size, const double* diff_rule) const
{
  MatrixXd matrix = MatrixXd::Zero(size, size);
  for (int i = 0; i < size; i++)
  {
    for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++)
    {
      int index = i + j;
      if (index < 0)
        continue;
      if (index >= size)
        continue;
      matrix(i, index) = diff_rule[j + DIFF_RULE_LENGTH / 2];
    }
  }
  return matrix;
}

double ChompCost::getMaxQuadCostInvValue() const
{
  return quad_cost_inv_.maxCoeff();
}

void ChompCost::scale(double scale)
{
  double inv_scale = 1.0 / scale;
  quad_cost_inv_ *= inv_scale;
  quad_cost_ *= scale;
  quad_cost_full_ *= scale;
}

ChompCost::~ChompCost() = default;
}  // namespace chomp
