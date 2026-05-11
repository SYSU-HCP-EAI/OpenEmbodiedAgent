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

#include <ros/ros.h>
#include <chomp_motion_planner/chomp_trajectory.h>

namespace chomp
{
ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, double duration,
                                 double discretization, const std::string& group_name)
  : ChompTrajectory(robot_model, static_cast<size_t>(duration / discretization) + 1, discretization, group_name)
{
}

ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, size_t num_points,
                                 double discretization, const std::string& group_name)
  : planning_group_name_(group_name)
  , num_points_(num_points)
  , discretization_(discretization)
  , duration_((num_points - 1) * discretization)
  , start_index_(1)
  , end_index_(num_points_ - 2)
{
  const moveit::core::JointModelGroup* model_group = robot_model->getJointModelGroup(planning_group_name_);
  num_joints_ = model_group->getActiveJointModels().size();
  init();
}

ChompTrajectory::ChompTrajectory(const ChompTrajectory& source_traj, const std::string& group_name, int diff_rule_length)
  : planning_group_name_(group_name), discretization_(source_traj.discretization_)
{
  num_joints_ = source_traj.getNumJoints();

  // figure out the num_points_:
  // we need diff_rule_length-1 extra points on either side:
  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

  num_points_ = source_traj.num_points_ + start_extra + end_extra;
  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
  duration_ = (num_points_ - 1) * discretization_;

  // allocate the memory:
  init();

  full_trajectory_index_.resize(num_points_);

  // now copy the trajectories over:
  for (size_t i = 0; i < num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (static_cast<size_t>(source_traj_point) >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_ - 1;
    full_trajectory_index_[i] = source_traj_point;
    getTrajectoryPoint(i) = const_cast<ChompTrajectory&>(source_traj).getTrajectoryPoint(source_traj_point);
  }
}

void ChompTrajectory::init()
{
  trajectory_.resize(num_points_, num_joints_);
}

void ChompTrajectory::updateFromGroupTrajectory(const ChompTrajectory& group_trajectory)
{
  size_t num_vars_free = end_index_ - start_index_ + 1;
  trajectory_.block(start_index_, 0, num_vars_free, num_joints_) =
      group_trajectory.trajectory_.block(group_trajectory.start_index_, 0, num_vars_free, num_joints_);
}

void ChompTrajectory::fillInLinearInterpolation()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  for (size_t i = 0; i < num_joints_; i++)
  {
    double theta = ((*this)(end_index, i) - (*this)(start_index, i)) / (end_index - 1);
    for (size_t j = start_index + 1; j < end_index; j++)
    {
      (*this)(j, i) = (*this)(start_index, i) + j * theta;
    }
  }
}

void ChompTrajectory::fillInCubicInterpolation()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double dt = 0.001;
  std::vector<double> coeffs(4, 0);
  double total_time = (end_index - 1) * dt;
  for (size_t i = 0; i < num_joints_; i++)
  {
    coeffs[0] = (*this)(start_index, i);
    coeffs[2] = (3 / (pow(total_time, 2))) * ((*this)(end_index, i) - (*this)(start_index, i));
    coeffs[3] = (-2 / (pow(total_time, 3))) * ((*this)(end_index, i) - (*this)(start_index, i));

    double t;
    for (size_t j = start_index + 1; j < end_index; j++)
    {
      t = j * dt;
      (*this)(j, i) = coeffs[0] + coeffs[2] * pow(t, 2) + coeffs[3] * pow(t, 3);
    }
  }
}

void ChompTrajectory::fillInMinJerk()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double td[6];  // powers of the time duration
  td[0] = 1.0;
  td[1] = (end_index - start_index) * discretization_;

  for (unsigned int i = 2; i <= 5; i++)
    td[i] = td[i - 1] * td[1];

  // calculate the spline coefficients for each joint:
  // (these are for the special case of zero start and end vel and acc)
  std::vector<double[6]> coeff(num_joints_);
  for (size_t i = 0; i < num_joints_; i++)
  {
    double x0 = (*this)(start_index, i);
    double x1 = (*this)(end_index, i);
    coeff[i][0] = x0;
    coeff[i][1] = 0;
    coeff[i][2] = 0;
    coeff[i][3] = (-20 * x0 + 20 * x1) / (2 * td[3]);
    coeff[i][4] = (30 * x0 - 30 * x1) / (2 * td[4]);
    coeff[i][5] = (-12 * x0 + 12 * x1) / (2 * td[5]);
  }

  // now fill in the joint positions at each time step
  for (size_t i = start_index + 1; i < end_index; i++)
  {
    double ti[6];  // powers of the time index point
    ti[0] = 1.0;
    ti[1] = (i - start_index) * discretization_;
    for (unsigned int k = 2; k <= 5; k++)
      ti[k] = ti[k - 1] * ti[1];

    for (size_t j = 0; j < num_joints_; j++)
    {
      (*this)(i, j) = 0.0;
      for (unsigned int k = 0; k <= 5; k++)
      {
        (*this)(i, j) += ti[k] * coeff[j][k];
      }
    }
  }
}

bool ChompTrajectory::fillInFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory)
{
  // check if input trajectory has less than two states (start and goal), function returns false if condition is true
  if (trajectory.getWayPointCount() < 2)
    return false;

  const size_t max_output_index = getNumPoints() - 1;
  const size_t max_input_index = trajectory.getWayPointCount() - 1;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  moveit::core::RobotState interpolated(trajectory.getRobotModel());
  for (size_t i = 0; i <= max_output_index; i++)
  {
    double fraction = static_cast<double>(i * max_input_index) / max_output_index;
    const size_t prev_idx = std::trunc(fraction);  // integer part
    fraction = fraction - prev_idx;                // fractional part
    const size_t next_idx = prev_idx == max_input_index ? prev_idx : prev_idx + 1;
    trajectory.getWayPoint(prev_idx).interpolate(trajectory.getWayPoint(next_idx), fraction, interpolated, group);
    assignCHOMPTrajectoryPointFromRobotState(interpolated, i, group);
  }
  return true;
}

void ChompTrajectory::assignCHOMPTrajectoryPointFromRobotState(const moveit::core::RobotState& source,
                                                               size_t chomp_trajectory_point_index,
                                                               const moveit::core::JointModelGroup* group)
{
  Eigen::MatrixXd::RowXpr target = getTrajectoryPoint(chomp_trajectory_point_index);
  assert(group->getActiveJointModels().size() == static_cast<size_t>(target.cols()));
  size_t joint_index = 0;
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
  {
    assert(jm->getVariableCount() == 1);
    target[joint_index++] = source.getVariablePosition(jm->getFirstVariableIndex());
  }
}

}  // namespace chomp
