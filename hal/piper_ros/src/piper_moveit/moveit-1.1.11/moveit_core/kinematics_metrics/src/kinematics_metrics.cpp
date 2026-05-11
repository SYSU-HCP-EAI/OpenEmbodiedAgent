/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Sachin Chitta */

#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <boost/math/constants/constants.hpp>

namespace kinematics_metrics
{
double KinematicsMetrics::getJointLimitsPenalty(const moveit::core::RobotState& state,
                                                const moveit::core::JointModelGroup* joint_model_group) const
{
  if (fabs(penalty_multiplier_) <= boost::math::tools::epsilon<double>())
    return 1.0;
  double joint_limits_multiplier(1.0);
  const std::vector<const moveit::core::JointModel*>& joint_model_vector = joint_model_group->getJointModels();
  for (const moveit::core::JointModel* joint_model : joint_model_vector)
  {
    if (joint_model->getType() == moveit::core::JointModel::REVOLUTE)
    {
      const moveit::core::RevoluteJointModel* revolute_model =
          static_cast<const moveit::core::RevoluteJointModel*>(joint_model);
      if (revolute_model->isContinuous())
        continue;
    }
    if (joint_model->getType() == moveit::core::JointModel::PLANAR)
    {
      const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();
      if (bounds[0].min_position_ == -std::numeric_limits<double>::max() ||
          bounds[0].max_position_ == std::numeric_limits<double>::max() ||
          bounds[1].min_position_ == -std::numeric_limits<double>::max() ||
          bounds[1].max_position_ == std::numeric_limits<double>::max() ||
          bounds[2].min_position_ == -boost::math::constants::pi<double>() ||
          bounds[2].max_position_ == boost::math::constants::pi<double>())
        continue;
    }
    if (joint_model->getType() == moveit::core::JointModel::FLOATING)
    {
      // Joint limits are not well-defined for floating joints
      continue;
    }
    const double* joint_values = state.getJointPositions(joint_model);
    const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();
    std::vector<double> lower_bounds, upper_bounds;
    for (const moveit::core::VariableBounds& bound : bounds)
    {
      lower_bounds.push_back(bound.min_position_);
      upper_bounds.push_back(bound.max_position_);
    }
    double lower_bound_distance = joint_model->distance(joint_values, &lower_bounds[0]);
    double upper_bound_distance = joint_model->distance(joint_values, &upper_bounds[0]);
    double range = lower_bound_distance + upper_bound_distance;
    if (range <= boost::math::tools::epsilon<double>())
      continue;
    joint_limits_multiplier *= (lower_bound_distance * upper_bound_distance / (range * range));
  }
  return (1.0 - exp(-penalty_multiplier_ * joint_limits_multiplier));
}

bool KinematicsMetrics::getManipulabilityIndex(const moveit::core::RobotState& state, const std::string& group_name,
                                               double& manipulability_index, bool translation) const
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (joint_model_group)
    return getManipulabilityIndex(state, joint_model_group, manipulability_index, translation);
  else
    return false;
}

bool KinematicsMetrics::getManipulabilityIndex(const moveit::core::RobotState& state,
                                               const moveit::core::JointModelGroup* joint_model_group,
                                               double& manipulability_index, bool translation) const
{
  // state.getJacobian() only works for chain groups.
  if (!joint_model_group->isChain())
  {
    return false;
  }

  Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group);
  // Get joint limits penalty
  double penalty = getJointLimitsPenalty(state, joint_model_group);
  if (translation)
  {
    if (jacobian.cols() < 6)
    {
      Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian.topLeftCorner(3, jacobian.cols()));
      Eigen::MatrixXd singular_values = svdsolver.singularValues();
      manipulability_index = 1.0;
      for (unsigned int i = 0; i < singular_values.rows(); ++i)
      {
        ROS_DEBUG_NAMED("kinematics_metrics", "Singular value: %d %f", i, singular_values(i, 0));
        manipulability_index *= singular_values(i, 0);
      }
      // Get manipulability index
      manipulability_index = penalty * manipulability_index;
    }
    else
    {
      Eigen::MatrixXd jacobian_2 = jacobian.topLeftCorner(3, jacobian.cols());
      Eigen::MatrixXd matrix = jacobian_2 * jacobian_2.transpose();
      // Get manipulability index
      manipulability_index = penalty * sqrt(matrix.determinant());
    }
  }
  else
  {
    if (jacobian.cols() < 6)
    {
      Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian);
      Eigen::MatrixXd singular_values = svdsolver.singularValues();
      manipulability_index = 1.0;
      for (unsigned int i = 0; i < singular_values.rows(); ++i)
      {
        ROS_DEBUG_NAMED("kinematics_metrics", "Singular value: %d %f", i, singular_values(i, 0));
        manipulability_index *= singular_values(i, 0);
      }
      // Get manipulability index
      manipulability_index = penalty * manipulability_index;
    }
    else
    {
      Eigen::MatrixXd matrix = jacobian * jacobian.transpose();
      // Get manipulability index
      manipulability_index = penalty * sqrt(matrix.determinant());
    }
  }
  return true;
}

bool KinematicsMetrics::getManipulabilityEllipsoid(const moveit::core::RobotState& state, const std::string& group_name,
                                                   Eigen::MatrixXcd& eigen_values,
                                                   Eigen::MatrixXcd& eigen_vectors) const
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (joint_model_group)
    return getManipulabilityEllipsoid(state, joint_model_group, eigen_values, eigen_vectors);
  else
    return false;
}

bool KinematicsMetrics::getManipulabilityEllipsoid(const moveit::core::RobotState& state,
                                                   const moveit::core::JointModelGroup* joint_model_group,
                                                   Eigen::MatrixXcd& eigen_values,
                                                   Eigen::MatrixXcd& eigen_vectors) const
{
  // state.getJacobian() only works for chain groups.
  if (!joint_model_group->isChain())
  {
    return false;
  }

  Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group);
  Eigen::MatrixXd matrix = jacobian * jacobian.transpose();
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix.block(0, 0, 3, 3));
  eigen_values = eigensolver.eigenvalues();
  eigen_vectors = eigensolver.eigenvectors();
  return true;
}

bool KinematicsMetrics::getManipulability(const moveit::core::RobotState& state, const std::string& group_name,
                                          double& manipulability, bool translation) const
{
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (joint_model_group)
    return getManipulability(state, joint_model_group, manipulability, translation);
  else
    return false;
}

bool KinematicsMetrics::getManipulability(const moveit::core::RobotState& state,
                                          const moveit::core::JointModelGroup* joint_model_group,
                                          double& manipulability, bool translation) const
{
  // state.getJacobian() only works for chain groups.
  if (!joint_model_group->isChain())
  {
    return false;
  }
  // Get joint limits penalty
  double penalty = getJointLimitsPenalty(state, joint_model_group);
  if (translation)
  {
    Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian.topLeftCorner(3, jacobian.cols()));
    Eigen::MatrixXd singular_values = svdsolver.singularValues();
    for (int i = 0; i < singular_values.rows(); ++i)
      ROS_DEBUG_NAMED("kinematics_metrics", "Singular value: %d %f", i, singular_values(i, 0));
    manipulability = penalty * singular_values.minCoeff() / singular_values.maxCoeff();
  }
  else
  {
    Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();
    for (int i = 0; i < singular_values.rows(); ++i)
      ROS_DEBUG_NAMED("kinematics_metrics", "Singular value: %d %f", i, singular_values(i, 0));
    manipulability = penalty * singular_values.minCoeff() / singular_values.maxCoeff();
  }
  return true;
}

}  // end of namespace kinematics_metrics
