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

#pragma once

#include <moveit/robot_state/robot_state.h>

/** @brief Namespace for kinematics metrics */
namespace kinematics_metrics
{
MOVEIT_CLASS_FORWARD(KinematicsMetrics);  // Defines KinematicsMetricsPtr, ConstPtr, WeakPtr... etc

/**
 * \brief Compute different kinds of metrics for kinematics evaluation. Currently includes
 * manipulability.
 */
class KinematicsMetrics
{
public:
  /** \brief Construct a KinematicsMetricss from a RobotModel */
  KinematicsMetrics(const moveit::core::RobotModelConstPtr& robot_model)
    : robot_model_(robot_model), penalty_multiplier_(0.0)
  {
  }

  /**
   * @brief Get the manipulability for a given group at a given joint configuration
   * @param state Complete kinematic state for the robot
   * @param group_name The group name (e.g. "arm")
   * @param manipulability_index The computed manipulability = sqrt(det(JJ^T))
   * @return False if the group was not found
   */
  bool getManipulabilityIndex(const moveit::core::RobotState& state, const std::string& group_name,
                              double& manipulability_index, bool translation = false) const;

  /**
   * @brief Get the manipulability for a given group at a given joint configuration
   * @param state Complete kinematic state for the robot
   * @param joint_model_group A pointer to the desired joint model group
   * @param manipulability_index The computed manipulability = sqrt(det(JJ^T))
   * @return False if the group was not found
   */
  bool getManipulabilityIndex(const moveit::core::RobotState& state,
                              const moveit::core::JointModelGroup* joint_model_group, double& manipulability_index,
                              bool translation = false) const;

  /**
   * @brief Get the (translation) manipulability ellipsoid for a given group at a given joint configuration
   * @param state Complete kinematic state for the robot
   * @param group_name The group name (e.g. "arm")
   * @param eigen_values The eigen values for the translation part of JJ^T
   * @param eigen_vectors The eigen vectors for the translation part of JJ^T
   * @return False if the group was not found
   */
  bool getManipulabilityEllipsoid(const moveit::core::RobotState& state, const std::string& group_name,
                                  Eigen::MatrixXcd& eigen_values, Eigen::MatrixXcd& eigen_vectors) const;

  /**
   * @brief Get the (translation) manipulability ellipsoid for a given group at a given joint configuration
   * @param state Complete kinematic state for the robot
   * @param joint_model_group A pointer to the desired joint model group
   * @param eigen_values The eigen values for the translation part of JJ^T
   * @param eigen_vectors The eigen vectors for the translation part of JJ^T
   * @return False if the group was not found
   */
  bool getManipulabilityEllipsoid(const moveit::core::RobotState& state,
                                  const moveit::core::JointModelGroup* joint_model_group,
                                  Eigen::MatrixXcd& eigen_values, Eigen::MatrixXcd& eigen_vectors) const;

  /**
   * @brief Get the manipulability = sigma_min/sigma_max
   * where sigma_min and sigma_max are the smallest and largest singular values
   * of the Jacobian matrix J
   * @param state Complete kinematic state for the robot
   * @param group_name The group name (e.g. "arm")
   * @param condition_number Condition number for JJ^T
   * @return False if the group was not found
   */
  bool getManipulability(const moveit::core::RobotState& state, const std::string& group_name, double& condition_number,
                         bool translation = false) const;

  /**
   * @brief Get the manipulability = sigma_min/sigma_max
   * where sigma_min and sigma_max are the smallest and largest singular values
   * of the Jacobian matrix J
   * @param state Complete kinematic state for the robot
   * @param joint_model_group A pointer to the desired joint model group
   * @param condition_number Condition number for JJ^T
   * @return False if the group was not found
   */
  bool getManipulability(const moveit::core::RobotState& state, const moveit::core::JointModelGroup* joint_model_group,
                         double& condition_number, bool translation = false) const;

  void setPenaltyMultiplier(double multiplier)
  {
    penalty_multiplier_ = fabs(multiplier);
  }

  const double& getPenaltyMultiplier() const
  {
    return penalty_multiplier_;
  }

protected:
  moveit::core::RobotModelConstPtr robot_model_;

private:
  /**
   * @brief Defines a multiplier for the manipulabilty
   * = 1 - exp ( -penalty_multipler_ * product_{i=1}{n} (distance_to_lower_limit *
   * distance_to_higher_limit/(joint_range*joint_range)))
   * where n is the number of joints in the group. Floating joints are ignored in this computation. Planar joints with
   * finite bounds
   * are considered.
   * Set penalty_multiplier_ to 0 if you don't want this multiplier to have any effect on the manipulability measures.
   * See "Workspace Geometric Characterization and Manipulability of Industrial Robots", Ming-June, Tsia, PhD Thesis,
   * Ohio State University, 1986, for more details.
   * @return multiplier that is multiplied with every manipulability measure computed here
   */
  double getJointLimitsPenalty(const moveit::core::RobotState& state,
                               const moveit::core::JointModelGroup* joint_model_group) const;

  double penalty_multiplier_;
};
}  // namespace kinematics_metrics
