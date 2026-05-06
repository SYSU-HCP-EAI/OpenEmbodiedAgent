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

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#pragma once

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// KDL
#include <kdl/config.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace KDL
{
class ChainIkSolverVelMimicSVD;
}

namespace kdl_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using KDL.
 * This version supports any kinematic chain, also including mimic joints.
 */
class KDLKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
   *  @brief Default constructor
   */
  KDLKinematicsPlugin();

  bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

  bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                  const std::string& base_frame, const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const override;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const override;

protected:
  typedef Eigen::Matrix<double, 6, 1> Twist;

  /// Solve position IK given initial joint values
  // NOLINTNEXTLINE(readability-identifier-naming)
  int CartToJnt(KDL::ChainIkSolverVelMimicSVD& ik_solver, const KDL::JntArray& q_init, const KDL::Frame& p_in,
                KDL::JntArray& q_out, const unsigned int max_iter, const Eigen::VectorXd& joint_weights,
                const Twist& cartesian_weights) const;

private:
  void getJointWeights();
  bool timedOut(const ros::WallTime& start_time, double duration) const;

  /** @brief Check whether the solution lies within the consistency limits of the seed state
   *  @param seed_state Seed state
   *  @param consistency_limits
   *  @param solution solution configuration
   *  @return true if check succeeds
   */
  bool checkConsistency(const Eigen::VectorXd& seed_state, const std::vector<double>& consistency_limits,
                        const Eigen::VectorXd& solution) const;

  void getRandomConfiguration(Eigen::VectorXd& jnt_array) const;

  /** @brief Get a random configuration within consistency limits close to the seed state
   *  @param seed_state Seed state
   *  @param consistency_limits
   *  @param jnt_array Returned random configuration
   */
  void getRandomConfiguration(const Eigen::VectorXd& seed_state, const std::vector<double>& consistency_limits,
                              Eigen::VectorXd& jnt_array) const;

  /// clip q_delta such that joint limits will not be violated
  void clipToJointLimits(const KDL::JntArray& q, KDL::JntArray& q_delta, Eigen::ArrayXd& weighting) const;

  bool initialized_;  ///< Internal variable that indicates whether solver is configured and ready

  unsigned int dimension_;                        ///< Dimension of the group
  moveit_msgs::KinematicSolverInfo solver_info_;  ///< Stores information for the inverse kinematics solver

  const moveit::core::JointModelGroup* joint_model_group_;
  moveit::core::RobotStatePtr state_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos> fk_solver_;
  std::vector<JointMimic> mimic_joints_;
  std::vector<double> joint_weights_;
  Eigen::VectorXd joint_min_, joint_max_;  ///< joint limits

  int max_solver_iterations_;
  double epsilon_;
  /** weight of orientation error vs position error
   *
   * < 1.0: orientation has less importance than position
   * > 1.0: orientation has more importance than position
   * = 0.0: perform position-only IK */
  double orientation_vs_position_weight_;
};
}  // namespace kdl_kinematics_plugin
