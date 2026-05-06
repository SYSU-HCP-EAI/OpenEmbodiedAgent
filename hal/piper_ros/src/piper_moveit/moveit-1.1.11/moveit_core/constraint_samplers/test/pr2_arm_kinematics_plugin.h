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

#include <kdl/config.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <urdf/model.h>

#include <moveit/kinematics_base/kinematics_base.h>

#include <memory>

#include "pr2_arm_ik.h"

namespace pr2_arm_kinematics
{
static const int NO_IK_SOLUTION = -1;
static const int TIMED_OUT = -2;

MOVEIT_CLASS_FORWARD(PR2ArmIKSolver);

// minimal stuff necessary
class PR2ArmIKSolver : public KDL::ChainIkSolverPos
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @class
   *  @brief ROS/KDL based interface for the inverse kinematics of the PR2 arm
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   *
   *  This class provides a KDL based interface to the inverse kinematics of the PR2 arm.
   *  It inherits from the KDL::ChainIkSolverPos class but also exposes additional functionality
   *  to return multiple solutions from an inverse kinematics computation.
   */
  PR2ArmIKSolver(const urdf::ModelInterface& robot_model, const std::string& root_frame_name,
                 const std::string& tip_frame_name, const double& search_discretization_angle, const int& free_angle);

  ~PR2ArmIKSolver() override{};

// TODO: simplify after kinetic support is dropped
#define KDL_VERSION_LESS(a, b, c) ((KDL_VERSION) < ((a << 16) | (b << 8) | c))
#if KDL_VERSION_LESS(1, 4, 0)
  void updateInternalDataStructures();
#else
  void updateInternalDataStructures() override;
#endif
#undef KDL_VERSION_LESS

  /**
   * @brief The PR2 inverse kinematics solver
   */
  PR2ArmIK pr2_arm_ik_;

  /**
   * @brief Indicates whether the solver has been successfully initialized
   */
  bool active_;

  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) override;

  int cartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray& q_out, const double& timeout);

  void getSolverInfo(moveit_msgs::KinematicSolverInfo& response)
  {
    pr2_arm_ik_.getSolverInfo(response);
  }

private:
  bool getCount(int& count, const int& max_count, const int& min_count);

  double search_discretization_angle_;

  int free_angle_;

  std::string root_frame_name_;
};

Eigen::Isometry3f KDLToEigenMatrix(const KDL::Frame& p);
double computeEuclideanDistance(const std::vector<double>& array_1, const KDL::JntArray& array_2);
void getKDLChainInfo(const KDL::Chain& chain, moveit_msgs::KinematicSolverInfo& chain_info);

MOVEIT_CLASS_FORWARD(PR2ArmKinematicsPlugin);

class PR2ArmKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /** @class
   *  @brief Plugin-able interface to the PR2 arm kinematics
   */
  PR2ArmKinematicsPlugin();

  /**
   *  @brief Specifies if the node is active or not
   *  @return True if the node is active, false otherwise.
   */
  bool isActive();

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_link_name - the name of the link for which IK is being computed
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;
  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
   * around those specified in the seed state are admissible and need to be searched.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a
   * timeout
   * @param response - the response contains stamped pose information for all the requested links
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

  /**
   * @brief  Initialization function for the kinematics
   * @return True if initialization was successful, false otherwise
   */
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
  bool active_;
  int free_angle_;
  pr2_arm_kinematics::PR2ArmIKSolverPtr pr2_arm_ik_solver_;
  std::string root_name_;
  int dimension_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
  KDL::Chain kdl_chain_;
  moveit_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;

  mutable IKCallbackFn desiredPoseCallback_;
  mutable IKCallbackFn solutionCallback_;

  void desiredPoseCallback(const KDL::JntArray& jnt_array, const KDL::Frame& ik_pose,
                           moveit_msgs::MoveItErrorCodes& error_code) const;

  void jointSolutionCallback(const KDL::JntArray& jnt_array, const KDL::Frame& ik_pose,
                             moveit_msgs::MoveItErrorCodes& error_code) const;
};
}  // namespace pr2_arm_kinematics
