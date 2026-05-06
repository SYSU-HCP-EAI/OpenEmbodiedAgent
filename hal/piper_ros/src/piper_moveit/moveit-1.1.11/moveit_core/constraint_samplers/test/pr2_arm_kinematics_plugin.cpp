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

#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <algorithm>

#include <moveit/robot_model/robot_model.h>
#include "pr2_arm_kinematics_plugin.h"

using namespace KDL;
using namespace std;

namespace pr2_arm_kinematics
{
bool PR2ArmIKSolver::getCount(int& count, const int& max_count, const int& min_count)
{
  if (count > 0)
  {
    if (-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if (count + 1 <= max_count)
    {
      count = count + 1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if (1 - count <= max_count)
    {
      count = 1 - count;
      return true;
    }
    else if (count - 1 >= min_count)
    {
      count = count - 1;
      return true;
    }
    else
      return false;
  }
}

PR2ArmIKSolver::PR2ArmIKSolver(const urdf::ModelInterface& robot_model, const std::string& root_frame_name,
                               const std::string& tip_frame_name, const double& search_discretization_angle,
                               const int& free_angle)
  : ChainIkSolverPos()
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  root_frame_name_ = root_frame_name;
  active_ = pr2_arm_ik_.init(robot_model, root_frame_name, tip_frame_name);
}

void PR2ArmIKSolver::updateInternalDataStructures()
{
  // TODO: move (re)allocation of any internal data structures here
  // to react to changes in chain
}

int PR2ArmIKSolver::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out)
{
  const bool verbose = false;
  Eigen::Isometry3f b = KDLToEigenMatrix(p_in);
  std::vector<std::vector<double> > solution_ik;
  if (free_angle_ == 0)
  {
    if (verbose)
      ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "Solving with %f", q_init(0));
    pr2_arm_ik_.computeIKShoulderPan(b, q_init(0), solution_ik);
  }
  else
  {
    pr2_arm_ik_.computeIKShoulderRoll(b, q_init(2), solution_ik);
  }

  if (solution_ik.empty())
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for (int i = 0; i < (int)solution_ik.size(); i++)
  {
    if (verbose)
    {
      ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "Solution : %d", (int)solution_ik.size());

      for (int j = 0; j < (int)solution_ik[i].size(); j++)
      {
        ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "%d: %f", j, solution_ik[i][j]);
      }
      ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", " ");
      ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", " ");
    }
    double tmp_distance = computeEuclideanDistance(solution_ik[i], q_init);
    if (tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if (min_index > -1)
  {
    q_out.resize((int)solution_ik[min_index].size());
    for (int i = 0; i < (int)solution_ik[min_index].size(); i++)
    {
      q_out(i) = solution_ik[min_index][i];
    }
    return 1;
  }
  else
    return -1;
}

int PR2ArmIKSolver::cartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, KDL::JntArray& q_out,
                                    const double& timeout)
{
  const bool verbose = false;
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments =
      (int)((pr2_arm_ik_.solver_info_.limits[free_angle_].max_position - initial_guess) / search_discretization_angle_);
  int num_negative_increments =
      (int)((initial_guess - pr2_arm_ik_.solver_info_.limits[free_angle_].min_position) / search_discretization_angle_);
  if (verbose)
    ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "%f %f %f %d %d \n\n", initial_guess,
                    pr2_arm_ik_.solver_info_.limits[free_angle_].max_position,
                    pr2_arm_ik_.solver_info_.limits[free_angle_].min_position, num_positive_increments,
                    num_negative_increments);
  while (loop_time < timeout)
  {
    if (CartToJnt(q_init, p_in, q_out) > 0)
      return 1;
    if (!getCount(count, num_positive_increments, -num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    if (verbose)
      ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "%d, %f", count, q_init(free_angle_));
    loop_time = (ros::Time::now() - start_time).toSec();
  }
  if (loop_time >= timeout)
  {
    ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "IK Timed out in %f seconds", timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

bool getKDLChain(const urdf::ModelInterface& model, const std::string& root_name, const std::string& tip_name,
                 KDL::Chain& kdl_chain)
{
  // create robot chain from root to tip
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, kdl_chain))
  {
    ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name << " tip " << tip_name);
    return false;
  }
  return true;
}

Eigen::Isometry3f KDLToEigenMatrix(const KDL::Frame& p)
{
  Eigen::Isometry3f b = Eigen::Isometry3f::Identity();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      b(i, j) = p.M(i, j);
    }
    b(i, 3) = p.p(i);
  }
  return b;
}

double computeEuclideanDistance(const std::vector<double>& array_1, const KDL::JntArray& array_2)
{
  double distance = 0.0;
  for (int i = 0; i < (int)array_1.size(); i++)
  {
    distance += (array_1[i] - array_2(i)) * (array_1[i] - array_2(i));
  }
  return ::sqrt(distance);
}

void getKDLChainInfo(const KDL::Chain& chain, moveit_msgs::KinematicSolverInfo& chain_info)
{
  int i = 0;  // segment number
  while (i < (int)chain.getNrOfSegments())
  {
    chain_info.link_names.push_back(chain.getSegment(i).getName());
    i++;
  }
}

PR2ArmKinematicsPlugin::PR2ArmKinematicsPlugin() : active_(false)
{
}

bool PR2ArmKinematicsPlugin::isActive()
{
  return active_;
}

bool PR2ArmKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                        const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                        double search_discretization)
{
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  const bool verbose = false;
  std::string xml_string;
  dimension_ = 7;

  ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "Loading KDL Tree");
  if (!getKDLChain(*robot_model.getURDF(), base_frame_, tip_frames_[0], kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }
  jnt_to_pose_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  free_angle_ = 2;

  pr2_arm_ik_solver_ = std::make_shared<pr2_arm_kinematics::PR2ArmIKSolver>(
      *robot_model.getURDF(), base_frame_, tip_frames_[0], search_discretization, free_angle_);
  if (!pr2_arm_ik_solver_->active_)
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {
    pr2_arm_ik_solver_->getSolverInfo(ik_solver_info_);
    pr2_arm_kinematics::getKDLChainInfo(kdl_chain_, fk_solver_info_);
    fk_solver_info_.joint_names = ik_solver_info_.joint_names;

    if (verbose)
    {
      for (const std::string& joint_name : ik_solver_info_.joint_names)
      {
        ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "PR2Kinematics:: joint name: %s", joint_name.c_str());
      }
      for (const std::string& link_name : ik_solver_info_.link_names)
      {
        ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "PR2Kinematics can solve IK for %s", link_name.c_str());
      }
      for (const std::string& link_name : fk_solver_info_.link_names)
      {
        ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "PR2Kinematics can solve FK for %s", link_name.c_str());
      }
      ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "PR2KinematicsPlugin::active for %s", group_name.c_str());
    }
    active_ = true;
  }
  return active_;
}

bool PR2ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& /*ik_pose*/,
                                           const std::vector<double>& /*ik_seed_state*/,
                                           std::vector<double>& /*solution*/,
                                           moveit_msgs::MoveItErrorCodes& /*error_code*/,
                                           const kinematics::KinematicsQueryOptions& /*options*/) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, double timeout,
                                              std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& /*options*/) const
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED;
    return false;
  }
  KDL::Frame pose_desired;
  tf2::fromMsg(ik_pose, pose_desired);

  // Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for (int i = 0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->cartToJntSearch(jnt_pos_in, pose_desired, jnt_pos_out, timeout);
  if (ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_valid >= 0)
  {
    solution.resize(dimension_);
    for (int i = 0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG_NAMED("pr2_arm_kinematics_plugin", "An IK solution could not be found");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& /*ik_pose*/,
                                              const std::vector<double>& /*ik_seed_state*/, double /*timeout*/,
                                              const std::vector<double>& /*consistency_limit*/,
                                              std::vector<double>& /*solution*/,
                                              moveit_msgs::MoveItErrorCodes& /*error_code*/,
                                              const kinematics::KinematicsQueryOptions& /*options*/) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& /*ik_pose*/,
                                              const std::vector<double>& /*ik_seed_state*/, double /*timeout*/,
                                              std::vector<double>& /*solution*/,
                                              const IKCallbackFn& /*solution_callback*/,
                                              moveit_msgs::MoveItErrorCodes& /*error_code*/,
                                              const kinematics::KinematicsQueryOptions& /*options*/) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& /*ik_pose*/,
                                              const std::vector<double>& /*ik_seed_state*/, double /*timeout*/,
                                              const std::vector<double>& /*consistency_limit*/,
                                              std::vector<double>& /*solution*/,
                                              const IKCallbackFn& /*solution_callback*/,
                                              moveit_msgs::MoveItErrorCodes& /*error_code*/,
                                              const kinematics::KinematicsQueryOptions& /*options*/) const
{
  return false;
}

bool PR2ArmKinematicsPlugin::getPositionFK(const std::vector<std::string>& /*link_names*/,
                                           const std::vector<double>& /*joint_angles*/,
                                           std::vector<geometry_msgs::Pose>& /*poses*/) const
{
  return false;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getJointNames() const
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getLinkNames() const
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

}  // namespace pr2_arm_kinematics
