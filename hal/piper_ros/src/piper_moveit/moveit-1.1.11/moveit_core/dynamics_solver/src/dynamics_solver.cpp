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

#include <moveit/dynamics_solver/dynamics_solver.h>

// KDL
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

namespace dynamics_solver
{
namespace
{
inline geometry_msgs::Vector3 transformVector(const Eigen::Isometry3d& transform, const geometry_msgs::Vector3& vector)
{
  Eigen::Vector3d p;
  p = Eigen::Vector3d(vector.x, vector.y, vector.z);
  // transform has to be a valid isometry; the caller is responsible for the check
  p = transform.linear() * p;

  geometry_msgs::Vector3 result;
  result.x = p.x();
  result.y = p.y();
  result.z = p.z();

  return result;
}
}  // namespace

DynamicsSolver::DynamicsSolver(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                               const geometry_msgs::Vector3& gravity_vector)
{
  robot_model_ = robot_model;
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return;

  if (!joint_model_group_->isChain())
  {
    ROS_ERROR_NAMED("dynamics_solver", "Group '%s' is not a chain. Will not initialize dynamics solver",
                    group_name.c_str());
    joint_model_group_ = nullptr;
    return;
  }

  if (!joint_model_group_->getMimicJointModels().empty())
  {
    ROS_ERROR_NAMED("dynamics_solver", "Group '%s' has a mimic joint. Will not initialize dynamics solver",
                    group_name.c_str());
    joint_model_group_ = nullptr;
    return;
  }

  const moveit::core::JointModel* joint = joint_model_group_->getJointRoots()[0];
  if (!joint->getParentLinkModel())
  {
    ROS_ERROR_NAMED("dynamics_solver", "Group '%s' does not have a parent link", group_name.c_str());
    joint_model_group_ = nullptr;
    return;
  }

  base_name_ = joint->getParentLinkModel()->getName();

  tip_name_ = joint_model_group_->getLinkModelNames().back();
  ROS_DEBUG_NAMED("dynamics_solver", "Base name: '%s', Tip name: '%s'", base_name_.c_str(), tip_name_.c_str());

  const urdf::ModelInterfaceSharedPtr urdf_model = robot_model_->getURDF();
  const srdf::ModelConstSharedPtr srdf_model = robot_model_->getSRDF();
  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree))
  {
    ROS_ERROR_NAMED("dynamics_solver", "Could not initialize tree object");
    joint_model_group_ = nullptr;
    return;
  }
  if (!tree.getChain(base_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR_NAMED("dynamics_solver", "Could not initialize chain object");
    joint_model_group_ = nullptr;
    return;
  }
  num_joints_ = kdl_chain_.getNrOfJoints();
  num_segments_ = kdl_chain_.getNrOfSegments();

  state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  state_->setToDefaultValues();

  const std::vector<std::string>& joint_model_names = joint_model_group_->getJointModelNames();
  for (const std::string& joint_model_name : joint_model_names)
  {
    const urdf::Joint* ujoint = urdf_model->getJoint(joint_model_name).get();
    if (ujoint && ujoint->limits)
      max_torques_.push_back(ujoint->limits->effort);
    else
      max_torques_.push_back(0.0);
  }

  KDL::Vector gravity(gravity_vector.x, gravity_vector.y,
                      gravity_vector.z);  // \todo Not sure if KDL expects the negative of this (Sachin)
  gravity_ = gravity.Norm();
  ROS_DEBUG_NAMED("dynamics_solver", "Gravity norm set to %f", gravity_);

  chain_id_solver_ = std::make_shared<KDL::ChainIdSolver_RNE>(kdl_chain_, gravity);
}

bool DynamicsSolver::getTorques(const std::vector<double>& joint_angles, const std::vector<double>& joint_velocities,
                                const std::vector<double>& joint_accelerations,
                                const std::vector<geometry_msgs::Wrench>& wrenches, std::vector<double>& torques) const
{
  if (!joint_model_group_)
  {
    ROS_DEBUG_NAMED("dynamics_solver", "Did not construct DynamicsSolver object properly. "
                                       "Check error logs.");
    return false;
  }
  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Joint angles vector should be size %d", num_joints_);
    return false;
  }
  if (joint_velocities.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Joint velocities vector should be size %d", num_joints_);
    return false;
  }
  if (joint_accelerations.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Joint accelerations vector should be size %d", num_joints_);
    return false;
  }
  if (wrenches.size() != num_segments_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Wrenches vector should be size %d", num_segments_);
    return false;
  }
  if (torques.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Torques vector should be size %d", num_joints_);
    return false;
  }

  KDL::JntArray kdl_angles(num_joints_), kdl_velocities(num_joints_), kdl_accelerations(num_joints_),
      kdl_torques(num_joints_);
  KDL::Wrenches kdl_wrenches(num_segments_);

  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    kdl_angles(i) = joint_angles[i];
    kdl_velocities(i) = joint_velocities[i];
    kdl_accelerations(i) = joint_accelerations[i];
  }

  for (unsigned int i = 0; i < num_segments_; ++i)
  {
    kdl_wrenches[i](0) = wrenches[i].force.x;
    kdl_wrenches[i](1) = wrenches[i].force.y;
    kdl_wrenches[i](2) = wrenches[i].force.z;

    kdl_wrenches[i](3) = wrenches[i].torque.x;
    kdl_wrenches[i](4) = wrenches[i].torque.y;
    kdl_wrenches[i](5) = wrenches[i].torque.z;
  }

  if (chain_id_solver_->CartToJnt(kdl_angles, kdl_velocities, kdl_accelerations, kdl_wrenches, kdl_torques) < 0)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Something went wrong computing torques");
    return false;
  }

  for (unsigned int i = 0; i < num_joints_; ++i)
    torques[i] = kdl_torques(i);

  return true;
}

bool DynamicsSolver::getMaxPayload(const std::vector<double>& joint_angles, double& payload,
                                   unsigned int& joint_saturated) const
{
  if (!joint_model_group_)
  {
    ROS_DEBUG_NAMED("dynamics_solver", "Did not construct DynamicsSolver object properly. "
                                       "Check error logs.");
    return false;
  }
  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Joint angles vector should be size %d", num_joints_);
    return false;
  }
  std::vector<double> joint_velocities(num_joints_, 0.0), joint_accelerations(num_joints_, 0.0);
  std::vector<double> torques(num_joints_, 0.0), zero_torques(num_joints_, 0.0);

  std::vector<geometry_msgs::Wrench> wrenches(num_segments_);
  if (!getTorques(joint_angles, joint_velocities, joint_accelerations, wrenches, zero_torques))
    return false;

  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    if (fabs(zero_torques[i]) >= max_torques_[i])
    {
      payload = 0.0;
      joint_saturated = i;
      return true;
    }
  }

  state_->setJointGroupPositions(joint_model_group_, joint_angles);
  const Eigen::Isometry3d& base_frame = state_->getFrameTransform(base_name_);  // valid isometry by contract
  const Eigen::Isometry3d& tip_frame = state_->getFrameTransform(tip_name_);    // valid isometry by contract
  Eigen::Isometry3d transform = tip_frame.inverse() * base_frame;               // valid isometry by construction
  wrenches.back().force.z = 1.0;
  wrenches.back().force = transformVector(transform, wrenches.back().force);
  wrenches.back().torque = transformVector(transform, wrenches.back().torque);

  ROS_DEBUG_NAMED("dynamics_solver", "New wrench (local frame): %f %f %f", wrenches.back().force.x,
                  wrenches.back().force.y, wrenches.back().force.z);

  if (!getTorques(joint_angles, joint_velocities, joint_accelerations, wrenches, torques))
    return false;

  double min_payload = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    double payload_joint = std::max<double>((max_torques_[i] - zero_torques[i]) / (torques[i] - zero_torques[i]),
                                            (-max_torques_[i] - zero_torques[i]) /
                                                (torques[i] - zero_torques[i]));  // because we set the payload to 1.0
    ROS_DEBUG_NAMED("dynamics_solver", "Joint: %d, Actual Torque: %f, Max Allowed: %f, Gravity: %f", i, torques[i],
                    max_torques_[i], zero_torques[i]);
    ROS_DEBUG_NAMED("dynamics_solver", "Joint: %d, Payload Allowed (N): %f", i, payload_joint);
    if (payload_joint < min_payload)
    {
      min_payload = payload_joint;
      joint_saturated = i;
    }
  }
  payload = min_payload / gravity_;
  ROS_DEBUG_NAMED("dynamics_solver", "Max payload (kg): %f", payload);
  return true;
}

bool DynamicsSolver::getPayloadTorques(const std::vector<double>& joint_angles, double payload,
                                       std::vector<double>& joint_torques) const
{
  if (!joint_model_group_)
  {
    ROS_DEBUG_NAMED("dynamics_solver", "Did not construct DynamicsSolver object properly. "
                                       "Check error logs.");
    return false;
  }
  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Joint angles vector should be size %d", num_joints_);
    return false;
  }
  if (joint_torques.size() != num_joints_)
  {
    ROS_ERROR_NAMED("dynamics_solver", "Joint torques vector should be size %d", num_joints_);
    return false;
  }
  std::vector<double> joint_velocities(num_joints_, 0.0), joint_accelerations(num_joints_, 0.0);
  std::vector<geometry_msgs::Wrench> wrenches(num_segments_);
  state_->setJointGroupPositions(joint_model_group_, joint_angles);

  const Eigen::Isometry3d& base_frame = state_->getFrameTransform(base_name_);  // valid isometry by contract
  const Eigen::Isometry3d& tip_frame = state_->getFrameTransform(tip_name_);    // valid isometry by contract
  Eigen::Isometry3d transform = tip_frame.inverse() * base_frame;               // valid isometry by construction
  wrenches.back().force.z = payload * gravity_;
  wrenches.back().force = transformVector(transform, wrenches.back().force);
  wrenches.back().torque = transformVector(transform, wrenches.back().torque);

  ROS_DEBUG_NAMED("dynamics_solver", "New wrench (local frame): %f %f %f", wrenches.back().force.x,
                  wrenches.back().force.y, wrenches.back().force.z);

  return getTorques(joint_angles, joint_velocities, joint_accelerations, wrenches, joint_torques);
}

const std::vector<double>& DynamicsSolver::getMaxTorques() const
{
  return max_torques_;
}

}  // end of namespace dynamics_solver
