/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Sachin Chitta
 *********************************************************************/

// // ROS msgs
#include <moveit/kinematics_constraint_aware/kinematics_constraint_aware.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <Eigen/Geometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <functional>

namespace kinematics_constraint_aware
{
KinematicsConstraintAware::KinematicsConstraintAware(const moveit::core::RobotModelConstPtr& kinematic_model,
                                                     const std::string& group_name)
{
  if (!kinematic_model->hasJointModelGroup(group_name))
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware", "The group %s does not exist", group_name.c_str());
    joint_model_group_ = NULL;
    return;
  }
  kinematic_model_ = kinematic_model;
  group_name_ = group_name;
  joint_model_group_ = kinematic_model_->getJointModelGroup(group_name);
  if (joint_model_group_->getSolverInstance())
  {
    has_sub_groups_ = false;
    sub_groups_names_.push_back(group_name_);
  }
  else
  {
    ROS_DEBUG_NAMED("kinematics_constraint_aware", "No kinematics solver instance defined for group %s",
                    group_name.c_str());
    bool is_solvable_group = true;
    if (!(joint_model_group_->getSubgroupNames().empty()))
    {
      const std::vector<std::string> sub_groups_names = joint_model_group_->getSubgroupNames();
      for (std::size_t i = 0; i < sub_groups_names.size(); ++i)
      {
        if (!kinematic_model_->getJointModelGroup(sub_groups_names[i])->getSolverInstance())
        {
          is_solvable_group = false;
          break;
        }
      }
      if (is_solvable_group)
      {
        ROS_DEBUG_NAMED("kinematics_constraint_aware", "Group %s is a group for which we can solve IK",
                        joint_model_group_->getName().c_str());
        sub_groups_names_ = sub_groups_names;
      }
      else
      {
        joint_model_group_ = NULL;
        return;
      }
    }
    else
    {
      joint_model_group_ = NULL;
      ROS_INFO_NAMED("kinematics_constraint_aware", "No solver allocated for group %s", group_name.c_str());
    }
    has_sub_groups_ = true;
  }
  ik_attempts_ = 10;
}

bool KinematicsConstraintAware::getIK(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                      const kinematics_constraint_aware::KinematicsRequest& request,
                                      kinematics_constraint_aware::KinematicsResponse& response) const
{
  if (!joint_model_group_)
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware", "This solver has not been constructed properly");
    return false;
  }

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware", "Planning scene must be allocated");
    return false;
  }

  if (!response.solution_)
  {
    response.solution_ = std::make_shared<moveit::core::RobotState>(planning_scene->getCurrentState());
  }

  ros::WallTime start_time = ros::WallTime::now();
  if (request.group_name_ != group_name_)
  {
    response.error_code_.val = response.error_code_.INVALID_GROUP_NAME;
    return false;
  }

  // Setup the seed and the values for all other joints in the robot
  moveit::core::RobotState kinematic_state = *request.robot_state_;
  std::vector<std::string> ik_link_names = request.ik_link_names_;

  // Transform request to tip frame if necessary
  if (!request.ik_link_names_.empty())
  {
    for (std::size_t i = 0; i < request.pose_stamped_vector_.size(); ++i)
    {
      geometry_msgs::PoseStamped tmp_pose = request.pose_stamped_vector_[i];
      // The assumption is that this new link is rigidly attached to the tip link for the group
      if (!kinematic_model_->getJointModelGroup(sub_groups_names_[i])->hasLinkModel(request.ik_link_names_[i]) &&
          kinematic_model_->getJointModelGroup(sub_groups_names_[i])->isLinkUpdated(request.ik_link_names_[i]))
      {
        tmp_pose.pose = getTipFramePose(planning_scene, kinematic_state, request.pose_stamped_vector_[i].pose,
                                        request.ik_link_names_[i], i);
        ik_link_names[i] =
            kinematic_model_->getJointModelGroup(sub_groups_names_[i])->getSolverInstance()->getTipFrame();
      }
      else if (!kinematic_model_->getJointModelGroup(sub_groups_names_[i])->canSetStateFromIK(request.ik_link_names_[i]))
      {
        ROS_ERROR_NAMED("kinematics_constraint_aware", "Could not find IK solver for link %s for group %s",
                        request.ik_link_names_[i].c_str(), sub_groups_names_[i].c_str());
        return false;
      }
    }
  }

  // Transform the requests to the base frame of the kinematic model
  EigenSTL::vector_Isometry3d goals =
      transformPoses(planning_scene, kinematic_state, request.pose_stamped_vector_, kinematic_model_->getModelFrame());

  moveit::core::StateValidityCallbackFn constraint_callback_fn = [this, &planning_scene, &request,
                                                                  &response](moveit::core::JointStateGroup* jsg,
                                                                             const std::vector<double>& jg_values) {
    validityCallbackFn(planning_scene, request, response, jsg, jg_values);
  };

  bool result = false;
  if (has_sub_groups_)
  {
    result = kinematic_state.getJointStateGroup(group_name_)
                 ->setFromIK(goals, ik_link_names, ik_attempts_, request.timeout_.toSec(), constraint_callback_fn);
  }
  else
  {
    result =
        ik_link_names.empty() ?
            kinematic_state.getJointStateGroup(group_name_)
                ->setFromIK(goals[0], ik_attempts_, request.timeout_.toSec(), constraint_callback_fn) :
            kinematic_state.getJointStateGroup(group_name_)
                ->setFromIK(goals[0], ik_link_names[0], ik_attempts_, request.timeout_.toSec(), constraint_callback_fn);
  }

  if (result)
  {
    std::vector<double> solution_values;
    kinematic_state.getJointStateGroup(group_name_)->getVariableValues(solution_values);
    response.solution_->getJointStateGroup(group_name_)->setVariableValues(solution_values);
    response.error_code_.val = response.error_code_.SUCCESS;
  }

  if (response.error_code_.val == 0)
  {
    response.error_code_.val = response.error_code_.NO_IK_SOLUTION;
  }
  return result;
}

bool KinematicsConstraintAware::validityCallbackFn(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                   const kinematics_constraint_aware::KinematicsRequest& request,
                                                   kinematics_constraint_aware::KinematicsResponse& response,
                                                   moveit::core::JointStateGroup* joint_state_group,
                                                   const std::vector<double>& joint_group_variable_values) const
{
  joint_state_group->setVariableValues(joint_group_variable_values);

  // Now check for collisions
  if (request.check_for_collisions_)
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.group_name = request.group_name_;
    planning_scene->checkCollision(collision_request, collision_result, *joint_state_group->getRobotState());
    if (collision_result.collision)
    {
      ROS_DEBUG_NAMED("kinematics_constraint_aware", "IK solution is in collision");
      response.error_code_.val = response.error_code_.GOAL_IN_COLLISION;
      return false;
    }
  }

  // Now check for constraints
  if (request.constraints_)
  {
    kinematic_constraints::ConstraintEvaluationResult constraint_result;
    constraint_result =
        request.constraints_->decide(*joint_state_group->getRobotState(), response.constraint_eval_results_);
    if (!constraint_result.satisfied)
    {
      ROS_DEBUG_NAMED("kinematics_constraint_aware", "IK solution violates constraints");
      response.error_code_.val = response.error_code_.GOAL_VIOLATES_PATH_CONSTRAINTS;
      return false;
    }
  }

  // Now check for user specified constraints
  if (request.constraint_callback_)
  {
    if (!request.constraint_callback_(joint_state_group, joint_group_variable_values))
    {
      ROS_DEBUG_NAMED("kinematics_constraint_aware", "IK solution violates user specified constraints");
      response.error_code_.val = response.error_code_.GOAL_VIOLATES_PATH_CONSTRAINTS;
      return false;
    }
  }

  return true;
}

bool KinematicsConstraintAware::getIK(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                      const moveit_msgs::GetConstraintAwarePositionIK::Request& request,
                                      moveit_msgs::GetConstraintAwarePositionIK::Response& response) const
{
  if (!joint_model_group_)
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware", "This solver has not been constructed properly");
    return false;
  }

  if (!planning_scene)
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware", "Planning scene must be allocated");
    return false;
  }

  kinematics_constraint_aware::KinematicsRequest kinematics_request;
  kinematics_constraint_aware::KinematicsResponse kinematics_response;

  if (!convertServiceRequest(planning_scene, request, kinematics_request, kinematics_response))
  {
    response.error_code = kinematics_response.error_code_;
    return false;
  }

  bool result = getIK(planning_scene, kinematics_request, kinematics_response);
  response.error_code = kinematics_response.error_code_;
  kinematics_response.solution_->getStateValues(response.solution.joint_state);
  return result;
}

bool KinematicsConstraintAware::convertServiceRequest(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit_msgs::GetConstraintAwarePositionIK::Request& request,
    kinematics_constraint_aware::KinematicsRequest& kinematics_request,
    kinematics_constraint_aware::KinematicsResponse& kinematics_response) const
{
  if (request.ik_request.group_name != group_name_)
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware", "This kinematics solver does not support requests for group: %s",
                    request.ik_request.group_name.c_str());
    kinematics_response.error_code_.val = kinematics_response.error_code_.INVALID_GROUP_NAME;
    return false;
  }

  if (!request.ik_request.pose_stamped_vector.empty() &&
      request.ik_request.pose_stamped_vector.size() != sub_groups_names_.size())
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware",
                    "Number of poses in request: %d must match number of sub groups %d in this group",
                    request.ik_request.pose_stamped_vector.size(), sub_groups_names_.size());
    kinematics_response.error_code_.val = kinematics_response.error_code_.INVALID_GROUP_NAME;
    return false;
  }

  if (!request.ik_request.ik_link_names.empty() && request.ik_request.ik_link_names.size() != sub_groups_names_.size())
  {
    ROS_ERROR_NAMED("kinematics_constraint_aware",
                    "Number of ik_link_names in request: "
                    "%d must match number of sub groups %d in this group or must be zero",
                    request.ik_request.ik_link_names.size(), sub_groups_names_.size());
    kinematics_response.error_code_.val = kinematics_response.error_code_.INVALID_GROUP_NAME;
    return false;
  }

  if (request.ik_request.ik_link_names.empty() && request.ik_request.ik_link_name != "")
    kinematics_request.ik_link_names_.push_back(request.ik_request.ik_link_name);
  else
    kinematics_request.ik_link_names_ = request.ik_request.ik_link_names;

  if (request.ik_request.pose_stamped_vector.empty())
    kinematics_request.pose_stamped_vector_.push_back(request.ik_request.pose_stamped);
  else
    kinematics_request.pose_stamped_vector_ = request.ik_request.pose_stamped_vector;

  kinematics_request.robot_state_ = std::make_shared<moveit::core::RobotState>(planning_scene->getCurrentState());
  kinematics_request.robot_state_->setStateValues(request.ik_request.robot_state.joint_state);
  kinematics_request.constraints_ = std::make_shared<kinematic_constraints::KinematicConstraintSet>(
      kinematic_model_, planning_scene->getTransforms());
  kinematics_request.constraints_->add(request.constraints);
  kinematics_request.timeout_ = request.ik_request.timeout;
  kinematics_request.group_name_ = request.ik_request.group_name;
  kinematics_request.check_for_collisions_ = true;

  kinematics_response.solution_ = std::make_shared<moveit::core::RobotState>(planning_scene->getCurrentState());

  return true;
}

EigenSTL::vector_Isometry3d KinematicsConstraintAware::transformPoses(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit::core::RobotState& kinematic_state,
    const std::vector<geometry_msgs::PoseStamped>& poses, const std::string& target_frame) const
{
  Eigen::Isometry3d eigen_pose, eigen_pose_2;
  EigenSTL::vector_Isometry3d result(poses.size());
  bool target_frame_is_root_frame = (target_frame == kinematic_state.getRobotModel()->getModelFrame());
  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    geometry_msgs::Pose pose = poses[i].pose;
    tf2::fromMsg(pose, eigen_pose_2);
    planning_scene->getTransforms()->transformPose(kinematic_state, poses[i].header.frame_id, eigen_pose_2, eigen_pose);
    if (!target_frame_is_root_frame)
    {
      eigen_pose_2 = planning_scene->getTransforms()->getTransform(kinematic_state, target_frame);
      eigen_pose = eigen_pose_2.inverse() * eigen_pose;
    }
    result[i] = eigen_pose;
  }
  return result;
}

geometry_msgs::Pose KinematicsConstraintAware::getTipFramePose(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit::core::RobotState& kinematic_state,
    const geometry_msgs::Pose& pose, const std::string& link_name, unsigned int sub_group_index) const
{
  geometry_msgs::Pose result;
  Eigen::Isometry3d eigen_pose_in, eigen_pose_link, eigen_pose_tip;
  std::string tip_name =
      kinematic_model_->getJointModelGroup(sub_groups_names_[sub_group_index])->getSolverInstance()->getTipFrame();
  tf2::fromMsg(pose, eigen_pose_in);
  eigen_pose_link = planning_scene->getTransforms()->getTransform(kinematic_state, link_name);
  eigen_pose_tip = planning_scene->getTransforms()->getTransform(kinematic_state, tip_name);
  eigen_pose_in = eigen_pose_in * (eigen_pose_link.inverse() * eigen_pose_tip);
  result = tf2::toMsg(eigen_pose_in, result);
  return result;
}
}  // namespace kinematics_constraint_aware
