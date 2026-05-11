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

#pragma once

// System
#include <boost/function.hpp>

// ROS msgs
#include <moveit_msgs/GetConstraintAwarePositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotState.h>

// Plugin
#include <moveit/kinematics_base/kinematics_base.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematics_constraint_aware/kinematics_request_response.h>

namespace kinematics_constraint_aware
{
class KinematicsConstraintAware;
typedef std::shared_ptr<KinematicsConstraintAware> KinematicsConstraintAwarePtr;
typedef std::shared_ptr<const KinematicsConstraintAware> KinematicsConstraintAwareConstPtr;

/**
 * @class A kinematics solver that can be used with multiple arms
 */
class KinematicsConstraintAware
{
public:
  /** @brief Default constructor
   * @param kinematic_model An instance of a kinematic model
   * @param group_name The name of the group to configure this solver for
   * @return False if any error occurs
   */
  KinematicsConstraintAware(const moveit::core::RobotModelConstPtr& kinematic_model, const std::string& group_name);

  /** @brief Solve the planning problem
   * @param planning_scene A const reference to the planning scene
   * @param request A const reference to the kinematics request
   * @param response The solution (if it exists)
   * @return False if group_name is invalid or ik fails
   */
  bool getIK(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const kinematics_constraint_aware::KinematicsRequest& request,
             kinematics_constraint_aware::KinematicsResponse& response) const;

  /** @brief Solve the planning problem
   * @param planning_scene A const reference to the planning scene
   * @param request A const reference to the kinematics request
   * @param response The solution (if it exists)
   * @return False if group_name is invalid or ik fails
   */
  bool getIK(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetConstraintAwarePositionIK::Request& request,
             moveit_msgs::GetConstraintAwarePositionIK::Response& response) const;

  const std::string& getGroupName() const
  {
    return group_name_;
  }

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return kinematic_model_;
  }

private:
  EigenSTL::vector_Isometry3d transformPoses(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                             const moveit::core::RobotState& kinematic_state,
                                             const std::vector<geometry_msgs::PoseStamped>& poses,
                                             const std::string& target_frame) const;

  bool convertServiceRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             const moveit_msgs::GetConstraintAwarePositionIK::Request& request,
                             kinematics_constraint_aware::KinematicsRequest& kinematics_request,
                             kinematics_constraint_aware::KinematicsResponse& kinematics_response) const;

  geometry_msgs::Pose getTipFramePose(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                      const moveit::core::RobotState& kinematic_state, const geometry_msgs::Pose& pose,
                                      const std::string& link_name, unsigned int sub_group_index) const;

  bool validityCallbackFn(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const kinematics_constraint_aware::KinematicsRequest& request,
                          kinematics_constraint_aware::KinematicsResponse& response,
                          moveit::core::JointStateGroup* joint_state_group,
                          const std::vector<double>& joint_group_variable_values) const;

  std::vector<std::string> sub_groups_names_;

  moveit::core::RobotModelConstPtr kinematic_model_;

  const moveit::core::JointModelGroup* joint_model_group_;

  std::string group_name_;

  bool has_sub_groups_;

  unsigned int ik_attempts_;
};
}  // namespace kinematics_constraint_aware
