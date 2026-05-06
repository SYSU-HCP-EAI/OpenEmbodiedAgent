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

/* Author: Ioan Sucan, Sachin Chitta */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/plan_execution/plan_representation.h>
#include <moveit_msgs/GripperTranslation.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>
#include <string>
#include <vector>

namespace pick_place
{
MOVEIT_STRUCT_FORWARD(ManipulationPlanSharedData);

struct ManipulationPlanSharedData
{
  ManipulationPlanSharedData()
    : planning_group_(nullptr)
    , end_effector_group_(nullptr)
    , ik_link_(nullptr)
    , max_goal_sampling_attempts_(0)
    , minimize_object_distance_(false)
  {
  }

  const moveit::core::JointModelGroup* planning_group_;
  const moveit::core::JointModelGroup* end_effector_group_;
  const moveit::core::LinkModel* ik_link_;

  unsigned int max_goal_sampling_attempts_;

  std::string planner_id_;

  bool minimize_object_distance_;

  moveit_msgs::Constraints path_constraints_;

  moveit_msgs::AttachedCollisionObject diff_attached_object_;

  ros::WallTime timeout_;
};

MOVEIT_STRUCT_FORWARD(ManipulationPlan);

struct ManipulationPlan
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ManipulationPlan(const ManipulationPlanSharedDataConstPtr& shared_data)
    : shared_data_(shared_data), processing_stage_(0)
  {
  }

  /// Restore this plan to a state that makes it look like it never was processed by the manipulation pipeline
  void clear()
  {
    goal_sampler_.reset();
    trajectories_.clear();
    approach_state_.reset();
    possible_goal_states_.clear();
    processing_stage_ = 0;
  }

  // Shared data between manipulation plans (set at initialization)
  ManipulationPlanSharedDataConstPtr shared_data_;

  // the approach motion towards the goal
  moveit_msgs::GripperTranslation approach_;

  // the retreat motion away from the goal
  moveit_msgs::GripperTranslation retreat_;

  // the kinematic configuration of the end effector when approaching the goal (an open gripper)
  trajectory_msgs::JointTrajectory approach_posture_;

  // the kinematic configuration of the end effector when retreating from the goal (a closed gripper)
  trajectory_msgs::JointTrajectory retreat_posture_;

  // -------------- computed data --------------------------
  geometry_msgs::PoseStamped goal_pose_;
  Eigen::Isometry3d transformed_goal_pose_;

  moveit_msgs::Constraints goal_constraints_;

  // Allows for the sampling of a kineamtic state for a particular group of a robot
  constraint_samplers::ConstraintSamplerPtr goal_sampler_;

  std::vector<moveit::core::RobotStatePtr> possible_goal_states_;

  moveit::core::RobotStatePtr approach_state_;

  // The sequence of trajectories produced for execution
  std::vector<plan_execution::ExecutableTrajectory> trajectories_;

  // An error code reflecting what went wrong (if anything)
  moveit_msgs::MoveItErrorCodes error_code_;

  // The processing stage that was last working on this plan, or was about to work on this plan
  std::size_t processing_stage_;

  // An id for this plan; this is usually the index of the Grasp / PlaceLocation in the input request
  std::size_t id_;
};
}  // namespace pick_place
