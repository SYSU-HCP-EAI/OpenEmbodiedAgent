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

/* Author: Ioan Sucan */

#include <moveit/pick_place/reachable_valid_pose_filter.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <functional>
#include <ros/console.h>

pick_place::ReachableAndValidPoseFilter::ReachableAndValidPoseFilter(
    const planning_scene::PlanningSceneConstPtr& scene,
    const collision_detection::AllowedCollisionMatrixConstPtr& collision_matrix,
    const constraint_samplers::ConstraintSamplerManagerPtr& constraints_sampler_manager)
  : ManipulationStage("reachable & valid pose filter")
  , planning_scene_(scene)
  , collision_matrix_(collision_matrix)
  , constraints_sampler_manager_(constraints_sampler_manager)
{
}

namespace
{
bool isStateCollisionFree(const planning_scene::PlanningScene* planning_scene,
                          const collision_detection::AllowedCollisionMatrix* collision_matrix, bool verbose,
                          const pick_place::ManipulationPlan* manipulation_plan, moveit::core::RobotState* state,
                          const moveit::core::JointModelGroup* group, const double* joint_group_variable_values)
{
  collision_detection::CollisionRequest req;
  req.verbose = verbose;
  req.group_name = manipulation_plan->shared_data_->planning_group_->getName();

  state->setJointGroupPositions(group, joint_group_variable_values);

  if (manipulation_plan->approach_posture_.points.empty())
  {
    state->update();
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, *state, *collision_matrix);
    if (res.collision)
      return false;
  }
  else
    // apply approach posture for the end effector (we always apply it here since it could be the case the sampler
    // changes this posture)
    for (std::size_t i = 0; i < manipulation_plan->approach_posture_.points.size(); ++i)
    {
      state->setVariablePositions(manipulation_plan->approach_posture_.joint_names,
                                  manipulation_plan->approach_posture_.points[i].positions);
      state->update();
      collision_detection::CollisionResult res;
      planning_scene->checkCollision(req, res, *state, *collision_matrix);
      if (res.collision)
        return false;
    }
  return planning_scene->isStateFeasible(*state);
}
}  // namespace

bool pick_place::ReachableAndValidPoseFilter::isEndEffectorFree(const ManipulationPlanPtr& plan,
                                                                moveit::core::RobotState& token_state) const
{
  tf2::fromMsg(plan->goal_pose_.pose, plan->transformed_goal_pose_);
  plan->transformed_goal_pose_ =
      planning_scene_->getFrameTransform(token_state, plan->goal_pose_.header.frame_id) * plan->transformed_goal_pose_;
  token_state.updateStateWithLinkAt(plan->shared_data_->ik_link_, plan->transformed_goal_pose_);
  collision_detection::CollisionRequest req;
  req.verbose = verbose_;
  collision_detection::CollisionResult res;
  req.group_name = plan->shared_data_->end_effector_group_->getName();
  planning_scene_->checkCollision(req, res, token_state, *collision_matrix_);
  return !res.collision;
}

bool pick_place::ReachableAndValidPoseFilter::evaluate(const ManipulationPlanPtr& plan) const
{
  // initialize with scene state
  moveit::core::RobotStatePtr token_state(new moveit::core::RobotState(planning_scene_->getCurrentState()));
  if (isEndEffectorFree(plan, *token_state))
  {
    // update the goal pose message if anything has changed; this is because the name of the frame in the input goal
    // pose
    // can be that of objects in the collision world but most components are unaware of those transforms,
    // so we convert to a frame that is certainly known
    if (!moveit::core::Transforms::sameFrame(planning_scene_->getPlanningFrame(), plan->goal_pose_.header.frame_id))
    {
      plan->goal_pose_.pose = tf2::toMsg(plan->transformed_goal_pose_);
      plan->goal_pose_.header.frame_id = planning_scene_->getPlanningFrame();
    }

    // convert the pose we want to reach to a set of constraints
    plan->goal_constraints_ =
        kinematic_constraints::constructGoalConstraints(plan->shared_data_->ik_link_->getName(), plan->goal_pose_);

    const std::string& planning_group = plan->shared_data_->planning_group_->getName();

    // construct a sampler for the specified constraints; this can end up calling just IK, but it is more general
    // and allows for robot-specific samplers, producing samples that also change the base position if needed, etc
    plan->goal_sampler_ =
        constraints_sampler_manager_->selectSampler(planning_scene_, planning_group, plan->goal_constraints_);
    if (plan->goal_sampler_)
    {
      plan->goal_sampler_->setGroupStateValidityCallback(
          [scene = planning_scene_.get(), acm = collision_matrix_.get(), verbose = verbose_,
           p = plan.get()](moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group,
                           const double* joint_group_variable_values) {
            return isStateCollisionFree(scene, acm, verbose, p, robot_state, joint_group, joint_group_variable_values);
          });
      plan->goal_sampler_->setVerbose(verbose_);
      if (plan->goal_sampler_->sample(*token_state, plan->shared_data_->max_goal_sampling_attempts_))
      {
        plan->possible_goal_states_.push_back(token_state);
        return true;
      }
      else if (verbose_)
        ROS_INFO_NAMED("manipulation", "Sampler failed to produce a state");
    }
    else
      ROS_ERROR_THROTTLE_NAMED(1, "manipulation", "No sampler was constructed");
  }
  plan->error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
  return false;
}
