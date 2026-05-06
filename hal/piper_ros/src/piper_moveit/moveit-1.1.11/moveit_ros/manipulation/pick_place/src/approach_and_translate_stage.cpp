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

#include <moveit/pick_place/pick_place.h>
#include <moveit/pick_place/approach_and_translate_stage.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/console.h>

namespace pick_place
{
ApproachAndTranslateStage::ApproachAndTranslateStage(
    const planning_scene::PlanningSceneConstPtr& scene,
    const collision_detection::AllowedCollisionMatrixConstPtr& collision_matrix)
  : ManipulationStage("approach & translate"), planning_scene_(scene), collision_matrix_(collision_matrix)
{
  max_goal_count_ = GetGlobalPickPlaceParams().max_goal_count_;
  max_fail_ = GetGlobalPickPlaceParams().max_fail_;
  max_step_ = GetGlobalPickPlaceParams().max_step_;
  jump_factor_ = GetGlobalPickPlaceParams().jump_factor_;
}

namespace
{
bool isStateCollisionFree(const planning_scene::PlanningScene* planning_scene,
                          const collision_detection::AllowedCollisionMatrix* collision_matrix, bool verbose,
                          const trajectory_msgs::JointTrajectory* grasp_posture, moveit::core::RobotState* state,
                          const moveit::core::JointModelGroup* group, const double* joint_group_variable_values)
{
  state->setJointGroupPositions(group, joint_group_variable_values);

  collision_detection::CollisionRequest req;
  req.verbose = verbose;
  req.group_name = group->getName();

  if (!grasp_posture->joint_names.empty())
  {
    // apply the grasp posture for the end effector (we always apply it here since it could be the case the sampler
    // changes this posture)
    for (std::size_t i = 0; i < grasp_posture->points.size(); ++i)
    {
      state->setVariablePositions(grasp_posture->joint_names, grasp_posture->points[i].positions);
      collision_detection::CollisionResult res;
      planning_scene->checkCollision(req, res, *state, *collision_matrix);
      if (res.collision)
        return false;
    }
  }
  else
  {
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, *state, *collision_matrix);
    if (res.collision)
      return false;
  }
  return planning_scene->isStateFeasible(*state);
}

bool samplePossibleGoalStates(const ManipulationPlanPtr& plan, const moveit::core::RobotState& reference_state,
                              double min_distance, unsigned int attempts)
{
  // initialize with scene state
  moveit::core::RobotStatePtr token_state(new moveit::core::RobotState(reference_state));
  for (unsigned int j = 0; j < attempts; ++j)
  {
    double min_d = std::numeric_limits<double>::infinity();

    // Samples given the constraints, populating the joint state group
    if (plan->goal_sampler_->sample(*token_state, plan->shared_data_->max_goal_sampling_attempts_))
    {
      // Check if this new sampled state is closest we've found so far
      for (std::size_t i = 0; i < plan->possible_goal_states_.size(); ++i)
      {
        double d = plan->possible_goal_states_[i]->distance(*token_state, plan->shared_data_->planning_group_);
        if (d < min_d)
          min_d = d;
      }
      if (min_d >= min_distance)
      {
        plan->possible_goal_states_.push_back(token_state);
        return true;
      }
    }
  }
  return false;
}

// This function is called during trajectory execution, after the gripper is closed, to attach the currently gripped
// object
bool executeAttachObject(const ManipulationPlanSharedDataConstPtr& shared_plan_data,
                         const trajectory_msgs::JointTrajectory& detach_posture,
                         const plan_execution::ExecutableMotionPlan& motion_plan)
{
  if (shared_plan_data->diff_attached_object_.object.id.empty())
  {
    // the user did not provide an object to attach, so just return successfully
    return true;
  }

  ROS_DEBUG_NAMED("manipulation", "Applying attached object diff to maintained planning scene (attaching/detaching "
                                  "object to end effector)");
  bool ok = false;
  {
    planning_scene_monitor::LockedPlanningSceneRW ps(motion_plan.planning_scene_monitor_);
    moveit_msgs::AttachedCollisionObject msg = shared_plan_data->diff_attached_object_;
    // remember the configuration of the gripper before the grasp;
    // this configuration will be set again when releasing the object
    msg.detach_posture = detach_posture;
    ok = ps->processAttachedCollisionObjectMsg(msg);
  }
  motion_plan.planning_scene_monitor_->triggerSceneUpdateEvent(
      (planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType)(
          planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY +
          planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE));
  return ok;
}

// Add the close end effector trajectory to the overall plan (after the approach trajectory, before the retreat)
void addGripperTrajectory(const ManipulationPlanPtr& plan,
                          const collision_detection::AllowedCollisionMatrixConstPtr& collision_matrix,
                          const std::string& name)
{
  // Check if a "closed" end effector configuration was specified
  if (!plan->retreat_posture_.joint_names.empty())
  {
    moveit::core::RobotStatePtr ee_closed_state(
        new moveit::core::RobotState(plan->trajectories_.back().trajectory_->getLastWayPoint()));

    robot_trajectory::RobotTrajectoryPtr ee_closed_traj(new robot_trajectory::RobotTrajectory(
        ee_closed_state->getRobotModel(), plan->shared_data_->end_effector_group_->getName()));
    ee_closed_traj->setRobotTrajectoryMsg(*ee_closed_state, plan->retreat_posture_);
    // If user has defined a time for it's gripper movement time, don't add the
    // DEFAULT_GRASP_POSTURE_COMPLETION_DURATION
    if (!plan->retreat_posture_.points.empty() &&
        plan->retreat_posture_.points.back().time_from_start > ros::Duration(0.0))
    {
      ee_closed_traj->addPrefixWayPoint(ee_closed_state, 0.0);
    }
    else
    {  // Do what was done before
      ROS_INFO_STREAM("Adding default duration of " << PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION
                                                    << " seconds to the grasp closure time. Assign time_from_start to "
                                                    << "your trajectory to avoid this.");
      ee_closed_traj->addPrefixWayPoint(ee_closed_state, PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION);
    }

    plan_execution::ExecutableTrajectory et(ee_closed_traj, name);

    // Add a callback to attach the object to the EE after closing the gripper
    et.effect_on_success_ = [plan](const plan_execution::ExecutableMotionPlan* motion_plan) {
      return executeAttachObject(plan->shared_data_, plan->approach_posture_, *motion_plan);
    };
    et.allowed_collision_matrix_ = collision_matrix;
    plan->trajectories_.push_back(et);
  }
  else
  {
    ROS_WARN_NAMED("manipulation", "No joint states of grasp postures have been defined in the pick place action.");
  }
}

}  // namespace

bool ApproachAndTranslateStage::evaluate(const ManipulationPlanPtr& plan) const
{
  const moveit::core::JointModelGroup* jmg = plan->shared_data_->planning_group_;
  // compute what the maximum distance reported between any two states in the planning group could be, and keep 1% of
  // that;
  // this is the minimum distance between sampled goal states
  const double min_distance = 0.01 * jmg->getMaximumExtent();

  // convert approach direction and retreat direction to Eigen structures
  Eigen::Vector3d approach_direction, retreat_direction;
  tf2::fromMsg(plan->approach_.direction.vector, approach_direction);
  tf2::fromMsg(plan->retreat_.direction.vector, retreat_direction);

  // if translation vectors are specified in the frame of the ik link name, then we assume the frame is local;
  // otherwise, the frame is global
  bool approach_direction_is_global_frame = !moveit::core::Transforms::sameFrame(
      plan->approach_.direction.header.frame_id, plan->shared_data_->ik_link_->getName());
  bool retreat_direction_is_global_frame = !moveit::core::Transforms::sameFrame(
      plan->retreat_.direction.header.frame_id, plan->shared_data_->ik_link_->getName());

  // transform the input vectors in accordance to frame specified in the header;
  if (approach_direction_is_global_frame)
    // getFrameTransform() returns a valid isometry by contract
    approach_direction =
        planning_scene_->getFrameTransform(plan->approach_.direction.header.frame_id).linear() * approach_direction;
  if (retreat_direction_is_global_frame)
    // getFrameTransform() returns a valid isometry by contract
    retreat_direction =
        planning_scene_->getFrameTransform(plan->retreat_.direction.header.frame_id).linear() * retreat_direction;

  // state validity checking during the approach must ensure that the gripper posture is that for pre-grasping
  moveit::core::GroupStateValidityCallbackFn approach_valid_callback =
      [scene = planning_scene_.get(), acm = collision_matrix_.get(), verbose = this->verbose_,
       approach_posture = &plan->approach_posture_](moveit::core::RobotState* robot_state,
                                                    const moveit::core::JointModelGroup* joint_group,
                                                    const double* joint_group_variable_values) {
        return isStateCollisionFree(scene, acm, verbose, approach_posture, robot_state, joint_group,
                                    joint_group_variable_values);
      };
  plan->goal_sampler_->setVerbose(verbose_);
  std::size_t attempted_possible_goal_states = 0;
  do  // continously sample possible goal states
  {
    for (std::size_t i = attempted_possible_goal_states; i < plan->possible_goal_states_.size() && !signal_stop_;
         ++i, ++attempted_possible_goal_states)
    {
      // if we are trying to get as close as possible to the goal (maximum one meter)
      if (plan->shared_data_->minimize_object_distance_)
      {
        static const double MAX_CLOSE_UP_DIST = 1.0;
        moveit::core::RobotStatePtr close_up_state(new moveit::core::RobotState(*plan->possible_goal_states_[i]));
        std::vector<moveit::core::RobotStatePtr> close_up_states;
        double d_close_up = moveit::core::CartesianInterpolator::computeCartesianPath(
            close_up_state.get(), plan->shared_data_->planning_group_, close_up_states, plan->shared_data_->ik_link_,
            approach_direction, approach_direction_is_global_frame, MAX_CLOSE_UP_DIST,
            moveit::core::MaxEEFStep(max_step_), moveit::core::JumpThreshold(jump_factor_), approach_valid_callback);
        // if progress towards the object was made, update the desired goal state
        if (d_close_up > 0.0 && close_up_states.size() > 1)
          *plan->possible_goal_states_[i] = *close_up_states[close_up_states.size() - 2];
      }

      // try to compute a straight line path that arrives at the goal using the specified approach direction
      moveit::core::RobotStatePtr first_approach_state(new moveit::core::RobotState(*plan->possible_goal_states_[i]));

      std::vector<moveit::core::RobotStatePtr> approach_states;
      double d_approach = moveit::core::CartesianInterpolator::computeCartesianPath(
          first_approach_state.get(), plan->shared_data_->planning_group_, approach_states,
          plan->shared_data_->ik_link_, -approach_direction, approach_direction_is_global_frame,
          plan->approach_.desired_distance, moveit::core::MaxEEFStep(max_step_),
          moveit::core::JumpThreshold(jump_factor_), approach_valid_callback);

      // if we were able to follow the approach direction for sufficient length, try to compute a retreat direction
      if (d_approach > plan->approach_.min_distance && !signal_stop_)
      {
        if (plan->retreat_.desired_distance > 0.0)
        {
          // construct a planning scene that is just a diff on top of our actual planning scene
          planning_scene::PlanningScenePtr planning_scene_after_approach = planning_scene_->diff();

          // assume the current state of the diff world is the one we plan to reach
          planning_scene_after_approach->getCurrentStateNonConst() = *plan->possible_goal_states_[i];

          // apply the difference message to this world that virtually attaches the object we are manipulating
          planning_scene_after_approach->processAttachedCollisionObjectMsg(plan->shared_data_->diff_attached_object_);

          // state validity checking during the retreat after the grasp must ensure the gripper posture is that of the
          // actual grasp
          moveit::core::GroupStateValidityCallbackFn retreat_valid_callback =
              [scene = planning_scene_after_approach.get(), acm = collision_matrix_.get(), verbose = verbose_,
               retreat_posture = &plan->retreat_posture_](moveit::core::RobotState* robot_state,
                                                          const moveit::core::JointModelGroup* joint_group,
                                                          const double* joint_group_variable_values) {
                return isStateCollisionFree(scene, acm, verbose, retreat_posture, robot_state, joint_group,
                                            joint_group_variable_values);
              };

          // try to compute a straight line path that moves from the goal in a desired direction
          moveit::core::RobotStatePtr last_retreat_state(
              new moveit::core::RobotState(planning_scene_after_approach->getCurrentState()));
          std::vector<moveit::core::RobotStatePtr> retreat_states;
          double d_retreat = moveit::core::CartesianInterpolator::computeCartesianPath(
              last_retreat_state.get(), plan->shared_data_->planning_group_, retreat_states,
              plan->shared_data_->ik_link_, retreat_direction, retreat_direction_is_global_frame,
              plan->retreat_.desired_distance, moveit::core::MaxEEFStep(max_step_),
              moveit::core::JumpThreshold(jump_factor_), retreat_valid_callback);

          // if sufficient progress was made in the desired direction, we have a goal state that we can consider for
          // future stages
          if (d_retreat > plan->retreat_.min_distance && !signal_stop_)
          {
            // Create approach trajectory
            std::reverse(approach_states.begin(), approach_states.end());
            robot_trajectory::RobotTrajectoryPtr approach_traj(new robot_trajectory::RobotTrajectory(
                planning_scene_->getRobotModel(), plan->shared_data_->planning_group_->getName()));
            for (const moveit::core::RobotStatePtr& approach_state : approach_states)
              approach_traj->addSuffixWayPoint(approach_state, 0.0);

            // Create retreat trajectory
            robot_trajectory::RobotTrajectoryPtr retreat_traj(new robot_trajectory::RobotTrajectory(
                planning_scene_->getRobotModel(), plan->shared_data_->planning_group_->getName()));
            for (const moveit::core::RobotStatePtr& retreat_state : retreat_states)
              retreat_traj->addSuffixWayPoint(retreat_state, 0.0);

            // Add timestamps to approach|retreat trajectories
            time_param_.computeTimeStamps(*approach_traj);
            time_param_.computeTimeStamps(*retreat_traj);

            // Convert approach trajectory to an executable trajectory
            plan_execution::ExecutableTrajectory et_approach(approach_traj, "approach");
            et_approach.allowed_collision_matrix_ = collision_matrix_;
            plan->trajectories_.push_back(et_approach);

            // Add gripper close trajectory
            addGripperTrajectory(plan, collision_matrix_, "grasp");

            // Convert retreat trajectory to an executable trajectory
            plan_execution::ExecutableTrajectory et_retreat(retreat_traj, "retreat");
            et_retreat.allowed_collision_matrix_ = collision_matrix_;
            plan->trajectories_.push_back(et_retreat);

            plan->approach_state_ = approach_states.front();
            return true;
          }
        }
        else  // No retreat was specified, so package up approach and grip trajectories.
        {
          // Reset the approach_state_ RobotStatePtr so that we can retry computing the cartesian path
          plan->approach_state_.swap(first_approach_state);

          // Create approach trajectory
          std::reverse(approach_states.begin(), approach_states.end());
          robot_trajectory::RobotTrajectoryPtr approach_traj(new robot_trajectory::RobotTrajectory(
              planning_scene_->getRobotModel(), plan->shared_data_->planning_group_->getName()));
          for (const moveit::core::RobotStatePtr& approach_state : approach_states)
            approach_traj->addSuffixWayPoint(approach_state, 0.0);

          // Add timestamps to approach trajectories
          time_param_.computeTimeStamps(*approach_traj);

          // Convert approach trajectory to an executable trajectory
          plan_execution::ExecutableTrajectory et_approach(approach_traj, "approach");
          et_approach.allowed_collision_matrix_ = collision_matrix_;
          plan->trajectories_.push_back(et_approach);

          // Add gripper close trajectory
          addGripperTrajectory(plan, collision_matrix_, "grasp");

          plan->approach_state_ = approach_states.front();

          return true;
        }
      }
    }
  } while (plan->possible_goal_states_.size() < max_goal_count_ && !signal_stop_ &&
           samplePossibleGoalStates(plan, planning_scene_->getCurrentState(), min_distance, max_fail_));

  plan->error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;

  return false;
}
}  // namespace pick_place
