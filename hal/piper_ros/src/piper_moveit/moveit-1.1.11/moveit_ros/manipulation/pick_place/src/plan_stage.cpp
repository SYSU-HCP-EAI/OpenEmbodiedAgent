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
#include <moveit/pick_place/plan_stage.h>
#include <moveit/kinematic_constraints/utils.h>
#include <ros/console.h>

namespace pick_place
{
PlanStage::PlanStage(const planning_scene::PlanningSceneConstPtr& scene,
                     const planning_pipeline::PlanningPipelinePtr& planning_pipeline)
  : ManipulationStage("plan"), planning_scene_(scene), planning_pipeline_(planning_pipeline)
{
}

void PlanStage::signalStop()
{
  ManipulationStage::signalStop();
  planning_pipeline_->terminate();
}

// Plan the arm movement to the approach location
bool PlanStage::evaluate(const ManipulationPlanPtr& plan) const
{
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = plan->shared_data_->planning_group_->getName();
  req.num_planning_attempts = 1;
  req.allowed_planning_time = (plan->shared_data_->timeout_ - ros::WallTime::now()).toSec();
  req.path_constraints = plan->shared_data_->path_constraints_;
  req.planner_id = plan->shared_data_->planner_id_;
  req.start_state.is_diff = true;

  req.goal_constraints.resize(1, kinematic_constraints::constructGoalConstraints(*plan->approach_state_,
                                                                                 plan->shared_data_->planning_group_));
  unsigned int attempts = 0;
  do  // give the planner two chances
  {
    attempts++;
    if (!signal_stop_ && planning_pipeline_->generatePlan(planning_scene_, req, res) &&
        res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS && res.trajectory_ && !res.trajectory_->empty())
    {
      // We have a valid motion plan, now apply pre-approach end effector posture (open gripper) if applicable
      if (!plan->approach_posture_.joint_names.empty())
      {
        moveit::core::RobotStatePtr pre_approach_state(new moveit::core::RobotState(res.trajectory_->getLastWayPoint()));
        robot_trajectory::RobotTrajectoryPtr pre_approach_traj(new robot_trajectory::RobotTrajectory(
            pre_approach_state->getRobotModel(), plan->shared_data_->end_effector_group_->getName()));
        pre_approach_traj->setRobotTrajectoryMsg(*pre_approach_state, plan->approach_posture_);

        // Apply the open gripper state to the waypoint
        // If user has defined a time for it's gripper movement time, don't add the
        // DEFAULT_GRASP_POSTURE_COMPLETION_DURATION
        if (!plan->approach_posture_.points.empty() &&
            plan->approach_posture_.points.back().time_from_start > ros::Duration(0.0))
        {
          pre_approach_traj->addPrefixWayPoint(pre_approach_state, 0.0);
        }
        else
        {  // Do what was done before
          ROS_INFO_STREAM("Adding default duration of " << PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION
                                                        << " seconds to the grasp closure time. Assign time_from_start "
                                                        << "to your trajectory to avoid this.");
          pre_approach_traj->addPrefixWayPoint(pre_approach_state, PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION);
        }

        // Add the open gripper trajectory to the plan
        plan_execution::ExecutableTrajectory et(pre_approach_traj, "pre_grasp");
        plan->trajectories_.insert(plan->trajectories_.begin(), et);
      }

      // Add the pre-approach trajectory to the plan
      plan_execution::ExecutableTrajectory et(res.trajectory_, name_);
      plan->trajectories_.insert(plan->trajectories_.begin(), et);

      plan->error_code_ = res.error_code_;

      return true;
    }
    else
      plan->error_code_ = res.error_code_;
  }
  // if the planner reported an invalid plan, give it a second chance
  while (!signal_stop_ && plan->error_code_.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN && attempts < 2);

  return false;
}
}  // namespace pick_place
