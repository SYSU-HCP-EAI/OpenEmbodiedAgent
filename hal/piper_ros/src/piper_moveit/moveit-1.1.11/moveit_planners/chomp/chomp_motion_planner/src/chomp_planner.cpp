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

/* Author: E. Gil Jones */

#include <ros/ros.h>
#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_optimizer.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace chomp
{
bool ChompPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const planning_interface::MotionPlanRequest& req, const ChompParameters& params,
                         planning_interface::MotionPlanDetailedResponse& res) const
{
  ros::WallTime start_time = ros::WallTime::now();
  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "No planning scene initialized.");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  // get the specified start state
  moveit::core::RobotState start_state = planning_scene->getCurrentState();
  moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

  if (!start_state.satisfiesBounds())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Start state violates joint limits");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  ChompTrajectory trajectory(planning_scene->getRobotModel(), 3.0, .03, req.group_name);
  robotStateToArray(start_state, req.group_name, trajectory.getTrajectoryPoint(0));

  if (req.goal_constraints.size() != 1)
  {
    ROS_ERROR_NAMED("chomp_planner", "Expecting exactly one goal constraint, got: %zd", req.goal_constraints.size());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  if (req.goal_constraints[0].joint_constraints.empty() || !req.goal_constraints[0].position_constraints.empty() ||
      !req.goal_constraints[0].orientation_constraints.empty())
  {
    ROS_ERROR_STREAM("Only joint-space goals are supported");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  const size_t goal_index = trajectory.getNumPoints() - 1;
  moveit::core::RobotState goal_state(start_state);
  for (const moveit_msgs::JointConstraint& joint_constraint : req.goal_constraints[0].joint_constraints)
    goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
  if (!goal_state.satisfiesBounds())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Goal state violates joint limits");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }
  robotStateToArray(goal_state, req.group_name, trajectory.getTrajectoryPoint(goal_index));

  const moveit::core::JointModelGroup* model_group =
      planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (size_t i = 0; i < model_group->getActiveJointModels().size(); i++)
  {
    const moveit::core::JointModel* model = model_group->getActiveJointModels()[i];
    const moveit::core::RevoluteJointModel* revolute_joint =
        dynamic_cast<const moveit::core::RevoluteJointModel*>(model);

    if (revolute_joint != nullptr)
    {
      if (revolute_joint->isContinuous())
      {
        double start = (trajectory)(0, i);
        double end = (trajectory)(goal_index, i);
        ROS_INFO_STREAM("Start is " << start << " end " << end << " short " << shortestAngularDistance(start, end));
        (trajectory)(goal_index, i) = start + shortestAngularDistance(start, end);
      }
    }
  }

  // fill in an initial trajectory based on user choice from the chomp_config.yaml file
  if (params.trajectory_initialization_method_.compare("quintic-spline") == 0)
    trajectory.fillInMinJerk();
  else if (params.trajectory_initialization_method_.compare("linear") == 0)
    trajectory.fillInLinearInterpolation();
  else if (params.trajectory_initialization_method_.compare("cubic") == 0)
    trajectory.fillInCubicInterpolation();
  else if (params.trajectory_initialization_method_.compare("fillTrajectory") == 0)
  {
    if (res.trajectory_.empty())
    {
      ROS_ERROR_STREAM_NAMED("chomp_planner", "No input trajectory specified");
      return false;
    }
    else if (!(trajectory.fillInFromTrajectory(*res.trajectory_[0])))
    {
      ROS_ERROR_STREAM_NAMED("chomp_planner", "Input trajectory has less than 2 points, "
                                              "trajectory must contain at least start and goal state");
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "invalid interpolation method specified in the chomp_planner file");
    return false;
  }

  ROS_INFO_NAMED("chomp_planner", "CHOMP trajectory initialized using method: %s ",
                 (params.trajectory_initialization_method_).c_str());

  // optimize!
  ros::WallTime create_time = ros::WallTime::now();

  int replan_count = 0;
  bool replan_flag = false;
  double org_learning_rate = 0.04, org_ridge_factor = 0.0, org_planning_time_limit = 10;
  int org_max_iterations = 200;

  // storing the initial chomp parameters values
  org_learning_rate = params.learning_rate_;
  org_ridge_factor = params.ridge_factor_;
  org_planning_time_limit = params.planning_time_limit_;
  org_max_iterations = params.max_iterations_;

  std::unique_ptr<ChompOptimizer> optimizer;

  // create a non_const_params variable which stores the non constant version of the const params variable
  ChompParameters params_nonconst = params;

  // while loop for replanning (recovery behaviour) if collision free optimized solution not found
  while (true)
  {
    if (replan_flag)
    {
      // increase learning rate in hope to find a successful path; increase ridge factor to avoid obstacles; add 5
      // additional secs in hope to find a solution; increase maximum iterations
      params_nonconst.setRecoveryParams(params_nonconst.learning_rate_ + 0.02, params_nonconst.ridge_factor_ + 0.002,
                                        params_nonconst.planning_time_limit_ + 5, params_nonconst.max_iterations_ + 50);
    }

    // initialize a ChompOptimizer object to load up the optimizer with default parameters or with updated parameters in
    // case of a recovery behaviour
    optimizer =
        std::make_unique<ChompOptimizer>(&trajectory, planning_scene, req.group_name, &params_nonconst, start_state);
    if (!optimizer->isInitialized())
    {
      ROS_ERROR_STREAM_NAMED("chomp_planner", "Could not initialize optimizer");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    ROS_DEBUG_NAMED("chomp_planner", "Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());

    bool optimization_result = optimizer->optimize();

    // replan with updated parameters if no solution is found
    if (params_nonconst.enable_failure_recovery_)
    {
      ROS_INFO_NAMED("chomp_planner",
                     "Planned with Chomp Parameters (learning_rate, ridge_factor, "
                     "planning_time_limit, max_iterations), attempt: # %d ",
                     (replan_count + 1));
      ROS_INFO_NAMED("chomp_planner", "Learning rate: %f ridge factor: %f planning time limit: %f max_iterations %d ",
                     params_nonconst.learning_rate_, params_nonconst.ridge_factor_,
                     params_nonconst.planning_time_limit_, params_nonconst.max_iterations_);

      if (!optimization_result && replan_count < params_nonconst.max_recovery_attempts_)
      {
        replan_count++;
        replan_flag = true;
      }
      else
      {
        break;
      }
    }
    else
      break;
  }  // end of while loop

  // resetting the CHOMP Parameters to the original values after a successful plan
  params_nonconst.setRecoveryParams(org_learning_rate, org_ridge_factor, org_planning_time_limit, org_max_iterations);

  ROS_DEBUG_NAMED("chomp_planner", "Optimization actually took %f sec to run",
                  (ros::WallTime::now() - create_time).toSec());
  create_time = ros::WallTime::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  ROS_DEBUG_NAMED("chomp_planner", "Output trajectory has %zd joints", trajectory.getNumJoints());

  auto result = std::make_shared<robot_trajectory::RobotTrajectory>(planning_scene->getRobotModel(), req.group_name);
  // fill in the entire trajectory
  for (size_t i = 0; i < trajectory.getNumPoints(); i++)
  {
    const Eigen::MatrixXd::RowXpr source = trajectory.getTrajectoryPoint(i);
    auto state = std::make_shared<moveit::core::RobotState>(start_state);
    size_t joint_index = 0;
    for (const moveit::core::JointModel* jm : result->getGroup()->getActiveJointModels())
    {
      assert(jm->getVariableCount() == 1);
      state->setVariablePosition(jm->getFirstVariableIndex(), source[joint_index++]);
    }
    result->addSuffixWayPoint(state, 0.0);
  }

  res.trajectory_.resize(1);
  res.trajectory_[0] = result;
  res.description_.resize(1);
  res.description_[0] = "plan";

  ROS_DEBUG_NAMED("chomp_planner", "Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_DEBUG_NAMED("chomp_planner", "Serviced planning request in %f wall-seconds",
                  (ros::WallTime::now() - start_time).toSec());

  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time_.resize(1);
  res.processing_time_[0] = (ros::WallTime::now() - start_time).toSec();

  // report planning failure if path has collisions
  if (not optimizer->isCollisionFree())
  {
    ROS_ERROR_STREAM_NAMED("chomp_planner", "Motion plan is invalid.");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // check that final state is within goal tolerances
  kinematic_constraints::JointConstraint jc(planning_scene->getRobotModel());
  const moveit::core::RobotState& last_state = result->getLastWayPoint();
  for (const moveit_msgs::JointConstraint& constraint : req.goal_constraints[0].joint_constraints)
  {
    if (!jc.configure(constraint) || !jc.decide(last_state).satisfied)
    {
      ROS_ERROR_STREAM_NAMED("chomp_planner", "Goal constraints are violated: " << constraint.joint_name);
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
      return false;
    }
  }

  return true;
}
}  // namespace chomp
