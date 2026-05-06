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

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/moveit_error_code.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>
#include <string>

void move_group::MoveGroupCapability::setContext(const MoveGroupContextPtr& context)
{
  context_ = context;
}

void move_group::MoveGroupCapability::convertToMsg(const std::vector<plan_execution::ExecutableTrajectory>& trajectory,
                                                   moveit_msgs::RobotState& first_state_msg,
                                                   std::vector<moveit_msgs::RobotTrajectory>& trajectory_msg) const
{
  if (!trajectory.empty())
  {
    bool first = true;
    trajectory_msg.resize(trajectory.size());
    for (std::size_t i = 0; i < trajectory.size(); ++i)
    {
      if (trajectory[i].trajectory_)
      {
        if (first && !trajectory[i].trajectory_->empty())
        {
          moveit::core::robotStateToRobotStateMsg(trajectory[i].trajectory_->getFirstWayPoint(), first_state_msg);
          first = false;
        }
        trajectory[i].trajectory_->getRobotTrajectoryMsg(trajectory_msg[i]);
      }
    }
  }
}

void move_group::MoveGroupCapability::convertToMsg(const robot_trajectory::RobotTrajectoryPtr& trajectory,
                                                   moveit_msgs::RobotState& first_state_msg,
                                                   moveit_msgs::RobotTrajectory& trajectory_msg) const
{
  if (trajectory && !trajectory->empty())
  {
    moveit::core::robotStateToRobotStateMsg(trajectory->getFirstWayPoint(), first_state_msg);
    trajectory->getRobotTrajectoryMsg(trajectory_msg);
  }
}

void move_group::MoveGroupCapability::convertToMsg(const std::vector<plan_execution::ExecutableTrajectory>& trajectory,
                                                   moveit_msgs::RobotState& first_state_msg,
                                                   moveit_msgs::RobotTrajectory& trajectory_msg) const
{
  if (trajectory.size() > 1)
    ROS_ERROR_STREAM("Internal logic error: trajectory component ignored. !!! THIS IS A SERIOUS ERROR !!!");
  if (!trajectory.empty())
    convertToMsg(trajectory[0].trajectory_, first_state_msg, trajectory_msg);
}

planning_interface::MotionPlanRequest
move_group::MoveGroupCapability::clearRequestStartState(const planning_interface::MotionPlanRequest& request) const
{
  planning_interface::MotionPlanRequest r = request;
  r.start_state = moveit_msgs::RobotState();
  r.start_state.is_diff = true;
  ROS_WARN("Execution of motions should always start at the robot's current state. Ignoring the state supplied as "
           "start state in the motion planning request");
  return r;
}

moveit_msgs::PlanningScene
move_group::MoveGroupCapability::clearSceneRobotState(const moveit_msgs::PlanningScene& scene) const
{
  moveit_msgs::PlanningScene r = scene;
  r.robot_state = moveit_msgs::RobotState();
  r.robot_state.is_diff = true;
  ROS_WARN("Execution of motions should always start at the robot's current state. Ignoring the state supplied as "
           "difference in the planning scene diff");
  return r;
}

std::string move_group::MoveGroupCapability::getActionResultString(const moveit_msgs::MoveItErrorCodes& error_code,
                                                                   bool planned_trajectory_empty, bool plan_only)
{
  switch (error_code.val)
  {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      if (planned_trajectory_empty)
        return "Requested path and goal constraints are already met.";
      else
      {
        if (plan_only)
          return "Motion plan was computed succesfully.";
        else
          return "Solution was found and executed.";
      }
    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
      return "Invalid group in motion plan request";
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      if (planned_trajectory_empty)
        return "No motion plan found. No execution attempted.";
      else
        return "Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
      return "Motion plan was found but it seems to be too costly and looking around did not help.";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "Solution found but the environment changed during execution and the path was aborted";
    default:
      return moveit::core::MoveItErrorCode::toString(error_code);
  }
}

std::string move_group::MoveGroupCapability::stateToStr(MoveGroupState state) const
{
  switch (state)
  {
    case IDLE:
      return "IDLE";
    case PLANNING:
      return "PLANNING";
    case MONITOR:
      return "MONITOR";
    case LOOK:
      return "LOOK";
    default:
      return "UNKNOWN";
  }
}

bool move_group::MoveGroupCapability::performTransform(geometry_msgs::PoseStamped& pose_msg,
                                                       const std::string& target_frame) const
{
  if (!context_ || !context_->planning_scene_monitor_->getTFClient())
    return false;
  if (pose_msg.header.frame_id == target_frame)
    return true;
  if (pose_msg.header.frame_id.empty())
  {
    pose_msg.header.frame_id = target_frame;
    return true;
  }

  try
  {
    geometry_msgs::TransformStamped common_tf = context_->planning_scene_monitor_->getTFClient()->lookupTransform(
        pose_msg.header.frame_id, target_frame, ros::Time(0.0));
    geometry_msgs::PoseStamped pose_msg_in(pose_msg);
    pose_msg_in.header.stamp = common_tf.header.stamp;
    context_->planning_scene_monitor_->getTFClient()->transform(pose_msg_in, pose_msg, target_frame);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("TF Problem: %s", ex.what());
    return false;
  }
  return true;
}

planning_pipeline::PlanningPipelinePtr
move_group::MoveGroupCapability::resolvePlanningPipeline(const std::string& pipeline_id) const
{
  if (pipeline_id.empty())
  {
    // Without specified planning pipeline we use the default
    return context_->planning_pipeline_;
  }
  else
  {
    // Attempt to get the planning pipeline for the specified identifier
    try
    {
      auto pipeline = context_->moveit_cpp_->getPlanningPipelines().at(pipeline_id);
      ROS_INFO_NAMED(getName(), "Using planning pipeline '%s'", pipeline_id.c_str());
      return pipeline;
    }
    catch (const std::out_of_range&)
    {
      ROS_WARN_NAMED(getName(), "Couldn't find requested planning pipeline '%s'", pipeline_id.c_str());
    }
  }

  return planning_pipeline::PlanningPipelinePtr();
}
