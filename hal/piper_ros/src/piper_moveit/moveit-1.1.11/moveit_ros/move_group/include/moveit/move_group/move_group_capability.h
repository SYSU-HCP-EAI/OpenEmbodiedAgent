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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/plan_execution/plan_representation.h>
#include <moveit/move_group/move_group_context.h>

namespace move_group
{
enum MoveGroupState
{
  IDLE,
  PLANNING,
  MONITOR,
  LOOK
};

MOVEIT_CLASS_FORWARD(MoveGroupCapability);  // Defines MoveGroupCapabilityPtr, ConstPtr, WeakPtr... etc

class MoveGroupCapability
{
public:
  MoveGroupCapability(const std::string& capability_name) : node_handle_("~"), capability_name_(capability_name)
  {
  }

  virtual ~MoveGroupCapability()
  {
  }

  void setContext(const MoveGroupContextPtr& context);

  virtual void initialize() = 0;

  const std::string& getName() const
  {
    return capability_name_;
  }

protected:
  std::string getActionResultString(const moveit_msgs::MoveItErrorCodes& error_code, bool planned_trajectory_empty,
                                    bool plan_only);
  std::string stateToStr(MoveGroupState state) const;

  void convertToMsg(const std::vector<plan_execution::ExecutableTrajectory>& trajectory,
                    moveit_msgs::RobotState& first_state_msg,
                    std::vector<moveit_msgs::RobotTrajectory>& trajectory_msg) const;
  void convertToMsg(const robot_trajectory::RobotTrajectoryPtr& trajectory, moveit_msgs::RobotState& first_state_msg,
                    moveit_msgs::RobotTrajectory& trajectory_msg) const;
  void convertToMsg(const std::vector<plan_execution::ExecutableTrajectory>& trajectory,
                    moveit_msgs::RobotState& first_state_msg, moveit_msgs::RobotTrajectory& trajectory_msg) const;

  planning_interface::MotionPlanRequest
  clearRequestStartState(const planning_interface::MotionPlanRequest& request) const;
  moveit_msgs::PlanningScene clearSceneRobotState(const moveit_msgs::PlanningScene& scene) const;
  bool performTransform(geometry_msgs::PoseStamped& pose_msg, const std::string& target_frame) const;

  planning_pipeline::PlanningPipelinePtr resolvePlanningPipeline(const std::string& pipeline_id) const;

  ros::NodeHandle root_node_handle_;
  ros::NodeHandle node_handle_;
  std::string capability_name_;
  MoveGroupContextPtr context_;
};
}  // namespace move_group
