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

#include "plan_service_capability.h"
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupPlanService::MoveGroupPlanService() : MoveGroupCapability("MotionPlanService")
{
}

void MoveGroupPlanService::initialize()
{
  plan_service_ =
      root_node_handle_.advertiseService(PLANNER_SERVICE_NAME, &MoveGroupPlanService::computePlanService, this);
}

bool MoveGroupPlanService::computePlanService(moveit_msgs::GetMotionPlan::Request& req,
                                              moveit_msgs::GetMotionPlan::Response& res)
{
  ROS_INFO_NAMED(getName(), "Received new planning service request...");
  // before we start planning, ensure that we have the latest robot state received...
  if (static_cast<bool>(req.motion_plan_request.start_state.is_diff))
    context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline =
      resolvePlanningPipeline(req.motion_plan_request.pipeline_id);
  if (!planning_pipeline)
  {
    res.motion_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return true;
  }

  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
  try
  {
    planning_interface::MotionPlanResponse mp_res;
    planning_pipeline->generatePlan(ps, req.motion_plan_request, mp_res);
    mp_res.getMessage(res.motion_plan_response);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    res.motion_plan_response.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  return true;
}
}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupPlanService, move_group::MoveGroupCapability)
