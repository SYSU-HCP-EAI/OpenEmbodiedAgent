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

#include <string>
#include <moveit/macros/class_forward.h>

namespace moveit_cpp
{
MOVEIT_CLASS_FORWARD(MoveItCpp);
}

namespace planning_scene_monitor
{
MOVEIT_CLASS_FORWARD(PlanningSceneMonitor);  // Defines PlanningSceneMonitorPtr, ConstPtr, WeakPtr... etc
}

namespace planning_pipeline
{
MOVEIT_CLASS_FORWARD(PlanningPipeline);  // Defines PlanningPipelinePtr, ConstPtr, WeakPtr... etc
}

namespace plan_execution
{
MOVEIT_CLASS_FORWARD(PlanExecution);    // Defines PlanExecutionPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(PlanWithSensing);  // Defines PlanWithSensingPtr, ConstPtr, WeakPtr... etc
}  // namespace plan_execution

namespace trajectory_execution_manager
{
MOVEIT_CLASS_FORWARD(TrajectoryExecutionManager);  // Defines TrajectoryExecutionManagerPtr, ConstPtr, WeakPtr... etc
}

namespace move_group
{
MOVEIT_STRUCT_FORWARD(MoveGroupContext);

struct MoveGroupContext
{
  MoveGroupContext(const moveit_cpp::MoveItCppPtr& moveit_cpp, const std::string& default_planning_pipeline,
                   bool allow_trajectory_execution = false, bool debug = false);
  ~MoveGroupContext();

  bool status() const;

  moveit_cpp::MoveItCppPtr moveit_cpp_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  plan_execution::PlanExecutionPtr plan_execution_;
  plan_execution::PlanWithSensingPtr plan_with_sensing_;
  bool allow_trajectory_execution_;
  bool debug_;
};
}  // namespace move_group
