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
#include <moveit/plan_execution/plan_representation.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>

#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <moveit/sensor_manager/sensor_manager.h>
#include <pluginlib/class_loader.hpp>

#include <memory>

namespace plan_execution
{
MOVEIT_CLASS_FORWARD(PlanWithSensing);  // Defines PlanWithSensingPtr, ConstPtr, WeakPtr... etc

class PlanWithSensing
{
public:
  PlanWithSensing(const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution);
  ~PlanWithSensing();

  const trajectory_execution_manager::TrajectoryExecutionManagerPtr& getTrajectoryExecutionManager() const
  {
    return trajectory_execution_manager_;
  }

  bool computePlan(ExecutableMotionPlan& plan, const ExecutableMotionPlanComputationFn& motion_planner,
                   unsigned int max_look_attempts, double max_safe_path_cost);

  double getMaxSafePathCost() const
  {
    return default_max_safe_path_cost_;
  }

  void setMaxSafePathCost(double max_safe_path_cost)
  {
    default_max_safe_path_cost_ = max_safe_path_cost;
  }

  void setMaxLookAttempts(unsigned int attempts)
  {
    default_max_look_attempts_ = attempts;
  }

  unsigned int getMaxLookAttempts() const
  {
    return default_max_look_attempts_;
  }

  unsigned int getMaxCostSources() const
  {
    return max_cost_sources_;
  }

  void setMaxCostSources(unsigned int value)
  {
    max_cost_sources_ = value;
  }

  double getDiscardOverlappingCostSources() const
  {
    return discard_overlapping_cost_sources_;
  }

  void setDiscardOverlappingCostSources(double value)
  {
    discard_overlapping_cost_sources_ = value;
  }

  void setBeforeLookCallback(const boost::function<void()>& callback)
  {
    before_look_callback_ = callback;
  }

  void displayCostSources(bool flag);

private:
  bool lookAt(const std::set<collision_detection::CostSource>& cost_sources, const std::string& frame_id);

  ros::NodeHandle node_handle_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

  std::unique_ptr<pluginlib::ClassLoader<moveit_sensor_manager::MoveItSensorManager> > sensor_manager_loader_;
  moveit_sensor_manager::MoveItSensorManagerPtr sensor_manager_;
  unsigned int default_max_look_attempts_;
  double default_max_safe_path_cost_;

  double discard_overlapping_cost_sources_;
  unsigned int max_cost_sources_;

  bool display_cost_sources_;
  ros::Publisher cost_sources_publisher_;

  boost::function<void()> before_look_callback_;

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl* reconfigure_impl_;
};
}  // namespace plan_execution
