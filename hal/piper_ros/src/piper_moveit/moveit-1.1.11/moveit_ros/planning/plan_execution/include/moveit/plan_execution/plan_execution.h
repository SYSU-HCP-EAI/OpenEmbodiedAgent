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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/trajectory_monitor.h>
#include <moveit/sensor_manager/sensor_manager.h>
#include <pluginlib/class_loader.hpp>

#include <atomic>

/** \brief This namespace includes functionality specific to the execution and monitoring of motion plans */
namespace plan_execution
{
MOVEIT_CLASS_FORWARD(PlanExecution);  // Defines PlanExecutionPtr, ConstPtr, WeakPtr... etc

class PlanExecution
{
public:
  struct Options
  {
    Options() : replan_(false), replan_attempts_(0), replan_delay_(0.0)
    {
    }

    /// Flag indicating whether replanning is allowed
    bool replan_;

    /// If replanning is allowed, this variable specifies how many replanning attempts there can be, at most, before
    /// failure
    unsigned int replan_attempts_;

    /// The amount of time to wait in between replanning attempts (in seconds)
    double replan_delay_;

    /// Callback for computing motion plans. This callback must always be specified.
    ExecutableMotionPlanComputationFn plan_callback_;

    /// Callback for repairing motion plans. This is optional. A new plan is re-computed if repairing routines are not
    /// specified.
    /// To aid in the repair process, the position that the controller had reached in the execution of the previous plan
    /// is also passed as argument.
    /// The format is the same as what the trajectory_execution_manager::TrajectoryExecutionManager reports: a pair of
    /// two integers where the first
    /// one is the index of the last trajectory being executed (from the sequence of trajectories specified in the
    /// ExecutableMotionPlan) and the second
    /// one is the index of the closest waypoint along that trajectory.
    boost::function<bool(ExecutableMotionPlan& plan_to_update, const std::pair<int, int>& trajectory_index)>
        repair_plan_callback_;

    boost::function<void()> before_plan_callback_;
    boost::function<void()> before_execution_callback_;
    boost::function<void()> done_callback_;
  };

  PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution);
  ~PlanExecution();

  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor() const
  {
    return planning_scene_monitor_;
  }

  const trajectory_execution_manager::TrajectoryExecutionManagerPtr& getTrajectoryExecutionManager() const
  {
    return trajectory_execution_manager_;
  }

  double getTrajectoryStateRecordingFrequency() const
  {
    if (trajectory_monitor_)
      return trajectory_monitor_->getSamplingFrequency();
    else
      return 0.0;
  }

  void setTrajectoryStateRecordingFrequency(double freq)
  {
    if (trajectory_monitor_)
      trajectory_monitor_->setSamplingFrequency(freq);
  }

  void setMaxReplanAttempts(unsigned int attempts)
  {
    default_max_replan_attempts_ = attempts;
  }

  unsigned int getMaxReplanAttempts() const
  {
    return default_max_replan_attempts_;
  }

  void planAndExecute(ExecutableMotionPlan& plan, const Options& opt);
  void planAndExecute(ExecutableMotionPlan& plan, const moveit_msgs::PlanningScene& scene_diff, const Options& opt);

  /** \brief Execute and monitor a previously created \e plan.

      In case there is no \e planning_scene or \e planning_scene_monitor set in the \e plan they will be set at the
      start of the method. They are then used to monitor the execution. */
  moveit_msgs::MoveItErrorCodes executeAndMonitor(ExecutableMotionPlan& plan, bool reset_preempted = true);

  void stop();

private:
  void planAndExecuteHelper(ExecutableMotionPlan& plan, const Options& opt);
  bool isRemainingPathValid(const ExecutableMotionPlan& plan, const std::pair<int, int>& path_segment);

  void planningSceneUpdatedCallback(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);
  void doneWithTrajectoryExecution(const moveit_controller_manager::ExecutionStatus& status);
  void successfulTrajectorySegmentExecution(const ExecutableMotionPlan& plan, std::size_t index);

  ros::NodeHandle node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  planning_scene_monitor::TrajectoryMonitorPtr trajectory_monitor_;

  unsigned int default_max_replan_attempts_;

  class
  {
  private:
    std::atomic<bool> preemption_requested{ false };

  public:
    /** \brief check *and clear* the preemption flag

        A true return value has to trigger full execution stop, as it consumes the request */
    inline bool checkAndClear()
    {
      return preemption_requested.exchange(false);
    }
    inline void request()
    {
      preemption_requested.store(true);
    }
  } preempt_;

  bool new_scene_update_;

  bool execution_complete_;
  bool path_became_invalid_;

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl* reconfigure_impl_;
};
}  // namespace plan_execution
