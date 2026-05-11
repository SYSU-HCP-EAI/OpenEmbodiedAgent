/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include "pilz_industrial_motion_planner/joint_limits_container.h"
#include "pilz_industrial_motion_planner/trajectory_generator.h"

#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <atomic>
#include <thread>

namespace pilz_industrial_motion_planner
{
/**
 * @brief PlanningContext for obtaining trajectories
 */
template <typename GeneratorT>
class PlanningContextBase : public planning_interface::PlanningContext
{
public:
  PlanningContextBase<GeneratorT>(const std::string& name, const std::string& group,
                                  const moveit::core::RobotModelConstPtr& model,
                                  const pilz_industrial_motion_planner::LimitsContainer& limits)
    : planning_interface::PlanningContext(name, group)
    , terminated_(false)
    , model_(model)
    , limits_(limits)
    , generator_(model, limits_, group)
  {
  }

  ~PlanningContextBase() override
  {
  }

  /**
   * @brief Calculates a trajectory for the request this context is currently
   * set for
   * @param res The result containing the respective trajectory, or error_code
   * on failure
   * @return true on success, false otherwise
   */
  bool solve(planning_interface::MotionPlanResponse& res) override;

  /**
   * @brief Will return the same trajectory as
   * solve(planning_interface::MotionPlanResponse& res)
   * This function just delegates to the common response however here the same
   * trajectory is stored with the
   * descriptions "plan", "simplify", "interpolate"
   * @param res The detailed response
   * @return true on success, false otherwise
   */
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /**
   * @brief Will terminate solve()
   * @return
   * @note Currently will not stop a runnning solve but not start future solves.
   */
  bool terminate() override;

  /**
   * @copydoc planning_interface::PlanningContext::clear()
   */
  void clear() override;

  /// Flag if terminated
  std::atomic_bool terminated_;

  /// The robot model
  robot_model::RobotModelConstPtr model_;

  /// Joint limits to be used during planning
  pilz_industrial_motion_planner::LimitsContainer limits_;

protected:
  GeneratorT generator_;
};

template <typename GeneratorT>
bool pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::solve(planning_interface::MotionPlanResponse& res)
{
  if (!terminated_)
  {
    // Use current state as start state if not set
    if (request_.start_state.joint_state.name.empty())
    {
      moveit_msgs::RobotState current_state;
      moveit::core::robotStateToRobotStateMsg(getPlanningScene()->getCurrentState(), current_state);
      request_.start_state = current_state;
    }
    bool result = generator_.generate(getPlanningScene(), request_, res);
    return result;
    // res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    // return false; // TODO
  }

  ROS_ERROR("Using solve on a terminated planning context!");
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  return false;
}

template <typename GeneratorT>
bool pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::solve(
    planning_interface::MotionPlanDetailedResponse& res)
{
  // delegate to regular response
  planning_interface::MotionPlanResponse undetailed_response;
  bool result = solve(undetailed_response);

  res.description_.push_back("plan");
  res.trajectory_.push_back(undetailed_response.trajectory_);
  res.processing_time_.push_back(undetailed_response.planning_time_);

  res.description_.push_back("simplify");
  res.trajectory_.push_back(undetailed_response.trajectory_);
  res.processing_time_.push_back(0);

  res.description_.push_back("interpolate");
  res.trajectory_.push_back(undetailed_response.trajectory_);
  res.processing_time_.push_back(0);

  res.error_code_ = undetailed_response.error_code_;
  return result;
}

template <typename GeneratorT>
bool pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::terminate()
{
  ROS_DEBUG_STREAM("Terminate called");
  terminated_ = true;
  return true;
}

template <typename GeneratorT>
void pilz_industrial_motion_planner::PlanningContextBase<GeneratorT>::clear()
{
  // No structures that need cleaning
  return;
}

}  // namespace pilz_industrial_motion_planner
