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

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>

namespace planning_interface
{
struct MotionPlanResponse
{
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  double planning_time_;
  moveit::core::MoveItErrorCode error_code_;
  moveit_msgs::RobotState start_state_;
  std::string planner_id_;

  [[deprecated("Use trajectory_ instead.")]] const robot_trajectory::RobotTrajectoryPtr& trajectory;
  [[deprecated("Use planning_time_ instead.")]] const double& planning_time;
  [[deprecated("Use error_code_ instead.")]] const moveit::core::MoveItErrorCode& error_code;
  [[deprecated("Use start_state_ instead.")]] const moveit_msgs::RobotState& start_state;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MotionPlanResponse()
    : planning_time_(0.0)
    , trajectory(trajectory_)
    , planning_time(planning_time_)
    , error_code(error_code_)
    , start_state(start_state_)
  {
  }
#pragma GCC diagnostic pop

  MotionPlanResponse(const MotionPlanResponse& response) : MotionPlanResponse()
  {
    *this = response;
  }

  MotionPlanResponse& operator=(const MotionPlanResponse& response)
  {
    trajectory_ = response.trajectory_;
    planning_time_ = response.planning_time_;
    error_code_ = response.error_code_;
    start_state_ = response.start_state_;
    planner_id_ = response.planner_id_;
    return *this;
  }

  void getMessage(moveit_msgs::MotionPlanResponse& msg) const;

  // Enable checking of query success or failure, for example if(response) ...
  explicit operator bool() const
  {
    return bool(error_code_);
  }
};

struct MotionPlanDetailedResponse
{
  void getMessage(moveit_msgs::MotionPlanDetailedResponse& msg) const;

  std::vector<robot_trajectory::RobotTrajectoryPtr> trajectory_;
  std::vector<std::string> description_;
  std::vector<double> processing_time_;
  moveit::core::MoveItErrorCode error_code_;
  moveit_msgs::RobotState start_state_;
  std::string planner_id_;

  // Enable checking of query success or failure, for example if(response) ...
  explicit operator bool() const
  {
    return bool(error_code_);
  }
};

}  // namespace planning_interface
