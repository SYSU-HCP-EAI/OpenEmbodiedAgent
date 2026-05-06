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

#include <eigen3/Eigen/Eigen>
#include <kdl/rotational_interpolation_sa.hpp>
#include <moveit/planning_scene/planning_scene.h>

#include "pilz_industrial_motion_planner/trajectory_generator.h"
#include "pilz_industrial_motion_planner/velocity_profile_atrap.h"

using namespace pilz_industrial_motion_planner;

namespace pilz_industrial_motion_planner
{
// TODO date type of units

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LinTrajectoryConversionFailure, moveit_msgs::MoveItErrorCodes::FAILURE);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(JointNumberMismatch, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LinJointMissingInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(LinInverseForGoalIncalculable, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);

/**
 * @brief This class implements a linear trajectory generator in Cartesian
 * space.
 * The Cartesian trajetory are based on trapezoid velocity profile.
 */
class TrajectoryGeneratorLIN : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of LIN Trajectory Generator
   * @throw TrajectoryGeneratorInvalidLimitsException
   * @param model: robot model
   * @param planner_limits: limits in joint and Cartesian spaces
   */
  TrajectoryGeneratorLIN(const robot_model::RobotModelConstPtr& robot_model,
                         const pilz_industrial_motion_planner::LimitsContainer& planner_limits,
                         const std::string& group_name);

private:
  void extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                             const planning_interface::MotionPlanRequest& req, MotionPlanInfo& info) const final;

  void plan(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
            const MotionPlanInfo& plan_info, const double& sampling_time,
            trajectory_msgs::JointTrajectory& joint_trajectory) override;

  /**
   * @brief construct a KDL::Path object for a Cartesian straight line
   * @return a unique pointer of the path object. null_ptr in case of an error.
   */
  std::unique_ptr<KDL::Path> setPathLIN(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& goal_pose) const;
};

}  // namespace pilz_industrial_motion_planner
