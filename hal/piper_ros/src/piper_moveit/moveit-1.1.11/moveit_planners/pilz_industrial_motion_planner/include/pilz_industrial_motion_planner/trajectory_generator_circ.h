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
#include <kdl/path.hpp>
#include <kdl/velocityprofile.hpp>
#include <moveit/planning_scene/planning_scene.h>

#include "pilz_industrial_motion_planner/trajectory_generator.h"

using namespace pilz_industrial_motion_planner;

namespace pilz_industrial_motion_planner
{
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircleNoPlane, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircleToSmall, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CenterPointDifferentRadius, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircTrajectoryConversionFailure, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(UnknownPathConstraintName, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoPositionConstraints, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoPrimitivePose, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(UnknownLinkNameOfAuxiliaryPoint, moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NumberOfConstraintsMismatch, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircJointMissingInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(CircInverseForGoalIncalculable, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);

/**
 * @brief This class implements a trajectory generator of arcs in Cartesian
 * space.
 * The arc is specified by a start pose, a goal pose and a interim point on the
 * arc,
 * or a point as the center of the circle which forms the arc. Complete circle
 * is not
 * covered by this generator.
 */
class TrajectoryGeneratorCIRC : public TrajectoryGenerator
{
public:
  /**
   * @brief Constructor of CIRC Trajectory Generator.
   *
   * @param planner_limits Limits in joint and Cartesian spaces
   *
   * @throw TrajectoryGeneratorInvalidLimitsException
   *
   */
  TrajectoryGeneratorCIRC(const robot_model::RobotModelConstPtr& robot_model,
                          const pilz_industrial_motion_planner::LimitsContainer& planner_limits,
                          const std::string& group_name);

private:
  void cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest& req) const override;

  void extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                             const planning_interface::MotionPlanRequest& req, MotionPlanInfo& info) const final;

  void plan(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
            const MotionPlanInfo& plan_info, const double& sampling_time,
            trajectory_msgs::JointTrajectory& joint_trajectory) override;

  /**
   * @brief Construct a KDL::Path object for a Cartesian path of an arc.
   *
   * @return A unique pointer of the path object, null_ptr in case of an error.
   *
   * @throws CircleNoPlane if the plane in which the circle resides,
   * could not be determined.
   *
   * @throws CircleToSmall if the specified circ radius is to small.
   *
   * @throws CenterPointDifferentRadius if the distances between start-center
   * and goal-center are different.
   */
  std::unique_ptr<KDL::Path> setPathCIRC(const MotionPlanInfo& info) const;
};

}  // namespace pilz_industrial_motion_planner
