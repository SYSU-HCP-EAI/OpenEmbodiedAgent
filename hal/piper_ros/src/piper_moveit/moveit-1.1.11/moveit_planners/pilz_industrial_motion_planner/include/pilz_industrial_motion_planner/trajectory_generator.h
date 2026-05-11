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

#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include "pilz_industrial_motion_planner/joint_limits_extension.h"
#include "pilz_industrial_motion_planner/limits_container.h"
#include "pilz_industrial_motion_planner/trajectory_functions.h"
#include "pilz_industrial_motion_planner/trajectory_generation_exceptions.h"

using namespace pilz_industrial_motion_planner;

namespace pilz_industrial_motion_planner
{
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(TrajectoryGeneratorInvalidLimitsException, moveit_msgs::MoveItErrorCodes::FAILURE);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(VelocityScalingIncorrect, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(AccelerationScalingIncorrect, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(UnknownPlanningGroup, moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoJointNamesInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(SizeMismatchInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(JointsOfStartStateOutOfRange, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NonZeroVelocityInStartState, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NotExactlyOneGoalConstraintGiven,
                                   moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(OnlyOneGoalTypeAllowed, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(StartStateGoalStateMismatch, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(JointConstraintDoesNotBelongToGroup,
                                   moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(JointsOfGoalOutOfRange, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PositionConstraintNameMissing,
                                   moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(OrientationConstraintNameMissing,
                                   moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(PositionOrientationConstraintNameMismatch,
                                   moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoIKSolverAvailable, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoPrimitivePoseGiven, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

/**
 * @brief Base class of trajectory generators
 *
 * Note: All derived classes cannot have a start velocity
 */
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(const robot_model::RobotModelConstPtr& robot_model,
                      const pilz_industrial_motion_planner::LimitsContainer& planner_limits)
    : robot_model_(robot_model), planner_limits_(planner_limits)
  {
  }

  virtual ~TrajectoryGenerator() = default;

  /**
   * @brief generate robot trajectory with given sampling time
   * @param req: motion plan request
   * @param res: motion plan response
   * @param sampling_time: sampling time of the generate trajectory
   * @return motion plan succeed/fail, detailed information in motion plan
   * response
   */
  bool generate(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
                planning_interface::MotionPlanResponse& res, double sampling_time = 0.1);

protected:
  /**
   * @brief This class is used to extract needed information from motion plan
   * request.
   */
  class MotionPlanInfo
  {
  public:
    std::string group_name;
    std::string link_name;
    Eigen::Isometry3d start_pose;
    Eigen::Isometry3d goal_pose;
    std::map<std::string, double> start_joint_position;
    std::map<std::string, double> goal_joint_position;
    std::pair<std::string, Eigen::Vector3d> circ_path_point;
  };

  /**
   * @brief build cartesian velocity profile for the path
   *
   * Uses the path to get the cartesian length and the angular distance from
   * start to goal.
   * The trap profile returns uses the longer distance of translational and
   * rotational motion.
   */
  std::unique_ptr<KDL::VelocityProfile> cartesianTrapVelocityProfile(const double& max_velocity_scaling_factor,
                                                                     const double& max_acceleration_scaling_factor,
                                                                     const std::unique_ptr<KDL::Path>& path) const;

private:
  virtual void cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest& req) const;

  /**
   * @brief Extract needed information from a motion plan request in order to
   * simplify
   * further usages.
   * @param scene: planning scene
   * @param req: motion plan request
   * @param info: information extracted from motion plan request which is
   * necessary for the planning
   */
  virtual void extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                     const planning_interface::MotionPlanRequest& req, MotionPlanInfo& info) const = 0;

  virtual void plan(const planning_scene::PlanningSceneConstPtr& scene,
                    const planning_interface::MotionPlanRequest& req, const MotionPlanInfo& plan_info,
                    const double& sampling_time, trajectory_msgs::JointTrajectory& joint_trajectory) = 0;

private:
  /**
   * @brief Validate the motion plan request based on the common requirements of
   * trajectroy generator
   * Checks that:
   *    - req.max_velocity_scaling_factor [0.0001, 1],
   * moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN on failure
   *    - req.max_acceleration_scaling_factor [0.0001, 1] ,
   * moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN on
   * failure
   *    - req.group_name is a JointModelGroup of the Robotmodel,
   * moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME on
   * failure
   *    - req.start_state.joint_state is not empty,
   * moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE on failure
   *    - req.start_state.joint_state is within the limits,
   * moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE on
   * failure
   *    - req.start_state.joint_state is all zero,
   * moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE on failure
   *    - req.goal_constraints must have exactly 1 defined cartesian oder joint
   * constraint
   *      moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   * A joint goal is checked for:
   *    - StartState joint-names matching goal joint-names,
   * moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on
   * failure
   *    - Beeing defined in the req.group_name JointModelGroup
   *    - Beeing with the defined limits
   * A cartesian goal is checked for:
   *    - A defined link_name for the constraint,
   * moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   *    - Matching link_name for position and orientation constraints,
   * moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   *    - A IK solver exists for the given req.group_name and constraint
   * link_name,
   * moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION on failure
   *    - A goal pose define in
   * position_constraints[0].constraint_region.primitive_poses,
   * moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS on failure
   * @param req: motion plan request
   */
  void validateRequest(const planning_interface::MotionPlanRequest& req) const;

  /**
   * @brief set MotionPlanResponse from joint trajectory
   */
  void setSuccessResponse(const moveit::core::RobotState& start_rs, const std::string& group_name,
                          const trajectory_msgs::JointTrajectory& joint_trajectory, const ros::Time& planning_start,
                          planning_interface::MotionPlanResponse& res) const;

  void setFailureResponse(const ros::Time& planning_start, planning_interface::MotionPlanResponse& res) const;

  void checkForValidGroupName(const std::string& group_name) const;

  /**
   * @brief Validate that the start state of the request matches the
   * requirements of the trajectory generator.
   *
   * These requirements are:
   *     - Names of the joints and given joint position match in size and are
   * non-zero
   *     - The start state is withing the position limits
   *     - The start state velocity is below
   * TrajectoryGenerator::VELOCITY_TOLERANCE
   */
  void checkStartState(const moveit_msgs::RobotState& start_state, const std::string& group) const;

  void checkGoalConstraints(const moveit_msgs::MotionPlanRequest::_goal_constraints_type& goal_constraints,
                            const std::vector<std::string>& expected_joint_names, const std::string& group_name) const;

  void checkJointGoalConstraint(const moveit_msgs::Constraints& constraint,
                                const std::vector<std::string>& expected_joint_names,
                                const std::string& group_name) const;

  void checkCartesianGoalConstraint(const moveit_msgs::Constraints& constraint, const std::string& group_name) const;

private:
  /**
   * @return joint state message including only active joints in group
   */
  sensor_msgs::JointState filterGroupValues(const sensor_msgs::JointState& robot_state, const std::string& group) const;

  /**
   * @return True if scaling factor is valid, otherwise false.
   */
  static bool isScalingFactorValid(const double& scaling_factor);
  static void checkVelocityScaling(const double& scaling_factor);
  static void checkAccelerationScaling(const double& scaling_factor);

  /**
   * @return True if ONE position + ONE orientation constraint given,
   * otherwise false.
   */
  static bool isCartesianGoalGiven(const moveit_msgs::Constraints& constraint);

  /**
   * @return True if joint constraint given, otherwise false.
   */
  static bool isJointGoalGiven(const moveit_msgs::Constraints& constraint);

  /**
   * @return True if ONLY joint constraint or
   * ONLY cartesian constraint (position+orientation) given, otherwise false.
   */
  static bool isOnlyOneGoalTypeGiven(const moveit_msgs::Constraints& constraint);

protected:
  const robot_model::RobotModelConstPtr robot_model_;
  const pilz_industrial_motion_planner::LimitsContainer planner_limits_;
  static constexpr double MIN_SCALING_FACTOR{ 0.0001 };
  static constexpr double MAX_SCALING_FACTOR{ 1. };
  static constexpr double VELOCITY_TOLERANCE{ 1e-8 };
};

inline bool TrajectoryGenerator::isScalingFactorValid(const double& scaling_factor)
{
  return (scaling_factor > MIN_SCALING_FACTOR && scaling_factor <= MAX_SCALING_FACTOR);
}

inline bool TrajectoryGenerator::isCartesianGoalGiven(const moveit_msgs::Constraints& constraint)
{
  return constraint.position_constraints.size() == 1 && constraint.orientation_constraints.size() == 1;
}

inline bool TrajectoryGenerator::isJointGoalGiven(const moveit_msgs::Constraints& constraint)
{
  return !constraint.joint_constraints.empty();
}

inline bool TrajectoryGenerator::isOnlyOneGoalTypeGiven(const moveit_msgs::Constraints& constraint)
{
  return (isJointGoalGiven(constraint) && !isCartesianGoalGiven(constraint)) ||
         (!isJointGoalGiven(constraint) && isCartesianGoalGiven(constraint));
}

}  // namespace pilz_industrial_motion_planner
