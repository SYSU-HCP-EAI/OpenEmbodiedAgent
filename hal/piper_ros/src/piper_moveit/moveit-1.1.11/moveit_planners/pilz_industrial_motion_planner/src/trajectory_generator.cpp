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

#include "pilz_industrial_motion_planner/trajectory_generator.h"

#include <cassert>

#include <boost/range/combine.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <moveit/robot_state/conversions.h>

#include "pilz_industrial_motion_planner/limits_container.h"

namespace pilz_industrial_motion_planner
{
sensor_msgs::JointState TrajectoryGenerator::filterGroupValues(const sensor_msgs::JointState& robot_state,
                                                               const std::string& group) const
{
  const std::vector<std::string>& group_joints{ robot_model_->getJointModelGroup(group)->getActiveJointModelNames() };
  sensor_msgs::JointState group_state;
  group_state.name.reserve(group_joints.size());
  group_state.position.reserve(group_joints.size());
  group_state.velocity.reserve(group_joints.size());

  for (size_t i = 0; i < robot_state.name.size(); ++i)
  {
    if (std::find(group_joints.begin(), group_joints.end(), robot_state.name.at(i)) != group_joints.end())
    {
      group_state.name.push_back(robot_state.name.at(i));
      group_state.position.push_back(robot_state.position.at(i));
      if (i < robot_state.velocity.size())
      {
        group_state.velocity.push_back(robot_state.velocity.at(i));
      }
    }
  }
  return group_state;
}

void TrajectoryGenerator::cmdSpecificRequestValidation(const planning_interface::MotionPlanRequest& /*req*/) const
{
  // Empty implementation, in case the derived class does not want
  // to provide a command specific request validation.
}

void TrajectoryGenerator::checkVelocityScaling(const double& scaling_factor)
{
  if (!isScalingFactorValid(scaling_factor))
  {
    std::ostringstream os;
    os << "Velocity scaling not in range [" << MIN_SCALING_FACTOR << ", " << MAX_SCALING_FACTOR << "], "
       << "actual value is: " << scaling_factor;
    throw VelocityScalingIncorrect(os.str());
  }
}

void TrajectoryGenerator::checkAccelerationScaling(const double& scaling_factor)
{
  if (!isScalingFactorValid(scaling_factor))
  {
    std::ostringstream os;
    os << "Acceleration scaling not in range [" << MIN_SCALING_FACTOR << ", " << MAX_SCALING_FACTOR << "], "
       << "actual value is: " << scaling_factor;
    throw AccelerationScalingIncorrect(os.str());
  }
}

void TrajectoryGenerator::checkForValidGroupName(const std::string& group_name) const
{
  if (!robot_model_->hasJointModelGroup(group_name))
  {
    std::ostringstream os;
    os << "Unknown planning group: " << group_name;
    throw UnknownPlanningGroup(os.str());
  }
}

void TrajectoryGenerator::checkStartState(const moveit_msgs::RobotState& start_state, const std::string& group) const
{
  if (start_state.joint_state.name.empty())
  {
    throw NoJointNamesInStartState("No joint names for state state given");
  }

  if (start_state.joint_state.name.size() != start_state.joint_state.position.size())
  {
    throw SizeMismatchInStartState("Joint state name and position do not match in start state");
  }

  sensor_msgs::JointState group_start_state{ filterGroupValues(start_state.joint_state, group) };

  // verify joint position limits
  const JointLimitsContainer& limits{ planner_limits_.getJointLimitContainer() };
  std::string error_msg;
  for (auto joint : boost::combine(group_start_state.name, group_start_state.position))
  {
    if (!limits.verifyPositionLimit(joint.get<0>(), joint.get<1>()))
    {
      error_msg.append(error_msg.empty() ? "start state joints outside their position limits: " : ", ");
      error_msg.append(joint.get<0>());
    }
  }
  if (!error_msg.empty())
  {
    throw JointsOfStartStateOutOfRange(error_msg);
  }

  // does not allow start velocity
  if (!std::all_of(group_start_state.velocity.begin(), group_start_state.velocity.end(),
                   [this](double v) { return std::fabs(v) < this->VELOCITY_TOLERANCE; }))
  {
    throw NonZeroVelocityInStartState("Trajectory Generator does not allow non-zero start velocity");
  }
}

void TrajectoryGenerator::checkJointGoalConstraint(const moveit_msgs::Constraints& constraint,
                                                   const std::vector<std::string>& expected_joint_names,
                                                   const std::string& group_name) const
{
  for (auto const& joint_constraint : constraint.joint_constraints)
  {
    const std::string& curr_joint_name{ joint_constraint.joint_name };
    if (std::find(expected_joint_names.cbegin(), expected_joint_names.cend(), curr_joint_name) ==
        expected_joint_names.cend())
    {
      std::ostringstream os;
      os << "Cannot find joint \"" << curr_joint_name << "\" from start state in goal constraint";
      throw StartStateGoalStateMismatch(os.str());
    }

    if (!robot_model_->getJointModelGroup(group_name)->hasJointModel(curr_joint_name))
    {
      std::ostringstream os;
      os << "Joint \"" << curr_joint_name << "\" does not belong to group \"" << group_name << "\"";
      throw JointConstraintDoesNotBelongToGroup(os.str());
    }

    if (!planner_limits_.getJointLimitContainer().verifyPositionLimit(curr_joint_name, joint_constraint.position))
    {
      std::ostringstream os;
      os << "Joint \"" << curr_joint_name << "\" violates joint limits in goal constraints";
      throw JointsOfGoalOutOfRange(os.str());
    }
  }
}

void TrajectoryGenerator::checkCartesianGoalConstraint(const moveit_msgs::Constraints& constraint,
                                                       const std::string& group_name) const
{
  assert(constraint.position_constraints.size() == 1);
  assert(constraint.orientation_constraints.size() == 1);
  const moveit_msgs::PositionConstraint& pos_constraint{ constraint.position_constraints.front() };
  const moveit_msgs::OrientationConstraint& ori_constraint{ constraint.orientation_constraints.front() };

  if (pos_constraint.link_name.empty())
  {
    throw PositionConstraintNameMissing("Link name of position constraint missing");
  }

  if (ori_constraint.link_name.empty())
  {
    throw OrientationConstraintNameMissing("Link name of orientation constraint missing");
  }

  if (pos_constraint.link_name != ori_constraint.link_name)
  {
    std::ostringstream os;
    os << "Position and orientation constraint name do not match"
       << "(Position constraint name: \"" << pos_constraint.link_name << "\" | Orientation constraint name: \""
       << ori_constraint.link_name << "\")";
    throw PositionOrientationConstraintNameMismatch(os.str());
  }

  if (!robot_model_->getJointModelGroup(group_name)->canSetStateFromIK(pos_constraint.link_name))
  {
    std::ostringstream os;
    os << "No IK solver available for link: \"" << pos_constraint.link_name << "\"";
    throw NoIKSolverAvailable(os.str());
  }

  if (pos_constraint.constraint_region.primitive_poses.empty())
  {
    throw NoPrimitivePoseGiven("Primitive pose in position constraints of goal missing");
  }
}

void TrajectoryGenerator::checkGoalConstraints(
    const moveit_msgs::MotionPlanRequest::_goal_constraints_type& goal_constraints,
    const std::vector<std::string>& expected_joint_names, const std::string& group_name) const
{
  if (goal_constraints.size() != 1)
  {
    std::ostringstream os;
    os << "Exactly one goal constraint required, but " << goal_constraints.size() << " goal constraints given";
    throw NotExactlyOneGoalConstraintGiven(os.str());
  }

  const moveit_msgs::Constraints& goal_con{ goal_constraints.front() };
  if (!isOnlyOneGoalTypeGiven(goal_con))
  {
    throw OnlyOneGoalTypeAllowed("Only cartesian XOR joint goal allowed");
  }

  if (isJointGoalGiven(goal_con))
  {
    checkJointGoalConstraint(goal_con, expected_joint_names, group_name);
  }
  else
  {
    checkCartesianGoalConstraint(goal_con, group_name);
  }
}

void TrajectoryGenerator::validateRequest(const planning_interface::MotionPlanRequest& req) const
{
  checkVelocityScaling(req.max_velocity_scaling_factor);
  checkAccelerationScaling(req.max_acceleration_scaling_factor);
  checkForValidGroupName(req.group_name);
  checkStartState(req.start_state, req.group_name);
  checkGoalConstraints(req.goal_constraints, req.start_state.joint_state.name, req.group_name);
}

void TrajectoryGenerator::setSuccessResponse(const moveit::core::RobotState& start_state, const std::string& group_name,
                                             const trajectory_msgs::JointTrajectory& joint_trajectory,
                                             const ros::Time& planning_start,
                                             planning_interface::MotionPlanResponse& res) const
{
  robot_trajectory::RobotTrajectoryPtr rt(new robot_trajectory::RobotTrajectory(robot_model_, group_name));
  rt->setRobotTrajectoryMsg(start_state, joint_trajectory);

  res.trajectory_ = rt;
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.planning_time_ = (ros::Time::now() - planning_start).toSec();
}

void TrajectoryGenerator::setFailureResponse(const ros::Time& planning_start,
                                             planning_interface::MotionPlanResponse& res) const
{
  if (res.trajectory_)
  {
    res.trajectory_->clear();
  }
  res.planning_time_ = (ros::Time::now() - planning_start).toSec();
}

std::unique_ptr<KDL::VelocityProfile>
TrajectoryGenerator::cartesianTrapVelocityProfile(const double& max_velocity_scaling_factor,
                                                  const double& max_acceleration_scaling_factor,
                                                  const std::unique_ptr<KDL::Path>& path) const
{
  std::unique_ptr<KDL::VelocityProfile> vp_trans(new KDL::VelocityProfile_Trap(
      max_velocity_scaling_factor * planner_limits_.getCartesianLimits().getMaxTranslationalVelocity(),
      max_acceleration_scaling_factor * planner_limits_.getCartesianLimits().getMaxTranslationalAcceleration()));

  if (path->PathLength() > std::numeric_limits<double>::epsilon())  // avoid division by zero
  {
    vp_trans->SetProfile(0, path->PathLength());
  }
  else
  {
    vp_trans->SetProfile(0, std::numeric_limits<double>::epsilon());
  }
  return vp_trans;
}

bool TrajectoryGenerator::generate(const planning_scene::PlanningSceneConstPtr& scene,
                                   const planning_interface::MotionPlanRequest& req,
                                   planning_interface::MotionPlanResponse& res, double sampling_time)
{
  ROS_INFO_STREAM("Generating " << req.planner_id << " trajectory...");
  ros::Time planning_begin = ros::Time::now();

  try
  {
    validateRequest(req);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    res.error_code_.val = ex.getErrorCode();
    setFailureResponse(planning_begin, res);
    return false;
  }

  try
  {
    cmdSpecificRequestValidation(req);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    res.error_code_.val = ex.getErrorCode();
    setFailureResponse(planning_begin, res);
    return false;
  }

  MotionPlanInfo plan_info;
  try
  {
    extractMotionPlanInfo(scene, req, plan_info);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    res.error_code_.val = ex.getErrorCode();
    setFailureResponse(planning_begin, res);
    return false;
  }

  trajectory_msgs::JointTrajectory joint_trajectory;
  try
  {
    plan(scene, req, plan_info, sampling_time, joint_trajectory);
  }
  catch (const MoveItErrorCodeException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    res.error_code_.val = ex.getErrorCode();
    setFailureResponse(planning_begin, res);
    return false;
  }

  moveit::core::RobotState start_state(scene->getCurrentState());
  moveit::core::robotStateMsgToRobotState(req.start_state, start_state, true);
  setSuccessResponse(start_state, req.group_name, joint_trajectory, planning_begin, res);
  return true;
}

}  // namespace pilz_industrial_motion_planner
