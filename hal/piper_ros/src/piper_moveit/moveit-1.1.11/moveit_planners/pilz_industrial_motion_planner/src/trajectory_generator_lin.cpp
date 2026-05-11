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

#include "pilz_industrial_motion_planner/trajectory_generator_lin.h"

#include <cassert>
#include <ros/ros.h>
#include <sstream>
#include <time.h>

#include <moveit/robot_state/conversions.h>

#include <kdl/path_line.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>

#include <tf2_kdl/tf2_kdl.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pilz_industrial_motion_planner
{
TrajectoryGeneratorLIN::TrajectoryGeneratorLIN(const moveit::core::RobotModelConstPtr& robot_model,
                                               const LimitsContainer& planner_limits, const std::string& /*group_name*/)
  : TrajectoryGenerator::TrajectoryGenerator(robot_model, planner_limits)
{
  if (!planner_limits_.hasFullCartesianLimits())
  {
    ROS_ERROR("Cartesian limits not set for LIN trajectory generator.");
    throw TrajectoryGeneratorInvalidLimitsException("Cartesian limits are not fully set for LIN trajectory generator.");
  }
}

void TrajectoryGeneratorLIN::extractMotionPlanInfo(const planning_scene::PlanningSceneConstPtr& scene,
                                                   const planning_interface::MotionPlanRequest& req,
                                                   TrajectoryGenerator::MotionPlanInfo& info) const
{
  ROS_DEBUG("Extract necessary information from motion plan request.");

  info.group_name = req.group_name;
  std::string frame_id{ robot_model_->getModelFrame() };

  // goal given in joint space
  if (!req.goal_constraints.front().joint_constraints.empty())
  {
    info.link_name = robot_model_->getJointModelGroup(req.group_name)->getSolverInstance()->getTipFrame();

    if (req.goal_constraints.front().joint_constraints.size() !=
        robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size())
    {
      std::ostringstream os;
      os << "Number of joints in goal does not match number of joints of group "
            "(Number joints goal: "
         << req.goal_constraints.front().joint_constraints.size() << " | Number of joints of group: "
         << robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames().size() << ")";
      throw JointNumberMismatch(os.str());
    }
    // initializing all joints of the model
    for (const auto& joint_name : robot_model_->getVariableNames())
    {
      info.goal_joint_position[joint_name] = 0;
    }

    for (const auto& joint_item : req.goal_constraints.front().joint_constraints)
    {
      info.goal_joint_position[joint_item.joint_name] = joint_item.position;
    }

    // Ignored return value because at this point the function should always
    // return 'true'.
    computeLinkFK(robot_model_, info.link_name, info.goal_joint_position, info.goal_pose);
  }
  // goal given in Cartesian space
  else
  {
    info.link_name = req.goal_constraints.front().position_constraints.front().link_name;
    if (req.goal_constraints.front().position_constraints.front().header.frame_id.empty() ||
        req.goal_constraints.front().orientation_constraints.front().header.frame_id.empty())
    {
      ROS_WARN("Frame id is not set in position/orientation constraints of "
               "goal. Use model frame as default");
      frame_id = robot_model_->getModelFrame();
    }
    else
    {
      frame_id = req.goal_constraints.front().position_constraints.front().header.frame_id;
    }
    info.goal_pose = getConstraintPose(req.goal_constraints.front());
  }

  assert(req.start_state.joint_state.name.size() == req.start_state.joint_state.position.size());
  for (const auto& joint_name : robot_model_->getJointModelGroup(req.group_name)->getActiveJointModelNames())
  {
    auto it{ std::find(req.start_state.joint_state.name.cbegin(), req.start_state.joint_state.name.cend(), joint_name) };
    if (it == req.start_state.joint_state.name.cend())
    {
      std::ostringstream os;
      os << "Could not find joint \"" << joint_name << "\" of group \"" << req.group_name
         << "\" in start state of request";
      throw LinJointMissingInStartState(os.str());
    }
    size_t index = it - req.start_state.joint_state.name.cbegin();
    info.start_joint_position[joint_name] = req.start_state.joint_state.position[index];
  }

  // Ignored return value because at this point the function should always
  // return 'true'.
  computeLinkFK(robot_model_, info.link_name, info.start_joint_position, info.start_pose);

  // check goal pose ik before Cartesian motion plan starts
  std::map<std::string, double> ik_solution;
  if (!computePoseIK(scene, info.group_name, info.link_name, info.goal_pose, frame_id, info.start_joint_position,
                     ik_solution))
  {
    std::ostringstream os;
    os << "Failed to compute inverse kinematics for link: " << info.link_name << " of goal pose";
    throw LinInverseForGoalIncalculable(os.str());
  }
}

void TrajectoryGeneratorLIN::plan(const planning_scene::PlanningSceneConstPtr& scene,
                                  const planning_interface::MotionPlanRequest& req, const MotionPlanInfo& plan_info,
                                  const double& sampling_time, trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // create Cartesian path for lin
  std::unique_ptr<KDL::Path> path(setPathLIN(plan_info.start_pose, plan_info.goal_pose));

  // create velocity profile
  std::unique_ptr<KDL::VelocityProfile> vp(
      cartesianTrapVelocityProfile(req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor, path));

  // combine path and velocity profile into Cartesian trajectory
  // with the third parameter set to false, KDL::Trajectory_Segment does not
  // take
  // the ownship of Path and Velocity Profile
  KDL::Trajectory_Segment cart_trajectory(path.get(), vp.get(), false);

  moveit_msgs::MoveItErrorCodes error_code;
  // sample the Cartesian trajectory and compute joint trajectory using inverse
  // kinematics
  if (!generateJointTrajectory(scene, planner_limits_.getJointLimitContainer(), cart_trajectory, plan_info.group_name,
                               plan_info.link_name, plan_info.start_joint_position, sampling_time, joint_trajectory,
                               error_code))
  {
    std::ostringstream os;
    os << "Failed to generate valid joint trajectory from the Cartesian path";
    throw LinTrajectoryConversionFailure(os.str(), error_code.val);
  }
}

std::unique_ptr<KDL::Path> TrajectoryGeneratorLIN::setPathLIN(const Eigen::Affine3d& start_pose,
                                                              const Eigen::Affine3d& goal_pose) const
{
  ROS_DEBUG("Set Cartesian path for LIN command.");

  KDL::Frame kdl_start_pose, kdl_goal_pose;
  tf2::fromMsg(tf2::toMsg(start_pose), kdl_start_pose);
  tf2::fromMsg(tf2::toMsg(goal_pose), kdl_goal_pose);
  double eqradius = planner_limits_.getCartesianLimits().getMaxTranslationalVelocity() /
                    planner_limits_.getCartesianLimits().getMaxRotationalVelocity();
  KDL::RotationalInterpolation* rot_interpo = new KDL::RotationalInterpolation_SingleAxis();

  return std::unique_ptr<KDL::Path>(new KDL::Path_Line(kdl_start_pose, kdl_goal_pose, rot_interpo, eqradius, true));
}

}  // namespace pilz_industrial_motion_planner
