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

#include <gtest/gtest.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "test_utils.h"

pilz_industrial_motion_planner::JointLimitsContainer
testutils::createFakeLimits(const std::vector<std::string>& joint_names)
{
  pilz_industrial_motion_planner::JointLimitsContainer container;

  for (const std::string& name : joint_names)
  {
    JointLimit limit;
    limit.has_position_limits = true;
    limit.max_position = 2.967;
    limit.min_position = -2.967;
    limit.has_velocity_limits = true;
    limit.max_velocity = 1;
    limit.has_acceleration_limits = true;
    limit.max_acceleration = 0.5;
    limit.has_deceleration_limits = true;
    limit.max_deceleration = -1;

    container.addLimit(name, limit);
  }

  return container;
}

bool testutils::getExpectedGoalPose(const moveit::core::RobotModelConstPtr& robot_model,
                                    const planning_interface::MotionPlanRequest& req, std::string& link_name,
                                    Eigen::Isometry3d& goal_pose_expect)
{
  // ++++++++++++++++++++++++++++++++++
  // + Get goal from joint constraint +
  // ++++++++++++++++++++++++++++++++++
  if (!req.goal_constraints.front().joint_constraints.empty())
  {
    std::map<std::string, double> goal_joint_position;

    // initializing all joints of the model
    for (const auto& joint_name : robot_model->getVariableNames())
    {
      goal_joint_position[joint_name] = 0;
    }

    for (const auto& joint_item : req.goal_constraints.front().joint_constraints)
    {
      goal_joint_position[joint_item.joint_name] = joint_item.position;
    }

    link_name = robot_model->getJointModelGroup(req.group_name)->getSolverInstance()->getTipFrame();

    if (!computeLinkFK(robot_model, link_name, goal_joint_position, goal_pose_expect))
    {
      std::cerr << "Failed to compute forward kinematics for link in goal "
                   "constraints \n";
      return false;
    }
    return true;
  }

  // ++++++++++++++++++++++++++++++++++++++
  // + Get goal from cartesian constraint +
  // ++++++++++++++++++++++++++++++++++++++
  // TODO frame id
  link_name = req.goal_constraints.front().position_constraints.front().link_name;
  geometry_msgs::Pose goal_pose_msg;
  goal_pose_msg.position =
      req.goal_constraints.front().position_constraints.front().constraint_region.primitive_poses.front().position;
  goal_pose_msg.orientation = req.goal_constraints.front().orientation_constraints.front().orientation;
  normalizeQuaternion(goal_pose_msg.orientation);
  tf2::fromMsg(goal_pose_msg, goal_pose_expect);
  return true;
}

bool testutils::isGoalReached(const trajectory_msgs::JointTrajectory& trajectory,
                              const std::vector<moveit_msgs::JointConstraint>& goal,
                              const double joint_position_tolerance, const double joint_velocity_tolerance)
{
  trajectory_msgs::JointTrajectoryPoint last_point = trajectory.points.back();

  for (unsigned int i = 0; i < trajectory.joint_names.size(); ++i)
  {
    if (fabs(last_point.velocities.at(i)) > joint_velocity_tolerance)
    {
      std::cerr << "[ Fail     ] goal has non zero velocity."
                << " Joint Name: " << trajectory.joint_names.at(i) << "; Velocity: " << last_point.velocities.at(i)
                << std::endl;
      return false;
    }

    for (const auto& joint_goal : goal)
    {
      if (trajectory.joint_names.at(i) == joint_goal.joint_name)
      {
        if (fabs(last_point.positions.at(i) - joint_goal.position) > joint_position_tolerance)
        {
          std::cerr << "[ Fail     ] goal position not reached."
                    << " Joint Name: " << trajectory.joint_names.at(i)
                    << "; Actual Position: " << last_point.positions.at(i)
                    << "; Expected Position: " << joint_goal.position << std::endl;
          return false;
        }
      }
    }
  }

  return true;
}

bool testutils::isGoalReached(const moveit::core::RobotModelConstPtr& robot_model,
                              const trajectory_msgs::JointTrajectory& trajectory,
                              const planning_interface::MotionPlanRequest& req, const double pos_tolerance,
                              const double orientation_tolerance)
{
  std::string link_name;
  Eigen::Isometry3d goal_pose_expect;
  if (!testutils::getExpectedGoalPose(robot_model, req, link_name, goal_pose_expect))
  {
    return false;
  }

  // compute the actual goal pose in model frame
  trajectory_msgs::JointTrajectoryPoint last_point = trajectory.points.back();
  Eigen::Isometry3d goal_pose_actual;
  std::map<std::string, double> joint_state;

  // initializing all joints of the model
  for (const auto& joint_name : robot_model->getVariableNames())
  {
    joint_state[joint_name] = 0;
  }

  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    joint_state[trajectory.joint_names.at(i)] = last_point.positions.at(i);
  }

  if (!computeLinkFK(robot_model, link_name, joint_state, goal_pose_actual))
  {
    std::cerr << "[ Fail     ] forward kinematics computation failed for link: " << link_name << std::endl;
  }

  auto actual_rotation{ goal_pose_actual.rotation() };
  auto expected_rotation{ goal_pose_expect.rotation() };
  auto rot_diff{ actual_rotation - expected_rotation };
  if (rot_diff.norm() > orientation_tolerance)
  {
    std::cout << "\nOrientation difference = " << rot_diff.norm() << " (eps=" << orientation_tolerance << ")"
              << "\n### Expected goal orientation: ### \n"
              << expected_rotation << std::endl
              << "### Actual goal orientation ### \n"
              << actual_rotation << std::endl;
    return false;
  }

  auto actual_position{ goal_pose_actual.translation() };
  auto expected_position{ goal_pose_expect.translation() };
  auto pos_diff{ actual_position - expected_position };
  if (pos_diff.norm() > pos_tolerance)
  {
    std::cout << "\nPosition difference = " << pos_diff.norm() << " (eps=" << pos_tolerance << ")"
              << "\n### Expected goal position: ### \n"
              << expected_position << std::endl
              << "### Actual goal position ### \n"
              << actual_position << std::endl;
    return false;
  }

  return true;
}

bool testutils::isGoalReached(const moveit::core::RobotModelConstPtr& robot_model,
                              const trajectory_msgs::JointTrajectory& trajectory,
                              const planning_interface::MotionPlanRequest& req, const double tolerance)
{
  return isGoalReached(robot_model, trajectory, req, tolerance, tolerance);
}

bool testutils::checkCartesianLinearity(const moveit::core::RobotModelConstPtr& robot_model,
                                        const trajectory_msgs::JointTrajectory& trajectory,
                                        const planning_interface::MotionPlanRequest& req,
                                        const double translation_norm_tolerance, const double rot_axis_norm_tolerance,
                                        const double /*rot_angle_tolerance*/)
{
  std::string link_name;
  Eigen::Isometry3d goal_pose_expect;
  if (!testutils::getExpectedGoalPose(robot_model, req, link_name, goal_pose_expect))
  {
    return false;
  }

  // compute start pose
  robot_state::RobotState rstate(robot_model);
  rstate.setToDefaultValues();
  moveit::core::jointStateToRobotState(req.start_state.joint_state, rstate);
  Eigen::Isometry3d start_pose = rstate.getFrameTransform(link_name);

  // start goal angle axis
  Eigen::AngleAxisd start_goal_aa(start_pose.rotation().transpose() * goal_pose_expect.rotation());

  // check the linearity
  for (trajectory_msgs::JointTrajectoryPoint way_point : trajectory.points)
  {
    Eigen::Isometry3d way_point_pose;
    std::map<std::string, double> way_point_joint_state;

    // initializing all joints of the model
    for (const auto& joint_name : robot_model->getVariableNames())
    {
      way_point_joint_state[joint_name] = 0;
    }

    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
    {
      way_point_joint_state[trajectory.joint_names.at(i)] = way_point.positions.at(i);
    }

    if (!computeLinkFK(robot_model, link_name, way_point_joint_state, way_point_pose))
    {
      std::cerr << "Failed to compute forward kinematics for link in goal "
                   "constraints \n";
      return false;
    }

    // translational linearity
    Eigen::Vector3d goal_start_translation = goal_pose_expect.translation() - start_pose.translation();
    // https://de.wikipedia.org/wiki/Geradengleichung
    // Determined form of a straight line in 3D space
    if (fabs((goal_start_translation.cross(way_point_pose.translation()) -
              goal_start_translation.cross(start_pose.translation()))
                 .norm()) > fabs(translation_norm_tolerance))
    {
      std::cout << "Translational linearity is violated. \n"
                << "goal tanslation: " << goal_pose_expect.translation() << std::endl
                << "start translation: " << start_pose.translation() << std::endl
                << "acutual translation " << way_point_pose.translation() << std::endl;
      return false;
    }

    if (!checkSLERP(start_pose, goal_pose_expect, way_point_pose, rot_axis_norm_tolerance))
    {
      return false;
    }
  }

  return true;
}

bool testutils::checkSLERP(const Eigen::Isometry3d& start_pose, const Eigen::Isometry3d& goal_pose,
                           const Eigen::Isometry3d& wp_pose, const double rot_axis_norm_tolerance,
                           const double rot_angle_tolerance)
{
  // rotational linearity
  // start way point angle axis
  Eigen::AngleAxisd start_goal_aa(start_pose.rotation().transpose() * goal_pose.rotation());
  Eigen::AngleAxisd start_wp_aa(start_pose.rotation().transpose() * wp_pose.rotation());

  // parallel rotation axis
  // it is possible the axis opposite is
  // if the angle is zero, axis is arbitrary
  if (!(((start_goal_aa.axis() - start_wp_aa.axis()).norm() < fabs(rot_axis_norm_tolerance)) ||
        ((start_goal_aa.axis() + start_wp_aa.axis()).norm() < fabs(rot_axis_norm_tolerance)) ||
        (fabs(start_wp_aa.angle()) < fabs(rot_angle_tolerance))))
  {
    std::cout << "Rotational linearity is violated. \n"
              << std::endl
              << "start goal angle: " << start_goal_aa.angle() << "; axis: " << std::endl
              << start_goal_aa.axis() << std::endl
              << "start waypoint angle: " << start_wp_aa.angle() << "; axis: " << std::endl
              << start_wp_aa.axis() << std::endl;

    return false;
  }

  return true;
}

bool testutils::computeLinkFK(const moveit::core::RobotModelConstPtr& robot_model, const std::string& link_name,
                              const std::map<std::string, double>& joint_state, Eigen::Isometry3d& pose)
{
  // create robot state
  robot_state::RobotState rstate(robot_model);
  rstate.setToDefaultValues();

  // check the reference frame of the target pose
  if (!rstate.knowsFrameTransform(link_name))
  {
    ROS_ERROR_STREAM("The target link " << link_name << " is not known by robot.");
    return false;
  }

  // set the joint positions
  try
  {
    rstate.setVariablePositions(joint_state);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }

  // update the frame
  rstate.update();
  pose = rstate.getFrameTransform(link_name);

  return true;
}

bool testutils::isVelocityBounded(const trajectory_msgs::JointTrajectory& trajectory,
                                  const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  for (const auto& point : trajectory.points)
  {
    for (std::size_t i = 0; i < point.velocities.size(); ++i)
    {
      if (fabs(point.velocities.at(i)) > fabs(joint_limits.getLimit(trajectory.joint_names.at(i)).max_velocity))
      {
        std::cerr << "[ Fail     ] Joint velocity limit violated in " << i << "th waypoint of joint: "
                  << " Joint Name: " << trajectory.joint_names.at(i) << "; Position: " << point.positions.at(i)
                  << "; Velocity: " << point.velocities.at(i) << "; Acceleration: " << point.accelerations.at(i)
                  << "; Time from start: " << point.time_from_start.toSec()
                  << "; Velocity Limit: " << joint_limits.getLimit(trajectory.joint_names.at(i)).max_velocity
                  << std::endl;

        return false;
      }
    }
  }

  return true;
}

bool testutils::isTrajectoryConsistent(const trajectory_msgs::JointTrajectory& trajectory)
{
  for (const auto& point : trajectory.points)
  {
    if (trajectory.joint_names.size() != point.positions.size() ||
        trajectory.joint_names.size() != point.velocities.size() ||
        trajectory.joint_names.size() != point.accelerations.size())
    {
      return false;
    }
  }

  // TODO check value is consistant (time, position, velocity, acceleration)

  return true;
}

bool testutils::isAccelerationBounded(const trajectory_msgs::JointTrajectory& trajectory,
                                      const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  for (const auto& point : trajectory.points)
  {
    for (std::size_t i = 0; i < point.velocities.size(); ++i)
    {
      // deceleration
      if (point.velocities.at(i) * point.accelerations.at(i) <= 0)
      {
        if (fabs(point.accelerations.at(i)) > fabs(joint_limits.getLimit(trajectory.joint_names.at(i)).max_deceleration))
        {
          std::cerr << "[ Fail     ] Deceleration limit violated of joint: " << trajectory.joint_names.at(i)
                    << ": Position: " << point.positions.at(i) << "; Velocity: " << point.velocities.at(i)
                    << "; Acceleration: " << point.accelerations.at(i)
                    << "; Time from start: " << point.time_from_start.toSec()
                    << ". Deceleration Limit: " << joint_limits.getLimit(trajectory.joint_names.at(i)).max_deceleration
                    << std::endl;
          return false;
        }
      }
      // acceleration
      else
      {
        if (fabs(point.accelerations.at(i)) > fabs(joint_limits.getLimit(trajectory.joint_names.at(i)).max_acceleration))
        {
          std::cerr << "[ Fail     ] Acceleration limit violated of joint: " << trajectory.joint_names.at(i)
                    << ": Position: " << point.positions.at(i) << "; Velocity: " << point.velocities.at(i)
                    << "; Acceleration: " << point.accelerations.at(i)
                    << "; Time from start: " << point.time_from_start.toSec()
                    << ". Acceleration Limit: " << joint_limits.getLimit(trajectory.joint_names.at(i)).max_acceleration
                    << std::endl;

          return false;
        }
      }
    }
  }

  return true;
}

bool testutils::isPositionBounded(const trajectory_msgs::JointTrajectory& trajectory,
                                  const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  for (const auto& point : trajectory.points)
  {
    for (std::size_t i = 0; i < point.positions.size(); ++i)
    {
      if (point.positions.at(i) > joint_limits.getLimit(trajectory.joint_names.at(i)).max_position ||
          point.positions.at(i) < joint_limits.getLimit(trajectory.joint_names.at(i)).min_position)
      {
        std::cerr << "[ Fail     ] Joint position limit violated in " << i << "th waypoint of joint: "
                  << " Joint Name: " << trajectory.joint_names.at(i) << "; Position: " << point.positions.at(i)
                  << "; Velocity: " << point.velocities.at(i) << "; Acceleration: " << point.accelerations.at(i)
                  << "; Time from start: " << point.time_from_start.toSec()
                  << "; Max Position: " << joint_limits.getLimit(trajectory.joint_names.at(i)).max_position
                  << "; Min Position: " << joint_limits.getLimit(trajectory.joint_names.at(i)).min_position
                  << std::endl;

        return false;
      }
    }
  }

  return true;
}

bool testutils::checkJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                     const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits)
{
  if (!testutils::isTrajectoryConsistent(trajectory))
  {
    std::cout << "Joint trajectory is not consistent." << std::endl;
    return false;
  }
  if (!testutils::isPositionBounded(trajectory, joint_limits))
  {
    std::cout << "Joint poisiton violated the limits." << std::endl;
    return false;
  }
  if (!testutils::isVelocityBounded(trajectory, joint_limits))
  {
    std::cout << "Joint velocity violated the limits." << std::endl;
    return false;
  }
  if (!testutils::isAccelerationBounded(trajectory, joint_limits))
  {
    std::cout << "Joint acceleration violated the limits." << std::endl;
    return false;
  }

  return true;
}

::testing::AssertionResult testutils::hasStrictlyIncreasingTime(const robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  // Check for strictly positively increasing time steps
  for (unsigned int i = 1; i < trajectory->getWayPointCount(); ++i)
  {
    if (trajectory->getWayPointDurationFromPrevious(i) <= 0.0)
    {
      return ::testing::AssertionFailure()
             << "Waypoint " << (i) << " has " << trajectory->getWayPointDurationFromPrevious(i)
             << " time between itself and its predecessor."
             << " Total points in trajectory: " << trajectory->getWayPointCount() << ".";
    }
  }

  return ::testing::AssertionSuccess();
}

void testutils::createDummyRequest(const moveit::core::RobotModelConstPtr& robot_model,
                                   const std::string& planning_group, planning_interface::MotionPlanRequest& req)
{
  // valid motion plan request with goal in joint space
  req.group_name = planning_group;
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;
  robot_state::RobotState rstate(robot_model);
  rstate.setToDefaultValues();
  moveit::core::robotStateToRobotStateMsg(rstate, req.start_state, false);
}

void testutils::createPTPRequest(const std::string& planning_group, const robot_state::RobotState& start_state,
                                 const robot_state::RobotState& goal_state, planning_interface::MotionPlanRequest& req)
{
  // valid motion plan request with goal in joint space
  req.planner_id = "PTP";
  req.group_name = planning_group;
  req.max_velocity_scaling_factor = 0.5;
  req.max_acceleration_scaling_factor = 0.5;
  // start state
  moveit::core::robotStateToRobotStateMsg(start_state, req.start_state, false);
  // goal state
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      goal_state, goal_state.getRobotModel()->getJointModelGroup(planning_group)));
}

bool testutils::toTCPPose(const moveit::core::RobotModelConstPtr& robot_model, const std::string& link_name,
                          const std::vector<double>& joint_values, geometry_msgs::Pose& pose,
                          const std::string& joint_prefix)
{
  {
    std::map<std::string, double> joint_state;
    auto joint_values_it = joint_values.begin();

    // initializing all joints of the model
    for (const auto& joint_name : robot_model->getVariableNames())
    {
      joint_state[joint_name] = 0;
    }

    for (std::size_t i = 0; i < joint_values.size(); ++i)
    {
      joint_state[testutils::getJointName(i + 1, joint_prefix)] = *joint_values_it;
      ++joint_values_it;
    }

    Eigen::Isometry3d eig_pose;
    if (!testutils::computeLinkFK(robot_model, link_name, joint_state, eig_pose))
    {
      return false;
    }
    pose = tf2::toMsg(eig_pose);
    return true;
  }
}

bool testutils::checkOriginalTrajectoryAfterBlending(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                                                     const pilz_industrial_motion_planner::TrajectoryBlendResponse& res,
                                                     const double time_tolerance)
{
  for (std::size_t i = 0; i < res.first_trajectory->getWayPointCount(); ++i)
  {
    for (const std::string& joint_name :
         res.first_trajectory->getWayPoint(i).getJointModelGroup(req.group_name)->getActiveJointModelNames())
    {
      // check joint position
      if (res.first_trajectory->getWayPoint(i).getVariablePosition(joint_name) !=
          req.first_trajectory->getWayPoint(i).getVariablePosition(joint_name))
      {
        std::cout << i << "th position of the first trajectory is not same." << std::endl;
        return false;
      }

      // check joint velocity
      if (res.first_trajectory->getWayPoint(i).getVariableVelocity(joint_name) !=
          req.first_trajectory->getWayPoint(i).getVariableVelocity(joint_name))
      {
        std::cout << i << "th velocity of the first trajectory is not same." << std::endl;
        return false;
      }

      // check joint acceleration
      if (res.first_trajectory->getWayPoint(i).getVariableAcceleration(joint_name) !=
          req.first_trajectory->getWayPoint(i).getVariableAcceleration(joint_name))
      {
        std::cout << i << "th acceleration of the first trajectory is not same." << std::endl;
        return false;
      }
    }

    // check time from start
    if (res.first_trajectory->getWayPointDurationFromStart(i) != req.first_trajectory->getWayPointDurationFromStart(i))
    {
      std::cout << i << "th time_from_start of the first trajectory is not same." << std::endl;
      return false;
    }
  }

  size_t size_second = res.second_trajectory->getWayPointCount();
  size_t size_second_original = req.second_trajectory->getWayPointCount();
  for (std::size_t i = 0; i < size_second; ++i)
  {
    for (const std::string& joint_name : res.second_trajectory->getWayPoint(size_second - i - 1)
                                             .getJointModelGroup(req.group_name)
                                             ->getActiveJointModelNames())
    {
      // check joint position
      if (res.second_trajectory->getWayPoint(size_second - i - 1).getVariablePosition(joint_name) !=
          req.second_trajectory->getWayPoint(size_second_original - i - 1).getVariablePosition(joint_name))
      {
        std::cout << i - 1 << "th position of the second trajectory is not same." << std::endl;
        return false;
      }

      // check joint velocity
      if (res.second_trajectory->getWayPoint(size_second - i - 1).getVariableVelocity(joint_name) !=
          req.second_trajectory->getWayPoint(size_second_original - i - 1).getVariableVelocity(joint_name))
      {
        std::cout << i - 1 << "th position of the second trajectory is not same." << std::endl;
        return false;
      }

      // check joint acceleration
      if (res.second_trajectory->getWayPoint(size_second - i - 1).getVariableAcceleration(joint_name) !=
          req.second_trajectory->getWayPoint(size_second_original - i - 1).getVariableAcceleration(joint_name))
      {
        std::cout << i - 1 << "th position of the second trajectory is not same." << std::endl;
        return false;
      }
    }

    // check time from start
    if (i < size_second - 1)
    {
      if (fabs((res.second_trajectory->getWayPointDurationFromStart(size_second - i - 1) -
                res.second_trajectory->getWayPointDurationFromStart(size_second - i - 2)) -
               (req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 1) -
                req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 2))) > time_tolerance)
      {
        std::cout << size_second - i - 1 << "th time from start of the second trajectory is not same."
                  << res.second_trajectory->getWayPointDurationFromStart(size_second - i - 1) << ", "
                  << res.second_trajectory->getWayPointDurationFromStart(size_second - i - 2) << ", "
                  << req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 1) << ", "
                  << req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 2) << std::endl;
        return false;
      }
    }
    else
    {
      if (fabs((res.second_trajectory->getWayPointDurationFromStart(size_second - i - 1)) -
               (req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 1) -
                req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 2))) > time_tolerance)
      {
        std::cout << size_second - i - 1 << "th time from start of the second trajectory is not same."
                  << res.second_trajectory->getWayPointDurationFromStart(size_second - i - 1) << ", "
                  << req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 1) -
                         req.second_trajectory->getWayPointDurationFromStart(size_second_original - i - 2)
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

bool testutils::checkBlendingJointSpaceContinuity(const pilz_industrial_motion_planner::TrajectoryBlendResponse& res,
                                                  double joint_velocity_tolerance, double joint_accleration_tolerance)
{
  // convert to msgs
  moveit_msgs::RobotTrajectory first_traj, blend_traj, second_traj;
  res.first_trajectory->getRobotTrajectoryMsg(first_traj);
  res.blend_trajectory->getRobotTrajectoryMsg(blend_traj);
  res.second_trajectory->getRobotTrajectoryMsg(second_traj);

  // check the continuity between first trajectory and blend trajectory
  trajectory_msgs::JointTrajectoryPoint first_end, blend_start;
  first_end = first_traj.joint_trajectory.points.back();
  blend_start = blend_traj.joint_trajectory.points.front();

  // check the dimensions
  if (first_end.positions.size() != blend_start.positions.size() ||
      first_end.velocities.size() != blend_start.velocities.size() ||
      first_end.accelerations.size() != blend_start.accelerations.size())
  {
    std::cout << "Different sizes of the position/velocity/acceleration "
                 "between first trajectory and blend trajectory."
              << std::endl;
    return false;
  }

  // check the velocity and acceleration
  for (std::size_t i = 0; i < first_end.positions.size(); ++i)
  {
    double blend_start_velo =
        (blend_start.positions.at(i) - first_end.positions.at(i)) / blend_start.time_from_start.toSec();
    if (fabs(blend_start_velo - blend_start.velocities.at(i)) > joint_velocity_tolerance)
    {
      std::cout << "Velocity computed from positions are different from the "
                   "value in trajectory."
                << std::endl
                << "position: " << blend_start.positions.at(i) << "; " << first_end.positions.at(i)
                << "computed: " << blend_start_velo << "; "
                << "in trajectory: " << blend_start.velocities.at(i) << std::endl;
      return false;
    }

    double blend_start_acc = (blend_start_velo - first_end.velocities.at(i)) / blend_start.time_from_start.toSec();
    if (fabs(blend_start_acc - blend_start.accelerations.at(i)) > joint_velocity_tolerance)
    {
      std::cout << "Acceleration computed from positions/velocities are "
                   "different from the value in trajectory."
                << std::endl
                << "computed: " << blend_start_acc << "; "
                << "in trajectory: " << blend_start.accelerations.at(i) << std::endl;
      return false;
    }
  }

  // check the continuity between blend trajectory and second trajectory
  trajectory_msgs::JointTrajectoryPoint blend_end, second_start;
  blend_end = blend_traj.joint_trajectory.points.back();
  second_start = second_traj.joint_trajectory.points.front();

  // check the dimensions
  if (blend_end.positions.size() != second_start.positions.size() ||
      blend_end.velocities.size() != second_start.velocities.size() ||
      blend_end.accelerations.size() != second_start.accelerations.size())
  {
    std::cout << "Different sizes of the position/velocity/acceleration "
                 "between first trajectory and blend trajectory."
              << std::endl
              << blend_end.positions.size() << ", " << second_start.positions.size() << std::endl
              << blend_end.velocities.size() << ", " << second_start.positions.size() << std::endl
              << blend_end.accelerations.size() << ", " << second_start.accelerations.size() << std::endl;
    return false;
  }

  // check the velocity and acceleration
  for (std::size_t i = 0; i < blend_end.positions.size(); ++i)
  {
    double second_start_velo =
        (second_start.positions.at(i) - blend_end.positions.at(i)) / second_start.time_from_start.toSec();
    if (fabs(second_start_velo - second_start.velocities.at(i)) > joint_accleration_tolerance)
    {
      std::cout << "Velocity computed from positions are different from the "
                   "value in trajectory."
                << std::endl
                << "computed: " << second_start_velo << "; "
                << "in trajectory: " << second_start.velocities.at(i) << std::endl;
      return false;
    }

    double second_start_acc = (second_start_velo - blend_end.velocities.at(i)) / second_start.time_from_start.toSec();
    if (fabs(second_start_acc - second_start.accelerations.at(i)) > joint_accleration_tolerance)
    {
      std::cout << "Acceleration computed from positions/velocities are "
                   "different from the value in trajectory."
                << std::endl
                << "computed: " << second_start_acc << "; "
                << "in trajectory: " << second_start.accelerations.at(i) << std::endl;
      return false;
    }
  }

  return true;
}

bool testutils::checkBlendingCartSpaceContinuity(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                                                 const pilz_industrial_motion_planner::TrajectoryBlendResponse& res,
                                                 const pilz_industrial_motion_planner::LimitsContainer& planner_limits)
{
  // sampling time
  double duration = res.blend_trajectory->getWayPointDurationFromPrevious(res.blend_trajectory->getWayPointCount() - 1);
  if (duration == 0.0)
  {
    std::cout << "Cannot perform check of cartesian space continuity with "
                 "sampling time 0.0"
              << std::endl;
    return false;
  }

  // limits
  double max_trans_velo = planner_limits.getCartesianLimits().getMaxTranslationalVelocity();
  double max_trans_acc = planner_limits.getCartesianLimits().getMaxTranslationalAcceleration();
  double max_rot_velo = planner_limits.getCartesianLimits().getMaxRotationalVelocity();
  double max_rot_acc = max_trans_acc / max_trans_velo * max_rot_velo;

  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // +++ check the connection points between three trajectories +++
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // connection points
  Eigen::Isometry3d pose_first_end, pose_first_end_1, pose_blend_start, pose_blend_start_1, pose_blend_end,
      pose_blend_end_1, pose_second_start, pose_second_start_1;
  // one sample before last point of first trajectory
  pose_first_end_1 = res.first_trajectory->getWayPointPtr(res.first_trajectory->getWayPointCount() - 2)
                         ->getFrameTransform(req.link_name);
  // last point of first trajectory
  pose_first_end = res.first_trajectory->getLastWayPointPtr()->getFrameTransform(req.link_name);
  // first point of blend trajectory
  pose_blend_start = res.blend_trajectory->getFirstWayPointPtr()->getFrameTransform(req.link_name);
  // second point of blend trajectory
  pose_blend_start_1 = res.blend_trajectory->getWayPointPtr(1)->getFrameTransform(req.link_name);
  // one sample before last point of blend trajectory
  pose_blend_end_1 = res.blend_trajectory->getWayPointPtr(res.blend_trajectory->getWayPointCount() - 2)
                         ->getFrameTransform(req.link_name);
  // last point of blend trajectory
  pose_blend_end = res.blend_trajectory->getLastWayPointPtr()->getFrameTransform(req.link_name);
  // first point of second trajectory
  pose_second_start = res.second_trajectory->getFirstWayPointPtr()->getFrameTransform(req.link_name);
  // second point of second trajectory
  pose_second_start_1 = res.second_trajectory->getWayPointPtr(1)->getFrameTransform(req.link_name);

  //  std::cout << "### sample duration: " << duration << " ###" << std::endl;
  //  std::cout << "### end pose of first trajectory ###" << std::endl;
  //  std::cout << pose_first_end.matrix() << std::endl;
  //  std::cout << "### start pose of blend trajectory ###" << std::endl;
  //  std::cout << pose_blend_start.matrix() << std::endl;
  //  std::cout << "### end pose of blend trajectory ###" << std::endl;
  //  std::cout << pose_blend_end.matrix() << std::endl;
  //  std::cout << "### start pose of second trajectory ###" << std::endl;
  //  std::cout << pose_second_start.matrix() << std::endl;

  //  std::cout << "### v_1 ###" << std::endl;
  //  std::cout << v_1 << std::endl;
  //  std::cout << "### w_1 ###" << std::endl;
  //  std::cout << w_1 << std::endl;
  //  std::cout << "### v_2 ###" << std::endl;
  //  std::cout << v_2 << std::endl;
  //  std::cout << "### w_2 ###" << std::endl;
  //  std::cout << w_2 << std::endl;
  //  std::cout << "### v_3 ###" << std::endl;
  //  std::cout << v_3 << std::endl;
  //  std::cout << "### w_3 ###" << std::endl;
  //  std::cout << w_3 << std::endl;

  // check the connection points between first trajectory and blend trajectory
  Eigen::Vector3d v_1, w_1, v_2, w_2, v_3, w_3;
  computeCartVelocity(pose_first_end_1, pose_first_end, duration, v_1, w_1);
  computeCartVelocity(pose_first_end, pose_blend_start, duration, v_2, w_2);
  computeCartVelocity(pose_blend_start, pose_blend_start_1, duration, v_3, w_3);

  // translational velocity
  if (v_2.norm() > max_trans_velo)
  {
    std::cout << "Translational velocity between first trajectory and blend "
                 "trajectory exceeds the limit."
              << "Actual velocity (norm): " << v_2.norm() << "; "
              << "Limits: " << max_trans_velo << std::endl;
  }
  // rotational velocity
  if (w_2.norm() > max_rot_velo)
  {
    std::cout << "Rotational velocity between first trajectory and blend "
                 "trajectory exceeds the limit."
              << "Actual velocity (norm): " << w_2.norm() << "; "
              << "Limits: " << max_rot_velo << std::endl;
  }
  // translational acceleration
  Eigen::Vector3d a_1 = (v_2 - v_1) / duration;
  Eigen::Vector3d a_2 = (v_3 - v_2) / duration;
  if (a_1.norm() > max_trans_acc || a_2.norm() > max_trans_acc)
  {
    std::cout << "Translational acceleration between first trajectory and "
                 "blend trajectory exceeds the limit."
              << "Actual acceleration (norm): " << a_1.norm() << ", " << a_1.norm() << "; "
              << "Limits: " << max_trans_acc << std::endl;
  }

  // rotational acceleration
  a_1 = (w_2 - w_1) / duration;
  a_2 = (w_3 - w_2) / duration;
  if (a_1.norm() > max_rot_acc || a_2.norm() > max_rot_acc)
  {
    std::cout << "Rotational acceleration between first trajectory and blend "
                 "trajectory exceeds the limit."
              << "Actual acceleration (norm): " << a_1.norm() << ", " << a_1.norm() << "; "
              << "Limits: " << max_rot_acc << std::endl;
  }

  computeCartVelocity(pose_blend_end_1, pose_blend_end, duration, v_1, w_1);
  computeCartVelocity(pose_blend_end, pose_second_start, duration, v_2, w_2);
  computeCartVelocity(pose_second_start, pose_second_start_1, duration, v_3, w_3);

  if (v_2.norm() > max_trans_velo)
  {
    std::cout << "Translational velocity between blend trajectory and second "
                 "trajectory exceeds the limit."
              << "Actual velocity (norm): " << v_2.norm() << "; "
              << "Limits: " << max_trans_velo << std::endl;
  }
  if (w_2.norm() > max_rot_velo)
  {
    std::cout << "Rotational velocity between blend trajectory and second "
                 "trajectory exceeds the limit."
              << "Actual velocity (norm): " << w_2.norm() << "; "
              << "Limits: " << max_rot_velo << std::endl;
  }
  a_1 = (v_2 - v_1) / duration;
  a_2 = (v_3 - v_2) / duration;
  if (a_1.norm() > max_trans_acc || a_2.norm() > max_trans_acc)
  {
    std::cout << "Translational acceleration between blend trajectory and "
                 "second trajectory exceeds the limit."
              << "Actual acceleration (norm): " << a_1.norm() << ", " << a_1.norm() << "; "
              << "Limits: " << max_trans_acc << std::endl;
  }
  // check rotational acceleration
  a_1 = (w_2 - w_1) / duration;
  a_2 = (w_3 - w_2) / duration;
  if (a_1.norm() > max_rot_acc || a_2.norm() > max_rot_acc)
  {
    std::cout << "Rotational acceleration between blend trajectory and second "
                 "trajectory exceeds the limit."
              << "Actual acceleration (norm): " << a_1.norm() << ", " << a_1.norm() << "; "
              << "Limits: " << max_rot_acc << std::endl;
  }

  return true;
}

bool testutils::checkThatPointsInRadius(const std::string& link_name, const double& r, Eigen::Isometry3d& circ_pose,
                                        const pilz_industrial_motion_planner::TrajectoryBlendResponse& res)
{
  bool result = true;
  for (size_t i = 1; i < res.blend_trajectory->getWayPointCount() - 1; ++i)
  {
    Eigen::Isometry3d curr_pos(res.blend_trajectory->getWayPointPtr(i)->getFrameTransform(link_name));
    if ((curr_pos.translation() - circ_pose.translation()).norm() > r)
    {
      std::cout << "Point " << i << " does not lie within blending radius (dist: "
                << ((curr_pos.translation() - circ_pose.translation()).norm() - r) << ")." << std::endl;
      result = false;
    }
  }
  return result;
}

void testutils::computeCartVelocity(const Eigen::Isometry3d& pose_1, const Eigen::Isometry3d& pose_2, double duration,
                                    Eigen::Vector3d& v, Eigen::Vector3d& w)
{
  // translational velocity
  v = (pose_2.translation() - pose_1.translation()) / duration;

  // angular velocity
  // reference: A Mathematical Introduction to Robotics Manipulation 2.4.1
  // Rotational velocity
  Eigen::Matrix3d rm_1 = pose_1.linear();
  Eigen::Matrix3d rm_2 = pose_2.linear();
  Eigen::Matrix3d rm_dot = (rm_2 - rm_1) / duration;
  Eigen::Matrix3d w_hat = rm_dot * rm_1.transpose();
  w(0) = w_hat(2, 1);
  w(1) = w_hat(0, 2);
  w(2) = w_hat(1, 0);
}

void testutils::getLinLinPosesWithoutOriChange(const std::string& frame_id, sensor_msgs::JointState& initialJointState,
                                               geometry_msgs::PoseStamped& p1, geometry_msgs::PoseStamped& p2)
{
  initialJointState =
      testutils::generateJointState({ 0., 0.007881892504574495, -1.8157263253868452, 0., 1.8236082178909834, 0. });

  p1.header.frame_id = frame_id;
  p1.pose.position.x = 0.25;
  p1.pose.position.y = 0.3;
  p1.pose.position.z = 0.65;
  p1.pose.orientation.x = 0.;
  p1.pose.orientation.y = 0.;
  p1.pose.orientation.z = 0.;
  p1.pose.orientation.w = 1.;

  p2 = p1;
  p2.pose.position.x -= 0.15;
}

void testutils::getOriChange(Eigen::Matrix3d& ori1, Eigen::Matrix3d& ori2)
{
  ori1 = Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d::UnitZ());
  ori2 = Eigen::AngleAxisd(0.4 * M_PI, Eigen::Vector3d::UnitZ());
}

void testutils::createFakeCartTraj(const robot_trajectory::RobotTrajectoryPtr& traj, const std::string& link_name,
                                   moveit_msgs::RobotTrajectory& fake_traj)
{
  fake_traj.joint_trajectory.joint_names.push_back("x");
  fake_traj.joint_trajectory.joint_names.push_back("y");
  fake_traj.joint_trajectory.joint_names.push_back("z");
  //  fake_traj.joint_trajectory.joint_names.push_back("a");
  //  fake_traj.joint_trajectory.joint_names.push_back("b");
  //  fake_traj.joint_trajectory.joint_names.push_back("c");

  for (size_t i = 0; i < traj->getWayPointCount(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint waypoint;
    waypoint.time_from_start = ros::Duration(traj->getWayPointDurationFromStart(i));
    Eigen::Isometry3d waypoint_pose = traj->getWayPointPtr(i)->getFrameTransform(link_name);
    Eigen::Vector3d waypoint_position = waypoint_pose.translation();
    waypoint.positions.push_back(waypoint_position(0));
    waypoint.positions.push_back(waypoint_position(1));
    waypoint.positions.push_back(waypoint_position(2));
    waypoint.velocities.push_back(0);
    waypoint.velocities.push_back(0);
    waypoint.velocities.push_back(0);
    waypoint.accelerations.push_back(0);
    waypoint.accelerations.push_back(0);
    waypoint.accelerations.push_back(0);
    fake_traj.joint_trajectory.points.push_back(waypoint);
  }
}

bool testutils::getBlendTestData(const ros::NodeHandle& nh, const size_t& dataset_num, const std::string& name_prefix,
                                 std::vector<testutils::BlendTestData>& datasets)
{
  datasets.clear();
  testutils::BlendTestData dataset;
  for (size_t i = 1; i < dataset_num + 1; ++i)
  {
    std::string data_set_name = "blend_set_" + std::to_string(i);
    if (nh.getParam(name_prefix + data_set_name + "/start_position", dataset.start_position) &&
        nh.getParam(name_prefix + data_set_name + "/mid_position", dataset.mid_position) &&
        nh.getParam(name_prefix + data_set_name + "/end_position", dataset.end_position))
    {
      datasets.push_back(dataset);
    }
    else
    {
      return false;
    }
  }
  return true;
}

bool testutils::generateTrajFromBlendTestData(
    const planning_scene::PlanningSceneConstPtr& scene,
    const std::shared_ptr<pilz_industrial_motion_planner::TrajectoryGenerator>& tg, const std::string& group_name,
    const std::string& link_name, const testutils::BlendTestData& data, const double& sampling_time_1,
    const double& sampling_time_2, planning_interface::MotionPlanResponse& res_1,
    planning_interface::MotionPlanResponse& res_2, double& dis_1, double& dis_2)
{
  const robot_model::RobotModelConstPtr robot_model = scene->getRobotModel();

  // generate first trajectory
  planning_interface::MotionPlanRequest req_1;
  req_1.group_name = group_name;
  req_1.max_velocity_scaling_factor = 0.1;
  req_1.max_acceleration_scaling_factor = 0.1;
  // start state
  robot_state::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  start_state.setJointGroupPositions(group_name, data.start_position);
  moveit::core::robotStateToRobotStateMsg(start_state, req_1.start_state, false);
  // goal constraint
  robot_state::RobotState goal_state_1(robot_model);
  goal_state_1.setToDefaultValues();
  goal_state_1.setJointGroupPositions(group_name, data.mid_position);
  req_1.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      goal_state_1, goal_state_1.getRobotModel()->getJointModelGroup(group_name)));

  // trajectory generation
  if (!tg->generate(scene, req_1, res_1, sampling_time_1))
  {
    std::cout << "Failed to generate first trajectory." << std::endl;
    return false;
  }

  // generate second LIN trajectory
  planning_interface::MotionPlanRequest req_2;
  req_2.group_name = group_name;
  req_2.max_velocity_scaling_factor = 0.1;
  req_2.max_acceleration_scaling_factor = 0.1;
  // start state
  moveit::core::robotStateToRobotStateMsg(goal_state_1, req_2.start_state, false);
  // goal state
  robot_state::RobotState goal_state_2(robot_model);
  goal_state_2.setToDefaultValues();
  goal_state_2.setJointGroupPositions(group_name, data.end_position);
  req_2.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      goal_state_2, goal_state_2.getRobotModel()->getJointModelGroup(group_name)));
  // trajectory generation
  if (!tg->generate(scene, req_2, res_2, sampling_time_2))
  {
    std::cout << "Failed to generate second trajectory." << std::endl;
    return false;
  }

  // estimate a proper blend radius
  dis_1 =
      (start_state.getFrameTransform(link_name).translation() - goal_state_1.getFrameTransform(link_name).translation())
          .norm();

  dis_2 = (goal_state_1.getFrameTransform(link_name).translation() -
           goal_state_2.getFrameTransform(link_name).translation())
              .norm();

  return true;
}

bool testutils::checkBlendResult(const pilz_industrial_motion_planner::TrajectoryBlendRequest& blend_req,
                                 const pilz_industrial_motion_planner::TrajectoryBlendResponse& blend_res,
                                 const pilz_industrial_motion_planner::LimitsContainer& limits,
                                 double joint_velocity_tolerance, double joint_acceleration_tolerance,
                                 double /*cartesian_velocity_tolerance*/,
                                 double /*cartesian_angular_velocity_tolerance*/)
{
  // ++++++++++++++++++++++
  // + Check trajectories +
  // ++++++++++++++++++++++
  moveit_msgs::RobotTrajectory traj_msg;
  blend_res.first_trajectory->getRobotTrajectoryMsg(traj_msg);
  if (!testutils::checkJointTrajectory(traj_msg.joint_trajectory, limits.getJointLimitContainer()))
  {
    return false;
  }

  blend_res.blend_trajectory->getRobotTrajectoryMsg(traj_msg);
  if (!testutils::checkJointTrajectory(traj_msg.joint_trajectory, limits.getJointLimitContainer()))
  {
    return false;
  };

  blend_res.second_trajectory->getRobotTrajectoryMsg(traj_msg);
  if (!testutils::checkJointTrajectory(traj_msg.joint_trajectory, limits.getJointLimitContainer()))
  {
    return false;
  };

  Eigen::Isometry3d circ_pose =
      blend_req.first_trajectory->getLastWayPointPtr()->getFrameTransform(blend_req.link_name);
  if (!testutils::checkThatPointsInRadius(blend_req.link_name, blend_req.blend_radius, circ_pose, blend_res))
  {
    return false;
  }

  // check the first and second trajectories, if they are still the same before
  // and after the bendling phase
  if (!testutils::checkOriginalTrajectoryAfterBlending(blend_req, blend_res, 10e-5))
  {
    return false;
  }

  // check the continuity between the trajectories in joint space
  if (!testutils::checkBlendingJointSpaceContinuity(blend_res, joint_velocity_tolerance, joint_acceleration_tolerance))
  {
    return false;
  }

  // check the continuity between the trajectories in cart space
  if (!testutils::checkBlendingCartSpaceContinuity(blend_req, blend_res, limits))
  {
    return false;
  }

  // ++++++++++++++++++++++++
  // + Visualize the result +
  // ++++++++++++++++++++++++
  //  ros::NodeHandle nh;
  //  ros::Publisher pub =
  //  nh.advertise<moveit_msgs::DisplayTrajectory>("my_planned_path", 1);
  //  ros::Duration duration(1.0);
  //  duration.sleep();

  //  // visualize the joint trajectory
  //  moveit_msgs::DisplayTrajectory displayTrajectory;
  //  moveit_msgs::RobotTrajectory res_first_traj_msg, res_blend_traj_msg,
  //  res_second_traj_msg;
  //  blend_res.first_trajectory->getRobotTrajectoryMsg(res_first_traj_msg);
  //  blend_res.blend_trajectory->getRobotTrajectoryMsg(res_blend_traj_msg);
  //  blend_res.second_trajectory->getRobotTrajectoryMsg(res_second_traj_msg);
  //  displayTrajectory.trajectory.push_back(res_first_traj_msg);
  //  displayTrajectory.trajectory.push_back(res_blend_traj_msg);
  //  displayTrajectory.trajectory.push_back(res_second_traj_msg);

  //  pub.publish(displayTrajectory);

  return true;
}

void testutils::generateRequestMsgFromBlendTestData(const moveit::core::RobotModelConstPtr& robot_model,
                                                    const testutils::BlendTestData& data, const std::string& planner_id,
                                                    const std::string& group_name, const std::string& link_name,
                                                    moveit_msgs::MotionSequenceRequest& req_list)
{
  // motion plan request of first trajectory
  planning_interface::MotionPlanRequest req_1;
  req_1.planner_id = planner_id;
  req_1.group_name = group_name;
  req_1.max_velocity_scaling_factor = 0.1;
  req_1.max_acceleration_scaling_factor = 0.1;
  // start state
  robot_state::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  start_state.setJointGroupPositions(group_name, data.start_position);
  moveit::core::robotStateToRobotStateMsg(start_state, req_1.start_state, false);
  // goal constraint
  robot_state::RobotState goal_state_1(robot_model);
  goal_state_1.setToDefaultValues();
  goal_state_1.setJointGroupPositions(group_name, data.mid_position);
  req_1.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      goal_state_1, goal_state_1.getRobotModel()->getJointModelGroup(group_name)));

  // motion plan request of second trajectory
  planning_interface::MotionPlanRequest req_2;
  req_2.planner_id = planner_id;
  req_2.group_name = group_name;
  req_2.max_velocity_scaling_factor = 0.1;
  req_2.max_acceleration_scaling_factor = 0.1;
  // start state
  // moveit::core::robotStateToRobotStateMsg(goal_state_1, req_2.start_state,
  // false);
  // goal state
  robot_state::RobotState goal_state_2(robot_model);
  goal_state_2.setToDefaultValues();
  goal_state_2.setJointGroupPositions(group_name, data.end_position);
  req_2.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      goal_state_2, goal_state_2.getRobotModel()->getJointModelGroup(group_name)));

  // select a proper blending radius
  // estimate a proper blend radius
  double dis_1 =
      (start_state.getFrameTransform(link_name).translation() - goal_state_1.getFrameTransform(link_name).translation())
          .norm();

  double dis_2 = (goal_state_1.getFrameTransform(link_name).translation() -
                  goal_state_2.getFrameTransform(link_name).translation())
                     .norm();

  double blend_radius = 0.5 * std::min(dis_1, dis_2);

  moveit_msgs::MotionSequenceItem blend_req_1, blend_req_2;
  blend_req_1.req = req_1;
  blend_req_1.blend_radius = blend_radius;
  blend_req_2.req = req_2;
  blend_req_2.blend_radius = 0.0;

  req_list.items.push_back(blend_req_1);
  req_list.items.push_back(blend_req_2);
}

void testutils::checkRobotModel(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                                const std::string& link_name)
{
  ASSERT_TRUE(robot_model != nullptr) << "failed to load robot model";
  ASSERT_FALSE(robot_model->isEmpty()) << "robot model is empty";
  ASSERT_TRUE(robot_model->hasJointModelGroup(group_name)) << group_name << " is not known to robot";
  ASSERT_TRUE(robot_model->hasLinkModel(link_name)) << link_name << " is not known to robot";
  ASSERT_TRUE(robot_state::RobotState(robot_model).knowsFrameTransform(link_name))
      << "Transform of " << link_name << " is unknown";
}

::testing::AssertionResult testutils::hasTrapezoidVelocity(std::vector<double> accelerations, const double acc_tol)
{
  // Check that acceleration is monotonously decreasing
  if (!isMonotonouslyDecreasing(accelerations, acc_tol))
  {
    return ::testing::AssertionFailure() << "Cannot be a trapezoid since "
                                            "acceleration is not monotonously "
                                            "decreasing!";
  }

  // Check accelerations
  double first_acc = accelerations.front();
  auto it_last_acc = std::find_if(accelerations.begin(), accelerations.end(),
                                  [first_acc, acc_tol](double el) { return (std::abs(el - first_acc) > acc_tol); }) -
                     1;

  auto it_last_intermediate =
      std::find_if(it_last_acc + 1, accelerations.end(), [acc_tol](double el) { return (el < acc_tol); }) - 1;

  // Check that there are 1 or 2 intermediate points
  auto n_intermediate_1 = std::distance(it_last_acc, it_last_intermediate);

  if (n_intermediate_1 != 1 && n_intermediate_1 != 2)
  {
    return ::testing::AssertionFailure() << "Expected 1 or 2 intermediate points between acceleration and "
                                            "constant "
                                            "velocity phase but found: "
                                         << n_intermediate_1;
  }

  // Const phase (vel == 0)
  auto it_first_const = it_last_intermediate + 1;
  auto it_last_const =
      std::find_if(it_first_const, accelerations.end(), [acc_tol](double el) { return (std::abs(el) > acc_tol); }) - 1;
  // This test makes sense only if the generated traj has enough points such
  // that the trapezoid is not degenerated.
  if (std::distance(it_first_const, it_last_const) < 3)
  {
    return ::testing::AssertionFailure() << "Exptected the have at least 3 points during the phase of "
                                            "constant "
                                            "velocity.";
  }

  // Deceleration
  double deceleration = accelerations.back();
  auto it_first_dec =
      std::find_if(it_last_const + 1, accelerations.end(),
                   [deceleration, acc_tol](double el) { return (std::abs(el - deceleration) > acc_tol); }) +
      1;

  // Points between const and deceleration (again 1 or 2)
  auto n_intermediate_2 = std::distance(it_last_const, it_first_dec);
  if (n_intermediate_2 != 1 && n_intermediate_2 != 2)
  {
    return ::testing::AssertionFailure() << "Expected 1 or 2 intermediate points between constant velocity "
                                            "and "
                                            "deceleration phase but found: "
                                         << n_intermediate_2;
  }

  std::stringstream debug_stream;
  for (auto it = accelerations.begin(); it != it_last_acc + 1; it++)
  {
    debug_stream << *it << "(Acc)" << std::endl;
  }

  for (auto it = it_last_acc + 1; it != it_last_intermediate + 1; it++)
  {
    debug_stream << *it << "(Inter1)" << std::endl;
  }

  for (auto it = it_first_const; it != it_last_const + 1; it++)
  {
    debug_stream << *it << "(Const)" << std::endl;
  }

  for (auto it = it_last_const + 1; it != it_first_dec; it++)
  {
    debug_stream << *it << "(Inter2)" << std::endl;
  }

  for (auto it = it_first_dec; it != accelerations.end(); it++)
  {
    debug_stream << *it << "(Dec)" << std::endl;
  }

  ROS_DEBUG_STREAM(debug_stream.str());

  return ::testing::AssertionSuccess();
}

::testing::AssertionResult
testutils::checkCartesianTranslationalPath(const robot_trajectory::RobotTrajectoryConstPtr& trajectory,
                                           const std::string& link_name, const double acc_tol)
{
  // We require the following such that the test makes sense, else the traj
  // would have a degenerated velocity profile
  EXPECT_GT(trajectory->getWayPointCount(), 9u);

  std::vector<double> accelerations;

  // Iterate over waypoints collect accelerations
  for (size_t i = 2; i < trajectory->getWayPointCount(); ++i)
  {
    auto waypoint_pose_0 = trajectory->getWayPoint(i - 2).getFrameTransform(link_name);
    auto waypoint_pose_1 = trajectory->getWayPoint(i - 1).getFrameTransform(link_name);
    auto waypoint_pose_2 = trajectory->getWayPoint(i).getFrameTransform(link_name);

    auto t1 = trajectory->getWayPointDurationFromPrevious(i - 1);
    auto t2 = trajectory->getWayPointDurationFromPrevious(i);

    auto vel1 = (waypoint_pose_1.translation() - waypoint_pose_0.translation()).norm() / t1;
    auto vel2 = (waypoint_pose_2.translation() - waypoint_pose_1.translation()).norm() / t2;
    auto acc_transl = (vel2 - vel1) / (t1 + t2);
    accelerations.push_back(acc_transl);
  }

  return hasTrapezoidVelocity(accelerations, acc_tol);
}

::testing::AssertionResult
testutils::checkCartesianRotationalPath(const robot_trajectory::RobotTrajectoryConstPtr& trajectory,
                                        const std::string& link_name, const double rot_axis_tol, const double acc_tol)
{
  // skip computations if rotation angle is zero
  if (trajectory->getFirstWayPoint().getFrameTransform(link_name).rotation().isApprox(
          trajectory->getLastWayPoint().getFrameTransform(link_name).rotation(), rot_axis_tol))
  {
    return ::testing::AssertionSuccess();
  }

  // We require the following such that the test makes sense, else the traj
  // would have a degenerated velocity profile
  EXPECT_GT(trajectory->getWayPointCount(), 9u);

  std::vector<double> accelerations;
  std::vector<Eigen::AngleAxisd> rotation_axes;

  // Iterate over waypoints collect accelerations and rotation axes
  for (size_t i = 2; i < trajectory->getWayPointCount(); ++i)
  {
    auto waypoint_pose_0 = trajectory->getWayPoint(i - 2).getFrameTransform(link_name);
    auto waypoint_pose_1 = trajectory->getWayPoint(i - 1).getFrameTransform(link_name);
    auto waypoint_pose_2 = trajectory->getWayPoint(i).getFrameTransform(link_name);

    auto t1 = trajectory->getWayPointDurationFromPrevious(i - 1);
    auto t2 = trajectory->getWayPointDurationFromPrevious(i);

    Eigen::Quaterniond orientation0(waypoint_pose_0.rotation());
    Eigen::Quaterniond orientation1(waypoint_pose_1.rotation());
    Eigen::Quaterniond orientation2(waypoint_pose_2.rotation());

    Eigen::AngleAxisd axis1(orientation0 * orientation1.inverse());
    Eigen::AngleAxisd axis2(orientation1 * orientation2.inverse());
    if (i == 2)
    {
      rotation_axes.push_back(axis1);
    }
    rotation_axes.push_back(axis2);

    double angular_vel1 = axis1.angle() / t1;
    double angular_vel2 = axis2.angle() / t2;
    double angular_acc = (angular_vel2 - angular_vel1) / (t1 + t2);
    accelerations.push_back(angular_acc);
  }

  // Check that rotation axis is fixed
  if (std::adjacent_find(rotation_axes.begin(), rotation_axes.end(),
                         [rot_axis_tol](const Eigen::AngleAxisd& axis1, const Eigen::AngleAxisd& axis2) {
                           return ((axis1.axis() - axis2.axis()).norm() > rot_axis_tol);
                         }) != rotation_axes.end())
  {
    return ::testing::AssertionFailure() << "Rotational path of trajectory "
                                            "does not have a fixed rotation "
                                            "axis";
  }

  return hasTrapezoidVelocity(accelerations, acc_tol);
}
