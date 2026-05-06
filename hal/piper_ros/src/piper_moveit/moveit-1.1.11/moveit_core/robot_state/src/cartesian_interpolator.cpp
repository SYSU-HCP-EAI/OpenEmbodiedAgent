/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  Copyright (c) 2019, PickNik LLC.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Sachin Chitta, Acorn Pooley, Mario Prats, Dave Coleman */

#include <memory>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <geometric_shapes/check_isometry.h>

namespace moveit
{
namespace core
{
/** \brief It is recommended that there are at least 10 steps per trajectory
 * for testing jump thresholds with computeCartesianPath. With less than 10 steps
 * it is difficult to choose a jump_threshold parameter that effectively separates
 * valid paths from paths with large joint space jumps. */
static const std::size_t MIN_STEPS_FOR_JUMP_THRESH = 10;

const std::string LOGNAME = "cartesian_interpolator";

double CartesianInterpolator::computeCartesianPath(RobotState* start_state, const JointModelGroup* group,
                                                   std::vector<RobotStatePtr>& traj, const LinkModel* link,
                                                   const Eigen::Vector3d& translation, bool global_reference_frame,
                                                   const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                                                   const GroupStateValidityCallbackFn& validCallback,
                                                   const kinematics::KinematicsQueryOptions& options)
{
  const double distance = translation.norm();
  // The target pose is obtained by adding the translation vector to the link's current pose
  Eigen::Isometry3d pose = start_state->getGlobalLinkTransform(link);

  // the translation direction can be specified w.r.t. the local link frame (then rotate into global frame)
  pose.translation() += global_reference_frame ? translation : pose.linear() * translation;

  // call computeCartesianPath for the computed target pose in the global reference frame
  return distance * computeCartesianPath(start_state, group, traj, link, pose, true, max_step, jump_threshold,
                                         validCallback, options);
}

double CartesianInterpolator::computeCartesianPath(RobotState* start_state, const JointModelGroup* group,
                                                   std::vector<RobotStatePtr>& traj, const LinkModel* link,
                                                   const Eigen::Isometry3d& target, bool global_reference_frame,
                                                   const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                                                   const GroupStateValidityCallbackFn& validCallback,
                                                   const kinematics::KinematicsQueryOptions& options,
                                                   const Eigen::Isometry3d& link_offset)
{
  // check unsanitized inputs for non-isometry
  ASSERT_ISOMETRY(target)
  ASSERT_ISOMETRY(link_offset)

  const std::vector<const JointModel*>& cjnt = group->getContinuousJointModels();
  // make sure that continuous joints wrap
  for (const JointModel* joint : cjnt)
    start_state->enforceBounds(joint);

  // Cartesian pose we start from
  Eigen::Isometry3d start_pose = start_state->getGlobalLinkTransform(link) * link_offset;
  Eigen::Isometry3d offset = link_offset.inverse();

  // the target can be in the local reference frame (in which case we rotate it)
  Eigen::Isometry3d rotated_target = global_reference_frame ? target : start_pose * target;

  Eigen::Quaterniond start_quaternion(start_pose.linear());
  Eigen::Quaterniond target_quaternion(rotated_target.linear());

  if (max_step.translation <= 0.0 && max_step.rotation <= 0.0)
  {
    ROS_ERROR_NAMED(LOGNAME,
                    "Invalid MaxEEFStep passed into computeCartesianPath. Both the MaxEEFStep.rotation and "
                    "MaxEEFStep.translation components must be non-negative and at least one component must be "
                    "greater than zero");
    return 0.0;
  }

  double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

  // decide how many steps we will need for this trajectory
  std::size_t translation_steps = 0;
  if (max_step.translation > 0.0)
    translation_steps = floor(translation_distance / max_step.translation);

  std::size_t rotation_steps = 0;
  if (max_step.rotation > 0.0)
    rotation_steps = floor(rotation_distance / max_step.rotation);

  // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
  std::size_t steps = std::max(translation_steps, rotation_steps) + 1;
  if (jump_threshold.factor > 0 && steps < MIN_STEPS_FOR_JUMP_THRESH)
    steps = MIN_STEPS_FOR_JUMP_THRESH;

  // To limit absolute joint-space jumps, we pass consistency limits to the IK solver
  std::vector<double> consistency_limits;
  if (jump_threshold.prismatic > 0 || jump_threshold.revolute > 0)
    for (const JointModel* jm : group->getActiveJointModels())
    {
      double limit;
      switch (jm->getType())
      {
        case JointModel::REVOLUTE:
          limit = jump_threshold.revolute;
          break;
        case JointModel::PRISMATIC:
          limit = jump_threshold.prismatic;
          break;
        default:
          limit = 0.0;
      }
      if (limit == 0.0)
        limit = jm->getMaximumExtent();
      consistency_limits.push_back(limit);
    }

  traj.clear();
  traj.push_back(std::make_shared<moveit::core::RobotState>(*start_state));

  double last_valid_percentage = 0.0;
  for (std::size_t i = 1; i <= steps; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

    // Explicitly use a single IK attempt only: We want a smooth trajectory.
    // Random seeding (of additional attempts) would probably create IK jumps.
    if (start_state->setFromIK(group, pose * offset, link->getName(), consistency_limits, 0.0, validCallback, options))
      traj.push_back(std::make_shared<moveit::core::RobotState>(*start_state));
    else
      break;

    last_valid_percentage = percentage;
  }

  last_valid_percentage *= checkJointSpaceJump(group, traj, jump_threshold);

  return last_valid_percentage;
}

double CartesianInterpolator::computeCartesianPath(
    RobotState* start_state, const JointModelGroup* group, std::vector<RobotStatePtr>& traj, const LinkModel* link,
    const EigenSTL::vector_Isometry3d& waypoints, bool global_reference_frame, const MaxEEFStep& max_step,
    const JumpThreshold& jump_threshold, const GroupStateValidityCallbackFn& validCallback,
    const kinematics::KinematicsQueryOptions& options, const Eigen::Isometry3d& link_offset)
{
  double percentage_solved = 0.0;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    // Don't test joint space jumps for every waypoint, test them later on the whole trajectory.
    static const JumpThreshold NO_JOINT_SPACE_JUMP_TEST;
    std::vector<RobotStatePtr> waypoint_traj;
    double wp_percentage_solved =
        computeCartesianPath(start_state, group, waypoint_traj, link, waypoints[i], global_reference_frame, max_step,
                             NO_JOINT_SPACE_JUMP_TEST, validCallback, options, link_offset);
    if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
    {
      percentage_solved = (double)(i + 1) / (double)waypoints.size();
      std::vector<RobotStatePtr>::iterator start = waypoint_traj.begin();
      if (i > 0 && !waypoint_traj.empty())
        std::advance(start, 1);
      traj.insert(traj.end(), start, waypoint_traj.end());
    }
    else
    {
      percentage_solved += wp_percentage_solved / (double)waypoints.size();
      std::vector<RobotStatePtr>::iterator start = waypoint_traj.begin();
      if (i > 0 && !waypoint_traj.empty())
        std::advance(start, 1);
      traj.insert(traj.end(), start, waypoint_traj.end());
      break;
    }
  }

  percentage_solved *= checkJointSpaceJump(group, traj, jump_threshold);

  return percentage_solved;
}

double CartesianInterpolator::checkJointSpaceJump(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                                  const JumpThreshold& jump_threshold)
{
  double percentage_solved = 1.0;
  if (traj.size() <= 1)
    return percentage_solved;

  if (jump_threshold.factor > 0.0)
    percentage_solved *= checkRelativeJointSpaceJump(group, traj, jump_threshold.factor);

  if (jump_threshold.revolute > 0.0 || jump_threshold.prismatic > 0.0)
    percentage_solved *= checkAbsoluteJointSpaceJump(group, traj, jump_threshold.revolute, jump_threshold.prismatic);

  return percentage_solved;
}

double CartesianInterpolator::checkRelativeJointSpaceJump(const JointModelGroup* group,
                                                          std::vector<RobotStatePtr>& traj,
                                                          double jump_threshold_factor)
{
  if (traj.size() < MIN_STEPS_FOR_JUMP_THRESH)
  {
    ROS_WARN_NAMED(LOGNAME,
                   "The computed trajectory is too short to detect jumps in joint-space "
                   "Need at least %zu steps, only got %zu. Try a lower max_step.",
                   MIN_STEPS_FOR_JUMP_THRESH, traj.size());
  }

  std::vector<double> dist_vector;
  dist_vector.reserve(traj.size() - 1);
  double total_dist = 0.0;
  for (std::size_t i = 1; i < traj.size(); ++i)
  {
    double dist_prev_point = traj[i]->distance(*traj[i - 1], group);
    dist_vector.push_back(dist_prev_point);
    total_dist += dist_prev_point;
  }

  double percentage = 1.0;
  // compute the average distance between the states we looked at
  double thres = jump_threshold_factor * (total_dist / (double)dist_vector.size());
  for (std::size_t i = 0; i < dist_vector.size(); ++i)
    if (dist_vector[i] > thres)
    {
      ROS_DEBUG_NAMED(LOGNAME, "Truncating Cartesian path due to detected jump in joint-space distance");
      percentage = (double)(i + 1) / (double)(traj.size());
      traj.resize(i + 1);
      break;
    }

  return percentage;
}

double CartesianInterpolator::checkAbsoluteJointSpaceJump(const JointModelGroup* group,
                                                          std::vector<RobotStatePtr>& traj, double revolute_threshold,
                                                          double prismatic_threshold)
{
  bool check_revolute = revolute_threshold > 0.0;
  bool check_prismatic = prismatic_threshold > 0.0;

  double joint_threshold;
  bool check_joint;
  bool still_valid = true;
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (std::size_t traj_ix = 0, ix_end = traj.size() - 1; traj_ix != ix_end; ++traj_ix)
  {
    for (auto& joint : joints)
    {
      switch (joint->getType())
      {
        case JointModel::REVOLUTE:
          check_joint = check_revolute;
          joint_threshold = revolute_threshold;
          break;
        case JointModel::PRISMATIC:
          check_joint = check_prismatic;
          joint_threshold = prismatic_threshold;
          break;
        default:
          ROS_WARN_NAMED(LOGNAME,
                         "Joint %s has not supported type %s. \n"
                         "checkAbsoluteJointSpaceJump only supports prismatic and revolute joints.",
                         joint->getName().c_str(), joint->getTypeName().c_str());
          continue;
      }
      if (check_joint)
      {
        double distance = traj[traj_ix]->distance(*traj[traj_ix + 1], joint);
        if (distance > joint_threshold)
        {
          ROS_DEBUG_NAMED(LOGNAME, "Truncating Cartesian path due to detected jump of %.4f > %.4f in joint %s",
                          distance, joint_threshold, joint->getName().c_str());
          still_valid = false;
          break;
        }
      }
    }

    if (!still_valid)
    {
      double percent_valid = (double)(traj_ix + 1) / (double)(traj.size());
      traj.resize(traj_ix + 1);
      return percent_valid;
    }
  }
  return 1.0;
}

}  // end of namespace core
}  // end of namespace moveit
