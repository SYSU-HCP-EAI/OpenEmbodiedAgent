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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan, Mike Lautman */

#pragma once

#include <moveit/robot_state/robot_state.h>

#include <boost/assert.hpp>

namespace moveit
{
namespace core
{
/** \brief Struct for containing jump_threshold.

    For the purposes of maintaining API, we support both \e jump_threshold_factor which provides a scaling factor for
    detecting joint space jumps and \e revolute_jump_threshold and \e prismatic_jump_threshold which provide absolute
    thresholds for detecting joint space jumps. */
struct JumpThreshold
{
  double factor;
  double revolute;   // Radians
  double prismatic;  // Meters

  explicit JumpThreshold() : factor(0.0), revolute(0.0), prismatic(0.0)
  {
  }

  explicit JumpThreshold(double jt_factor) : JumpThreshold()
  {
    factor = jt_factor;
  }

  explicit JumpThreshold(double jt_revolute, double jt_prismatic) : JumpThreshold()
  {
    revolute = jt_revolute;    // Radians
    prismatic = jt_prismatic;  // Meters
  }
};

/** \brief Struct for containing max_step for computeCartesianPath

    Setting translation to zero will disable checking for translations. The same goes for rotation.
    Initializing with only one value (translation) sets the rotation such that
    1 cm of allowed translation = 2 degrees of allowed rotation. */
struct MaxEEFStep
{
  MaxEEFStep(double translation, double rotation) : translation(translation), rotation(rotation)
  {
  }

  MaxEEFStep(double step_size) : translation(step_size), rotation(3.5 * step_size)  // 0.035 rad = 2 deg
  {
  }

  double translation;  // Meters
  double rotation;     // Radians
};

class CartesianInterpolator
{
  // TODO(mlautman): Eventually, this planner should be moved out of robot_state

public:
  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path for a particular link.

     The Cartesian path to be followed is specified as a \e translation vector to be followed by the robot \e link.
     This vector is assumed to be specified either in the global reference frame or in the local
     reference frame of the link.
     The resulting joint values are stored in the vector \e traj, one by one. The maximum distance in
     Cartesian space between consecutive points on the resulting path is specified in the \e MaxEEFStep struct which
     provides two fields: translation and rotation. If a \e validCallback is specified, this is passed to the internal
     call to setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to
     the distance that was achieved and for which corresponding states were added to the path.  At the end of the
     function call, the state of the group corresponds to the last attempted Cartesian pose.

     During the computation of the trajectory, it is usually preferred if consecutive joint values do not 'jump' by a
     large amount in joint space, even if the Cartesian distance between the corresponding points is small as expected.
     To account for this, the \e jump_threshold struct is provided, which comprises three fields:
     \e jump_threshold_factor, \e revolute_jump_threshold and \e prismatic_jump_threshold.
     If either \e revolute_jump_threshold or \e prismatic_jump_threshold  are non-zero, we test for absolute jumps.
     If \e jump_threshold_factor is non-zero, we test for relative jumps. Otherwise (all params are zero), jump
     detection is disabled.

     For relative jump detection, the average joint-space distance between consecutive points in the trajectory is
     computed. If any individual joint-space motion delta is larger then this average distance by a factor of
     \e jump_threshold_factor, this step is considered a failure and the returned path is truncated up to just
     before the jump.

     For absolute jump thresholds, if any individual joint-space motion delta is larger then \e revolute_jump_threshold
     for revolute joints or \e prismatic_jump_threshold for prismatic joints then this step is considered a failure and
     the returned path is truncated up to just before the jump.*/
  static double
  computeCartesianPath(RobotState* start_state, const JointModelGroup* group,
                       std::vector<std::shared_ptr<RobotState>>& traj, const LinkModel* link,
                       const Eigen::Vector3d& translation, bool global_reference_frame, const MaxEEFStep& max_step,
                       const JumpThreshold& jump_threshold,
                       const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                       const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions());

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular link.

     In contrast to the previous function, the translation vector is specified as a (unit) direction vector and
     a distance. */
  static double
  computeCartesianPath(RobotState* start_state, const JointModelGroup* group,
                       std::vector<std::shared_ptr<RobotState>>& traj, const LinkModel* link,
                       const Eigen::Vector3d& direction, bool global_reference_frame, double distance,
                       const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                       const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                       const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions())
  {
    return computeCartesianPath(start_state, group, traj, link, distance * direction, global_reference_frame, max_step,
                                jump_threshold, validCallback, options);
  }

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular frame.

     In contrast to the previous function, the Cartesian path is specified as a target frame to be reached (\e target)
     for a virtual frame attached to the robot \e link with the given \e link_offset.
     The target frame is assumed to be specified either w.r.t. to the global reference frame or the virtual link frame.
     This function returns the fraction (0..1) of path that was achieved. All other comments from the previous function apply. */
  static double
  computeCartesianPath(RobotState* start_state, const JointModelGroup* group,
                       std::vector<std::shared_ptr<RobotState>>& traj, const LinkModel* link,
                       const Eigen::Isometry3d& target, bool global_reference_frame, const MaxEEFStep& max_step,
                       const JumpThreshold& jump_threshold,
                       const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                       const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                       const Eigen::Isometry3d& link_offset = Eigen::Isometry3d::Identity());

  /** \brief Compute the sequence of joint values that perform a general Cartesian path.

     In contrast to the previous functions, the Cartesian path is specified as a set of \e waypoints to be sequentially
     reached by the virtual frame attached to the robot \e link. The waypoints are transforms given either w.r.t. the global
     reference frame or the virtual frame at the immediately preceding waypoint. The virtual frame needs
     to move in a straight line between two consecutive waypoints. All other comments apply. */
  static double
  computeCartesianPath(RobotState* start_state, const JointModelGroup* group,
                       std::vector<std::shared_ptr<RobotState>>& traj, const LinkModel* link,
                       const EigenSTL::vector_Isometry3d& waypoints, bool global_reference_frame,
                       const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
                       const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
                       const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                       const Eigen::Isometry3d& link_offset = Eigen::Isometry3d::Identity());

  /** \brief Tests joint space jumps of a trajectory.

     If \e jump_threshold_factor is non-zero, we test for relative jumps.
     If \e revolute_jump_threshold  or \e prismatic_jump_threshold are non-zero, we test for absolute jumps.
     Both tests can be combined. If all params are zero, jump detection is disabled.
     For relative jump detection, the average joint-space distance between consecutive points in the trajectory is
     computed. If any individual joint-space motion delta is larger then this average distance by a factor of
     \e jump_threshold_factor, this step is considered a failure and the returned path is truncated up to just
     before the jump.

     @param group The joint model group of the robot state.
     @param traj The trajectory that should be tested.
     @param jump_threshold The struct holding jump thresholds to determine if a joint space jump has occurred.
     @return The fraction of the trajectory that passed.

     TODO: move to more appropriate location
  */
  static double checkJointSpaceJump(const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& traj,
                                    const JumpThreshold& jump_threshold);

  /** \brief Tests for relative joint space jumps of the trajectory \e traj.

     First, the average distance between adjacent trajectory points is computed. If two adjacent trajectory points
     have distance > \e jump_threshold_factor * average, the trajectory is truncated at this point.

     @param group The joint model group of the robot state.
     @param traj The trajectory that should be tested.
     @param jump_threshold_factor The threshold to determine if a joint space jump has occurred .
     @return The fraction of the trajectory that passed.

     TODO: move to more appropriate location
   */
  static double checkRelativeJointSpaceJump(const JointModelGroup* group,
                                            std::vector<std::shared_ptr<RobotState>>& traj,
                                            double jump_threshold_factor);

  /** \brief Tests for absolute joint space jumps of the trajectory \e traj.

     The joint-space difference between consecutive waypoints is computed for each active joint and compared to the
     absolute thresholds \e revolute_jump_threshold for revolute joints and \e prismatic_jump_threshold for prismatic
     joints. If these thresholds are exceeded, the trajectory is truncated.

     @param group The joint model group of the robot state.
     @param traj The trajectory that should be tested.
     @param revolute_jump_threshold Absolute joint-space threshold for revolute joints.
     @param prismatic_jump_threshold Absolute joint-space threshold for prismatic joints.
     @return The fraction of the trajectory that passed.

     TODO: move to more appropriate location
  */
  static double checkAbsoluteJointSpaceJump(const JointModelGroup* group,
                                            std::vector<std::shared_ptr<RobotState>>& traj,
                                            double revolute_jump_threshold, double prismatic_jump_threshold);
};

}  // end of namespace core
}  // end of namespace moveit
