/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI International
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

/* Author: Acorn Pooley */

#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>

namespace robot_interaction
{
// Options for inverse kinematics calculations.
//
// This is intended to be lightweight and passable by value.  No virtual
// functions and no destructor.
struct KinematicOptions
{
  /// Constructor - set all options to reasonable default values
  KinematicOptions();

  /// Bits corresponding to each member.
  /// NOTE: when adding fields to this structure also add the field to this
  /// enum and to the setOptions() method.
  enum OptionBitmask
  {
    TIMEOUT = 0x00000001,                      // timeout_seconds_
    STATE_VALIDITY_CALLBACK = 0x00000002,      // state_validity_callback_
    LOCK_REDUNDANT_JOINTS = 0x00000004,        // options_.lock_redundant_joints
    RETURN_APPROXIMATE_SOLUTION = 0x00000008,  // options_.return_approximate_solution
    DISCRETIZATION_METHOD = 0x00000010,
    ALL_QUERY_OPTIONS = LOCK_REDUNDANT_JOINTS | RETURN_APPROXIMATE_SOLUTION | DISCRETIZATION_METHOD,
    ALL = 0x7fffffff
  };

  /// Set \e state using inverse kinematics
  /// @param state the state to set
  /// @param group name of group whose joints can move
  /// @param tip link that will be posed
  /// @param pose desired pose of tip link
  /// @param result true if IK succeeded.
  bool setStateFromIK(moveit::core::RobotState& state, const std::string& group, const std::string& tip,
                      const geometry_msgs::Pose& pose) const;

  /// Copy a subset of source to this.
  /// For each bit set in fields the corresponding member is copied from
  /// source to this.
  void setOptions(const KinematicOptions& source, OptionBitmask fields = ALL);

  /// max time an IK attempt can take before we give up.
  double timeout_seconds_;

  /// This is called to determine if the state is valid
  moveit::core::GroupStateValidityCallbackFn state_validity_callback_;

  /// other options
  kinematics::KinematicsQueryOptions options_;
};
}  // namespace robot_interaction
