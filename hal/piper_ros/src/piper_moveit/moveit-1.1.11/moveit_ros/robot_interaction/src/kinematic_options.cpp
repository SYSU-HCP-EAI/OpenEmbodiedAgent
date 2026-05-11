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

#include <moveit/robot_interaction/kinematic_options.h>
#include <boost/static_assert.hpp>
#include <ros/console.h>

robot_interaction::KinematicOptions::KinematicOptions() : timeout_seconds_(0.0)  // 0.0 = use default timeout
{
}

// This is intended to be called as a ModifyStateFunction to modify the state
// maintained by a LockedRobotState in place.
bool robot_interaction::KinematicOptions::setStateFromIK(moveit::core::RobotState& state, const std::string& group,
                                                         const std::string& tip, const geometry_msgs::Pose& pose) const
{
  const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group);
  if (!jmg)
  {
    ROS_ERROR("No getJointModelGroup('%s') found", group.c_str());
    return false;
  }
  bool result = state.setFromIK(jmg, pose, tip,
                                // limit timeout to 0.1s if set from JMG's default, i.e. when timeout_seconds_ == 0
                                timeout_seconds_ > 0.0 ? timeout_seconds_ : std::min(0.1, jmg->getDefaultIKTimeout()),
                                state_validity_callback_, options_);
  state.update();
  return result;
}

void robot_interaction::KinematicOptions::setOptions(const KinematicOptions& source, OptionBitmask fields)
{
// This function is implemented with the O_FIELDS and QO_FIELDS macros to
// ensure that any fields added to robot_interaction::KinematicOptions or
// kinematics::KinematicsQueryOptions are also added here and to the
// KinematicOptions::OptionBitmask enum.

// This needs to represent all the fields in
// robot_interaction::KinematicOptions except options_
#define O_FIELDS(F)                                                                                                    \
  F(double, timeout_seconds_, TIMEOUT)                                                                                 \
  F(moveit::core::GroupStateValidityCallbackFn, state_validity_callback_, STATE_VALIDITY_CALLBACK)

// This needs to represent all the fields in
// kinematics::KinematicsQueryOptions
#define QO_FIELDS(F)                                                                                                   \
  F(bool, lock_redundant_joints, LOCK_REDUNDANT_JOINTS)                                                                \
  F(bool, return_approximate_solution, RETURN_APPROXIMATE_SOLUTION)                                                    \
  F(::kinematics::DiscretizationMethods::DiscretizationMethod, discretization_method, DISCRETIZATION_METHOD)

  // This structure should be identical to kinematics::KinematicsQueryOptions
  // This is only used in the BOOST_STATIC_ASSERT below.
  struct DummyKinematicsQueryOptions
  {
#define F(type, member, enumval) type member;
    QO_FIELDS(F)
#undef F
  };
  // This structure should be identical to robot_interaction::KinematicOptions
  // This is only used in the BOOST_STATIC_ASSERT below.
  struct DummyKinematicOptions
  {
#define F(type, member, enumval) type member;
    O_FIELDS(F)
#undef F
    DummyKinematicsQueryOptions options_;
  };

  // If these asserts fails it means that fields were added to
  // kinematics::KinematicsQueryOptions or robot_interaction::KinematicOptions
  // and not added to the O_FIELDS and QO_FIELDS definitions above. To fix add
  // any new fields to the definitions above.
  BOOST_STATIC_ASSERT(sizeof(kinematics::KinematicsQueryOptions) == sizeof(DummyKinematicsQueryOptions));
  BOOST_STATIC_ASSERT(sizeof(KinematicOptions) == sizeof(DummyKinematicOptions));

// copy fields from other to this if its bit is set in fields
#define F(type, member, enumval)                                                                                       \
  if (fields & KinematicOptions::enumval)                                                                              \
    member = source.member;
  O_FIELDS(F)
#undef F

// copy fields from other.options_ to this.options_ if its bit is set in
// fields
#define F(type, member, enumval)                                                                                       \
  if (fields & KinematicOptions::enumval)                                                                              \
    options_.member = source.options_.member;
  QO_FIELDS(F)
#undef F
}
