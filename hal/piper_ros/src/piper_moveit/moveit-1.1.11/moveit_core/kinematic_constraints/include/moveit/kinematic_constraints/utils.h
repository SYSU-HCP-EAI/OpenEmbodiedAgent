/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit_msgs/Constraints.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <moveit/robot_state/robot_state.h>
#include <limits>

namespace XmlRpc
{
class XmlRpcValue;
}

namespace kinematic_constraints
{
/**
 * \brief Merge two sets of constraints into one.
 *
 * This just does appending of all constraints except joint
 * constraints. For members of type \ref JointConstraint, the bounds
 * specified in the parameter \e first take precedence over parameter
 * \e second
 *
 * @param [in] first The first constraint to merge
 * @param [in] second The second constraint to merge
 *
 * @return The merged set of constraints
 */
moveit_msgs::Constraints mergeConstraints(const moveit_msgs::Constraints& first, const moveit_msgs::Constraints& second);

/** \brief Check if any constraints were specified */
[[deprecated("Use moveit/utils/message_checks.h instead")]] bool isEmpty(const moveit_msgs::Constraints& constr);

std::size_t countIndividualConstraints(const moveit_msgs::Constraints& constr);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a joint group.  The full constraint will contain a
 * vector of type \ref JointConstraint, one for each DOF in the group.
 *
 * @param [in] state The state from which to generate goal joint constraints
 * @param [in] jmg The group for which to generate goal joint constraints
 * @param [in] tolerance_below The below tolerance to apply to all constraints
 * @param [in] tolerance_above The above tolerance to apply to all constraints
 *
 * @return A full constraint message containing all the joint constraints
 */
moveit_msgs::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                  const moveit::core::JointModelGroup* jmg, double tolerance_below,
                                                  double tolerance_above);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a joint group.  The full constraint will contain a
 * vector of type \ref JointConstraint, one for each DOF in the group.
 *
 * @param [in] state The state from which to generate goal joint constraints
 * @param [in] jmg The group for which to generate joint constraints
 * @param [in] tolerance A tolerance to apply both above and below for all constraints
 *
 * @return A full constraint message containing all the joint constraints
 */
moveit_msgs::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                  const moveit::core::JointModelGroup* jmg,
                                                  double tolerance = std::numeric_limits<double>::epsilon());

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint will contain a
 * \ref PositionConstraint and a \ref OrientationConstraint,
 * constructed from the pose. A sphere will be used to represent the
 * constraint region for the \ref PositionConstraint.
 *
 * @param [in] link_name The link name for both constraints
 * @param [in] pose The pose stamped to be used for the target region.
 * @param [in] tolerance_pos The dimension of the sphere associated with the target region of the \ref
 *PositionConstraint
 * @param [in] tolerance_angle The value to assign to the absolute tolerances of the \ref OrientationConstraint
 *
 * @return A full constraint message containing both constraints
 */
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name, const geometry_msgs::PoseStamped& pose,
                                                  double tolerance_pos = 1e-3, double tolerance_angle = 1e-2);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint will contain a
 * \ref PositionConstraint and a \ref OrientationConstraint,
 * constructed from the pose. A box  will be used to represent the
 * constraint region for the \ref PositionConstraint.
 *
 * @param [in] link_name The link name for both constraints
 * @param [in] pose The pose stamped to be used for the target region.
 * @param [in] tolerance_pos The dimensions of the box (xyz) associated with the target region of the \ref
 *PositionConstraint
 * @param [in] tolerance_angle The values to assign to the absolute tolerances (xyz) of the \ref OrientationConstraint
 *
 * @return A full constraint message containing both constraints
 */
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name, const geometry_msgs::PoseStamped& pose,
                                                  const std::vector<double>& tolerance_pos,
                                                  const std::vector<double>& tolerance_angle);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link. The full constraint message will
 * contain only an \ref OrientationConstraint.
 *
 * @param [in] link_name The link name for the \ref OrientationConstraint
 * @param [in] quat The quaternion for the \ref OrientationConstraint
 * @param [in] tolerance The absolute axes tolerances to apply to the \ref OrientationConstraint
 *
 * @return A full constraint message containing the orientation constraint
 */
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::QuaternionStamped& quat,
                                                  double tolerance = 1e-2);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint message will
 * contain only a \ref PositionConstraint.  A sphere will be used to
 * represent the constraint region.
 *
 * @param [in] link_name The link name for the \ref PositionConstraint
 * @param [in] reference_point A point corresponding to the target_point_offset of the \ref PositionConstraint
 * @param [in] goal_point The position associated with the constraint region
 * @param [in] tolerance The radius associated with the sphere volume associated with the constraint region
 *
 * @return A full constraint message containing the position constraint
 */
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::Point& reference_point,
                                                  const geometry_msgs::PointStamped& goal_point,
                                                  double tolerance = 1e-3);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint message will
 * contain only a \ref PositionConstraint.  A sphere will be used to
 * represent the constraint region.
 *
 * @param [in] link_name The link name for the \ref PositionConstraint
 * @param [in] goal_point The position associated with the constraint region
 * @param [in] tolerance The radius associated with the sphere volume associated with the constraint region
 *
 * @return A full constraint message containing the position constraint
 */
moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::PointStamped& goal_point,
                                                  double tolerance = 1e-3);

/**
 * \brief extract constraint message from XmlRpc node.
 *
 * This can be used to construct a Constraints message from
 * specifications uploaded on the parameter server.
 *
 * @param [in] params XmlRpc node of the parameter specification
 * @param [out] constraints The constructed constraints message
 *
 * @return was the construction successful?
 */
bool constructConstraints(XmlRpc::XmlRpcValue& params, moveit_msgs::Constraints& constraints);

/**
 * \brief Resolves frames used in constraints to links in the robot model.
 *
 * The link_name field of a constraint is changed from the name of an object's frame or subframe
 * to the name of the robot link that the object is attached to.
 *
 * This is used in a planning request adapter which ensures that the planning problem is defined
 * properly (the attached objects' frames are not known to the planner).
 *
 * @param [in] state The RobotState used to resolve frames.
 * @param [in] constraints The constraint to resolve.
 */
bool resolveConstraintFrames(const moveit::core::RobotState& state, moveit_msgs::Constraints& constraints);
}  // namespace kinematic_constraints
