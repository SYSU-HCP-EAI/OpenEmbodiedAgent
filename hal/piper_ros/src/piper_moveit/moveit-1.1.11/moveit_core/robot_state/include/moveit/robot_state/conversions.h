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

/* Author: Ioan Sucan, Dave Coleman */

#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit_msgs/RobotState.h>

namespace moveit
{
namespace core
{
/**
 * @brief Convert a joint state to a MoveIt robot state
 * @param joint_state The input joint state to be converted
 * @param state The resultant MoveIt robot state
 * @return True if successful, false if failed for any reason
 */
bool jointStateToRobotState(const sensor_msgs::JointState& joint_state, RobotState& state);

/**
 * @brief Convert a robot state msg (with accompanying extra transforms) to a MoveIt robot state
 * @param tf An instance of a transforms object
 * @param robot_state The input robot state msg
 * @param state The resultant MoveIt robot state
 * @param copy_attached_bodies Flag to include attached objects in robot state copy
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(const Transforms& tf, const moveit_msgs::RobotState& robot_state, RobotState& state,
                               bool copy_attached_bodies = true);

/**
 * @brief Convert a robot state msg (with accompanying extra transforms) to a MoveIt robot state
 * @param robot_state The input robot state msg
 * @param state The resultant MoveIt robot state
 * @param copy_attached_bodies Flag to include attached objects in robot state copy
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(const moveit_msgs::RobotState& robot_state, RobotState& state,
                               bool copy_attached_bodies = true);

/**
 * @brief Convert a MoveIt robot state to a robot state message
 * @param state The input MoveIt robot state object
 * @param robot_state The resultant RobotState *message
 * @param copy_attached_bodies Flag to include attached objects in robot state copy
 */
void robotStateToRobotStateMsg(const RobotState& state, moveit_msgs::RobotState& robot_state,
                               bool copy_attached_bodies = true);

/**
 * @brief Convert AttachedBodies to AttachedCollisionObjects
 * @param attached_bodies The input MoveIt attached body objects
 * @param attached_collision_objs The resultant AttachedCollisionObject messages
 */
void attachedBodiesToAttachedCollisionObjectMsgs(
    const std::vector<const AttachedBody*>& attached_bodies,
    std::vector<moveit_msgs::AttachedCollisionObject>& attached_collision_objs);
/**
 * @brief Convert a MoveIt robot state to a joint state message
 * @param state The input MoveIt robot state object
 * @param robot_state The resultant JointState message
 */
void robotStateToJointStateMsg(const RobotState& state, sensor_msgs::JointState& joint_state);

/**
 * @brief Convert a joint trajectory point to a MoveIt robot state
 * @param joint_trajectory The input msg
 * @param point_id The index of the trajectory point in the joint trajectory.
 * @param state The resultant MoveIt robot state
 * @return True if successful, false if failed for any reason
 */
bool jointTrajPointToRobotState(const trajectory_msgs::JointTrajectory& trajectory, std::size_t point_id,
                                RobotState& state);

/**
 * @brief Convert a MoveIt robot state to common separated values (CSV) on a single line that is
 *        outputted to a stream e.g. for file saving
 * @param state - The input MoveIt robot state object
 * @param out - a file stream, or any other stream
 * @param include_header - flag to prefix the output with a line of joint names.
 * @param separator - allows to override the comma seperator with any symbol, such as a white space
 */
void robotStateToStream(const RobotState& state, std::ostream& out, bool include_header = true,
                        const std::string& separator = ",");

/**
 * @brief Convert a MoveIt robot state to common separated values (CSV) on a single line that is
 *        outputted to a stream e.g. for file saving. This version can order by joint model groups
 * @param state - The input MoveIt robot state object
 * @param out - a file stream, or any other stream
 * @param joint_group_ordering - output joints based on ordering of joint groups
 * @param include_header - flag to prefix the output with a line of joint names.
 * @param separator - allows to override the comma seperator with any symbol, such as a white space
 */
void robotStateToStream(const RobotState& state, std::ostream& out,
                        const std::vector<std::string>& joint_groups_ordering, bool include_header = true,
                        const std::string& separator = ",");

/**
 * \brief Convert a string of joint values from a file (CSV) or input source into a RobotState
 * @param state - the output MoveIt robot state object
 * @param line - the input string of joint values
 * @param separator - allows to override the comma seperator with any symbol, such as a white space
 * \return true on success
 */
void streamToRobotState(RobotState& state, const std::string& line, const std::string& separator = ",");
}  // namespace core
}  // namespace moveit
