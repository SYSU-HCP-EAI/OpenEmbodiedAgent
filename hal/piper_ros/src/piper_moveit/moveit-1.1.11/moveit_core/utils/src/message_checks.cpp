/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Universitaet Hamburg.
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
 *   * Neither the name of Hamburg University nor the names of its
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

/* Author: Michael Goerner */

#include <moveit/utils/message_checks.h>

namespace moveit
{
namespace core
{
bool isEmpty(const moveit_msgs::PlanningScene& msg)
{
  return msg.name.empty() && msg.fixed_frame_transforms.empty() && msg.allowed_collision_matrix.entry_names.empty() &&
         msg.link_padding.empty() && msg.link_scale.empty() && isEmpty(msg.robot_state) && isEmpty(msg.world);
}

bool isEmpty(const moveit_msgs::PlanningSceneWorld& msg)
{
  return msg.collision_objects.empty() && msg.octomap.octomap.data.empty();
}

bool isEmpty(const moveit_msgs::RobotState& msg)
{
  /* a state is empty if it includes no information and it is a diff; if the state is not a diff, then the implicit
     information is
     that the set of attached bodies is empty, so they must be cleared from the state to be updated */
  return static_cast<bool>(msg.is_diff) && msg.multi_dof_joint_state.joint_names.empty() &&
         msg.joint_state.name.empty() && msg.attached_collision_objects.empty() && msg.joint_state.position.empty() &&
         msg.joint_state.velocity.empty() && msg.joint_state.effort.empty() &&
         msg.multi_dof_joint_state.transforms.empty() && msg.multi_dof_joint_state.twist.empty() &&
         msg.multi_dof_joint_state.wrench.empty();
}

bool isEmpty(const moveit_msgs::Constraints& constr)
{
  return constr.position_constraints.empty() && constr.orientation_constraints.empty() &&
         constr.visibility_constraints.empty() && constr.joint_constraints.empty();
}

bool isEmpty(const geometry_msgs::Quaternion& msg)
{
  return msg.x == 0.0 && msg.y == 0.0 && msg.z == 0.0 && msg.w == 0.0;
}

bool isEmpty(const geometry_msgs::Pose& msg)
{
  return msg.position.x == 0.0 && msg.position.y == 0.0 && msg.position.z == 0.0 && isEmpty(msg.orientation);
}

}  // namespace core
}  // namespace moveit
