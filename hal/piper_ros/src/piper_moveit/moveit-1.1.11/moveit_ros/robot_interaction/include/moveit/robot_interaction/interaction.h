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

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <moveit/robot_state/robot_state.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace moveit
{
namespace core
{
class RobotState;
}
}  // namespace moveit

namespace robot_interaction
{
namespace InteractionStyle
{
/// The different types of interaction that can be constructed for an end
/// effector This is a bitmask so OR together the parts you want.
enum InteractionStyle
{
  POSITION_ARROWS = 1,      // arrows to change position
  ORIENTATION_CIRCLES = 2,  // circles to change orientation
  POSITION_SPHERE = 4,      // sphere: drag to change position
  ORIENTATION_SPHERE = 8,   // sphere: drag to change orientation
  POSITION_EEF = 16,        // drag end effector to change position
  ORIENTATION_EEF = 32,     // drag end effector to change orientation
  FIXED = 64,               // keep arrow and circle axis fixed

  POSITION = POSITION_ARROWS | POSITION_SPHERE | POSITION_EEF,
  ORIENTATION = ORIENTATION_CIRCLES | ORIENTATION_SPHERE | ORIENTATION_EEF,
  SIX_DOF = POSITION | ORIENTATION,
  SIX_DOF_SPHERE = POSITION_SPHERE | ORIENTATION_SPHERE,
  POSITION_NOSPHERE = POSITION_ARROWS | POSITION_EEF,
  ORIENTATION_NOSPHERE = ORIENTATION_CIRCLES | ORIENTATION_EEF,
  SIX_DOF_NOSPHERE = POSITION_NOSPHERE | ORIENTATION_NOSPHERE
};
}  // namespace InteractionStyle

/// Type of function for constructing markers.
/// This callback sets up the marker used for an interaction.
/// @param state the current state of the robot
/// @param marker the function should fill this in with an InteractiveMarker
///          that will be used to control the interaction.
///  @returns true if the function succeeds, false if the function was not able
///          to fill in \e marker.
typedef boost::function<bool(const moveit::core::RobotState& state, visualization_msgs::InteractiveMarker& marker)>
    InteractiveMarkerConstructorFn;

/// Type of function for processing marker feedback.
/// This callback function handles feedback for an Interaction's marker.
/// Callback should update the robot state that was passed in according to
/// the new position of the marker.
///
/// @param state the current state of the robot
/// @param marker the function should fill this in with an InteractiveMarker
///          that will be used to control the interaction.
/// @returns false if the state was not successfully updated or the new state
///           is somehow invalid or erronious (e.g. in collision).  true if
///           everything worked well.
typedef boost::function<bool(moveit::core::RobotState& state,
                             const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)>
    ProcessFeedbackFn;

/// Type of function for updating marker pose for new state.
/// This callback is called when the robot state has changed.
/// Callback should calculate a new
/// pose for the marker based on the passed in robot state.
/// @param state the new state of the robot
/// @param pose the function should fill this in with the new pose of the
///              marker, given the new state of the robot.
/// @returns true if the pose was modified, false if no update is needed (i.e.
///              if the pose did not change).
typedef boost::function<bool(const moveit::core::RobotState&, geometry_msgs::Pose&)> InteractiveMarkerUpdateFn;

/// Representation of a generic interaction.
/// Displays one interactive marker.
struct GenericInteraction
{
  // Callback to construct interactive marker.
  // See comment on typedef above.
  InteractiveMarkerConstructorFn construct_marker;

  // Callback to handle interactive marker feedback messages.
  // See comment on typedef above.
  ProcessFeedbackFn process_feedback;

  // Callback to update marker pose when RobotState changes.
  // See comment on typedef above.
  InteractiveMarkerUpdateFn update_pose;

  // Suffix added to name of markers.
  // Automatically generated.
  std::string marker_name_suffix;
};

/// Representation of an interaction via an end-effector
/// Displays one marker for manipulating the EEF position.
struct EndEffectorInteraction
{
  /// The name of the group that sustains the end-effector (usually an arm)
  std::string parent_group;

  /// The name of the link in the parent group that connects to the end-effector
  std::string parent_link;

  /// The name of the group that defines the group joints
  std::string eef_group;

  /// Which degrees of freedom to enable for the end-effector
  InteractionStyle::InteractionStyle interaction;

  /// The size of the end effector group (diameter of enclosing sphere)
  double size;
};

/// Representation of a joint interaction.
/// Displays one marker for manipulating the joint.
struct JointInteraction
{
  /// The link in the robot model this joint is a parent of
  std::string connecting_link;

  /// The name of the frame that is a parent of this joint
  std::string parent_frame;

  /// The name of the joint
  std::string joint_name;

  /// The type of joint disguised as the number of DOF it has.  3=PLANAR in X/Y; 6=FLOATING
  unsigned int dof;

  /// The size of the connecting link  (diameter of enclosing sphere)
  double size;
};
}  // namespace robot_interaction
