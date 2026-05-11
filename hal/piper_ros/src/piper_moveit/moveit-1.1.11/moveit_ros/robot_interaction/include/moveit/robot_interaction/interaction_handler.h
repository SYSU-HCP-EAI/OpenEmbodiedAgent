/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Adam Leeper */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <moveit/macros/class_forward.h>
#include <moveit/robot_interaction/locked_robot_state.h>
//#include <moveit/robot_interaction/robot_interaction.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_ros/buffer.h>

namespace robot_interaction
{
MOVEIT_CLASS_FORWARD(InteractionHandler);   // Defines InteractionHandlerPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(RobotInteraction);     // Defines RobotInteractionPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(KinematicOptionsMap);  // Defines KinematicOptionsMapPtr, ConstPtr, WeakPtr... etc

struct EndEffectorInteraction;
struct JointInteraction;
struct GenericInteraction;

/// Function type for notifying client of RobotState changes.
///
/// This callback function is called by the InteractionHandler::handle*
/// functions, when changes are made to the internal robot state the handler
/// maintains.  The handler passes its own pointer as argument to the callback,
/// as well as a boolean flag that indicates whether the error state changed --
/// whether updates to the robot state performed in the
/// InteractionHandler::handle* functions have switched from failing to
/// succeeding or the other way around.
typedef boost::function<void(InteractionHandler*, bool)> InteractionHandlerCallbackFn;

/// Manage interactive markers to control a RobotState.
///
/// Each instance maintains one or more interactive markers to control
/// various joints in one group of one RobotState.
/// The group being controlled is maintained by the RobotInteraction object
/// that contains this InteractionHandler object.
/// All InteractionHandler objects in the same RobotInteraction are controlling
/// the same group.
class InteractionHandler : public LockedRobotState
{
public:
  // Use this constructor if you have an initial RobotState already.
  InteractionHandler(const RobotInteractionPtr& robot_interaction, const std::string& name,
                     const moveit::core::RobotState& initial_robot_state,
                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>());

  // Use this constructor to start with a default state.
  InteractionHandler(const RobotInteractionPtr& robot_interaction, const std::string& name,
                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>());

  // DEPRECATED.
  InteractionHandler(const std::string& name, const moveit::core::RobotState& initial_robot_state,
                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>());
  // DEPRECATED.
  InteractionHandler(const std::string& name, const moveit::core::RobotModelConstPtr& model,
                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>());

  ~InteractionHandler() override
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

  /// Get a copy of the RobotState maintained by this InteractionHandler.
  using LockedRobotState::getState;

  /// Set a new RobotState for this InteractionHandler.
  using LockedRobotState::setState;

  void setUpdateCallback(const InteractionHandlerCallbackFn& callback);
  const InteractionHandlerCallbackFn& getUpdateCallback() const;
  void setMeshesVisible(bool visible);
  bool getMeshesVisible() const;
  void setControlsVisible(bool visible);
  bool getControlsVisible() const;

  /** \brief Set the offset from EEF to its marker.
   * @param eef The target end-effector.
   * @param m The pose of the marker in the frame of the end-effector parent. */
  void setPoseOffset(const EndEffectorInteraction& eef, const geometry_msgs::Pose& m);

  /** \brief Set the offset from a joint to its marker.
   * @param j The target joint.
   * @param m The pose of the marker in the frame of the joint parent. */
  void setPoseOffset(const JointInteraction& j, const geometry_msgs::Pose& m);

  /** \brief Get the offset from EEF to its marker.
   * @param  The target end-effector.
   * @param  The pose offset (only valid if return value is true).
   * @return True if an offset was found for the given end-effector. */
  bool getPoseOffset(const EndEffectorInteraction& eef, geometry_msgs::Pose& m);

  /** \brief Get the offset from a joint to its marker.
   * @param vj The joint.
   * @param m The pose offset (only valid if return value is true).
   * @return True if an offset was found for the given joint. */
  bool getPoseOffset(const JointInteraction& vj, geometry_msgs::Pose& m);

  /** \brief Clear the interactive marker pose offset for the given
   * end-effector.
   * @param  The target end-effector. */
  void clearPoseOffset(const EndEffectorInteraction& eef);

  /** \brief Clear the interactive marker pose offset for the given joint.
   * @param  The target joint. */
  void clearPoseOffset(const JointInteraction& vj);

  /** \brief Clear the pose offset for all end-effectors and virtual joints. */
  void clearPoseOffsets();

  /** \brief Set the menu handler that defines menus and callbacks for all
   *         interactive markers drawn by this interaction handler.
   * @param  A menu handler. */
  void setMenuHandler(const std::shared_ptr<interactive_markers::MenuHandler>& mh);

  /** \brief Get the menu handler that defines menus and callbacks for all
   *         interactive markers drawn by this interaction handler.
   * @return  The menu handler. */
  const std::shared_ptr<interactive_markers::MenuHandler>& getMenuHandler();

  /** \brief Remove the menu handler for this interaction handler. */
  void clearMenuHandler();

  /** \brief Get the last interactive_marker command pose for an end-effector.
   * @param The end-effector in question.
   * @param A PoseStamped message containing the last (offset-removed) pose
   *           commanded for the end-effector.
   * @return True if a pose for that end-effector was found, false otherwise. */
  bool getLastEndEffectorMarkerPose(const EndEffectorInteraction& eef, geometry_msgs::PoseStamped& pose);

  /** \brief Get the last interactive_marker command pose for a joint.
   * @param The joint in question.
   * @param A PoseStamped message containing the last (offset-removed) pose
   *           commanded for the joint.
   * @return True if a pose for that joint was found, false otherwise. */
  bool getLastJointMarkerPose(const JointInteraction& vj, geometry_msgs::PoseStamped& pose);

  /** \brief Clear the last interactive_marker command pose for the given
   * end-effector.
   * @param  The target end-effector. */
  void clearLastEndEffectorMarkerPose(const EndEffectorInteraction& eef);

  /** \brief Clear the last interactive_marker command pose for the given joint.
   * @param  The target joint. */
  void clearLastJointMarkerPose(const JointInteraction& vj);

  /** \brief Clear the last interactive_marker command poses for all
   * end-effectors and joints. */
  void clearLastMarkerPoses();

  /** \brief Update the internal state maintained by the handler using
   * information from the received feedback message. */
  virtual void handleEndEffector(const EndEffectorInteraction& eef,
                                 const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /** \brief Update the internal state maintained by the handler using
   * information from the received feedback message. */
  virtual void handleJoint(const JointInteraction& vj,
                           const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /** \brief Update the internal state maintained by the handler using
   * information from the received feedback message. */
  virtual void handleGeneric(const GenericInteraction& g,
                             const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /** \brief Check if the marker corresponding to this end-effector leads to an
   * invalid state */
  virtual bool inError(const EndEffectorInteraction& eef) const;

  /** \brief Check if the marker corresponding to this joint leads to an
   * invalid state */
  virtual bool inError(const JointInteraction& vj) const;

  /** \brief Check if the generic marker to an invalid state */
  virtual bool inError(const GenericInteraction& g) const;

  /** \brief Clear any error settings.
   * This makes the markers appear as if the state is no longer invalid. */
  void clearError();

protected:
  bool transformFeedbackPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                             const geometry_msgs::Pose& offset, geometry_msgs::PoseStamped& tpose);

  const std::string name_;
  const std::string planning_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

private:
  typedef boost::function<void(InteractionHandler*)> StateChangeCallbackFn;

  // Update RobotState using a generic interaction feedback message.
  // YOU MUST LOCK state_lock_ BEFORE CALLING THIS.
  void updateStateGeneric(moveit::core::RobotState& state, const GenericInteraction& g,
                          const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                          StateChangeCallbackFn& callback);

  // Update RobotState for a new pose of an eef.
  // YOU MUST LOCK state_lock_ BEFORE CALLING THIS.
  void updateStateEndEffector(moveit::core::RobotState& state, const EndEffectorInteraction& eef,
                              const geometry_msgs::Pose& pose, StateChangeCallbackFn& callback);

  // Update RobotState for a new joint position.
  // YOU MUST LOCK state_lock_ BEFORE CALLING THIS.
  void updateStateJoint(moveit::core::RobotState& state, const JointInteraction& vj, const geometry_msgs::Pose& pose,
                        StateChangeCallbackFn& callback);

  // Set the error state for \e name.
  // Returns true if the error state for \e name changed.
  // YOU MUST LOCK state_lock_ BEFORE CALLING THIS.
  bool setErrorState(const std::string& name, bool new_error_state);

  /** Get the error state for \e name.
   * True if Interaction \e name is not in a valid pose. */
  bool getErrorState(const std::string& name) const;

  // Contains the (user-programmable) pose offset between the end-effector
  // parent link (or a virtual joint) and the desired control frame for the
  // interactive marker. The offset is expressed in the frame of the parent
  // link or virtual joint. For example, on a PR2 an offset of +0.20 along
  // the x-axis will move the center of the 6-DOF interactive marker from
  // the wrist to the finger tips.
  // PROTECTED BY offset_map_lock_
  std::map<std::string, geometry_msgs::Pose> offset_map_;

  // Contains the most recent poses received from interactive marker feedback,
  // with the offset removed (e.g. in theory, coinciding with the end-effector
  // parent or virtual joint). This allows a user application to query for the
  // interactive marker pose (which could be useful for robot control using
  // gradient-based methods) even when the IK solver failed to find a valid
  // robot state that satisfies the feedback pose.
  // PROTECTED BY pose_map_lock_
  std::map<std::string, geometry_msgs::PoseStamped> pose_map_;

  boost::mutex pose_map_lock_;
  boost::mutex offset_map_lock_;

  // per group options for doing kinematics.
  // PROTECTED BY state_lock_ - The POINTER is protected by state_lock_.  The
  // CONTENTS is protected internally.
  KinematicOptionsMapPtr kinematic_options_map_;

  // A set of Interactions for which the pose is invalid.
  // PROTECTED BY state_lock_
  std::set<std::string> error_state_;

  // For adding menus (and associated callbacks) to all the
  // end-effector and virtual-joint interactive markers
  //
  // PROTECTED BY state_lock_ - The POINTER is protected by state_lock_.  The
  // CONTENTS is not.
  std::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

  // Called when the RobotState maintained by the handler changes.
  // The caller may, for example, redraw the robot at the new state.
  // handler is the handler that changed.
  // error_state_changed is true if an end effector's error state may have
  // changed.
  //
  // PROTECTED BY state_lock_ - the function pointer is protected, but the call
  // is made without any lock held.
  boost::function<void(InteractionHandler* handler, bool error_state_changed)> update_callback_;

  // PROTECTED BY state_lock_
  bool display_meshes_;

  // PROTECTED BY state_lock_
  bool display_controls_;

  // remove '_' characters from name
  static std::string fixName(std::string name);
};
}  // namespace robot_interaction
