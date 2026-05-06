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

/* Author: Ioan Sucan, Dave Coleman, Adam Leeper, Sachin Chitta */

#pragma once

#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit/rviz_plugin_render_tools/trajectory_visualization.h>

#ifndef Q_MOC_RUN
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/dynamics_solver/dynamics_solver.h>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <moveit_msgs/DisplayTrajectory.h>
#endif

#include <memory>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Robot;
class Shape;
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
class ColorProperty;
class MovableText;
}  // namespace rviz

namespace moveit_rviz_plugin
{
class MotionPlanningDisplay : public PlanningSceneDisplay
{
  Q_OBJECT

public:
  MotionPlanningDisplay();

  ~MotionPlanningDisplay() override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  void setName(const QString& name) override;

  moveit::core::RobotStateConstPtr getQueryStartState() const
  {
    return query_start_state_->getState();
  }

  moveit::core::RobotStateConstPtr getQueryGoalState() const
  {
    return query_goal_state_->getState();
  }

  const moveit::core::RobotState& getPreviousState() const
  {
    return *previous_state_;
  }

  const robot_interaction::RobotInteractionPtr& getRobotInteraction() const
  {
    return robot_interaction_;
  }

  const robot_interaction::InteractionHandlerPtr& getQueryStartStateHandler() const
  {
    return query_start_state_;
  }

  const robot_interaction::InteractionHandlerPtr& getQueryGoalStateHandler() const
  {
    return query_goal_state_;
  }

  void dropVisualizedTrajectory()
  {
    trajectory_visual_->dropTrajectory();
  }

  void setQueryStartState(const moveit::core::RobotState& start);
  void setQueryGoalState(const moveit::core::RobotState& goal);

  void updateQueryStates(const moveit::core::RobotState& current_state);
  void updateQueryStartState();
  void updateQueryGoalState();
  void rememberPreviousStartState();

  void useApproximateIK(bool flag);

  // Pick Place
  void clearPlaceLocationsDisplay();
  void visualizePlaceLocations(const std::vector<geometry_msgs::PoseStamped>& place_poses);
  std::vector<std::shared_ptr<rviz::Shape> > place_locations_display_;

  std::string getCurrentPlanningGroup() const;

  void changePlanningGroup(const std::string& group);

  void addStatusText(const std::string& text);
  void addStatusText(const std::vector<std::string>& text);
  void setStatusTextColor(const QColor& color);
  void resetStatusTextColor();

  void toggleSelectPlanningGroupSubscription(bool enable);

Q_SIGNALS:
  // signals issued when start/goal states of a query changed
  void queryStartStateChanged();
  void queryGoalStateChanged();

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  void changedQueryStartState();
  void changedQueryGoalState();
  void changedQueryMarkerScale();
  void changedQueryStartColor();
  void changedQueryGoalColor();
  void changedQueryStartAlpha();
  void changedQueryGoalAlpha();
  void changedQueryCollidingLinkColor();
  void changedQueryJointViolationColor();
  void changedAttachedBodyColor() override;
  void changedPlanningGroup();
  void changedShowWeightLimit();
  void changedShowManipulabilityIndex();
  void changedShowManipulability();
  void changedShowJointTorques();
  void changedMetricsSetPayload();
  void changedMetricsTextHeight();
  void changedWorkspace();
  void resetInteractiveMarkers();
  void motionPanelVisibilityChange(bool enable);

protected:
  enum LinkDisplayStatus
  {
    COLLISION_LINK,
    OUTSIDE_BOUNDS_LINK
  };

  void clearRobotModel() override;
  void onRobotModelLoaded() override;
  void onNewPlanningSceneState() override;
  void onSceneMonitorReceivedUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type) override;
  void updateInternal(float wall_dt, float ros_dt) override;

  void renderWorkspaceBox();
  void updateLinkColors();

  void displayTable(const std::map<std::string, double>& values, const Ogre::ColourValue& color,
                    const Ogre::Vector3& pos, const Ogre::Quaternion& orient);
  void displayMetrics(bool start);

  void executeMainLoopJobs();
  void publishInteractiveMarkers(bool pose_update);

  void recomputeQueryStartStateMetrics();
  void recomputeQueryGoalStateMetrics();
  void drawQueryStartState();
  void drawQueryGoalState();
  void scheduleDrawQueryStartState(robot_interaction::InteractionHandler* handler, bool error_state_changed);
  void scheduleDrawQueryGoalState(robot_interaction::InteractionHandler* handler, bool error_state_changed);

  bool isIKSolutionCollisionFree(moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
                                 const double* ik_solution) const;

  void computeMetrics(bool start, const std::string& group, double payload);
  void computeMetricsInternal(std::map<std::string, double>& metrics,
                              const robot_interaction::EndEffectorInteraction& eef,
                              const moveit::core::RobotState& state, double payload);
  void updateStateExceptModified(moveit::core::RobotState& dest, const moveit::core::RobotState& src);
  void updateBackgroundJobProgressBar();
  void backgroundJobUpdate(moveit::tools::BackgroundProcessing::JobEvent event, const std::string& jobname);

  void setQueryStateHelper(bool use_start_state, const std::string& v);
  void populateMenuHandler(std::shared_ptr<interactive_markers::MenuHandler>& mh);

  void selectPlanningGroupCallback(const std_msgs::StringConstPtr& msg);

  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  RobotStateVisualizationPtr query_robot_start_;  ///< Handles drawing the robot at the start configuration
  RobotStateVisualizationPtr query_robot_goal_;   ///< Handles drawing the robot at the goal configuration

  Ogre::SceneNode* text_display_scene_node_;  ///< displays texts
  bool text_display_for_start_;               ///< indicates whether the text display is for the start state or not
  rviz::MovableText* text_to_display_;

  ros::Subscriber planning_group_sub_;
  ros::NodeHandle private_handle_, node_handle_;

  // render the workspace box
  std::unique_ptr<rviz::Shape> workspace_box_;

  // the planning frame
  MotionPlanningFrame* frame_;
  rviz::PanelDockWidget* frame_dock_;

  // robot interaction
  robot_interaction::RobotInteractionPtr robot_interaction_;
  robot_interaction::InteractionHandlerPtr query_start_state_;
  robot_interaction::InteractionHandlerPtr query_goal_state_;
  std::shared_ptr<interactive_markers::MenuHandler> menu_handler_start_;
  std::shared_ptr<interactive_markers::MenuHandler> menu_handler_goal_;
  std::map<std::string, LinkDisplayStatus> status_links_start_;
  std::map<std::string, LinkDisplayStatus> status_links_goal_;
  /// remember previous start state (updated before starting execution)
  moveit::core::RobotStatePtr previous_state_;

  /// Hold the names of the groups for which the query states have been updated (and should not be altered when new info
  /// is received from the planning scene)
  std::set<std::string> modified_groups_;

  /// The metrics are pairs of name-value for each of the active end effectors, for both start & goal states.
  /// computed_metrics_[std::make_pair(IS_START_STATE, GROUP_NAME)] = a map of key-value pairs
  std::map<std::pair<bool, std::string>, std::map<std::string, double> > computed_metrics_;
  /// Some groups use position only ik, calls to the metrics have to be modified appropriately
  std::map<std::string, bool> position_only_ik_;

  // Metric calculations
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  std::map<std::string, dynamics_solver::DynamicsSolverPtr> dynamics_solver_;
  boost::mutex update_metrics_lock_;

  // The trajectory playback component
  TrajectoryVisualizationPtr trajectory_visual_;

  // properties to show on side panel
  rviz::Property* path_category_;
  rviz::Property* plan_category_;
  rviz::Property* metrics_category_;

  rviz::EditableEnumProperty* planning_group_property_;
  rviz::BoolProperty* query_start_state_property_;
  rviz::BoolProperty* query_goal_state_property_;
  rviz::FloatProperty* query_marker_scale_property_;
  rviz::ColorProperty* query_start_color_property_;
  rviz::ColorProperty* query_goal_color_property_;
  rviz::FloatProperty* query_start_alpha_property_;
  rviz::FloatProperty* query_goal_alpha_property_;
  rviz::ColorProperty* query_colliding_link_color_property_;
  rviz::ColorProperty* query_outside_joint_limits_link_color_property_;

  rviz::BoolProperty* compute_weight_limit_property_;
  rviz::BoolProperty* show_manipulability_index_property_;
  rviz::BoolProperty* show_manipulability_property_;
  rviz::BoolProperty* show_joint_torques_property_;
  rviz::FloatProperty* metrics_set_payload_property_;
  rviz::FloatProperty* metrics_text_height_property_;
  rviz::BoolProperty* show_workspace_property_;

  rviz::Display* int_marker_display_;
};

}  // namespace moveit_rviz_plugin
