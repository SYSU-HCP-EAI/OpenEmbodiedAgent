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

/* Author: Dave Coleman */

#pragma once

#include <boost/thread/mutex.hpp>
#include <moveit/macros/class_forward.h>
#include <rviz/display.h>
#include <rviz/panel_dock_widget.h>

#ifndef Q_MOC_RUN
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/trajectory_panel.h>
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#endif

namespace rviz
{
class Robot;
class Shape;
class Property;
class IntProperty;
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
MOVEIT_CLASS_FORWARD(TrajectoryVisualization);  // Defines TrajectoryVisualizationPtr, ConstPtr, WeakPtr... etc

class TrajectoryVisualization : public QObject
{
  Q_OBJECT

public:
  /**
   * \brief Playback a trajectory from a planned path
   * \param widget - either a rviz::Display or rviz::Property
   * \param display - the rviz::Display from the parent
   * \return true on success
   */
  TrajectoryVisualization(rviz::Property* widget, rviz::Display* display);

  ~TrajectoryVisualization() override;

  virtual void update(float wall_dt, float sim_dt);
  virtual void reset();

  void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context, const ros::NodeHandle& update_nh);
  void clearRobotModel();
  void onRobotModelLoaded(const moveit::core::RobotModelConstPtr& robot_model);
  void onEnable();
  void onDisable();
  void setName(const QString& name);

  void dropTrajectory();

public Q_SLOTS:
  void interruptCurrentDisplay();
  void setDefaultAttachedObjectColor(const QColor& color);

private Q_SLOTS:

  /**
   * \brief Slot Event Functions
   */
  void changedDisplayPathVisualEnabled();
  void changedDisplayPathCollisionEnabled();
  void changedRobotPathAlpha();
  void changedLoopDisplay();
  void changedShowTrail();
  void changedTrailStepSize();
  void changedTrajectoryTopic();
  void changedStateDisplayTime();
  void changedRobotColor();
  void enabledRobotColor();
  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  /**
   * \brief ROS callback for an incoming path message
   */
  void incomingDisplayTrajectory(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
  /**
   * \brief get time to show each single robot state
   * \return Positive values indicate a fixed time per state
   *         Negative values indicate a realtime-factor
   */
  float getStateDisplayTime();
  void clearTrajectoryTrail();

  // Handles actually drawing the robot along motion plans
  RobotStateVisualizationPtr display_path_robot_;
  std_msgs::ColorRGBA default_attached_object_color_;

  // Handle colouring of robot
  void setRobotColor(rviz::Robot* robot, const QColor& color);
  void unsetRobotColor(rviz::Robot* robot);

  robot_trajectory::RobotTrajectoryPtr displaying_trajectory_message_;
  robot_trajectory::RobotTrajectoryPtr trajectory_message_to_display_;
  std::vector<RobotStateVisualizationUniquePtr> trajectory_trail_;
  ros::Subscriber trajectory_topic_sub_;
  bool animating_path_;
  bool drop_displaying_trajectory_;
  int current_state_;
  float current_state_time_;
  boost::mutex update_trajectory_message_;

  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

  // Pointers from parent display taht we save
  rviz::Display* display_;  // the parent display that this class populates
  rviz::Property* widget_;
  Ogre::SceneNode* scene_node_;
  rviz::DisplayContext* context_;
  ros::NodeHandle update_nh_;
  TrajectoryPanel* trajectory_slider_panel_;
  rviz::PanelDockWidget* trajectory_slider_dock_panel_;

  // Properties
  rviz::BoolProperty* display_path_visual_enabled_property_;
  rviz::BoolProperty* display_path_collision_enabled_property_;
  rviz::EditableEnumProperty* state_display_time_property_;
  rviz::RosTopicProperty* trajectory_topic_property_;
  rviz::FloatProperty* robot_path_alpha_property_;
  rviz::BoolProperty* loop_display_property_;
  rviz::BoolProperty* use_sim_time_property_;
  rviz::BoolProperty* trail_display_property_;
  rviz::BoolProperty* interrupt_display_property_;
  rviz::ColorProperty* robot_color_property_;
  rviz::BoolProperty* enable_robot_color_property_;
  rviz::IntProperty* trail_step_size_property_;
};

}  // namespace moveit_rviz_plugin
