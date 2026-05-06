/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>
#endif

namespace rviz
{
class Robot;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
}  // namespace rviz

namespace moveit_rviz_plugin
{
class RobotStateVisualization;

class RobotStateDisplay : public rviz::Display
{
  Q_OBJECT

public:
  RobotStateDisplay();
  ~RobotStateDisplay() override;

  void load(const rviz::Config& config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  void setLinkColor(const std::string& link_name, const QColor& color);
  void unsetLinkColor(const std::string& link_name);

public Q_SLOTS:
  void setVisible(bool visible);

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void changedRobotDescription();
  void changedRootLinkName();
  void changedRobotSceneAlpha();
  void changedAttachedBodyColor();
  void changedRobotStateTopic();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  void loadRobotModel();

  /**
   * \brief Set the scene node's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor& color);
  void unsetLinkColor(rviz::Robot* robot, const std::string& link_name);

  void newRobotStateCallback(const moveit_msgs::DisplayRobotState::ConstPtr& state);

  void setRobotHighlights(const moveit_msgs::DisplayRobotState::_highlight_links_type& highlight_links);
  void setHighlight(const std::string& link_name, const std_msgs::ColorRGBA& color);
  void unsetHighlight(const std::string& link_name);

  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  // render the robot
  ros::NodeHandle root_nh_;
  ros::Subscriber robot_state_subscriber_;

  RobotStateVisualizationPtr robot_;
  rdf_loader::RDFLoaderPtr rdf_loader_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  std::map<std::string, std_msgs::ColorRGBA> highlights_;
  bool update_state_;

  rviz::StringProperty* robot_description_property_;
  rviz::StringProperty* root_link_name_property_;
  rviz::RosTopicProperty* robot_state_topic_property_;
  rviz::FloatProperty* robot_alpha_property_;
  rviz::ColorProperty* attached_body_color_property_;
  rviz::BoolProperty* enable_link_highlight_;
  rviz::BoolProperty* enable_visual_visible_;
  rviz::BoolProperty* enable_collision_visible_;
  rviz::BoolProperty* show_all_links_;
};

}  // namespace moveit_rviz_plugin
