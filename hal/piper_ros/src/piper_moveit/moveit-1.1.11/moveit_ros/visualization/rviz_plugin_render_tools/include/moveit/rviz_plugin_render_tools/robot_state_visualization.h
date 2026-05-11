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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/rviz_plugin_render_tools/octomap_render.h>
#include <rviz/robot/robot.h>

namespace moveit_rviz_plugin
{
MOVEIT_CLASS_FORWARD(RenderShapes);             // Defines RenderShapesPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(RobotStateVisualization);  // Defines RobotStateVisualizationPtr, ConstPtr, WeakPtr... etc

/** \brief Update the links of an rviz::Robot using a moveit::core::RobotState */
class RobotStateVisualization
{
public:
  RobotStateVisualization(Ogre::SceneNode* root_node, rviz::DisplayContext* context, const std::string& name,
                          rviz::Property* parent_property);

  rviz::Robot& getRobot()
  {
    return robot_;
  }

  void load(const urdf::ModelInterface& descr, bool visual = true, bool collision = true);
  void clear();

  void update(const moveit::core::RobotStateConstPtr& kinematic_state);
  void update(const moveit::core::RobotStateConstPtr& kinematic_state,
              const std_msgs::ColorRGBA& default_attached_object_color);
  void update(const moveit::core::RobotStateConstPtr& kinematic_state,
              const std_msgs::ColorRGBA& default_attached_object_color,
              const std::map<std::string, std_msgs::ColorRGBA>& color_map);
  void updateKinematicState(const moveit::core::RobotStateConstPtr& kinematic_state);
  void setDefaultAttachedObjectColor(const std_msgs::ColorRGBA& default_attached_object_color);
  /// update color of all attached object shapes
  void updateAttachedObjectColors(const std_msgs::ColorRGBA& attached_object_color);

  bool isVisible() const
  {
    return visible_;
  }

  /**
   * \brief Set the robot as a whole to be visible or not
   * @param visible Should we be visible?
   */
  void setVisible(bool visible);

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible(bool visible);

  /**
   * \brief Set whether the collision meshes/primitives of the robot should be visible
   * @param visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible(bool visible);

  void setAlpha(float alpha);

private:
  void updateHelper(const moveit::core::RobotStateConstPtr& kinematic_state,
                    const std_msgs::ColorRGBA& default_attached_object_color,
                    const std::map<std::string, std_msgs::ColorRGBA>* color_map);
  rviz::Robot robot_;
  RenderShapesPtr render_shapes_;
  std_msgs::ColorRGBA default_attached_object_color_;
  OctreeVoxelRenderMode octree_voxel_render_mode_;
  OctreeVoxelColorMode octree_voxel_color_mode_;

  bool visible_;
  bool visual_visible_;
  bool collision_visible_;
};
}  // namespace moveit_rviz_plugin
