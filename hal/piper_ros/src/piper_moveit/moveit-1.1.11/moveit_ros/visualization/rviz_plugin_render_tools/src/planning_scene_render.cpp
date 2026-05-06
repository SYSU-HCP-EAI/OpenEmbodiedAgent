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

#include <moveit/rviz_plugin_render_tools/planning_scene_render.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/render_shapes.h>
#include <rviz/display_context.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace moveit_rviz_plugin
{
PlanningSceneRender::PlanningSceneRender(Ogre::SceneNode* node, rviz::DisplayContext* context,
                                         const RobotStateVisualizationPtr& robot)
  : planning_scene_geometry_node_(node->createChildSceneNode()), context_(context), scene_robot_(robot)
{
  render_shapes_ = std::make_shared<RenderShapes>(context);
}

PlanningSceneRender::~PlanningSceneRender()
{
  context_->getSceneManager()->destroySceneNode(planning_scene_geometry_node_);
}

void PlanningSceneRender::updateRobotPosition(const planning_scene::PlanningSceneConstPtr& scene)
{
  if (scene_robot_)
  {
    auto rs = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
    rs->update();
    scene_robot_->updateKinematicState(rs);
  }
}

void PlanningSceneRender::clear()
{
  render_shapes_->clear();
}

void PlanningSceneRender::renderPlanningScene(const planning_scene::PlanningSceneConstPtr& scene,
                                              const rviz::Color& default_env_color,
                                              const rviz::Color& default_attached_color,
                                              OctreeVoxelRenderMode octree_voxel_rendering,
                                              OctreeVoxelColorMode octree_color_mode, float default_scene_alpha)
{
  if (!scene)
    return;

  clear();

  if (scene_robot_)
  {
    moveit::core::RobotState* rs = new moveit::core::RobotState(scene->getCurrentState());
    rs->update();

    std_msgs::ColorRGBA color;
    color.r = default_attached_color.r_;
    color.g = default_attached_color.g_;
    color.b = default_attached_color.b_;
    color.a = 1.0f;
    planning_scene::ObjectColorMap color_map;
    scene->getKnownObjectColors(color_map);
    scene_robot_->update(moveit::core::RobotStateConstPtr(rs), color, color_map);
  }

  const std::vector<std::string>& ids = scene->getWorld()->getObjectIds();
  for (const std::string& id : ids)
  {
    collision_detection::CollisionEnv::ObjectConstPtr object = scene->getWorld()->getObject(id);
    rviz::Color color = default_env_color;
    float alpha = default_scene_alpha;
    if (scene->hasObjectColor(id))
    {
      const std_msgs::ColorRGBA& c = scene->getObjectColor(id);
      color.r_ = c.r;
      color.g_ = c.g;
      color.b_ = c.b;
      alpha = c.a;
    }
    for (std::size_t j = 0; j < object->shapes_.size(); ++j)
    {
      render_shapes_->renderShape(planning_scene_geometry_node_, object->shapes_[j].get(),
                                  object->global_shape_poses_[j], octree_voxel_rendering, octree_color_mode, color,
                                  alpha);
    }
  }
}
}  // namespace moveit_rviz_plugin
