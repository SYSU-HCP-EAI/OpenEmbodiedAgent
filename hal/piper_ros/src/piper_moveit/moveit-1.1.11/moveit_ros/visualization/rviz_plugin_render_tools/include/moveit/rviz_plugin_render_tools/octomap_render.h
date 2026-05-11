/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Julius Kammerl */

#pragma once

#include <memory>
#include <vector>
#include <rviz/ogre_helpers/point_cloud.h>
#include <moveit/rviz_plugin_render_tools/octomap_render.h>
#include <OGRE/OgrePrerequisites.h>

namespace octomap
{
class OcTree;
}

namespace moveit_rviz_plugin
{
enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

class OcTreeRender
{
public:
  OcTreeRender(const std::shared_ptr<const octomap::OcTree>& octree, OctreeVoxelRenderMode octree_voxel_rendering,
               OctreeVoxelColorMode octree_color_mode, std::size_t max_octree_depth, Ogre::SceneNode* parent_node);
  virtual ~OcTreeRender();

  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);

private:
  void setColor(double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point* point);
  void setProbColor(double prob, rviz::PointCloud::Point* point);

  void octreeDecoding(const std::shared_ptr<const octomap::OcTree>& octree,
                      OctreeVoxelRenderMode octree_voxel_rendering, OctreeVoxelColorMode octree_color_mode);

  // Ogre-rviz point clouds
  std::vector<rviz::PointCloud*> cloud_;
  std::shared_ptr<const octomap::OcTree> octree_;

  Ogre::SceneNode* scene_node_;

  double colorFactor_;
  std::size_t octree_depth_;
};
}  // namespace moveit_rviz_plugin
