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

#include <moveit/rviz_plugin_render_tools/planning_link_updater.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>

bool moveit_rviz_plugin::PlanningLinkUpdater::getLinkTransforms(const std::string& link_name,
                                                                Ogre::Vector3& visual_position,
                                                                Ogre::Quaternion& visual_orientation,
                                                                Ogre::Vector3& collision_position,
                                                                Ogre::Quaternion& collision_orientation) const
{
  const moveit::core::LinkModel* link_model = kinematic_state_->getLinkModel(link_name);

  if (!link_model)
  {
    return false;
  }

  // getGlobalLinkTransform() returns a valid isometry by contract
  const Eigen::Vector3d& robot_visual_position = kinematic_state_->getGlobalLinkTransform(link_model).translation();
  Eigen::Quaterniond robot_visual_orientation(kinematic_state_->getGlobalLinkTransform(link_model).linear());
  visual_position = Ogre::Vector3(robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z());
  visual_orientation = Ogre::Quaternion(robot_visual_orientation.w(), robot_visual_orientation.x(),
                                        robot_visual_orientation.y(), robot_visual_orientation.z());
  collision_position = visual_position;
  collision_orientation = visual_orientation;

  return true;
}
