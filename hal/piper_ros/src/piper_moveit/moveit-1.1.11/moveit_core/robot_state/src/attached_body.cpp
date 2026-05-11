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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/robot_state/attached_body.h>
#include <geometric_shapes/check_isometry.h>
#include <geometric_shapes/shapes.h>
#include <boost/algorithm/string/predicate.hpp>

namespace moveit
{
namespace core
{
AttachedBody::AttachedBody(const LinkModel* parent, const std::string& id, const Eigen::Isometry3d& pose,
                           const std::vector<shapes::ShapeConstPtr>& shapes,
                           const EigenSTL::vector_Isometry3d& shape_poses, const std::set<std::string>& touch_links,
                           const trajectory_msgs::JointTrajectory& detach_posture,
                           const FixedTransformsMap& subframe_poses)
  : parent_link_model_(parent)
  , id_(id)
  , pose_(pose)
  , shapes_(shapes)
  , shape_poses_(shape_poses)
  , touch_links_(touch_links)
  , detach_posture_(detach_posture)
  , subframe_poses_(subframe_poses)
  , global_subframe_poses_(subframe_poses)
{
  ASSERT_ISOMETRY(pose)  // unsanitized input, could contain a non-isometry
  for (const auto& t : shape_poses_)
  {
    ASSERT_ISOMETRY(t)  // unsanitized input, could contain a non-isometry
  }
  for (const auto& t : subframe_poses_)
  {
    ASSERT_ISOMETRY(t.second)  // unsanitized input, could contain a non-isometry
  }

  // Global poses are initialized to identity to allow efficient Isometry calculations
  global_pose_.setIdentity();
  global_collision_body_transforms_.resize(shape_poses.size());
  for (Eigen::Isometry3d& global_collision_body_transform : global_collision_body_transforms_)
    global_collision_body_transform.setIdentity();

  shape_poses_in_link_frame_.clear();
  shape_poses_in_link_frame_.reserve(shape_poses_.size());
  for (const auto& shape_pose : shape_poses_)
  {
    shape_poses_in_link_frame_.push_back(pose_ * shape_pose);
  }
}

AttachedBody::~AttachedBody() = default;

void AttachedBody::setScale(double scale)
{
  for (shapes::ShapeConstPtr& shape : shapes_)
  {
    // if this shape is only owned here (and because this is a non-const function), we can safely const-cast:
    if (shape.unique())
      const_cast<shapes::Shape*>(shape.get())->scale(scale);
    else
    {
      // if the shape is owned elsewhere, we make a copy:
      shapes::Shape* copy = shape->clone();
      copy->scale(scale);
      shape.reset(copy);
    }
  }
}

void AttachedBody::computeTransform(const Eigen::Isometry3d& parent_link_global_transform)
{
  ASSERT_ISOMETRY(parent_link_global_transform)  // unsanitized input, could contain a non-isometry
  global_pose_ = parent_link_global_transform * pose_;

  // update collision body transforms
  for (std::size_t i = 0; i < global_collision_body_transforms_.size(); ++i)
    global_collision_body_transforms_[i] = global_pose_ * shape_poses_[i];  // valid isometry

  // update subframe transforms
  for (auto global = global_subframe_poses_.begin(), end = global_subframe_poses_.end(), local = subframe_poses_.begin();
       global != end; ++global, ++local)
    global->second = global_pose_ * local->second;  // valid isometry
}

void AttachedBody::setPadding(double padding)
{
  for (shapes::ShapeConstPtr& shape : shapes_)
  {
    // if this shape is only owned here (and because this is a non-const function), we can safely const-cast:
    if (shape.unique())
      const_cast<shapes::Shape*>(shape.get())->padd(padding);
    else
    {
      // if the shape is owned elsewhere, we make a copy:
      shapes::Shape* copy = shape->clone();
      copy->padd(padding);
      shape.reset(copy);
    }
  }
}

const Eigen::Isometry3d& AttachedBody::getSubframeTransform(const std::string& frame_name, bool* found) const
{
  if (boost::starts_with(frame_name, id_) && frame_name[id_.length()] == '/')
  {
    auto it = subframe_poses_.find(frame_name.substr(id_.length() + 1));
    if (it != subframe_poses_.end())
    {
      if (found)
        *found = true;
      return it->second;
    }
  }
  static const Eigen::Isometry3d IDENTITY_TRANSFORM = Eigen::Isometry3d::Identity();
  if (found)
    *found = false;
  return IDENTITY_TRANSFORM;
}

const Eigen::Isometry3d& AttachedBody::getGlobalSubframeTransform(const std::string& frame_name, bool* found) const
{
  if (boost::starts_with(frame_name, id_) && frame_name[id_.length()] == '/')
  {
    auto it = global_subframe_poses_.find(frame_name.substr(id_.length() + 1));
    if (it != global_subframe_poses_.end())
    {
      if (found)
        *found = true;
      return it->second;
    }
  }
  static const Eigen::Isometry3d IDENTITY_TRANSFORM = Eigen::Isometry3d::Identity();
  if (found)
    *found = false;
  return IDENTITY_TRANSFORM;
}

bool AttachedBody::hasSubframeTransform(const std::string& frame_name) const
{
  bool found;
  getSubframeTransform(frame_name, &found);
  return found;
}

}  // namespace core
}  // namespace moveit
