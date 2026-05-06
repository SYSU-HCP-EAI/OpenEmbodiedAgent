/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/transforms/transforms.h>
#include <geometric_shapes/check_isometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/algorithm/string/trim.hpp>
#include <ros/console.h>

namespace moveit
{
namespace core
{
Transforms::Transforms(const std::string& target_frame) : target_frame_(target_frame)
{
  boost::trim(target_frame_);
  if (target_frame_.empty())
    ROS_ERROR_NAMED("transforms", "The target frame for MoveIt Transforms cannot be empty.");
  else
  {
    transforms_map_[target_frame_] = Eigen::Isometry3d::Identity();
  }
}

bool Transforms::sameFrame(const std::string& frame1, const std::string& frame2)
{
  if (frame1.empty() || frame2.empty())
    return false;
  return frame1 == frame2;
}

Transforms::~Transforms() = default;

const std::string& Transforms::getTargetFrame() const
{
  return target_frame_;
}

const FixedTransformsMap& Transforms::getAllTransforms() const
{
  return transforms_map_;
}

void Transforms::setAllTransforms(const FixedTransformsMap& transforms)
{
  for (const auto& t : transforms)
  {
    ASSERT_ISOMETRY(t.second)  // unsanitized input, could contain a non-isometry
  }
  transforms_map_ = transforms;
}

bool Transforms::isFixedFrame(const std::string& frame) const
{
  if (frame.empty())
    return false;
  else
    return transforms_map_.find(frame) != transforms_map_.end();
}

const Eigen::Isometry3d& Transforms::getTransform(const std::string& from_frame) const
{
  if (!from_frame.empty())
  {
    FixedTransformsMap::const_iterator it = transforms_map_.find(from_frame);
    if (it != transforms_map_.end())
      return it->second;
    // If no transform found in map, return identity
  }

  ROS_ERROR_NAMED("transforms", "Unable to transform from frame '%s' to frame '%s'. Returning identity.",
                  from_frame.c_str(), target_frame_.c_str());

  // return identity
  static const Eigen::Isometry3d IDENTITY = Eigen::Isometry3d::Identity();
  return IDENTITY;
}

bool Transforms::canTransform(const std::string& from_frame) const
{
  if (from_frame.empty())
    return false;
  else
    return transforms_map_.find(from_frame) != transforms_map_.end();
}

void Transforms::setTransform(const Eigen::Isometry3d& t, const std::string& from_frame)
{
  ASSERT_ISOMETRY(t)  // unsanitized input, could contain a non-isometry
  if (from_frame.empty())
    ROS_ERROR_NAMED("transforms", "Cannot record transform with empty name");
  else
    transforms_map_[from_frame] = t;
}

void Transforms::setTransform(const geometry_msgs::TransformStamped& transform)
{
  if (sameFrame(transform.child_frame_id, target_frame_))
  {
    // convert message manually to ensure correct normalization for double (error < 1e-12)
    // tf2 only enforces float normalization (error < 1e-5)
    const auto& trans = transform.transform.translation;
    const auto& rot = transform.transform.rotation;
    Eigen::Translation3d translation(trans.x, trans.y, trans.z);
    Eigen::Quaterniond rotation(rot.w, rot.x, rot.y, rot.z);
    rotation.normalize();

    setTransform(translation * rotation, transform.header.frame_id);
  }
  else
  {
    ROS_ERROR_NAMED("transforms", "Given transform is to frame '%s', but frame '%s' was expected.",
                    transform.child_frame_id.c_str(), target_frame_.c_str());
  }
}

void Transforms::setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms)
{
  for (const geometry_msgs::TransformStamped& transform : transforms)
    setTransform(transform);
}

void Transforms::copyTransforms(std::vector<geometry_msgs::TransformStamped>& transforms) const
{
  transforms.resize(transforms_map_.size());
  std::size_t i = 0;
  for (FixedTransformsMap::const_iterator it = transforms_map_.begin(); it != transforms_map_.end(); ++it, ++i)
  {
    transforms[i] = tf2::eigenToTransform(it->second);
    transforms[i].child_frame_id = target_frame_;
    transforms[i].header.frame_id = it->first;
  }
}

}  // end of namespace core
}  // end of namespace moveit
