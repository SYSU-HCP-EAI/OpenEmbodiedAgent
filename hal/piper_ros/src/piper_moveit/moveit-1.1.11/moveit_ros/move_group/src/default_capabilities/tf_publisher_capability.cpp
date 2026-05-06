/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Hamburg University
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
 *   * Neither the name of Hamburg University nor the names of its
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

/* Author: Jonas Tietz */

#include "tf_publisher_capability.h"
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/attached_body.h>

namespace move_group
{
TfPublisher::TfPublisher() : MoveGroupCapability("TfPublisher")
{
}

TfPublisher::~TfPublisher()
{
  keep_running_ = false;
  thread_.join();
}

namespace
{
void publishSubframes(tf2_ros::TransformBroadcaster& broadcaster, const moveit::core::FixedTransformsMap& subframes,
                      const std::string& parent_object, const ros::Time& stamp)
{
  geometry_msgs::TransformStamped transform;
  for (auto& subframe : subframes)
  {
    transform = tf2::eigenToTransform(subframe.second);
    transform.child_frame_id = parent_object + "/" + subframe.first;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_object;
    broadcaster.sendTransform(transform);
  }
}
}  // namespace

void TfPublisher::publishPlanningSceneFrames()
{
  tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transform;
  ros::Rate rate(rate_);

  while (keep_running_)
  {
    {
      ros::Time stamp = ros::Time::now();
      planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(context_->planning_scene_monitor_);
      collision_detection::WorldConstPtr world = locked_planning_scene->getWorld();
      std::string planning_frame = locked_planning_scene->getPlanningFrame();

      for (const auto& obj : *world)
      {
        std::string object_frame = prefix_ + obj.second->id_;
        transform = tf2::eigenToTransform(obj.second->pose_);
        transform.child_frame_id = object_frame;
        transform.header.stamp = stamp;
        transform.header.frame_id = planning_frame;
        broadcaster.sendTransform(transform);

        const moveit::core::FixedTransformsMap& subframes = obj.second->subframe_poses_;
        publishSubframes(broadcaster, subframes, object_frame, stamp);
      }

      const moveit::core::RobotState& rs = locked_planning_scene->getCurrentState();
      std::vector<const moveit::core::AttachedBody*> attached_collision_objects;
      rs.getAttachedBodies(attached_collision_objects);
      for (const moveit::core::AttachedBody* attached_body : attached_collision_objects)
      {
        std::string object_frame = prefix_ + attached_body->getName();
        transform = tf2::eigenToTransform(attached_body->getPose());
        transform.child_frame_id = object_frame;
        transform.header.stamp = stamp;
        transform.header.frame_id = attached_body->getAttachedLinkName();
        broadcaster.sendTransform(transform);

        const moveit::core::FixedTransformsMap& subframes = attached_body->getSubframes();
        publishSubframes(broadcaster, subframes, object_frame, stamp);
      }
    }

    rate.sleep();
  }
}

void TfPublisher::initialize()
{
  ros::NodeHandle nh = ros::NodeHandle("~");

  std::string prefix = nh.getNamespace() + "/";
  nh.param("planning_scene_frame_publishing_rate", rate_, 10);
  nh.param("planning_scene_tf_prefix", prefix_, prefix);
  keep_running_ = true;

  ROS_INFO("Initializing MoveGroupTfPublisher with a frame publishing rate of %d", rate_);
  thread_ = std::thread(&TfPublisher::publishPlanningSceneFrames, this);
}
}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::TfPublisher, move_group::MoveGroupCapability)
