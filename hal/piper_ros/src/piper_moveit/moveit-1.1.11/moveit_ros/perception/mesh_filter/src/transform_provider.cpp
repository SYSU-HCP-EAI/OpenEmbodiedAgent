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

/* Author: Suat Gedikli */

#include <moveit/mesh_filter/transform_provider.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

TransformProvider::TransformProvider(double update_rate) : stop_(true), update_rate_(update_rate)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_buffer_);
  psm_->startStateMonitor();
}

TransformProvider::~TransformProvider()
{
  stop();
}

void TransformProvider::start()
{
  stop_ = false;
  thread_ = std::thread(&TransformProvider::run, this);
}

void TransformProvider::stop()
{
  stop_ = true;
  thread_.join();
}

void TransformProvider::addHandle(mesh_filter::MeshHandle handle, const std::string& name)
{
  if (!stop_)
    throw std::runtime_error("Can not add handles if TransformProvider is running");

  handle2context_[handle] = std::make_shared<TransformContext>(name);
}

void TransformProvider::setFrame(const std::string& frame)
{
  if (frame_id_ != frame)
  {
    frame_id_ = frame;
    for (auto& context_it : handle2context_)
    {
      // invalidate transformations
      context_it.second->mutex_.lock();
      context_it.second->transformation_.matrix().setZero();
      context_it.second->mutex_.unlock();
    }
  }
}

bool TransformProvider::getTransform(mesh_filter::MeshHandle handle, Eigen::Isometry3d& transform) const
{
  auto context_it = handle2context_.find(handle);

  if (context_it == handle2context_.end())
  {
    ROS_ERROR("Unable to find mesh with handle %d", handle);
    return false;
  }
  context_it->second->mutex_.lock();
  transform = context_it->second->transformation_;
  context_it->second->mutex_.unlock();
  return !(transform.matrix().isZero(0));
}

void TransformProvider::run()
{
  if (handle2context_.empty())
    throw std::runtime_error("TransformProvider is listening to empty list of frames!");

  while (ros::ok() && !stop_)
  {
    updateTransforms();
    update_rate_.sleep();
  }
}

void TransformProvider::setUpdateRate(double update_rate)
{
  update_rate_ = ros::Rate(update_rate);
}

void TransformProvider::updateTransforms()
{
  // Don't bother if frame_id_ is empty (true initially)
  if (frame_id_.empty())
  {
    ROS_DEBUG_THROTTLE(2., "Not updating transforms because frame_id_ is empty.");
    return;
  }

  static tf2::Stamped<Eigen::Isometry3d> input_transform, output_transform;
  static moveit::core::RobotStatePtr robot_state;
  robot_state = psm_->getStateMonitor()->getCurrentState();
  try
  {
    geometry_msgs::TransformStamped common_tf =
        tf_buffer_->lookupTransform(frame_id_, psm_->getPlanningScene()->getPlanningFrame(), ros::Time(0.0));
    input_transform.stamp_ = common_tf.header.stamp;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("TF Problem: %s", ex.what());
    return;
  }
  input_transform.frame_id_ = psm_->getPlanningScene()->getPlanningFrame();

  for (auto& context_it : handle2context_)
  {
    try
    {
      // TODO: check logic here - which global collision body's transform should be used?
      input_transform.setData(robot_state->getGlobalLinkTransform(context_it.second->frame_id_));
      tf_buffer_->transform(input_transform, output_transform, frame_id_);
    }
    catch (const tf2::TransformException& ex)
    {
      handle2context_[context_it.first]->mutex_.lock();
      handle2context_[context_it.first]->transformation_.matrix().setZero();
      handle2context_[context_it.first]->mutex_.unlock();
      continue;
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Caught %s while updating transforms", ex.what());
    }
    handle2context_[context_it.first]->mutex_.lock();
    handle2context_[context_it.first]->transformation_ = static_cast<Eigen::Isometry3d>(output_transform);
    handle2context_[context_it.first]->mutex_.unlock();
  }
}
