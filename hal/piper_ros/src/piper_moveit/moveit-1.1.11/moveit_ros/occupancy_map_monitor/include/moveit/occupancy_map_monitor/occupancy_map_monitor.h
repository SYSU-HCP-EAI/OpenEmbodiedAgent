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

/* Author: Ioan Sucan, Jon Binney */

#pragma once

#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <pluginlib/class_loader.hpp>

#include <moveit_msgs/SaveMap.h>
#include <moveit_msgs/LoadMap.h>
#include <moveit/collision_detection/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>

#include <boost/thread/mutex.hpp>

#include <memory>

namespace occupancy_map_monitor
{
class OccupancyMapMonitor
{
public:
  OccupancyMapMonitor(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& map_frame = "",
                      double map_resolution = 0.0);
  OccupancyMapMonitor(double map_resolution = 0.0);
  OccupancyMapMonitor(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh,
                      const std::string& map_frame = "", double map_resolution = 0.0);

  ~OccupancyMapMonitor();

  /** @brief start the monitor (will begin updating the octomap */
  void startMonitor();

  void stopMonitor();

  /** @brief Get a pointer to the underlying octree for this monitor. Lock the tree before reading or writing using this
   *  pointer. The value of this pointer stays the same throughout the existance of the monitor instance. */
  const collision_detection::OccMapTreePtr& getOcTreePtr()
  {
    return tree_;
  }

  /** @brief Get a const pointer to the underlying octree for this monitor. Lock the
   *  tree before reading this pointer */
  const collision_detection::OccMapTreeConstPtr& getOcTreePtr() const
  {
    return tree_const_;
  }

  const std::string& getMapFrame() const
  {
    return map_frame_;
  }

  void setMapFrame(const std::string& frame);

  double getMapResolution() const
  {
    return map_resolution_;
  }

  const std::shared_ptr<tf2_ros::Buffer>& getTFClient() const
  {
    return tf_buffer_;
  }

  void addUpdater(const OccupancyMapUpdaterPtr& updater);

  /** \brief Add this shape to the set of shapes to be filtered out from the octomap */
  ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape);

  /** \brief Forget about this shape handle and the shapes it corresponds to */
  void forgetShape(ShapeHandle handle);

  /** @brief Set the callback to trigger when updates to the maintained octomap are received */
  void setUpdateCallback(const boost::function<void()>& update_callback)
  {
    tree_->setUpdateCallback(update_callback);
  }

  void setTransformCacheCallback(const TransformCacheProvider& transform_cache_callback);

  void publishDebugInformation(bool flag);

  bool isActive() const
  {
    return active_;
  }

private:
  void initialize();

  /** @brief Save the current octree to a binary file */
  bool saveMapCallback(moveit_msgs::SaveMap::Request& request, moveit_msgs::SaveMap::Response& response);

  /** @brief Load octree from a binary file (gets rid of current octree data) */
  bool loadMapCallback(moveit_msgs::LoadMap::Request& request, moveit_msgs::LoadMap::Response& response);

  bool getShapeTransformCache(std::size_t index, const std::string& target_frame, const ros::Time& target_time,
                              ShapeTransformCache& cache) const;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string map_frame_;
  double map_resolution_;
  boost::mutex parameters_lock_;

  collision_detection::OccMapTreePtr tree_;
  collision_detection::OccMapTreeConstPtr tree_const_;

  std::unique_ptr<pluginlib::ClassLoader<OccupancyMapUpdater> > updater_plugin_loader_;
  std::vector<OccupancyMapUpdaterPtr> map_updaters_;
  std::vector<std::map<ShapeHandle, ShapeHandle> > mesh_handles_;
  TransformCacheProvider transform_cache_callback_;
  bool debug_info_;

  std::size_t mesh_handle_count_;

  ros::NodeHandle root_nh_;
  ros::NodeHandle nh_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  bool active_;
};
}  // namespace occupancy_map_monitor
