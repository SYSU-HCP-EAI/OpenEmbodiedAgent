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

#include <ros/ros.h>
#include <moveit_msgs/SaveMap.h>
#include <moveit_msgs/LoadMap.h>
#include <moveit/collision_detection/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <XmlRpcException.h>

namespace occupancy_map_monitor
{
static const std::string LOGNAME = "occupancy_map_monitor";

OccupancyMapMonitor::OccupancyMapMonitor(double map_resolution)
  : map_resolution_(map_resolution), debug_info_(false), mesh_handle_count_(0), nh_("~"), active_(false)
{
  initialize();
}

OccupancyMapMonitor::OccupancyMapMonitor(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                         const std::string& map_frame, double map_resolution)
  : tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , map_resolution_(map_resolution)
  , debug_info_(false)
  , mesh_handle_count_(0)
  , nh_("~")
{
  initialize();
}

OccupancyMapMonitor::OccupancyMapMonitor(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh,
                                         const std::string& map_frame, double map_resolution)
  : tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , map_resolution_(map_resolution)
  , debug_info_(false)
  , mesh_handle_count_(0)
  , nh_(nh)
{
  initialize();
}

void OccupancyMapMonitor::initialize()
{
  /* load params from param server */
  if (map_resolution_ <= std::numeric_limits<double>::epsilon())
  {
    if (!nh_.getParam("octomap_resolution", map_resolution_))
    {
      map_resolution_ = 0.1;
      ROS_WARN_NAMED(LOGNAME, "Resolution not specified for Octomap. Assuming resolution = %g instead", map_resolution_);
    }
  }
  ROS_DEBUG_NAMED(LOGNAME, "Using resolution = %lf m for building octomap", map_resolution_);

  if (map_frame_.empty())
    if (!nh_.getParam("octomap_frame", map_frame_))
      if (tf_buffer_)
        ROS_WARN_NAMED(LOGNAME, "No target frame specified for Octomap. "
                                "No transforms will be applied to received data.");

  if (!tf_buffer_ && !map_frame_.empty())
    ROS_WARN_STREAM_NAMED(LOGNAME, "Target frame \"" << map_frame_
                                                     << "\" specified but no TF instance (buffer) specified. "
                                                        "No transforms will be applied to received data.");

  tree_ = std::make_shared<collision_detection::OccMapTree>(map_resolution_);
  tree_const_ = tree_;

  XmlRpc::XmlRpcValue sensor_list;
  if (nh_.getParam("sensors", sensor_list))
  {
    try
    {
      if (sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        for (int32_t i = 0; i < sensor_list.size(); ++i)
        {
          if (sensor_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
          {
            ROS_ERROR_NAMED(LOGNAME, "Params for octomap updater %d not a struct; ignoring.", i);
            continue;
          }

          if (!sensor_list[i].hasMember("sensor_plugin"))
          {
            ROS_ERROR_NAMED(LOGNAME, "No sensor plugin specified for octomap updater %d; ignoring.", i);
            continue;
          }

          std::string sensor_plugin = std::string(sensor_list[i]["sensor_plugin"]);
          if (sensor_plugin.empty() || sensor_plugin[0] == '~')
          {
            ROS_INFO_STREAM_NAMED(LOGNAME, "Skipping octomap updater plugin '" << sensor_plugin << "'");
            continue;
          }

          if (!updater_plugin_loader_)
          {
            try
            {
              updater_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<OccupancyMapUpdater>>(
                  "moveit_ros_perception", "occupancy_map_monitor::OccupancyMapUpdater");
            }
            catch (pluginlib::PluginlibException& ex)
            {
              ROS_FATAL_STREAM_NAMED(LOGNAME, "Exception while creating octomap updater plugin loader " << ex.what());
            }
          }

          OccupancyMapUpdaterPtr up;
          try
          {
            up = updater_plugin_loader_->createUniqueInstance(sensor_plugin);
            up->setMonitor(this);
          }
          catch (pluginlib::PluginlibException& ex)
          {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Exception while loading octomap updater '"
                                                << sensor_plugin << "': " << ex.what() << std::endl);
          }
          if (up)
          {
            /* pass the params struct directly in to the updater */
            if (!up->setParams(sensor_list[i]))
            {
              ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to configure updater of type " << up->getType());
              continue;
            }

            if (!up->initialize())
            {
              ROS_ERROR_NAMED(LOGNAME, "Unable to initialize map updater of type %s (plugin %s)", up->getType().c_str(),
                              sensor_plugin.c_str());
              continue;
            }

            addUpdater(up);
          }
        }
      else
        ROS_ERROR_NAMED(LOGNAME, "List of sensors must be an array!");
    }
    catch (XmlRpc::XmlRpcException& ex)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "XmlRpc Exception: " << ex.getMessage());
    }
  }
  else
    ROS_INFO_NAMED(LOGNAME, "No 3D sensor plugin(s) defined for octomap updates");

  /* advertise a service for loading octomaps from disk */
  save_map_srv_ = nh_.advertiseService("save_map", &OccupancyMapMonitor::saveMapCallback, this);
  load_map_srv_ = nh_.advertiseService("load_map", &OccupancyMapMonitor::loadMapCallback, this);
}

void OccupancyMapMonitor::addUpdater(const OccupancyMapUpdaterPtr& updater)
{
  if (updater)
  {
    map_updaters_.push_back(updater);
    updater->publishDebugInformation(debug_info_);
    if (map_updaters_.size() > 1)
    {
      mesh_handles_.resize(map_updaters_.size());
      // when we had one updater only, we passed the transform cache callback directly to that updater
      if (map_updaters_.size() == 2)
      {
        map_updaters_[0]->setTransformCacheCallback(
            [this](const std::string& frame, const ros::Time& stamp, ShapeTransformCache& cache) {
              return getShapeTransformCache(0, frame, stamp, cache);
            });
        map_updaters_[1]->setTransformCacheCallback(
            [this](const std::string& frame, const ros::Time& stamp, ShapeTransformCache& cache) {
              return getShapeTransformCache(1, frame, stamp, cache);
            });
      }
      else
        map_updaters_.back()->setTransformCacheCallback(
            [this, i = map_updaters_.size() - 1](const std::string& frame, const ros::Time& stamp,
                                                 ShapeTransformCache& cache) {
              return getShapeTransformCache(i, frame, stamp, cache);
            });
    }
    else
      updater->setTransformCacheCallback(transform_cache_callback_);
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "NULL updater was specified");
}

void OccupancyMapMonitor::publishDebugInformation(bool flag)
{
  debug_info_ = flag;
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->publishDebugInformation(debug_info_);
}

void OccupancyMapMonitor::setMapFrame(const std::string& frame)
{
  boost::mutex::scoped_lock _(parameters_lock_);  // we lock since an updater could specify a new frame for us
  map_frame_ = frame;
}

ShapeHandle OccupancyMapMonitor::excludeShape(const shapes::ShapeConstPtr& shape)
{
  // if we have just one updater, remove the additional level of indirection
  if (map_updaters_.size() == 1)
    return map_updaters_[0]->excludeShape(shape);

  ShapeHandle h = 0;
  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
  {
    ShapeHandle mh = map_updaters_[i]->excludeShape(shape);
    if (mh)
    {
      if (h == 0)
        h = ++mesh_handle_count_;
      mesh_handles_[i][h] = mh;
    }
  }
  return h;
}

void OccupancyMapMonitor::forgetShape(ShapeHandle handle)
{
  // if we have just one updater, remove the additional level of indirection
  if (map_updaters_.size() == 1)
  {
    map_updaters_[0]->forgetShape(handle);
    return;
  }

  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
  {
    std::map<ShapeHandle, ShapeHandle>::const_iterator it = mesh_handles_[i].find(handle);
    if (it == mesh_handles_[i].end())
      continue;
    map_updaters_[i]->forgetShape(it->second);
  }
}

void OccupancyMapMonitor::setTransformCacheCallback(const TransformCacheProvider& transform_callback)
{
  // if we have just one updater, we connect it directly to the transform provider
  if (map_updaters_.size() == 1)
    map_updaters_[0]->setTransformCacheCallback(transform_callback);
  else
    transform_cache_callback_ = transform_callback;
}

bool OccupancyMapMonitor::getShapeTransformCache(std::size_t index, const std::string& target_frame,
                                                 const ros::Time& target_time, ShapeTransformCache& cache) const
{
  if (transform_cache_callback_)
  {
    ShapeTransformCache temp_cache;
    if (transform_cache_callback_(target_frame, target_time, temp_cache))
    {
      for (std::pair<const ShapeHandle, Eigen::Isometry3d>& it : temp_cache)
      {
        std::map<ShapeHandle, ShapeHandle>::const_iterator jt = mesh_handles_[index].find(it.first);
        if (jt == mesh_handles_[index].end())
        {
          ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "Incorrect mapping of mesh handles");
          return false;
        }
        else
          cache[jt->second] = it.second;
      }
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

bool OccupancyMapMonitor::saveMapCallback(moveit_msgs::SaveMap::Request& request,
                                          moveit_msgs::SaveMap::Response& response)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Writing map to " << request.filename);
  tree_->lockRead();
  try
  {
    response.success = tree_->writeBinary(request.filename);
  }
  catch (...)
  {
    response.success = false;
  }
  tree_->unlockRead();
  return true;
}

bool OccupancyMapMonitor::loadMapCallback(moveit_msgs::LoadMap::Request& request,
                                          moveit_msgs::LoadMap::Response& response)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Reading map from " << request.filename);

  /* load the octree from disk */
  tree_->lockWrite();
  try
  {
    response.success = tree_->readBinary(request.filename);
  }
  catch (...)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to load map from file");
    response.success = false;
  }
  tree_->unlockWrite();

  if (response.success)
    tree_->triggerUpdateCallback();

  return true;
}

void OccupancyMapMonitor::startMonitor()
{
  active_ = true;
  /* initialize all of the occupancy map updaters */
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->start();
}

void OccupancyMapMonitor::stopMonitor()
{
  active_ = false;
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->stop();
}

OccupancyMapMonitor::~OccupancyMapMonitor()
{
  stopMonitor();
}
}  // namespace occupancy_map_monitor
