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

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>

namespace occupancy_map_monitor
{
static const std::string LOGNAME = "occupancy_map_monitor";

OccupancyMapUpdater::OccupancyMapUpdater(const std::string& type) : type_(type)
{
}

OccupancyMapUpdater::~OccupancyMapUpdater() = default;

void OccupancyMapUpdater::setMonitor(OccupancyMapMonitor* monitor)
{
  monitor_ = monitor;
  tree_ = monitor->getOcTreePtr();
}

void OccupancyMapUpdater::readXmlParam(XmlRpc::XmlRpcValue& params, const std::string& param_name, double* value)
{
  if (params.hasMember(param_name))
  {
    if (params[param_name].getType() == XmlRpc::XmlRpcValue::TypeInt)
      *value = (int)params[param_name];
    else
      *value = (double)params[param_name];
  }
}

void OccupancyMapUpdater::readXmlParam(XmlRpc::XmlRpcValue& params, const std::string& param_name, unsigned int* value)
{
  if (params.hasMember(param_name))
    *value = (int)params[param_name];
}

bool OccupancyMapUpdater::updateTransformCache(const std::string& target_frame, const ros::Time& target_time)
{
  transform_cache_.clear();
  if (transform_provider_callback_)
  {
    bool success = transform_provider_callback_(target_frame, target_time, transform_cache_);
    if (!success)
      ROS_ERROR_THROTTLE_NAMED(
          1, LOGNAME,
          "Transform cache was not updated. Self-filtering may fail. If transforms were not available yet, consider "
          "setting robot_description_planning/shape_transform_cache_lookup_wait_time to wait longer for transforms");
    return success;
  }
  else
  {
    ROS_WARN_THROTTLE_NAMED(1, LOGNAME, "No callback provided for updating the transform cache for octomap updaters");
    return false;
  }
}
}  // namespace occupancy_map_monitor
