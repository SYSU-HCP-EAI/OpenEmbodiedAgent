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

#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection/occupancy_map.h>
#include <geometric_shapes/shapes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace occupancy_map_monitor
{
using ShapeHandle = unsigned int;
using ShapeTransformCache = std::map<ShapeHandle, Eigen::Isometry3d, std::less<ShapeHandle>,
                                     Eigen::aligned_allocator<std::pair<const ShapeHandle, Eigen::Isometry3d> > >;
using TransformCacheProvider = boost::function<bool(const std::string&, const ros::Time&, ShapeTransformCache&)>;

class OccupancyMapMonitor;

MOVEIT_CLASS_FORWARD(OccupancyMapUpdater);  // Defines OccupancyMapUpdaterPtr, ConstPtr, WeakPtr... etc

/** \brief Base class for classes which update the occupancy map.
 */
class OccupancyMapUpdater
{
public:
  OccupancyMapUpdater(const std::string& type);
  virtual ~OccupancyMapUpdater();

  /** \brief This is the first function to be called after construction */
  void setMonitor(OccupancyMapMonitor* monitor);

  /** @brief Set updater params using struct that comes from parsing a yaml string. This must be called after
   * setMonitor() */
  virtual bool setParams(XmlRpc::XmlRpcValue& params) = 0;

  /** @brief Do any necessary setup (subscribe to ros topics, etc.). This call assumes setMonitor() and setParams() have
   * been previously called. */
  virtual bool initialize() = 0;

  virtual void start() = 0;

  virtual void stop() = 0;

  virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) = 0;

  virtual void forgetShape(ShapeHandle handle) = 0;

  const std::string& getType() const
  {
    return type_;
  }

  void setTransformCacheCallback(const TransformCacheProvider& transform_callback)
  {
    transform_provider_callback_ = transform_callback;
  }

  void publishDebugInformation(bool flag)
  {
    debug_info_ = flag;
  }

protected:
  OccupancyMapMonitor* monitor_;
  std::string type_;
  collision_detection::OccMapTreePtr tree_;
  TransformCacheProvider transform_provider_callback_;
  ShapeTransformCache transform_cache_;
  bool debug_info_;

  bool updateTransformCache(const std::string& target_frame, const ros::Time& target_time);

  static void readXmlParam(XmlRpc::XmlRpcValue& params, const std::string& param_name, double* value);
  static void readXmlParam(XmlRpc::XmlRpcValue& params, const std::string& param_name, unsigned int* value);
};
}  // namespace occupancy_map_monitor
