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

#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <geometric_shapes/bodies.h>
#include <boost/function.hpp>
#include <vector>
#include <set>
#include <map>

#include <boost/thread/mutex.hpp>

namespace point_containment_filter
{
typedef unsigned int ShapeHandle;

/** \brief Computing a mask for a pointcloud that states which points are inside the robot */
class ShapeMask
{
public:
  /** \brief The possible values of a mask computed for a point */
  enum
  {
    INSIDE = 0,
    OUTSIDE = 1,
    CLIP = 2
  };

  typedef boost::function<bool(ShapeHandle, Eigen::Isometry3d&)> TransformCallback;

  /** \brief Construct the filter */
  ShapeMask(const TransformCallback& transform_callback = TransformCallback());

  /** \brief Destructor to clean up */
  virtual ~ShapeMask();

  ShapeHandle addShape(const shapes::ShapeConstPtr& shape, double scale = 1.0, double padding = 0.0);
  void removeShape(ShapeHandle handle);

  void setTransformCallback(const TransformCallback& transform_callback);

  /** \brief Compute the containment mask (INSIDE or OUTSIDE) for a given pointcloud. If a mask element is INSIDE, the
     point
      is inside the robot. The point is outside if the mask element is OUTSIDE.
  */
  void maskContainment(const sensor_msgs::PointCloud2& data_in, const Eigen::Vector3d& sensor_pos,
                       const double min_sensor_dist, const double max_sensor_dist, std::vector<int>& mask);

  /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point.
      It is assumed the point is in the frame corresponding to the TransformCallback */
  int getMaskContainment(double x, double y, double z) const;

  /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point.
      It is assumed the point is in the frame corresponding to the TransformCallback */
  int getMaskContainment(const Eigen::Vector3d& pt) const;

protected:
  struct SeeShape
  {
    SeeShape()
    {
      body = nullptr;
    }

    bodies::Body* body;
    ShapeHandle handle;
    double volume;
  };

  struct SortBodies
  {
    bool operator()(const SeeShape& b1, const SeeShape& b2) const
    {
      if (b1.volume > b2.volume)
        return true;
      if (b1.volume < b2.volume)
        return false;
      return b1.handle < b2.handle;
    }
  };

  TransformCallback transform_callback_;

  /** \brief Protects, bodies_ and bspheres_. All public methods acquire this mutex for their whole duration. */
  mutable boost::mutex shapes_lock_;
  std::set<SeeShape, SortBodies> bodies_;
  std::vector<bodies::BoundingSphere> bspheres_;

private:
  /** \brief Free memory. */
  void freeMemory();

  ShapeHandle next_handle_;
  ShapeHandle min_handle_;
  std::map<ShapeHandle, std::set<SeeShape, SortBodies>::iterator> used_handles_;
};
}  // namespace point_containment_filter
