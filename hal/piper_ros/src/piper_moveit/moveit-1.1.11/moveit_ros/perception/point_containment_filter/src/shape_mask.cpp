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

#include <moveit/point_containment_filter/shape_mask.h>
#include <geometric_shapes/body_operations.h>
#include <ros/console.h>
#include <sensor_msgs/point_cloud2_iterator.h>

static const std::string LOGNAME = "shape_mask";

point_containment_filter::ShapeMask::ShapeMask(const TransformCallback& transform_callback)
  : transform_callback_(transform_callback), next_handle_(1), min_handle_(1)
{
}

point_containment_filter::ShapeMask::~ShapeMask()
{
  freeMemory();
}

void point_containment_filter::ShapeMask::freeMemory()
{
  for (const SeeShape& body : bodies_)
    delete body.body;
  bodies_.clear();
}

void point_containment_filter::ShapeMask::setTransformCallback(const TransformCallback& transform_callback)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  transform_callback_ = transform_callback;
}

point_containment_filter::ShapeHandle point_containment_filter::ShapeMask::addShape(const shapes::ShapeConstPtr& shape,
                                                                                    double scale, double padding)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  SeeShape ss;
  ss.body = bodies::createEmptyBodyFromShapeType(shape->type);
  if (ss.body)
  {
    ss.body->setDimensionsDirty(shape.get());
    ss.body->setScaleDirty(scale);
    ss.body->setPaddingDirty(padding);
    ss.body->updateInternalData();
    ss.volume = ss.body->computeVolume();
    ss.handle = next_handle_;
    std::pair<std::set<SeeShape, SortBodies>::iterator, bool> insert_op = bodies_.insert(ss);
    if (!insert_op.second)
      ROS_ERROR_NAMED(LOGNAME, "Internal error in management of bodies in ShapeMask. This is a serious error.");
    used_handles_[next_handle_] = insert_op.first;
  }
  else
    return 0;

  ShapeHandle ret = next_handle_;
  const std::size_t sz = min_handle_ + bodies_.size() + 1;
  for (std::size_t i = min_handle_; i < sz; ++i)
    if (used_handles_.find(i) == used_handles_.end())
    {
      next_handle_ = i;
      break;
    }
  min_handle_ = next_handle_;

  return ret;
}

void point_containment_filter::ShapeMask::removeShape(ShapeHandle handle)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  std::map<ShapeHandle, std::set<SeeShape, SortBodies>::iterator>::iterator it = used_handles_.find(handle);
  if (it != used_handles_.end())
  {
    delete it->second->body;
    bodies_.erase(it->second);
    used_handles_.erase(it);
    min_handle_ = handle;
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "Unable to remove shape handle %u", handle);
}

void point_containment_filter::ShapeMask::maskContainment(const sensor_msgs::PointCloud2& data_in,
                                                          const Eigen::Vector3d& /*sensor_origin*/,
                                                          const double min_sensor_dist, const double max_sensor_dist,
                                                          std::vector<int>& mask)
{
  boost::mutex::scoped_lock _(shapes_lock_);
  const unsigned int np = data_in.data.size() / data_in.point_step;
  mask.resize(np);

  if (bodies_.empty())
    std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
  else
  {
    Eigen::Isometry3d tmp;
    bspheres_.resize(bodies_.size());
    std::size_t j = 0;
    for (std::set<SeeShape>::const_iterator it = bodies_.begin(); it != bodies_.end(); ++it)
    {
      if (!transform_callback_(it->handle, tmp))
      {
        if (!it->body)
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Missing transform for shape with handle " << it->handle << " without a body");
        else
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Missing transform for shape " << it->body->getType() << " with handle "
                                                                         << it->handle);
      }
      else
      {
        it->body->setPose(tmp);
        it->body->computeBoundingSphere(bspheres_[j++]);
      }
    }

    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    const double radius_squared = bound.radius * bound.radius;

    // we now decide which points we keep
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(data_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(data_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(data_in, "z");

    // Cloud iterators are not incremented in the for loop, because of the pragma
    // Comment out below parallelization as it can result in very high CPU consumption
    //#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)np; ++i)
    {
      Eigen::Vector3d pt = Eigen::Vector3d(*(iter_x + i), *(iter_y + i), *(iter_z + i));
      double d = pt.norm();
      int out = OUTSIDE;
      if (d < min_sensor_dist || d > max_sensor_dist)
        out = CLIP;
      else if ((bound.center - pt).squaredNorm() < radius_squared)
        for (std::set<SeeShape>::const_iterator it = bodies_.begin(); it != bodies_.end() && out == OUTSIDE; ++it)
          if (it->body->containsPoint(pt))
            out = INSIDE;
      mask[i] = out;
    }
  }
}

int point_containment_filter::ShapeMask::getMaskContainment(const Eigen::Vector3d& pt) const
{
  boost::mutex::scoped_lock _(shapes_lock_);

  int out = OUTSIDE;
  for (std::set<SeeShape>::const_iterator it = bodies_.begin(); it != bodies_.end() && out == OUTSIDE; ++it)
    if (it->body->containsPoint(pt))
      out = INSIDE;
  return out;
}

int point_containment_filter::ShapeMask::getMaskContainment(double x, double y, double z) const
{
  return getMaskContainment(Eigen::Vector3d(x, y, z));
}
