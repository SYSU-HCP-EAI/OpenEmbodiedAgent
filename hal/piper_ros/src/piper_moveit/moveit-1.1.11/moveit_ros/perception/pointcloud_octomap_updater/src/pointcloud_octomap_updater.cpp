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

/* Author: Jon Binney, Ioan Sucan */

#include <cmath>
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <XmlRpcException.h>

#include <memory>

namespace occupancy_map_monitor
{
static const std::string LOGNAME = "occupancy_map_monitor";
PointCloudOctomapUpdater::PointCloudOctomapUpdater()
  : OccupancyMapUpdater("PointCloudUpdater")
  , private_nh_("~")
  , scale_(1.0)
  , padding_(0.0)
  , max_range_(std::numeric_limits<double>::infinity())
  , point_subsample_(1)
  , max_update_rate_(0)
  , point_cloud_subscriber_(nullptr)
  , point_cloud_filter_(nullptr)
{
}

PointCloudOctomapUpdater::~PointCloudOctomapUpdater()
{
  stopHelper();
}

bool PointCloudOctomapUpdater::setParams(XmlRpc::XmlRpcValue& params)
{
  try
  {
    if (!params.hasMember("point_cloud_topic"))
      return false;
    point_cloud_topic_ = static_cast<const std::string&>(params["point_cloud_topic"]);

    readXmlParam(params, "max_range", &max_range_);
    readXmlParam(params, "padding_offset", &padding_);
    readXmlParam(params, "padding_scale", &scale_);
    readXmlParam(params, "point_subsample", &point_subsample_);
    if (params.hasMember("max_update_rate"))
      readXmlParam(params, "max_update_rate", &max_update_rate_);
    if (params.hasMember("filtered_cloud_topic"))
      filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
    if (params.hasMember("ns"))
      ns_ = static_cast<const std::string&>(params["ns"]);
  }
  catch (XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "XmlRpc Exception: " << ex.getMessage());
    return false;
  }

  return true;
}

bool PointCloudOctomapUpdater::initialize()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, root_nh_);
  shape_mask_ = std::make_unique<point_containment_filter::ShapeMask>();
  shape_mask_->setTransformCallback(
      [this](ShapeHandle shape, Eigen::Isometry3d& tf) { return getShapeTransform(shape, tf); });

  std::string prefix = "";
  if (!ns_.empty())
    prefix = ns_ + "/";
  if (!filtered_cloud_topic_.empty())
    filtered_cloud_publisher_ =
        private_nh_.advertise<sensor_msgs::PointCloud2>(prefix + filtered_cloud_topic_, 10, false);
  return true;
}

void PointCloudOctomapUpdater::start()
{
  if (point_cloud_subscriber_)
    return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 5);
  if (tf_listener_ && tf_buffer_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_buffer_,
                                                                               monitor_->getMapFrame(), 5, root_nh_);
    point_cloud_filter_->registerCallback(
        [this](const sensor_msgs::PointCloud2::ConstPtr& cloud) { cloudMsgCallback(cloud); });
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(),
                   point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(
        [this](const sensor_msgs::PointCloud2::ConstPtr& cloud) { cloudMsgCallback(cloud); });
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void PointCloudOctomapUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void PointCloudOctomapUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = nullptr;
  point_cloud_subscriber_ = nullptr;
}

ShapeHandle PointCloudOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  ShapeHandle h = 0;
  if (shape_mask_)
    h = shape_mask_->addShape(shape, scale_, padding_);
  else
    ROS_ERROR_NAMED(LOGNAME, "Shape filter not yet initialized!");
  return h;
}

void PointCloudOctomapUpdater::forgetShape(ShapeHandle handle)
{
  if (shape_mask_)
    shape_mask_->removeShape(handle);
}

bool PointCloudOctomapUpdater::getShapeTransform(ShapeHandle h, Eigen::Isometry3d& transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it != transform_cache_.end())
  {
    transform = it->second;
  }
  return it != transform_cache_.end();
}

void PointCloudOctomapUpdater::updateMask(const sensor_msgs::PointCloud2& /*cloud*/,
                                          const Eigen::Vector3d& /*sensor_origin*/, std::vector<int>& /*mask*/)
{
}

void PointCloudOctomapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  ROS_DEBUG_NAMED(LOGNAME, "Received a new point cloud message");
  ros::WallTime start = ros::WallTime::now();

  if (max_update_rate_ > 0)
  {
    // ensure we are not updating the octomap representation too often
    if (ros::Time::now() - last_update_time_ <= ros::Duration(1.0 / max_update_rate_))
      return;
    last_update_time_ = ros::Time::now();
  }

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
    map_h_sensor.setIdentity();
  else
  {
    if (tf_buffer_)
    {
      try
      {
        tf2::fromMsg(tf_buffer_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id,
                                                 cloud_msg->header.stamp),
                     map_h_sensor);
      }
      catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Transform error of sensor data: " << ex.what() << "; quitting callback");
        return;
      }
    }
    else
      return;
  }

  /* compute sensor origin in map frame */
  const tf2::Vector3& sensor_origin_tf = map_h_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp))
    return;

  /* mask out points on the robot */
  shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_range_, mask_);
  updateMask(*cloud_msg, sensor_origin_eigen, mask_);

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
  std::unique_ptr<sensor_msgs::PointCloud2> filtered_cloud;

  // We only use these iterators if we are creating a filtered_cloud for
  // publishing. We cannot default construct these, so we use unique_ptr's
  // to defer construction
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_x;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_y;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_filtered_z;

  if (!filtered_cloud_topic_.empty())
  {
    filtered_cloud = std::make_unique<sensor_msgs::PointCloud2>();
    filtered_cloud->header = cloud_msg->header;
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

    // we have created a filtered_out, so we can create the iterators now
    iter_filtered_x = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*filtered_cloud, "x");
    iter_filtered_y = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*filtered_cloud, "y");
    iter_filtered_z = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*filtered_cloud, "z");
  }
  size_t filtered_cloud_size = 0;

  tree_->lockRead();

  try
  {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
     * should be occupied */
    for (unsigned int row = 0; row < cloud_msg->height; row += point_subsample_)
    {
      unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg, "x");
      // set iterator to point at start of the current row
      pt_iter += row_c;

      for (unsigned int col = 0; col < cloud_msg->width; col += point_subsample_, pt_iter += point_subsample_)
      {
        // if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;

        /* check for NaN */
        if (!std::isnan(pt_iter[0]) && !std::isnan(pt_iter[1]) && !std::isnan(pt_iter[2]))
        {
          /* occupied cell at ray endpoint if ray is shorter than max range and this point
             isn't on a part of the robot*/
          if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE)
          {
            // transform to map frame
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]);
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
          {
            tf2::Vector3 clipped_point_tf =
                map_h_sensor * (tf2::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]).normalize() * max_range_);
            clip_cells.insert(
                tree_->coordToKey(clipped_point_tf.getX(), clipped_point_tf.getY(), clipped_point_tf.getZ()));
          }
          else
          {
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(pt_iter[0], pt_iter[1], pt_iter[2]);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
            // build list of valid points if we want to publish them
            if (filtered_cloud)
            {
              **iter_filtered_x = pt_iter[0];
              **iter_filtered_y = pt_iter[1];
              **iter_filtered_z = pt_iter[2];
              ++filtered_cloud_size;
              ++*iter_filtered_x;
              ++*iter_filtered_y;
              ++*iter_filtered_z;
            }
          }
        }
      }
    }

    /* compute the free cells along each ray that ends at an occupied cell */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(occupied_cell), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    /* compute the free cells along each ray that ends at a model cell */
    for (const octomap::OcTreeKey& model_cell : model_cells)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(model_cell), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    /* compute the free cells along each ray that ends at a clipped cell */
    for (const octomap::OcTreeKey& clip_cell : clip_cells)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(clip_cell), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());
  }
  catch (...)
  {
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();

  /* cells that overlap with the model are not occupied */
  for (const octomap::OcTreeKey& model_cell : model_cells)
    occupied_cells.erase(model_cell);

  /* occupied cells are not free */
  for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
    free_cells.erase(occupied_cell);

  tree_->lockWrite();

  try
  {
    /* mark free cells only if not seen occupied in this cloud */
    for (const octomap::OcTreeKey& free_cell : free_cells)
      tree_->updateNode(free_cell, false);

    /* now mark all occupied cells */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      tree_->updateNode(occupied_cell, true);

    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (const octomap::OcTreeKey& model_cell : model_cells)
      tree_->updateNode(model_cell, lg);
  }
  catch (...)
  {
    ROS_ERROR_NAMED(LOGNAME, "Internal error while updating octree");
  }
  tree_->unlockWrite();
  ROS_DEBUG_NAMED(LOGNAME, "Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  tree_->triggerUpdateCallback();

  if (filtered_cloud)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
    pcd_modifier.resize(filtered_cloud_size);
    filtered_cloud_publisher_.publish(*filtered_cloud);
  }
}
}  // namespace occupancy_map_monitor
