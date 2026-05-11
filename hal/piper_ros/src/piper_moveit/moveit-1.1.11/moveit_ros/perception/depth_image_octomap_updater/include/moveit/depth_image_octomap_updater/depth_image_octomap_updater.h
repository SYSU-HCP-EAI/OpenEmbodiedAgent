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

/* Author: Ioan Sucan */

#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/mesh_filter/mesh_filter.h>
#include <moveit/mesh_filter/stereo_camera_model.h>
#include <moveit/lazy_free_space_updater/lazy_free_space_updater.h>
#include <image_transport/image_transport.h>
#include <memory>

namespace occupancy_map_monitor
{
class DepthImageOctomapUpdater : public OccupancyMapUpdater
{
public:
  DepthImageOctomapUpdater();
  ~DepthImageOctomapUpdater() override;

  bool setParams(XmlRpc::XmlRpcValue& params) override;
  bool initialize() override;
  void start() override;
  void stop() override;
  ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;
  void forgetShape(ShapeHandle handle) override;

private:
  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  bool getShapeTransform(mesh_filter::MeshHandle h, Eigen::Isometry3d& transform) const;
  void stopHelper();

  ros::NodeHandle nh_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  image_transport::ImageTransport input_depth_transport_;
  image_transport::ImageTransport model_depth_transport_;
  image_transport::ImageTransport filtered_depth_transport_;
  image_transport::ImageTransport filtered_label_transport_;

  image_transport::CameraSubscriber sub_depth_image_;
  image_transport::CameraPublisher pub_model_depth_image_;
  image_transport::CameraPublisher pub_filtered_depth_image_;
  image_transport::CameraPublisher pub_filtered_label_image_;

  ros::Time last_update_time_;

  std::string filtered_cloud_topic_;
  std::string ns_;
  std::string sensor_type_;
  std::string image_topic_;
  std::size_t queue_size_;
  double near_clipping_plane_distance_;
  double far_clipping_plane_distance_;
  double shadow_threshold_;
  double padding_scale_;
  double padding_offset_;
  double max_update_rate_;
  unsigned int skip_vertical_pixels_;
  unsigned int skip_horizontal_pixels_;

  unsigned int image_callback_count_;
  double average_callback_dt_;
  unsigned int good_tf_;
  unsigned int failed_tf_;

  std::unique_ptr<mesh_filter::MeshFilter<mesh_filter::StereoCameraModel> > mesh_filter_;
  std::unique_ptr<LazyFreeSpaceUpdater> free_space_updater_;

  std::vector<float> x_cache_, y_cache_;
  double inv_fx_, inv_fy_, K0_, K2_, K4_, K5_;
  std::vector<unsigned int> filtered_labels_;
  ros::WallTime last_depth_callback_start_;
};
}  // namespace occupancy_map_monitor
