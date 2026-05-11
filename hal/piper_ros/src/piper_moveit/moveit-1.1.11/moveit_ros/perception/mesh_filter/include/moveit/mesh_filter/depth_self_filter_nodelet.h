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

#pragma once

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <moveit/mesh_filter/transform_provider.h>
#include <moveit/mesh_filter/mesh_filter.h>
#include <moveit/mesh_filter/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <memory>

namespace mesh_filter
{
/**
 * \brief Nodelet for filtering meshes from depth images. e.g. meshes of the robot or any attached object where
 * a transformation can be provided for.
 * \author Suat Gedikli (gedikli@willowgarage.com)
 */
class DepthSelfFiltering : public nodelet::Nodelet
{
public:
  /** \brief Nodelet init callback*/
  void onInit() override;

private:
  ~DepthSelfFiltering() override;

  /**
   * \brief adding the meshes to a given mesh filter object.
   * \param[in,out] mesh_filter mesh filter object that gets meshes from the robot description added to
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void addMeshes(mesh_filter::MeshFilter<mesh_filter::StereoCameraModel>& mesh_filter);

  /**
   * \brief main filtering routine
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void filter(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  /**
   * \brief Callback for connection/deconnection of listener
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void connectCb();

  /**
   * \brief Callback for subscribed depth images
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * @param depth_msg depth image
   * @param info_msg camera information containing parameters frame, etc.
   */
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

private:
  // member variables to handle ros messages
  std::shared_ptr<image_transport::ImageTransport> input_depth_transport_;
  std::shared_ptr<image_transport::ImageTransport> filtered_label_transport_;
  std::shared_ptr<image_transport::ImageTransport> filtered_depth_transport_;
  std::shared_ptr<image_transport::ImageTransport> model_depth_transport_;
  std::shared_ptr<image_transport::ImageTransport> model_label_transport_;
  image_transport::CameraSubscriber sub_depth_image_;
  image_transport::CameraPublisher pub_filtered_depth_image_;
  image_transport::CameraPublisher pub_filtered_label_image_;
  image_transport::CameraPublisher pub_model_depth_image_;
  image_transport::CameraPublisher pub_model_label_image_;

  /** \brief required to avoid listener registration before we are all set*/
  std::mutex connect_mutex_;
  int queue_size_;
  TransformProvider transform_provider_;

  std::shared_ptr<cv_bridge::CvImage> filtered_depth_ptr_;
  std::shared_ptr<cv_bridge::CvImage> filtered_label_ptr_;
  std::shared_ptr<cv_bridge::CvImage> model_depth_ptr_;
  std::shared_ptr<cv_bridge::CvImage> model_label_ptr_;
  /** \brief distance of near clipping plane*/
  double near_clipping_plane_distance_;

  /** \brief distance of far clipping plane*/
  double far_clipping_plane_distance_;

  /** \brief threshold that indicates a pixel to be in shadow, rather than being filtered out */
  double shadow_threshold_;

  /** \brief the coefficient for the square component of padding function in  1/m*/
  double padding_scale_;

  /** \brief the coefficient for the linear component of the padding function*/
  double padding_offset_;

  /** mesh filter object*/
  MeshFilter<StereoCameraModel>::Ptr mesh_filter_;
};

}  // namespace mesh_filter
