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

#include <string>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/mesh_filter/mesh_filter_base.h>
#include <map>

namespace tf2_ros
{
class TransformListener;
class Buffer;
}  // namespace tf2_ros

/**
 * \brief Class that caches and updates transformations for given frames.
 * \author Suat Gedikli (gedikli@willowgarage.com)
 */
class TransformProvider
{
public:
  /**
   * \brief Constructor
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] update_rate update rate in Hz
   */
  TransformProvider(double update_rate = 30.);

  /** \brief Destructor */
  ~TransformProvider();

  /**
   * \brief returns the current transformation of a mesh given by its handle
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] handle handle of the mesh
   * \param[out] transform pose of the mesh in camera coordinate system
   * \return true if transform available, false otherwise
   */
  bool getTransform(mesh_filter::MeshHandle handle, Eigen::Isometry3d& transform) const;

  /**
   * \brief registers a mesh with its handle
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] handle handle of the mesh
   * \param[in] name frame_id_ of the mesh
   */
  void addHandle(mesh_filter::MeshHandle handle, const std::string& name);

  /**
   * \brief sets the camera frame id. The returned transformations are in respect to this coordinate frame
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param frame frame id of parent/camera coordinate system.
   */
  void setFrame(const std::string& frame);

  /**
   * \brief starts the updating process. Done in a seperate thread
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void start();

  /**
   * \brief stops the update process/thread.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void stop();

  /**
   * \brief sets the update rate in Hz. This should be slow enough to reduce the system load but fast enough to get
   * up-to-date transformations. For PSDK compatible devices this value should be around 30 Hz.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] update_rate update rate in Hz
   */
  void setUpdateRate(double update_rate);

private:
  /**
   * \brief this method is called periodically by the dedicated thread and updates all the transformations of the
   * registered frames.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void updateTransforms();

  MOVEIT_CLASS_FORWARD(TransformContext);  // Defines TransformContextPtr, ConstPtr, WeakPtr... etc

  /**
   * \brief Context Object for registered frames
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  class TransformContext
  {
  public:
    TransformContext(const std::string& name) : frame_id_(name)
    {
      transformation_.matrix().setZero();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string frame_id_;
    Eigen::Isometry3d transformation_;
    std::mutex mutex_;
  };

  /**
   * \brief The entry point of the dedicated thread that updates the transformations periodically.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void run();

  /** \brief mapping between the mesh handle and its context*/
  std::map<mesh_filter::MeshHandle, TransformContextPtr> handle2context_;

  /** \brief TransformListener used to listen and update transformations*/
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /** \brief SceneMonitor used to get current states*/
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  /** \brief the camera frame id*/
  std::string frame_id_;

  /** \brief thread object*/
  std::thread thread_;

  /** \flag to leave the update loop*/
  bool stop_;

  /** \brief update rate in Hz*/
  ros::Rate update_rate_;
};
