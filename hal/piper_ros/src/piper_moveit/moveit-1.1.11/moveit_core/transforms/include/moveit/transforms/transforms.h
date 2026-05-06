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

/* Author: Ioan Sucan */

#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <boost/noncopyable.hpp>
#include <moveit/macros/class_forward.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(Transforms);  // Defines TransformsPtr, ConstPtr, WeakPtr... etc

/// @brief Map frame names to the transformation matrix that can transform objects from the frame name to the planning
/// frame
using FixedTransformsMap = std::map<std::string, Eigen::Isometry3d, std::less<std::string>,
                                    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > >;

/** @brief Provides an implementation of a snapshot of a transform tree that can be easily queried for
    transforming different quantities. Transforms are maintained as a list of transforms to a particular frame.
    All stored transforms are considered fixed. */
class Transforms : private boost::noncopyable
{
public:
  /**
   * @brief Construct a transform list
   */
  Transforms(const std::string& target_frame);

  /**
   * @brief Destructor
   */
  virtual ~Transforms();

  /** \brief Check if two frames end up being the same once the missing / are added as prefix (if they are missing) */
  static bool sameFrame(const std::string& frame1, const std::string& frame2);

  /**
   * @brief Get the planning frame corresponding to this set of transforms
   * @return The planning frame
   */
  const std::string& getTargetFrame() const;

  /**
   * \name Setting and retrieving transforms maintained in this class
   */
  /**@{*/

  /**
   * @brief Return all the transforms
   * @return A map from string names of frames to corresponding Eigen::Isometry3d (w.r.t the planning frame). The
   * transforms are guaranteed to be valid isometries.
   */
  const FixedTransformsMap& getAllTransforms() const;

  /**
   * @brief Get a vector of all the transforms as ROS messages
   * @param transforms The output transforms. They are guaranteed to be valid isometries.
   */
  void copyTransforms(std::vector<geometry_msgs::TransformStamped>& transforms) const;

  /**
   * @brief Set a transform in the transform tree (adding it if necessary)
   * @param t The input transform (w.r.t the target frame)
   * @param from_frame The frame for which the input transform is specified
   */
  void setTransform(const Eigen::Isometry3d& t, const std::string& from_frame);

  /**
   * @brief Set a transform in the transform tree (adding it if necessary)
   * @param transform The input transform (the frame_id must match the target frame)
   */
  void setTransform(const geometry_msgs::TransformStamped& transform);

  /**
   * @brief Set a transform in the transform tree (adding it if necessary)
   * @param transform The input transforms (the frame_id must match the target frame)
   */
  void setTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms);

  /**
   * @brief Set all the transforms: a map from string names of frames to corresponding Eigen::Isometry3d (w.r.t the
   * planning frame)
   */
  void setAllTransforms(const FixedTransformsMap& transforms);

  /**@}*/

  /**
   * \name Applying transforms
   */
  /**@{*/

  /**
   * @brief Transform a vector in from_frame to the target_frame
   *        Note: assumes that v_in and v_out are "free" vectors, not points
   * @param from_frame The frame from which the transform is computed
   * @param v_in The input vector (in from_frame)
   * @param v_out The resultant (transformed) vector
   */
  void transformVector3(const std::string& from_frame, const Eigen::Vector3d& v_in, Eigen::Vector3d& v_out) const
  {
    // getTransform() returns a valid isometry by contract
    v_out = getTransform(from_frame).linear() * v_in;
  }

  /**
   * @brief Transform a quaternion in from_frame to the target_frame
   * @param from_frame The frame in which the input quaternion is specified
   * @param v_in The input quaternion (in from_frame). Make sure the quaternion is normalized.
   * @param v_out The resultant (transformed) quaternion. It is guaranteed to be a valid and normalized quaternion.
   */
  void transformQuaternion(const std::string& from_frame, const Eigen::Quaterniond& q_in,
                           Eigen::Quaterniond& q_out) const
  {
    // getTransform() returns a valid isometry by contract
    q_out = getTransform(from_frame).linear() * q_in;
  }

  /**
   * @brief Transform a rotation matrix in from_frame to the target_frame
   * @param from_frame The frame in which the input rotation matrix is specified
   * @param m_in The input rotation matrix (in from_frame). Make sure the matrix is a valid rotation matrix.
   * @param m_out The resultant (transformed) rotation matrix. It is guaranteed to be a valid rotation matrix.
   */
  void transformRotationMatrix(const std::string& from_frame, const Eigen::Matrix3d& m_in, Eigen::Matrix3d& m_out) const
  {
    // getTransform() returns a valid isometry by contract
    m_out = getTransform(from_frame).linear() * m_in;
  }

  /**
   * @brief Transform a pose in from_frame to the target_frame
   * @param from_frame The frame in which the input pose is specified
   * @param t_in The input pose (in from_frame). Make sure the pose is a valid isometry.
   * @param t_out The resultant (transformed) pose. It is guaranteed to be a valid isometry.
   */
  void transformPose(const std::string& from_frame, const Eigen::Isometry3d& t_in, Eigen::Isometry3d& t_out) const
  {
    // getTransform() returns a valid isometry by contract
    t_out = getTransform(from_frame) * t_in;
  }
  /**@}*/

  /**
   * @brief Check whether data can be transformed from a particular frame
   */
  virtual bool canTransform(const std::string& from_frame) const;

  /**
   * @brief Check whether a frame stays constant as the state of the robot model changes.
   * This is true for any transform mainatined by this object.
   */
  virtual bool isFixedFrame(const std::string& frame) const;

  /**
   * @brief Get transform for from_frame (w.r.t target frame)
   * @param from_frame The string id of the frame for which the transform is being computed
   * @return The required transform. It is guaranteed to be a valid isometry.
   */
  virtual const Eigen::Isometry3d& getTransform(const std::string& from_frame) const;

protected:
  std::string target_frame_;
  FixedTransformsMap transforms_map_;
};
}  // namespace core
}  // namespace moveit
