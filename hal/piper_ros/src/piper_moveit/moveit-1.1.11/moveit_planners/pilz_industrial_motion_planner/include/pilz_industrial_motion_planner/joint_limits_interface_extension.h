/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#ifndef JOINT_LIMITS_INTERFACE_EXTENSION_H
#define JOINT_LIMITS_INTERFACE_EXTENSION_H

#include "pilz_industrial_motion_planner/joint_limits_extension.h"
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace pilz_industrial_motion_planner
{
namespace joint_limits_interface
{
/**
 * @see joint_limits_inteface::getJointLimits(...)
 */
inline bool getJointLimits(const std::string& joint_name, const ros::NodeHandle& nh,
                           joint_limits_interface::JointLimits& limits)
{
  // Node handle scoped where the joint limits are
  // defined (copied from ::joint_limits_interface::getJointLimits(joint_name,
  // nh, limits)
  ros::NodeHandle limits_nh;
  try
  {
    const std::string limits_namespace = "joint_limits/" + joint_name;
    if (!nh.hasParam(limits_namespace))
    {
      ROS_DEBUG_STREAM("No joint limits specification found for joint '"
                       << joint_name << "' in the parameter server (namespace "
                       << nh.getNamespace() + "/" + limits_namespace << ").");
      return false;
    }
    limits_nh = ros::NodeHandle(nh, limits_namespace);
  }
  catch (const ros::InvalidNameException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  // Set the existing limits
  if (!::joint_limits_interface::getJointLimits(joint_name, nh, limits))
  {
    return false;  // LCOV_EXCL_LINE // The case where getJointLimits returns
                   // false is covered above.
  }

  // Deceleration limits
  bool has_deceleration_limits = false;
  if (limits_nh.getParam("has_deceleration_limits", has_deceleration_limits))
  {
    if (!has_deceleration_limits)
    {
      limits.has_deceleration_limits = false;
    }
    double max_dec;
    if (has_deceleration_limits && limits_nh.getParam("max_deceleration", max_dec))
    {
      limits.has_deceleration_limits = true;
      limits.max_deceleration = max_dec;
    }
  }

  return true;
}
}  // namespace joint_limits_interface
}  // namespace pilz_industrial_motion_planner

#endif  // JOINT_LIMITS_INTERFACE_EXTENSION_H
