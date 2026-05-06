/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <vector>
#include <string>
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/PointStamped.h>

/// Namespace for the base class of a MoveIt sensor manager
namespace moveit_sensor_manager
{
/** \brief Define the frame of reference and the frustum of a sensor (usually this is a visual sensor) */
struct SensorInfo
{
  SensorInfo() : min_dist(0.), max_dist(0.0), x_angle(0.0), y_angle(0.0)
  {
  }

  /// The name of the frame in which the sensor observation axis is Z and starts at (0,0,0)
  std::string origin_frame;

  /* Define the frustum (or approximation of the frustum) */

  /// The minumum distance along the Z axis at which observations start
  double min_dist;

  /// The maximum distance along the Z axis at which observations can be
  double max_dist;

  /// The range of observations (in radians) on the X axis
  double x_angle;

  /// The range of observations (in radians) on the Y axis
  double y_angle;
};

MOVEIT_CLASS_FORWARD(MoveItSensorManager);  // Defines MoveItSensorManagerPtr, ConstPtr, WeakPtr... etc

class MoveItSensorManager
{
public:
  MoveItSensorManager()
  {
  }

  virtual ~MoveItSensorManager()
  {
  }

  /** \brief Get the list of known sensors */
  virtual void getSensorsList(std::vector<std::string>& names) const = 0;

  /** \brief Get the sensor information for a particular sensor */
  virtual SensorInfo getSensorInfo(const std::string& name) const = 0;

  /** \brief Check if any sensors are known to this manager */
  virtual bool hasSensors() const = 0;

  /// Point sensor \e name towards a particular point in space (\e target). This may require executing a trajectory, but
  /// it may or may not execute that trajectory.
  /// If it does not, it returns it as part of \e sensor_trajectory. This is the recommended behaviour, since the caller
  /// of this function can perform checks on the safety of the trajectory.
  /// The function returns true on success (either completing execution succesfully or computing a trajecotory
  /// successufully)
  virtual bool pointSensorTo(const std::string& name, const geometry_msgs::PointStamped& target,
                             moveit_msgs::RobotTrajectory& sensor_trajectory) = 0;
};
}  // namespace moveit_sensor_manager
