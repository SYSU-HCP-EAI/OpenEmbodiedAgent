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

/* Author: Ioan Sucan, Acorn Pooley, Adam Leeper */

#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/math/constants/constants.hpp>

namespace robot_interaction
{
visualization_msgs::InteractiveMarker
makeEmptyInteractiveMarker(const std::string& name, const geometry_msgs::PoseStamped& stamped, double scale)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = stamped.header;
  int_marker.name = name;
  int_marker.scale = scale;
  int_marker.pose = stamped.pose;
  return int_marker;
}

void addTArrowMarker(visualization_msgs::InteractiveMarker& im)
{
  // create an arrow marker
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.scale.x = 0.6 * im.scale;
  m.scale.y = 0.12 * im.scale;
  m.scale.z = 0.12 * im.scale;
  m.ns = "goal_pose_arrow_marker";
  m.id = 1;
  m.action = visualization_msgs::Marker::ADD;
  m.header = im.header;
  m.pose = im.pose;
  // Arrow points along Z
  tf2::Quaternion imq, tmq;
  tf2::fromMsg(m.pose.orientation, imq);
  tmq.setRPY(0, -boost::math::constants::pi<double>() / 2.0, 0);
  imq = imq * tmq;
  m.pose.orientation = tf2::toMsg(imq);
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;

  visualization_msgs::Marker mc;
  mc.type = visualization_msgs::Marker::CYLINDER;
  mc.scale.x = 0.05 * im.scale;
  mc.scale.y = 0.05 * im.scale;
  mc.scale.z = 0.15 * im.scale;
  mc.ns = "goal_pose_arrow_marker";
  mc.id = 2;
  mc.action = visualization_msgs::Marker::ADD;
  mc.header = im.header;
  mc.pose = im.pose;
  // Cylinder points along Y
  tf2::fromMsg(mc.pose.orientation, imq);
  tmq.setRPY(boost::math::constants::pi<double>() / 2.0, 0, 0);
  imq = imq * tmq;
  mc.pose.orientation = tf2::toMsg(imq);
  mc.pose.position.x -= 0.04;
  mc.pose.position.z += 0.01;
  mc.color.r = 0.0f;
  mc.color.g = 1.0f;
  mc.color.b = 0.0f;
  mc.color.a = 1.0f;

  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = true;
  m_control.interaction_mode = m_control.BUTTON;
  m_control.markers.push_back(m);
  m_control.markers.push_back(mc);

  // add the control to the interactive marker
  im.controls.push_back(m_control);
}

void addErrorMarker(visualization_msgs::InteractiveMarker& im)
{
  // create a grey box marker
  visualization_msgs::Marker err;
  err.type = visualization_msgs::Marker::MESH_RESOURCE;
  err.scale.x = 0.002 * im.scale;
  err.scale.y = 0.002 * im.scale;
  err.scale.z = 0.002 * im.scale;
  err.mesh_resource = "package://moveit_ros_planning_interface/resources/access-denied.dae";
  err.ns = "robot_interaction_error";
  err.id = 1;
  err.action = visualization_msgs::Marker::ADD;
  err.header = im.header;
  err.pose = im.pose;
  err.pose.orientation.x = err.pose.orientation.y = 0.7071067811865476;
  err.pose.orientation.z = err.pose.orientation.w = 0.0;
  err.color.r = 1.0f;
  err.color.g = 0.0f;
  err.color.b = 0.0f;
  err.color.a = 1.0f;

  visualization_msgs::InteractiveMarkerControl err_control;
  err_control.always_visible = false;
  err_control.markers.push_back(err);

  // add the control to the interactive marker
  im.controls.push_back(err_control);
}

// value for normalized quaternion: 1.0 / std::sqrt(2.0)
static const double SQRT2INV = 0.707106781;

void addPlanarXYControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed)
{
  visualization_msgs::InteractiveMarkerControl control;

  if (orientation_fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = SQRT2INV;
  control.orientation.x = SQRT2INV;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = SQRT2INV;
  control.orientation.x = 0;
  control.orientation.y = SQRT2INV;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = SQRT2INV;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = SQRT2INV;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
}

void add6DOFControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed)
{
  addOrientationControl(int_marker, orientation_fixed);
  addPositionControl(int_marker, orientation_fixed);
}

void addOrientationControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed)
{
  visualization_msgs::InteractiveMarkerControl control;

  if (orientation_fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = SQRT2INV;
  control.orientation.x = SQRT2INV;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = SQRT2INV;
  control.orientation.x = 0;
  control.orientation.y = SQRT2INV;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = SQRT2INV;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = SQRT2INV;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
}

void addPositionControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed)
{
  visualization_msgs::InteractiveMarkerControl control;

  if (orientation_fixed)
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = SQRT2INV;
  control.orientation.x = SQRT2INV;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = SQRT2INV;
  control.orientation.x = 0;
  control.orientation.y = SQRT2INV;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = SQRT2INV;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = SQRT2INV;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
}

void addViewPlaneControl(visualization_msgs::InteractiveMarker& int_marker, double radius,
                         const std_msgs::ColorRGBA& color, bool position, bool orientation)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
  if (position && orientation)
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  else if (orientation)
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
  else
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
  control.independent_marker_orientation = true;
  control.name = "move";

  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = radius * 2.0;
  marker.scale.y = radius * 2.0;
  marker.scale.z = radius * 2.0;
  marker.color = color;

  control.markers.push_back(marker);
  control.always_visible = false;

  int_marker.controls.push_back(control);
}

visualization_msgs::InteractiveMarker makePlanarXYMarker(const std::string& name,
                                                         const geometry_msgs::PoseStamped& stamped, double scale,
                                                         bool orientation_fixed)
{
  visualization_msgs::InteractiveMarker int_marker = makeEmptyInteractiveMarker(name, stamped, scale);
  addPlanarXYControl(int_marker, orientation_fixed);
  return int_marker;
}

visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name, const geometry_msgs::PoseStamped& stamped,
                                                     double scale, bool orientation_fixed)
{
  visualization_msgs::InteractiveMarker int_marker = makeEmptyInteractiveMarker(name, stamped, scale);
  add6DOFControl(int_marker, orientation_fixed);
  return int_marker;
}
}  // namespace robot_interaction
