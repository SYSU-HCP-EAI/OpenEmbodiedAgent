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

#pragma once

#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>

namespace robot_interaction
{
visualization_msgs::InteractiveMarker
makeEmptyInteractiveMarker(const std::string& name, const geometry_msgs::PoseStamped& stamped, double scale);

visualization_msgs::InteractiveMarker make6DOFMarker(const std::string& name, const geometry_msgs::PoseStamped& stamped,
                                                     double scale, bool orientation_fixed = false);

visualization_msgs::InteractiveMarker makePlanarXYMarker(const std::string& name,
                                                         const geometry_msgs::PoseStamped& stamped, double scale,
                                                         bool orientation_fixed = false);

void addTArrowMarker(visualization_msgs::InteractiveMarker& im);

void addErrorMarker(visualization_msgs::InteractiveMarker& im);

void add6DOFControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed = false);

void addPlanarXYControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed = false);

void addOrientationControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed = false);

void addPositionControl(visualization_msgs::InteractiveMarker& int_marker, bool orientation_fixed = false);

void addViewPlaneControl(visualization_msgs::InteractiveMarker& int_marker, double radius,
                         const std_msgs::ColorRGBA& color, bool position = true, bool orientation = true);
}  // namespace robot_interaction
