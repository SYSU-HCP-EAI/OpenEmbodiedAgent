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

/* Author: Adam Leeper */

#pragma once

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_env.h>

namespace collision_detection
{
/** @brief Re-proceses contact normals for an octomap by estimating a metaball
 *   iso-surface using the centers of occupied cells in a neighborhood of the contact point.
 *
 *  This is an implementation of the algorithm described in:
 *  A. Leeper, S. Chan, K. Salisbury. Point Clouds Can Be Represented as Implicit
 *  Surfaces for Constraint-Based Haptic Rendering. ICRA, May 2012, St Paul, MN.
 *  http://adamleeper.com/research/papers/2012_ICRA_leeper-chan-salisbury.pdf
 *
 *  @param The octomap originally used for collision detection.
 *  @param The collision result (which will get its normals updated)
 *  @param The distance, as a multiple of the octomap cell size, from which to include neighboring cells.
 *  @param The minimum angle change required for a normal to be over-written
 *  @param Whether to request a depth estimate from the algorithm (experimental...)
 *  @param The iso-surface threshold value (0.5 is a reasonable default).
 *  @param The metaball radius, as a multiple of the octomap cell size (1.5 is a reasonable default)
 */
int refineContactNormals(const World::ObjectConstPtr& object, CollisionResult& res,
                         double cell_bbx_search_distance = 1.0, double allowed_angle_divergence = 0.0,
                         bool estimate_depth = false, double iso_value = 0.5, double metaball_radius_multiple = 1.5);
}  // namespace collision_detection
