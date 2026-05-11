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

/* Author: Dave Coleman */

#pragma once

#include <map>
#include <moveit/macros/class_forward.h>
namespace planning_scene
{
MOVEIT_CLASS_FORWARD(PlanningScene);  // Defines PlanningScenePtr, ConstPtr, WeakPtr... etc
}

namespace moveit_setup_assistant
{
/**
 * \brief Reasons for disabling link pairs. Append "in collision" for understanding.
 * NOT_DISABLED means the link pair DOES do self collision checking
 */
enum DisabledReason
{
  NEVER,
  DEFAULT,
  ADJACENT,
  ALWAYS,
  USER,
  NOT_DISABLED
};

/**
 * \brief Store details on a pair of links
 */
struct LinkPairData
{
  // by default all link pairs are NOT disabled for collision checking
  LinkPairData() : reason(NOT_DISABLED), disable_check(false){};
  DisabledReason reason;
  bool disable_check;
};

/**
 * \brief LinkPairMap is an adjacency list structure containing links in string-based form. Used for disabled links
 */
typedef std::map<std::pair<std::string, std::string>, LinkPairData> LinkPairMap;

/**
 * \brief Generate an adjacency list of links that are always and never in collision, to speed up collision detection
 * \param parent_scene A reference to the robot in the planning scene
 * \param include_never_colliding Flag to disable the check for links that are never in collision
 * \param trials Set the number random collision checks that are made. Increase the probability of correctness
 * \param min_collision_fraction If collisions are found between a pair of links >= this fraction, the are assumed
 * "always" in collision
 * \return Adj List of unique set of pairs of links in string-based form
 */
LinkPairMap computeDefaultCollisions(const planning_scene::PlanningSceneConstPtr& parent_scene, unsigned int* progress,
                                     const bool include_never_colliding, const unsigned int trials,
                                     const double min_collision_faction, const bool verbose);

/**
 * \brief Generate a list of unique link pairs for all links with geometry. Order pairs alphabetically. n choose 2 pairs
 * \param scene A reference to the robot in the planning scene
 * \param link_pairs List of all unique link pairs and each pair's properties
 **/
void computeLinkPairs(const planning_scene::PlanningScene& scene, LinkPairMap& link_pairs);

/**
 * \brief Converts a reason for disabling a link pair into a string
 * \param reason enum reason type
 * \return reason as string
 */
const std::string disabledReasonToString(DisabledReason reason);

/**
 * \brief Converts a string reason for disabling a link pair into a struct data type
 * \param reason string that should match one of the DisableReason types. If not, is set as "USER"
 * \return reason as struct
 */
DisabledReason disabledReasonFromString(const std::string& reason);
}  // namespace moveit_setup_assistant
