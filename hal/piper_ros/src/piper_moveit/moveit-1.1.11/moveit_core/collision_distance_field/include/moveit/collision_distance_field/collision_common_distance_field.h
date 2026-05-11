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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#pragma once

#include <moveit/robot_state/robot_state.h>
#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>

namespace collision_detection
{
MOVEIT_STRUCT_FORWARD(GroupStateRepresentation);
MOVEIT_STRUCT_FORWARD(DistanceFieldCacheEntry);

/** collision volume representation for a particular pose and link group
 *
 * This stores posed spheres making up the collision volume for a particular
 * link group in a particular pose.  It is associated with a particular dfce_
 * and can only be used with that dfce_ (DistanceFieldCacheEntry -- see below).
 * */
struct GroupStateRepresentation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GroupStateRepresentation(){};
  GroupStateRepresentation(const GroupStateRepresentation& gsr)
  {
    link_body_decompositions_.resize(gsr.link_body_decompositions_.size());
    for (unsigned int i = 0; i < gsr.link_body_decompositions_.size(); i++)
    {
      if (gsr.link_body_decompositions_[i])
      {
        link_body_decompositions_[i] =
            std::make_shared<PosedBodySphereDecomposition>(*gsr.link_body_decompositions_[i]);
      }
    }

    link_distance_fields_.assign(gsr.link_distance_fields_.begin(), gsr.link_distance_fields_.end());

    attached_body_decompositions_.resize(gsr.attached_body_decompositions_.size());
    for (unsigned int i = 0; i < gsr.attached_body_decompositions_.size(); i++)
    {
      (*attached_body_decompositions_[i]) = (*gsr.attached_body_decompositions_[i]);
    }
    gradients_ = gsr.gradients_;
  }

  /** dfce used to generate this GSR */
  DistanceFieldCacheEntryConstPtr dfce_;

  /** posed spheres representing collision volume for the links in the group
   * (dfce_.group_name_) and all links below the group (i.e. links that can
   * move if joints in the group move).  These are posed in the global frame
   * used for collision detection. */
  std::vector<PosedBodySphereDecompositionPtr> link_body_decompositions_;

  /** posed spheres representing collision volume for bodies attached to group
   * links */
  std::vector<PosedBodySphereDecompositionVectorPtr> attached_body_decompositions_;

  /** */
  std::vector<PosedDistanceFieldPtr> link_distance_fields_;

  /** information about detected collisions, collected during collision
   * detection.  One entry for each link and one entry for each attached
   * object. */
  std::vector<GradientInfo> gradients_;
};

/** collision volume representation for a particular link group of a robot
 *
 * This entry is specific to a particular robot model, a particular link group,
 * a particular ACM, and a particular configuration of attached objects.  It
 * assumes the poses of joints above this group have not changed, but can be
 * used with different poses of the joints within the group. */
struct DistanceFieldCacheEntry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** for checking collisions between this group and other objects */
  std::string group_name_;
  /** RobotState that this cache entry represents */
  moveit::core::RobotStatePtr state_;
  /** list of indices into the state_values_ vector.  One index for each joint
   * variable which is NOT in the group or a child of the group.  In other
   * words, variables which should not change if only joints in the group move.
   */
  std::vector<unsigned int> state_check_indices_;
  /** all the joint variables from all the joints in the entire robot (whether
   * in the group or not), excluding mimic joints.  If 2
   * DistanceFieldCacheEntrys have identical state_values_ then they are
   * equivalent (i.e. same robot state).  */
  std::vector<double> state_values_;
  /* the acm used when generating this dfce.  This dfce cannot be used to check
   * collisions with a different acm. */
  collision_detection::AllowedCollisionMatrix acm_;
  /** the distance field describing all links of the robot that are not in the
   * group and their attached bodies */
  distance_field::DistanceFieldPtr distance_field_;
  /** this can be used as a starting point for creating a
   * GroupStateRepresentation needed for collision checking */
  GroupStateRepresentationPtr pregenerated_group_state_representation_;
  /** names of all links in the group and all links below the group (links that
   * will move if any of the joints in the group move)
   */
  std::vector<std::string> link_names_;
  /** for each link in link_names_, true if the link has collision geometry */
  std::vector<bool> link_has_geometry_;
  /** for each link in link_names_, index into the
   * CollisionRobotDistanceField::link_body_decomposition_vector_ vector.  This
   * is 0 (and invalid) for links with no geometry.  The
   * link_body_decomposition_vector_ contains the (unposed) spheres that make
   * up the collision geometry for the link */
  std::vector<unsigned int> link_body_indices_;
  /** for each link in link_names_, index into the
   * RobotState::link_state_vector_ */
  std::vector<unsigned int> link_state_indices_;
  /** list of all bodies attached to links in link_names_ */
  std::vector<std::string> attached_body_names_;
  /** for each body in attached_body_names_, the index into the link state of
   * the link the body is attached to. */
  std::vector<unsigned int> attached_body_link_state_indices_;
  /** for each link in link_names_ and for each body in attached_body_names_,
   * true if this link should be checked for self collision */
  std::vector<bool> self_collision_enabled_;
  /** for each link in link_names_, a vector of bool indicating for each other
   * object (where object could be another link or an attacjed object) whether
   * to check for collisions between this link and that object.  Size of inner
   * and outer lists are the same and equal the sum of the size of link_names_
   * and attached_body_names_ */
  std::vector<std::vector<bool>> intra_group_collision_enabled_;
};

BodyDecompositionConstPtr getBodyDecompositionCacheEntry(const shapes::ShapeConstPtr& shape, double resolution);

PosedBodyPointDecompositionVectorPtr getCollisionObjectPointDecomposition(const collision_detection::World::Object& obj,
                                                                          double resolution);

PosedBodySphereDecompositionVectorPtr getAttachedBodySphereDecomposition(const moveit::core::AttachedBody* att,
                                                                         double resolution);

PosedBodyPointDecompositionVectorPtr getAttachedBodyPointDecomposition(const moveit::core::AttachedBody* att,
                                                                       double resolution);

void getBodySphereVisualizationMarkers(const GroupStateRepresentationPtr& gsr, const std::string& reference_frame,
                                       visualization_msgs::MarkerArray& body_marker_array);
}  // namespace collision_detection
