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

#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <moveit/macros/class_forward.h>

namespace constraint_samplers
{
MOVEIT_CLASS_FORWARD(ConstraintSamplerManager);  // Defines ConstraintSamplerManagerPtr, ConstPtr, WeakPtr... etc

/**
 * \brief This class assists in the generation of a ConstraintSampler for a
 * particular group from a moveit_msgs::Constraints.
 *
 * It contains logic that will generate either a
 * JointConstraintSampler, an IKConstraintSampler, or a
 * UnionConstraintSampler depending on the contents of the Constraints
 * message and the group in question.
 *
 */
class ConstraintSamplerManager
{
public:
  /**
   * \brief Empty constructor
   *
   */
  ConstraintSamplerManager()
  {
  }
  /**
   * \brief Allows the user to specify an alternate ConstraintSamplerAllocation
   *
   * @param sa The constraint sampler allocator that will be used
   */
  void registerSamplerAllocator(const ConstraintSamplerAllocatorPtr& sa)
  {
    sampler_alloc_.push_back(sa);
  }
  /**
   * \brief Selects among the potential sampler allocators.
   *
   * This function will iterate through the constraint sampler
   * allocators, trying to find one that can service the constraints.
   * The first one that can service the request will be called.  If no
   * allocators can service the Constraints, or there are no
   * allocators, the selectDefaultSampler will be called.
   *
   * @param scene The planning scene that will be passed into the constraint sampler
   * @param group_name The group name for which to allocate the constraint sampler
   * @param constr The constraints
   *
   * @return An allocated ConstraintSamplerPtr,
   * or an empty pointer if none could be allocated
   */
  ConstraintSamplerPtr selectSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                                     const moveit_msgs::Constraints& constr) const;

  /**
   * \brief Default logic to select a ConstraintSampler given a
   * constraints message.
   *
   * This function will generate a sampler using the joint_constraint,
   * position_constraint, and orientation_constraint vectors from the
   * Constraints argument.  The type of constraint sampler that is
   * produced depends on which constraint vectors have been populated.
   * The following rules are applied:
   *
   * - If every joint in the group indicated by group_name is
   * constrained by a valid joint constraint in the joint_constraints
   * vector, a JointConstraintSampler with all bounded joints in
   * returned.
   * - If not every joint is constrained, but no position and
   * orientation constraints are specified, or no valid
   * IKConstraintSampler can be created, then a JointConstraintSampler
   * with some unbounded joints is returned.
   * - If position and orientation constraints are present and there
   * is an IKSolver for the group, the function will attempt to
   * create an IKConstraintSampler.
   *   - If there are multiple valid position/constraint pairings, the
   *     one with the smallest volume will be kept.
   *   - If no full pose is available, the function will attempt to create a position-only IKConstraintSampler.
   *   - Finally, the function will attempt to create an orientation-only IKConstraintSampler.
   *   - If there is a valid IKConstraintSampler, then if no valid joint constraints are present then an
   *IKConstraintSampler will be returned.
   *   - If there are joint constraints, a UnionConstraintSampler with both the JointConstraintSampler and the
   *IKConstraintSampler will be returned.
   * - If there is no direct IK solver for the group, or no valid IKConstraintSampler could be generated, and there are
   *subgroup IKSolvers, the function will attempt to generate a sampler from the various subgroup solvers.
   *   - It will attempt to determine which constraints act on the IK link for the sub-group IK solvers, and attempts to
   *create ConstraintSampler functions by recursively calling \ref selectDefaultSampler for the sub-group.
   *   - If any samplers are valid, it adds them to a vector of type \ref ConstraintSamplerPtr.
   *   - Once it has iterated through each sub-group, if any samplers are valid, they are returned in a
   *UnionConstraintSampler, along with a JointConstraintSampler if one exists.
   * @param scene The planning scene that will be used to create the ConstraintSampler
   * @param group_name The group name for which to create a sampler
   * @param constr The set of constraints for which to create a sampler
   *
   * @return A valid \ref ConstraintSamplerPtr if one could be allocated, and otherwise an empty \ref
   *ConstraintSamplerPtr
   */
  static ConstraintSamplerPtr selectDefaultSampler(const planning_scene::PlanningSceneConstPtr& scene,
                                                   const std::string& group_name,
                                                   const moveit_msgs::Constraints& constr);

private:
  std::vector<ConstraintSamplerAllocatorPtr>
      sampler_alloc_; /**< \brief Holds the constraint sampler allocators, which will be tested in order  */
};
}  // namespace constraint_samplers
