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

#include <moveit/constraint_samplers/constraint_sampler.h>

namespace constraint_samplers
{
/**
 * \brief This class exists as a union of constraint samplers.  It
 * contains a vector of constraint samplers, and will sample from each
 * of them.
 *
 * When asked to sample it will call the samplers in a sorted order
 * that samples more general groups - like a robot's whole body -
 * before sampling more specific groups, such as a robot's arm.
 * Member samplers can operate on all or part of a joint state group
 * vector, with later samplers potentially overwriting previous
 * samplers.
 *
 */
class UnionConstraintSampler : public ConstraintSampler
{
public:
  /**
   * \brief Constructor, which will re-order its internal list of
   * samplers on construction.
   *
   * The samplers need not all refer to the same group, as long as all
   * are part of the kinematic model. The sampler will sort the
   * samplers based on a set of criteria - where A and B are two
   * samplers being considered for swapping by a sort algorithm:
   *
   * \li If the set of links updated by the group of A are a proper
   * subset of the set of links updated by the group of B, A and B are
   * not swapped.  If the updated links of B are a proper set of the
   * updated links of A, A and B are swapped.
   *
   * \li Otherwise, the groups associated with A and B are either
   * disjoint in terms of updated links or have an equivalent group.
   * In this case, it is determined if any updated links in the group for A
   * exist in the frame dependency of B, or vice-versa.
   *
   * \li If A depends on B, and B depends on A, a warning message is
   * printed that circular dependencies are likely to lead to bad
   * samples.  A and B are not swapped.
   *
   * \li If one of the frame dependencies of B is a link updated by A,
   * but not vice-versa, the samplers are swapped.
   *
   * \li If one of the frame dependencies of A is a link updated by B,
   * but not vice-versa, the samplers are not swapped.
   *
   * \li If no dependency exists, the samplers are swapped according
   * to alphabetical order.
   *
   * @param [in] scene The planning scene
   * @param [in] group_name The group name is ignored, as each sampler already has a group name
   * @param [in] samplers A vector of already configured samplers that will be applied for future samples
   *
   * @return
   */
  UnionConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name,
                         const std::vector<ConstraintSamplerPtr>& samplers);

  /**
   * \brief Gets the sorted internal list of constraint samplers
   *
   *
   * @return The sorted internal list of constraint samplers
   */
  const std::vector<ConstraintSamplerPtr>& getSamplers() const
  {
    return samplers_;
  }

  /**
   * \brief No-op, as the union constraint sampler is for already
   * configured samplers
   *
   * @param [in] constr Constraint message
   *
   * @return Always true
   */
  bool configure(const moveit_msgs::Constraints& /*constr*/) override
  {
    return true;
  }

  /**
   * \brief No-op, as the union constraint sampler can act on anything
   *
   * @param [in] constr Constraint message
   *
   * @return Always true
   */
  virtual bool canService(const moveit_msgs::Constraints& /*constr*/) const
  {
    return true;
  }

  /**
   * \brief Produces a sample from all configured samplers.
   *
   * This function will call each sampler in sorted order
   * independently of the group associated with the sampler.  The
   * function will also operate independently of the joint state group
   * passed in as an argument.  If any sampler fails, the sample fails
   * altogether.
   *
   * @param [in] state State where the group sample is written to
   * @param [in] reference_state Reference kinematic state that will be passed through to samplers
   * @param [in] max_attempts Max attempts, which will be passed through to samplers
   *
   * @return True if all invidual samplers return true
   */
  bool sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state,
              unsigned int max_attempts) override;

  /**
   * \brief Get the name of the constraint sampler, for debugging purposes
   * should be in CamelCase format.
   * \return string of name
   */
  const std::string& getName() const override
  {
    static const std::string SAMPLER_NAME = "UnionConstraintSampler";
    return SAMPLER_NAME;
  }

protected:
  std::vector<ConstraintSamplerPtr> samplers_; /**< \brief Holder for sorted internal list of samplers*/
};
}  // namespace constraint_samplers
