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

#include <ompl/base/StateSampler.h>
#include <moveit/constraint_samplers/constraint_sampler.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

/** @class ConstrainedSampler
 *  This class defines a sampler that tries to find a sample that satisfies the constraints*/
class ConstrainedSampler : public ompl::base::StateSampler
{
public:
  /** @brief Default constructor
   *  @param pg The planning group
   *  @param cs A pointer to a kinematic constraint sampler
   */
  ConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs);

  /** @brief Sample a state (uniformly)*/
  void sampleUniform(ompl::base::State* state) override;

  /** @brief Sample a state (uniformly) within a certain distance of another state*/
  void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override;

  /** @brief Sample a state using the specified Gaussian*/
  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev) override;

  double getConstrainedSamplingRate() const;

private:
  bool sampleC(ompl::base::State* state);

  const ModelBasedPlanningContext* planning_context_;
  ompl::base::StateSamplerPtr default_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  moveit::core::RobotState work_state_;
  unsigned int constrained_success_;
  unsigned int constrained_failure_;
  double inv_dim_;
};
}  // namespace ompl_interface
