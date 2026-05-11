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

#include <ompl/base/goals/GoalLazySamples.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/constraint_samplers/constraint_sampler.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

/** @class ConstrainedGoalSampler
 *  An interface to the OMPL goal lazy sampler*/
class ConstrainedGoalSampler : public ompl::base::GoalLazySamples
{
public:
  ConstrainedGoalSampler(const ModelBasedPlanningContext* pc, kinematic_constraints::KinematicConstraintSetPtr ks,
                         constraint_samplers::ConstraintSamplerPtr cs = constraint_samplers::ConstraintSamplerPtr());

private:
  bool sampleUsingConstraintSampler(const ompl::base::GoalLazySamples* gls, ompl::base::State* new_goal);
  bool stateValidityCallback(ompl::base::State* new_goal, moveit::core::RobotState const* state,
                             const moveit::core::JointModelGroup* /*jmg*/, const double* /*jpos*/,
                             bool verbose = false) const;
  bool checkStateValidity(ompl::base::State* new_goal, const moveit::core::RobotState& state,
                          bool verbose = false) const;

  const ModelBasedPlanningContext* planning_context_;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  ompl::base::StateSamplerPtr default_sampler_;
  moveit::core::RobotState work_state_;
  unsigned int invalid_sampled_constraints_;
  bool warned_invalid_samples_;
  unsigned int verbose_display_;
};
}  // namespace ompl_interface
