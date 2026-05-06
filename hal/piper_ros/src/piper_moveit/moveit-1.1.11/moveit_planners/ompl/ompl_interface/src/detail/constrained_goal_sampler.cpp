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

#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/profiler/profiler.h>

#include <utility>

namespace ompl_interface
{
constexpr char LOGNAME[] = "constrained_goal_sampler";
}  // namespace ompl_interface

ompl_interface::ConstrainedGoalSampler::ConstrainedGoalSampler(const ModelBasedPlanningContext* pc,
                                                               kinematic_constraints::KinematicConstraintSetPtr ks,
                                                               constraint_samplers::ConstraintSamplerPtr cs)
  : ob::GoalLazySamples(
        pc->getOMPLSimpleSetup()->getSpaceInformation(),
        [this](const GoalLazySamples* gls, ompl::base::State* state) {
          return sampleUsingConstraintSampler(gls, state);
        },
        false)
  , planning_context_(pc)
  , kinematic_constraint_set_(std::move(ks))
  , constraint_sampler_(std::move(cs))
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
{
  if (!constraint_sampler_)
    default_sampler_ = si_->allocStateSampler();
  ROS_DEBUG_NAMED(LOGNAME, "Constructed a ConstrainedGoalSampler instance at address %p", this);
  startSampling();
}

bool ompl_interface::ConstrainedGoalSampler::checkStateValidity(ob::State* new_goal,
                                                                const moveit::core::RobotState& state,
                                                                bool verbose) const
{
  planning_context_->getOMPLStateSpace()->copyToOMPLState(new_goal, state);
  return static_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose);
}

bool ompl_interface::ConstrainedGoalSampler::stateValidityCallback(ob::State* new_goal,
                                                                   moveit::core::RobotState const* state,
                                                                   const moveit::core::JointModelGroup* jmg,
                                                                   const double* jpos, bool verbose) const
{
  // we copy the state to not change the seed state
  moveit::core::RobotState solution_state(*state);
  solution_state.setJointGroupPositions(jmg, jpos);
  solution_state.update();
  return checkStateValidity(new_goal, solution_state, verbose);
}

bool ompl_interface::ConstrainedGoalSampler::sampleUsingConstraintSampler(const ob::GoalLazySamples* gls,
                                                                          ob::State* new_goal)
{
  //  moveit::Profiler::ScopedBlock sblock("ConstrainedGoalSampler::sampleUsingConstraintSampler");

  unsigned int max_attempts = planning_context_->getMaximumGoalSamplingAttempts();
  unsigned int attempts_so_far = gls->samplingAttemptsCount();

  // terminate after too many attempts
  if (attempts_so_far >= max_attempts)
    return false;

  // terminate after a maximum number of samples
  if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
    return false;

  // terminate the sampling thread when a solution has been found
  if (planning_context_->getOMPLSimpleSetup()->getProblemDefinition()->hasSolution())
    return false;

  unsigned int max_attempts_div2 = max_attempts / 2;
  for (unsigned int a = gls->samplingAttemptsCount(); a < max_attempts && gls->isSampling(); ++a)
  {
    bool verbose = false;
    if (gls->getStateCount() == 0 && a >= max_attempts_div2)
      if (verbose_display_ < 1)
      {
        verbose = true;
        verbose_display_++;
      }

    if (constraint_sampler_)
    {
      // makes the constraint sampler also perform a validity callback
      moveit::core::GroupStateValidityCallbackFn gsvcf = [this, new_goal,
                                                          verbose](moveit::core::RobotState* robot_state,
                                                                   const moveit::core::JointModelGroup* joint_group,
                                                                   const double* joint_group_variable_values) {
        return stateValidityCallback(new_goal, robot_state, joint_group, joint_group_variable_values, verbose);
      };
      constraint_sampler_->setGroupStateValidityCallback(gsvcf);

      if (constraint_sampler_->sample(work_state_, planning_context_->getMaximumStateSamplingAttempts()))
      {
        work_state_.update();
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
        {
          if (checkStateValidity(new_goal, work_state_, verbose))
            return true;
        }
        else
        {
          invalid_sampled_constraints_++;
          if (!warned_invalid_samples_ && invalid_sampled_constraints_ >= (attempts_so_far * 8) / 10)
          {
            warned_invalid_samples_ = true;
            ROS_WARN_NAMED(LOGNAME, "More than 80%% of the sampled goal states "
                                    "fail to satisfy the constraints imposed on the goal sampler. "
                                    "Is the constrained sampler working correctly?");
          }
        }
      }
    }
    else
    {
      default_sampler_->sampleUniform(new_goal);
      if (static_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose))
      {
        planning_context_->getOMPLStateSpace()->copyToRobotState(work_state_, new_goal);
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          return true;
      }
    }
  }
  return false;
}
