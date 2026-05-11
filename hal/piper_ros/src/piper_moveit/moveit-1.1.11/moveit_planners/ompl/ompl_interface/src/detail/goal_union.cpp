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

#include <moveit/ompl_interface/detail/goal_union.h>
#include <ompl/base/goals/GoalLazySamples.h>

namespace
{
ompl::base::SpaceInformationPtr getGoalsSI(const std::vector<ompl::base::GoalPtr>& goals)
{
  if (goals.empty())
    return ompl::base::SpaceInformationPtr();
  for (const ompl::base::GoalPtr& goal : goals)
    if (!goal->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
      throw ompl::Exception("Multiplexed goals must be instances of GoalSampleableRegion");
  for (const ompl::base::GoalPtr& goal : goals)
    if (goal->getSpaceInformation() != goals[0]->getSpaceInformation())
      throw ompl::Exception("The instance of SpaceInformation must be the same among the goals to be considered");
  return goals[0]->getSpaceInformation();
}
}  // namespace

ompl_interface::GoalSampleableRegionMux::GoalSampleableRegionMux(const std::vector<ompl::base::GoalPtr>& goals)
  : ompl::base::GoalSampleableRegion(getGoalsSI(goals)), goals_(goals), gindex_(0)
{
}

void ompl_interface::GoalSampleableRegionMux::startSampling()
{
  for (ompl::base::GoalPtr& goal : goals_)
    if (goal->hasType(ompl::base::GOAL_LAZY_SAMPLES))
      static_cast<ompl::base::GoalLazySamples*>(goal.get())->startSampling();
}

void ompl_interface::GoalSampleableRegionMux::stopSampling()
{
  for (ompl::base::GoalPtr& goal : goals_)
    if (goal->hasType(ompl::base::GOAL_LAZY_SAMPLES))
      static_cast<ompl::base::GoalLazySamples*>(goal.get())->stopSampling();
}

void ompl_interface::GoalSampleableRegionMux::sampleGoal(ompl::base::State* st) const
{
  for (std::size_t i = 0; i < goals_.size(); ++i)
  {
    if (goals_[gindex_]->as<ompl::base::GoalSampleableRegion>()->maxSampleCount() > 0)
    {
      goals_[gindex_]->as<ompl::base::GoalSampleableRegion>()->sampleGoal(st);
      return;
    }
    gindex_ = (gindex_ + 1) % goals_.size();
  }
  throw ompl::Exception("There are no states to sample");
}

unsigned int ompl_interface::GoalSampleableRegionMux::maxSampleCount() const
{
  unsigned int sc = 0;
  for (const ompl::base::GoalPtr& goal : goals_)
    sc += goal->as<GoalSampleableRegion>()->maxSampleCount();
  return sc;
}

bool ompl_interface::GoalSampleableRegionMux::canSample() const
{
  for (const ompl::base::GoalPtr& goal : goals_)
    if (goal->as<ompl::base::GoalSampleableRegion>()->canSample())
      return true;
  return false;
}

bool ompl_interface::GoalSampleableRegionMux::couldSample() const
{
  for (const ompl::base::GoalPtr& goal : goals_)
    if (goal->as<ompl::base::GoalSampleableRegion>()->couldSample())
      return true;
  return false;
}

bool ompl_interface::GoalSampleableRegionMux::isSatisfied(const ompl::base::State* st, double* distance) const
{
  for (const ompl::base::GoalPtr& goal : goals_)
    if (goal->isSatisfied(st, distance))
      return true;
  return false;
}

double ompl_interface::GoalSampleableRegionMux::distanceGoal(const ompl::base::State* st) const
{
  double min_d = std::numeric_limits<double>::infinity();
  for (const ompl::base::GoalPtr& goal : goals_)
  {
    double d = goal->as<ompl::base::GoalRegion>()->distanceGoal(st);
    if (d < min_d)
      min_d = d;
  }
  return min_d;
}

void ompl_interface::GoalSampleableRegionMux::print(std::ostream& out) const
{
  out << "MultiGoal [" << std::endl;
  for (const ompl::base::GoalPtr& goal : goals_)
    goal->print(out);
  out << "]" << std::endl;
}
