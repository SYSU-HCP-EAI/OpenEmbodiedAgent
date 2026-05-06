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

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <moveit/collision_detection/collision_common.h>
#include <ompl/base/StateValidityChecker.h>

namespace ompl_interface
{
class ModelBasedPlanningContext;

/** @class StateValidityChecker
    @brief An interface for a OMPL state validity checker*/
class StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(const ModelBasedPlanningContext* planning_context);

  bool isValid(const ompl::base::State* state) const override
  {
    return isValid(state, verbose_);
  }

  bool isValid(const ompl::base::State* state, double& dist) const override
  {
    return isValid(state, dist, verbose_);
  }

  bool isValid(const ompl::base::State* state, double& dist, ompl::base::State* /*validState*/,
               bool& /*validStateAvailable*/) const override
  {
    return isValid(state, dist, verbose_);
  }

  bool isValid(const ompl::base::State* state, bool verbose) const;
  bool isValid(const ompl::base::State* state, double& dist, bool verbose) const;

  virtual double cost(const ompl::base::State* state) const;
  double clearance(const ompl::base::State* state) const override;

  void setVerbose(bool flag);

protected:
  const ModelBasedPlanningContext* planning_context_;
  std::string group_name_;
  TSStateStorage tss_;
  collision_detection::CollisionRequest collision_request_simple_;
  collision_detection::CollisionRequest collision_request_with_distance_;
  collision_detection::CollisionRequest collision_request_simple_verbose_;
  collision_detection::CollisionRequest collision_request_with_distance_verbose_;

  collision_detection::CollisionRequest collision_request_with_cost_;
  bool verbose_;
};
}  // namespace ompl_interface
