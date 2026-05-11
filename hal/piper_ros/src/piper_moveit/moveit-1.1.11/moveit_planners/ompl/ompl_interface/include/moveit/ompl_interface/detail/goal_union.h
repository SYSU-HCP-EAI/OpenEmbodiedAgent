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

#include <ompl/base/goals/GoalSampleableRegion.h>

namespace ompl_interface
{
/** @class GoalSampleableRegionMux*/
class GoalSampleableRegionMux : public ompl::base::GoalSampleableRegion
{
public:
  /** @brief Constructor
   *  @param goals The input set of goals*/
  GoalSampleableRegionMux(const std::vector<ompl::base::GoalPtr>& goals);

  ~GoalSampleableRegionMux() override
  {
  }

  /** @brief Sample a goal*/
  void sampleGoal(ompl::base::State* st) const override;

  /** @brief Get the max sample count*/
  unsigned int maxSampleCount() const override;

  /** @brief Query if sampler can find any sample*/
  virtual bool canSample() const;

  /** @brief Query if sampler could find a sample in the future */
  bool couldSample() const override;

  /** @brief Is the goal satisfied for this state (given a distance)*/
  bool isSatisfied(const ompl::base::State* st, double* distance) const override;

  /** @brief Find the distance of this state from the goal*/
  double distanceGoal(const ompl::base::State* st) const override;

  /** @brief If there are any member lazy samplers, start them */
  void startSampling();

  /** @brief If there are any member lazy samplers, stop them */
  void stopSampling();

  /** @brief Pretty print goal information*/
  void print(std::ostream& out = std::cout) const override;

protected:
  std::vector<ompl::base::GoalPtr> goals_;
  mutable unsigned int gindex_;
};
}  // namespace ompl_interface
