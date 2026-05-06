/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2013, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2014, SRI International
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

/* Author: Ioan Sucan, Acorn Pooley */

#include <moveit/robot_interaction/locked_robot_state.h>

robot_interaction::LockedRobotState::LockedRobotState(const moveit::core::RobotState& state)
  : state_(new moveit::core::RobotState(state))
{
  state_->update();
}

robot_interaction::LockedRobotState::LockedRobotState(const moveit::core::RobotModelPtr& model)
  : state_(new moveit::core::RobotState(model))
{
  state_->setToDefaultValues();
  state_->update();
}

robot_interaction::LockedRobotState::~LockedRobotState() = default;

moveit::core::RobotStateConstPtr robot_interaction::LockedRobotState::getState() const
{
  boost::mutex::scoped_lock lock(state_lock_);
  return state_;
}

void robot_interaction::LockedRobotState::setState(const moveit::core::RobotState& state)
{
  {
    boost::mutex::scoped_lock lock(state_lock_);

    // If someone else has a reference to the state, then make a new copy.
    // The old state is orphaned (does not change, but is now out of date).
    if (state_.unique())
      *state_ = state;
    else
      state_ = std::make_shared<moveit::core::RobotState>(state);

    state_->update();
  }
  robotStateChanged();
}

void robot_interaction::LockedRobotState::modifyState(const ModifyStateFunction& modify)
{
  {
    boost::mutex::scoped_lock lock(state_lock_);

    // If someone else has a reference to the state, then make a copy.
    // The old state is orphaned (does not change, but is now out of date).
    if (!state_.unique())
      state_ = std::make_shared<moveit::core::RobotState>(*state_);

    modify(state_.get());
    state_->update();
  }
  robotStateChanged();
}

void robot_interaction::LockedRobotState::robotStateChanged()
{
}
