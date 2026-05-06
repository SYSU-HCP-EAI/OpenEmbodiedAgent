/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace robot_interaction
{
MOVEIT_CLASS_FORWARD(LockedRobotState);  // Defines LockedRobotStatePtr, ConstPtr, WeakPtr... etc

/// Maintain a RobotState in a multithreaded environment.
//
// Only allow one thread to modify the RobotState at a time.
//
// Allow any thread access to the RobotState when it is not being modified.  If
// a (const) reference to the robot state is held when the RobotState needs to
// be modified, a copy is made and the copy is modified.  Any externally held
// references will be out of date but still valid.
//
// The RobotState can only be modified by passing a callback function which
// does the modification.
class LockedRobotState
{
public:
  LockedRobotState(const moveit::core::RobotState& state);
  LockedRobotState(const moveit::core::RobotModelPtr& model);

  virtual ~LockedRobotState();

  /// get read-only access to the state.
  //
  // This state may go stale, meaning the maintained state has been updated,
  // but it will never be modified while the caller is holding a reference to
  // it.
  //
  // The transforms in the returned state will always be up to date.
  moveit::core::RobotStateConstPtr getState() const;

  /// Set the state to the new value.
  void setState(const moveit::core::RobotState& state);

  // This is a function that can modify the maintained state.
  typedef boost::function<void(moveit::core::RobotState*)> ModifyStateFunction;

  // Modify the state.
  //
  // This modifies the state by calling \e modify on it.
  // The \e modify function is passed a reference to the state which it can
  // modify.  No threads will be given access to the state while the \e modify
  // function is running.
  void modifyState(const ModifyStateFunction& modify);

protected:
  // This is called when the internally maintained state has changed.
  // This is called with state_lock_ unlocked.
  // Default definition does nothing.  Override to get notification of state
  // change.
  // TODO: is this needed?
  virtual void robotStateChanged();

protected:
  // this locks all accesses to the state_ member.
  // The lock can also be used by subclasses to lock additional fields.
  mutable boost::mutex state_lock_;

private:
  // The state maintained by this class.
  // When a modify function is being called this is NULL.
  // PROTECTED BY state_lock_
  moveit::core::RobotStatePtr state_;
};
}  // namespace robot_interaction
