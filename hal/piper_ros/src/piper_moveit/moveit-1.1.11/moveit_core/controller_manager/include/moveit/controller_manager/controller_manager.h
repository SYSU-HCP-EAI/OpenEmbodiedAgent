/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <vector>
#include <string>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/macros/class_forward.h>

/// Namespace for the base class of a MoveIt controller manager
namespace moveit_controller_manager
{
/// The reported execution status
struct ExecutionStatus
{
  enum Value
  {
    UNKNOWN,
    RUNNING,
    SUCCEEDED,
    PREEMPTED,
    TIMED_OUT,
    ABORTED,
    FAILED
  };

  ExecutionStatus(Value value = UNKNOWN) : status_(value)
  {
  }

  operator Value() const
  {
    return status_;
  }

  explicit operator bool() const
  {
    return status_ == SUCCEEDED;
  }

  /// Convert the execution status to a string
  std::string asString() const
  {
    switch (status_)
    {
      case RUNNING:
        return "RUNNING";
      case SUCCEEDED:
        return "SUCCEEDED";
      case PREEMPTED:
        return "PREEMPTED";
      case TIMED_OUT:
        return "TIMED_OUT";
      case ABORTED:
        return "ABORTED";
      case FAILED:
        return "FAILED";
      default:
        return "UNKNOWN";
    }
  }

private:
  Value status_;
};

MOVEIT_CLASS_FORWARD(MoveItControllerHandle);  // Defines MoveItControllerHandlePtr, ConstPtr, WeakPtr... etc

/** \brief MoveIt sends commands to a controller via a handle that satisfies this interface. */
class MoveItControllerHandle
{
public:
  /** \brief Each controller has a name. The handle is initialized with that name */
  MoveItControllerHandle(const std::string& name) : name_(name)
  {
  }

  virtual ~MoveItControllerHandle()
  {
  }

  /** \brief Get the name of the controller this handle can send commands to */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief Send a trajectory to the controller.
   *
   * The controller is expected to execute the trajectory, but this function call should not block.
   * Blocking is achievable by calling waitForExecution().
   * Return false when the controller cannot accept the trajectory. */
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) = 0;

  /** \brief Cancel the execution of any motion using this controller.
   *
   * Report false if canceling is not possible.
   * If there is no execution in progress, this function is a no-op and returns true. */
  virtual bool cancelExecution() = 0;

  /** \brief Wait for the current execution to complete, or until the timeout is reached.
   *
   * Return true if the execution is complete (whether successful or not).
   * Return false if timeout was reached.
   * If timeout is 0 (default argument), wait until the execution is complete (no timeout). */
  virtual bool waitForExecution(const ros::Duration& timeout = ros::Duration(0)) = 0;

  /** \brief Return the execution status of the last trajectory sent to the controller. */
  virtual ExecutionStatus getLastExecutionStatus() = 0;

protected:
  std::string name_;
};

MOVEIT_CLASS_FORWARD(MoveItControllerManager);  // Defines MoveItControllerManagerPtr, ConstPtr, WeakPtr... etc

/** @brief MoveIt does not enforce how controllers are implemented.
    To make your controllers usable by MoveIt, this interface needs to be implemented.
    The main purpose of this interface is to expose the set of known controllers and
    potentially to allow activating and deactivating them, if multiple controllers are available.
 */
class MoveItControllerManager
{
public:
  /** \brief Each controller known to MoveIt has a state. This
      structure describes that controller's state. */
  struct ControllerState
  {
    ControllerState() : active_(false), default_(false)
    {
    }

    /** \brief A controller can be active or inactive. This means that MoveIt could activate the controller when
       needed, and de-activate controllers that overlap (control the same set of joints) */
    bool active_;

    /** \brief It is often the case that multiple controllers could be used to execute a motion. Marking a controller as
       default makes MoveIt prefer this controller when multiple options are available. */
    bool default_;
  };

  /** \brief Default constructor. This needs to have no arguments so that the plugin system can construct the object. */
  MoveItControllerManager()
  {
  }

  virtual ~MoveItControllerManager()
  {
  }

  /** \brief Return a given named controller. */
  virtual MoveItControllerHandlePtr getControllerHandle(const std::string& name) = 0;

  /** \brief Get the list of known controller names. */
  virtual void getControllersList(std::vector<std::string>& names) = 0;

  /** \brief Get the list of active controllers.
   *
   * If there is only one controller in the system, this will be active.  When multiple controllers exist,
   * and they operate on overlaping sets of joints, not all controllers should be active at the same time. */
  virtual void getActiveControllers(std::vector<std::string>& names) = 0;

  /** \brief Report the joints a controller operates on, given the controller name.
   *
   * In order to decide which controller to use, it is necessary to reason about the joints a controller
   * operates on. */
  virtual void getControllerJoints(const std::string& name, std::vector<std::string>& joints) = 0;

  /** \brief Report the state of a controller, given its name. */
  virtual ControllerState getControllerState(const std::string& name) = 0;

  /** \brief Activate and deactivate controllers */
  virtual bool switchControllers(const std::vector<std::string>& activate,
                                 const std::vector<std::string>& deactivate) = 0;
};
}  // namespace moveit_controller_manager
