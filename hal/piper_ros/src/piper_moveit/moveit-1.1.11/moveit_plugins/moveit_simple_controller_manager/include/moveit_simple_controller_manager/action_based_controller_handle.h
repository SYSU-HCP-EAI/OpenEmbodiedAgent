/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <moveit/controller_manager/controller_manager.h>
#include <moveit/macros/class_forward.h>
#include <memory>

namespace moveit_simple_controller_manager
{
/*
 * This exist solely to inject addJoint/getJoints into base non-templated class.
 */
class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandleBase(const std::string& name) : moveit_controller_manager::MoveItControllerHandle(name)
  {
  }

  virtual void addJoint(const std::string& name) = 0;
  virtual void getJoints(std::vector<std::string>& joints) = 0;
  virtual void configure(XmlRpc::XmlRpcValue& /* config */)
  {
  }
};

MOVEIT_CLASS_FORWARD(
    ActionBasedControllerHandleBase);  // Defines ActionBasedControllerHandleBasePtr, ConstPtr, WeakPtr... etc

/*
 * This is a simple base class, which handles all of the action creation/etc
 */
template <typename T>
class ActionBasedControllerHandle : public ActionBasedControllerHandleBase
{
public:
  ActionBasedControllerHandle(const std::string& name, const std::string& ns)
    : ActionBasedControllerHandleBase(name), nh_("~"), done_(true), namespace_(ns)
  {
    controller_action_client_ = std::make_shared<actionlib::SimpleActionClient<T>>(getActionName(), true);
    unsigned int attempts = 0;
    double timeout;
    nh_.param("trajectory_execution/controller_connection_timeout", timeout, 15.0);

    if (timeout == 0.0)
    {
      while (ros::ok() && !controller_action_client_->waitForServer(ros::Duration(5.0)))
      {
        ROS_WARN_STREAM_NAMED("ActionBasedController", "Waiting for " << getActionName() << " to come up");
        ros::Duration(1).sleep();
      }
    }
    else
    {
      while (ros::ok() && !controller_action_client_->waitForServer(ros::Duration(timeout / 3)) && ++attempts < 3)
      {
        ROS_WARN_STREAM_NAMED("ActionBasedController", "Waiting for " << getActionName() << " to come up");
        ros::Duration(1).sleep();
      }
    }
    if (!controller_action_client_->isServerConnected())
    {
      ROS_ERROR_STREAM_NAMED("ActionBasedController", "Action client not connected: " << getActionName());
      controller_action_client_.reset();
    }

    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  bool isConnected() const
  {
    return static_cast<bool>(controller_action_client_);
  }

  bool cancelExecution() override
  {
    if (!controller_action_client_)
      return false;
    if (!done_)
    {
      ROS_INFO_STREAM_NAMED("ActionBasedController", "Cancelling execution for " << name_);
      controller_action_client_->cancelGoal();
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      done_ = true;
    }
    return true;
  }

  bool waitForExecution(const ros::Duration& timeout = ros::Duration(0)) override
  {
    if (controller_action_client_ && !done_)
      return controller_action_client_->waitForResult(timeout);
#if 1  // TODO: remove when https://github.com/ros/actionlib/issues/155 is fixed
    // workaround for actionlib issue: waitForResult() might return before our doneCB finished
    ros::Time deadline = ros::Time::now() + ros::Duration(0.1);  // limit waiting to 0.1s
    while (!done_ && ros::ok() && ros::Time::now() < deadline)   // Check the done_ flag explicitly,
      ros::Duration(0.0001).sleep();                             // which is eventually set in finishControllerExecution
#endif
    return true;
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return last_exec_;
  }

  void addJoint(const std::string& name) override
  {
    joints_.push_back(name);
  }

  void getJoints(std::vector<std::string>& joints) override
  {
    joints = joints_;
  }

protected:
  ros::NodeHandle nh_;
  std::string getActionName() const
  {
    if (namespace_.empty())
      return name_;
    else
      return name_ + "/" + namespace_;
  }

  void finishControllerExecution(const actionlib::SimpleClientGoalState& state)
  {
    ROS_DEBUG_STREAM_NAMED("ActionBasedController", "Controller " << name_ << " is done with state " << state.toString()
                                                                  << ": " << state.getText());
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    else
      last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
  }

  /* execution status */
  moveit_controller_manager::ExecutionStatus last_exec_;
  bool done_;

  /* the controller namespace, for instance, topics will map to name/ns/goal,
   * name/ns/result, etc */
  std::string namespace_;

  /* the joints controlled by this controller */
  std::vector<std::string> joints_;

  /* action client */
  std::shared_ptr<actionlib::SimpleActionClient<T>> controller_action_client_;
};

}  // end namespace moveit_simple_controller_manager
