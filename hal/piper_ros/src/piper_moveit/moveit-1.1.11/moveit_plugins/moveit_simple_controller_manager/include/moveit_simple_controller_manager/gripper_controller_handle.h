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

#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <control_msgs/GripperCommandAction.h>
#include <set>

namespace moveit_simple_controller_manager
{
/*
 * This is an interface for a gripper using control_msgs/GripperCommandAction
 * action interface (single DOF).
 */
class GripperControllerHandle : public ActionBasedControllerHandle<control_msgs::GripperCommandAction>
{
public:
  /* Topics will map to name/ns/goal, name/ns/result, etc */
  GripperControllerHandle(const std::string& name, const std::string& ns, const double max_effort = 0.0)
    : ActionBasedControllerHandle<control_msgs::GripperCommandAction>(name, ns)
    , allow_failure_(false)
    , parallel_jaw_gripper_(false)
    , max_effort_(max_effort)
  {
  }

  bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override
  {
    ROS_DEBUG_STREAM_NAMED("GripperController", "Received new trajectory for " << name_);

    if (!controller_action_client_)
      return false;

    if (!trajectory.multi_dof_joint_trajectory.points.empty())
    {
      ROS_ERROR_NAMED("GripperController", "Gripper cannot execute multi-dof trajectories.");
      return false;
    }

    if (trajectory.joint_trajectory.points.empty())
    {
      ROS_ERROR_NAMED("GripperController", "GripperController requires at least one joint trajectory point.");
      return false;
    }

    if (trajectory.joint_trajectory.points.size() > 1)
    {
      ROS_DEBUG_STREAM_NAMED("GripperController", "Trajectory: " << trajectory.joint_trajectory);
    }

    if (trajectory.joint_trajectory.joint_names.empty())
    {
      ROS_ERROR_NAMED("GripperController", "No joint names specified");
      return false;
    }

    std::vector<std::size_t> gripper_joint_indexes;
    for (std::size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i)
    {
      if (command_joints_.find(trajectory.joint_trajectory.joint_names[i]) != command_joints_.end())
      {
        gripper_joint_indexes.push_back(i);
        if (!parallel_jaw_gripper_)
          break;
      }
    }

    if (gripper_joint_indexes.empty())
    {
      ROS_WARN_NAMED("GripperController", "No command_joint was specified for the MoveIt controller gripper handle. \
                      Please see GripperControllerHandle::addCommandJoint() and \
                      GripperControllerHandle::setCommandJoint(). Assuming index 0.");
      gripper_joint_indexes.push_back(0);
    }

    // goal to be sent
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.0;

    // send last point
    int tpoint = trajectory.joint_trajectory.points.size() - 1;
    ROS_DEBUG_NAMED("GripperController", "Sending command from trajectory point %d", tpoint);

    // fill in goal from last point
    for (std::size_t idx : gripper_joint_indexes)
    {
      if (trajectory.joint_trajectory.points[tpoint].positions.size() <= idx)
      {
        ROS_ERROR_NAMED("GripperController", "GripperController expects a joint trajectory with one \
                         point that specifies at least the position of joint \
                         '%s', but insufficient positions provided",
                        trajectory.joint_trajectory.joint_names[idx].c_str());
        return false;
      }
      goal.command.position += trajectory.joint_trajectory.points[tpoint].positions[idx];

      if (trajectory.joint_trajectory.points[tpoint].effort.size() > idx)
      {
        goal.command.max_effort = trajectory.joint_trajectory.points[tpoint].effort[idx];
      }
      else
      {
        goal.command.max_effort = max_effort_;
      }
    }

    controller_action_client_->sendGoal(
        goal, [this](const auto& state, const auto& result) { controllerDoneCallback(state, result); },
        [this] { controllerActiveCallback(); }, [this](const auto& feedback) { controllerFeedbackCallback(feedback); });

    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  void setCommandJoint(const std::string& name)
  {
    command_joints_.clear();
    addCommandJoint(name);
  }

  void addCommandJoint(const std::string& name)
  {
    command_joints_.insert(name);
  }

  void allowFailure(bool allow)
  {
    allow_failure_ = allow;
  }

  void setParallelJawGripper(const std::string& left, const std::string& right)
  {
    addCommandJoint(left);
    addCommandJoint(right);
    parallel_jaw_gripper_ = true;
  }

private:
  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::GripperCommandResultConstPtr& /* result */)
  {
    if (state == actionlib::SimpleClientGoalState::ABORTED && allow_failure_)
      finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
    else
      finishControllerExecution(state);
  }

  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM_NAMED("GripperController", name_ << " started execution");
  }

  void controllerFeedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr& /* feedback */)
  {
  }

  /*
   * Some gripper drivers may indicate a failure if they do not close all the way when
   * an object is in the gripper.
   */
  bool allow_failure_;

  /*
   * A common setup is where there are two joints that each move
   * half the overall distance. Thus, the command to the gripper
   * should be the sum of the two joint distances.
   *
   * When this is set, command_joints_ should be of size 2,
   * and the command will be the sum of the two joints.
   */
  bool parallel_jaw_gripper_;

  /*
   * The ``max_effort`` used in the GripperCommand message when no ``max_effort`` was
   * specified in the requested trajectory. Defaults to ``0.0``.
   */
  double max_effort_;

  /*
   * A GripperCommand message has only a single float64 for the
   * "command", thus only a single joint angle can be sent -- however,
   * due to the complexity of making grippers look correct in a URDF,
   * they typically have >1 joints. The "command_joint" is the joint
   * whose position value will be sent in the GripperCommand action. A
   * set of names is provided for acceptable joint names. If any of
   * the joints specified is found, the value corresponding to that
   * joint is considered the command.
   */
  std::set<std::string> command_joints_;
};

}  // end namespace moveit_simple_controller_manager
