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

#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <moveit/utils/xmlrpc_casts.h>

using namespace moveit::core;
static const std::string LOGNAME("SimpleControllerManager");

namespace moveit_simple_controller_manager
{
bool FollowJointTrajectoryControllerHandle::sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "new trajectory to " << name_);

  if (!controller_action_client_)
    return false;

  if (!trajectory.multi_dof_joint_trajectory.points.empty())
  {
    ROS_WARN_NAMED(LOGNAME, "%s cannot execute multi-dof trajectories.", name_.c_str());
  }

  if (done_)
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "sending trajectory to " << name_);
  else
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "sending continuation for the currently executed trajectory to " << name_);

  control_msgs::FollowJointTrajectoryGoal goal = goal_template_;
  goal.trajectory = trajectory.joint_trajectory;
  controller_action_client_->sendGoal(
      goal, [this](const auto& state, const auto& result) { controllerDoneCallback(state, result); },
      [this] { controllerActiveCallback(); }, [this](const auto& feedback) { controllerFeedbackCallback(feedback); });
  done_ = false;
  last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
  return true;
}

void FollowJointTrajectoryControllerHandle::configure(XmlRpc::XmlRpcValue& config)
{
  if (config.hasMember("path_tolerance"))
    configure(config["path_tolerance"], "path_tolerance", goal_template_.path_tolerance);
  if (config.hasMember("goal_tolerance"))
    configure(config["goal_tolerance"], "goal_tolerance", goal_template_.goal_tolerance);
  if (config.hasMember("goal_time_tolerance"))
    goal_template_.goal_time_tolerance = ros::Duration(parseDouble(config["goal_time_tolerance"]));
}

namespace
{
enum ToleranceVariables
{
  POSITION,
  VELOCITY,
  ACCELERATION
};
template <ToleranceVariables>
double& variable(control_msgs::JointTolerance& msg);

template <>
inline double& variable<POSITION>(control_msgs::JointTolerance& msg)
{
  return msg.position;
}
template <>
inline double& variable<VELOCITY>(control_msgs::JointTolerance& msg)
{
  return msg.velocity;
}
template <>
inline double& variable<ACCELERATION>(control_msgs::JointTolerance& msg)
{
  return msg.acceleration;
}

static std::map<ToleranceVariables, std::string> VAR_NAME = { { POSITION, "position" },
                                                              { VELOCITY, "velocity" },
                                                              { ACCELERATION, "acceleration" } };
static std::map<ToleranceVariables, decltype(&variable<POSITION>)> VAR_ACCESS = { { POSITION, &variable<POSITION> },
                                                                                  { VELOCITY, &variable<VELOCITY> },
                                                                                  { ACCELERATION,
                                                                                    &variable<ACCELERATION> } };

const char* errorCodeToMessage(int error_code)
{
  switch (error_code)
  {
    case control_msgs::FollowJointTrajectoryResult::SUCCESSFUL:
      return "SUCCESSFUL";
    case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL:
      return "INVALID_GOAL";
    case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP:
      return "OLD_HEADER_TIMESTAMP";
    case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
      return "PATH_TOLERANCE_VIOLATED";
    case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
      return "GOAL_TOLERANCE_VIOLATED";
    default:
      return "unknown error";
  }
}
}  // namespace

void FollowJointTrajectoryControllerHandle::configure(XmlRpc::XmlRpcValue& config, const std::string& config_name,
                                                      std::vector<control_msgs::JointTolerance>& tolerances)
{
  if (isStruct(config))  // config should be either a struct of position, velocity, acceleration
  {
    for (ToleranceVariables var : { POSITION, VELOCITY, ACCELERATION })
    {
      if (!config.hasMember(VAR_NAME[var]))
        continue;
      XmlRpc::XmlRpcValue values = config[VAR_NAME[var]];
      if (isArray(values, joints_.size()))
      {
        size_t i = 0;
        for (const auto& joint_name : joints_)
          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = parseDouble(values[i++]);
      }
      else
      {  // use common value for all joints
        double value = parseDouble(values);
        for (const auto& joint_name : joints_)
          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = value;
      }
    }
  }
  else if (isArray(config))  // or an array of JointTolerance msgs
  {
    for (int i = 0; i < config.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      control_msgs::JointTolerance& tol = getTolerance(tolerances, config[i]["name"]);
      for (ToleranceVariables var : { POSITION, VELOCITY, ACCELERATION })
      {
        if (!config[i].hasMember(VAR_NAME[var]))
          continue;
        VAR_ACCESS[var](tol) = parseDouble(config[i][VAR_NAME[var]]);
      }
    }
  }
  else
    ROS_WARN_STREAM_NAMED(LOGNAME, "Invalid " << config_name);
}

control_msgs::JointTolerance&
FollowJointTrajectoryControllerHandle::getTolerance(std::vector<control_msgs::JointTolerance>& tolerances,
                                                    const std::string& name)
{
  auto it =
      std::lower_bound(tolerances.begin(), tolerances.end(), name,
                       [](const control_msgs::JointTolerance& lhs, const std::string& rhs) { return lhs.name < rhs; });
  if (it == tolerances.cend() || it->name != name)
  {  // insert new entry if not yet available
    it = tolerances.insert(it, control_msgs::JointTolerance());
    it->name = name;
  }
  return *it;
}

void FollowJointTrajectoryControllerHandle::controllerDoneCallback(
    const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  // Output custom error message for FollowJointTrajectoryResult if necessary
  if (!result)
    ROS_WARN_STREAM_NAMED(LOGNAME, "Controller '" << name_ << "' done, no result returned");
  else if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
    ROS_INFO_STREAM_NAMED(LOGNAME, "Controller '" << name_ << "' successfully finished");
  else
    ROS_WARN_STREAM_NAMED(LOGNAME, "Controller '" << name_ << "' failed with error "
                                                  << errorCodeToMessage(result->error_code) << ": "
                                                  << result->error_string);
  finishControllerExecution(state);
}

void FollowJointTrajectoryControllerHandle::controllerActiveCallback()
{
  ROS_DEBUG_STREAM_NAMED(LOGNAME, name_ << " started execution");
}

void FollowJointTrajectoryControllerHandle::controllerFeedbackCallback(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr& /* feedback */)
{
}

}  // end namespace moveit_simple_controller_manager
