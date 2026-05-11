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

#include <moveit/planning_scene_monitor/current_state_monitor.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>

constexpr char LOGNAME[] = "current_state_monitor";

namespace planning_scene_monitor
{
CurrentStateMonitor::CurrentStateMonitor(const moveit::core::RobotModelConstPtr& robot_model,
                                         const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : CurrentStateMonitor(robot_model, tf_buffer, ros::NodeHandle())
{
}

CurrentStateMonitor::CurrentStateMonitor(const moveit::core::RobotModelConstPtr& robot_model,
                                         const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const ros::NodeHandle& nh)
  : nh_(nh)
  , tf_buffer_(tf_buffer)
  , robot_model_(robot_model)
  , robot_state_(robot_model)
  , state_monitor_started_(false)
  , copy_dynamics_(false)
  , error_(std::numeric_limits<double>::epsilon())
{
  robot_state_.setToDefaultValues();
}

CurrentStateMonitor::~CurrentStateMonitor()
{
  stopStateMonitor();
}

moveit::core::RobotStatePtr CurrentStateMonitor::getCurrentState() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  moveit::core::RobotState* result = new moveit::core::RobotState(robot_state_);
  return moveit::core::RobotStatePtr(result);
}

ros::Time CurrentStateMonitor::getCurrentStateTime() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  return current_state_time_;
}

std::pair<moveit::core::RobotStatePtr, ros::Time> CurrentStateMonitor::getCurrentStateAndTime() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  moveit::core::RobotState* result = new moveit::core::RobotState(robot_state_);
  return std::make_pair(moveit::core::RobotStatePtr(result), current_state_time_);
}

std::map<std::string, double> CurrentStateMonitor::getCurrentStateValues() const
{
  std::map<std::string, double> m;
  boost::mutex::scoped_lock slock(state_update_lock_);
  const double* pos = robot_state_.getVariablePositions();
  const std::vector<std::string>& names = robot_state_.getVariableNames();
  for (std::size_t i = 0; i < names.size(); ++i)
    m[names[i]] = pos[i];
  return m;
}

void CurrentStateMonitor::setToCurrentState(moveit::core::RobotState& upd) const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  const double* pos = robot_state_.getVariablePositions();
  upd.setVariablePositions(pos);
  if (copy_dynamics_)
  {
    if (robot_state_.hasVelocities())
    {
      const double* vel = robot_state_.getVariableVelocities();
      upd.setVariableVelocities(vel);
    }
    if (robot_state_.hasAccelerations())
    {
      const double* acc = robot_state_.getVariableAccelerations();
      upd.setVariableAccelerations(acc);
    }
    if (robot_state_.hasEffort())
    {
      const double* eff = robot_state_.getVariableEffort();
      upd.setVariableEffort(eff);
    }
  }
}

void CurrentStateMonitor::addUpdateCallback(const JointStateUpdateCallback& fn)
{
  if (fn)
    update_callbacks_.push_back(fn);
}

void CurrentStateMonitor::clearUpdateCallbacks()
{
  update_callbacks_.clear();
}

void CurrentStateMonitor::startStateMonitor(const std::string& joint_states_topic)
{
  if (!state_monitor_started_ && robot_model_)
  {
    joint_time_.clear();
    if (joint_states_topic.empty())
      ROS_ERROR_NAMED(LOGNAME, "The joint states topic cannot be an empty string");
    else
      joint_state_subscriber_ = nh_.subscribe(joint_states_topic, 25, &CurrentStateMonitor::jointStateCallback, this);
    if (tf_buffer_ && !robot_model_->getMultiDOFJointModels().empty())
    {
      tf_connection_ =
          std::make_shared<TFConnection>(tf_buffer_->_addTransformsChangedListener([this] { tfCallback(); }));
    }
    state_monitor_started_ = true;
    monitor_start_time_ = ros::Time::now();
    ROS_DEBUG_NAMED(LOGNAME, "Listening to joint states on topic '%s'", nh_.resolveName(joint_states_topic).c_str());
  }
}

bool CurrentStateMonitor::isActive() const
{
  return state_monitor_started_;
}

void CurrentStateMonitor::stopStateMonitor()
{
  if (state_monitor_started_)
  {
    joint_state_subscriber_.shutdown();
    if (tf_buffer_ && tf_connection_)
    {
      tf_buffer_->_removeTransformsChangedListener(*tf_connection_);
      tf_connection_.reset();
    }
    ROS_DEBUG_NAMED(LOGNAME, "No longer listening for joint states");
    state_monitor_started_ = false;
  }
}

std::string CurrentStateMonitor::getMonitoredTopic() const
{
  if (joint_state_subscriber_)
    return joint_state_subscriber_.getTopic();
  else
    return "";
}

bool CurrentStateMonitor::haveCompleteStateHelper(const ros::Time& oldest_allowed_update_time,
                                                  std::vector<std::string>* missing_joints) const
{
  const std::vector<const moveit::core::JointModel*>& active_joints = robot_model_->getActiveJointModels();
  boost::mutex::scoped_lock slock(state_update_lock_);
  for (const moveit::core::JointModel* joint : active_joints)
  {
    std::map<const moveit::core::JointModel*, ros::Time>::const_iterator it = joint_time_.find(joint);
    if (it == joint_time_.end())
    {
      ROS_DEBUG_NAMED(LOGNAME, "Joint '%s' has never been updated", joint->getName().c_str());
    }
    else if (it->second < oldest_allowed_update_time)
    {
      ROS_DEBUG_NAMED(LOGNAME, "Joint '%s' was last updated %0.3lf seconds before requested time",
                      joint->getName().c_str(), (oldest_allowed_update_time - it->second).toSec());
    }
    else
      continue;

    if (missing_joints)
      missing_joints->push_back(joint->getName());
    else
      return false;
  }
  return (missing_joints == nullptr) || missing_joints->empty();
}

bool CurrentStateMonitor::waitForCurrentState(const ros::Time t, double wait_time) const
{
  ros::WallTime start = ros::WallTime::now();
  ros::WallDuration elapsed(0, 0);
  ros::WallDuration timeout(wait_time);

  boost::mutex::scoped_lock lock(state_update_lock_);
  while (current_state_time_ < t)
  {
    state_update_condition_.wait_for(lock, boost::chrono::nanoseconds((timeout - elapsed).toNSec()));
    elapsed = ros::WallTime::now() - start;
    if (elapsed > timeout)
    {
      ROS_INFO_STREAM_NAMED(LOGNAME,
                            "Didn't receive robot state (joint angles) with recent timestamp within "
                                << wait_time << " seconds.\n"
                                << "Check clock synchronization if your are running ROS across multiple machines!");
      return false;
    }
  }
  return true;
}

bool CurrentStateMonitor::waitForCompleteState(double wait_time) const
{
  double slept_time = 0.0;
  double sleep_step_s = std::min(0.05, wait_time / 10.0);
  ros::Duration sleep_step(sleep_step_s);
  while (!haveCompleteState() && slept_time < wait_time)
  {
    sleep_step.sleep();
    slept_time += sleep_step_s;
  }
  return haveCompleteState();
}

bool CurrentStateMonitor::waitForCompleteState(const std::string& group, double wait_time) const
{
  if (waitForCompleteState(wait_time))
    return true;
  bool ok = true;

  // check to see if we have a fully known state for the joints we want to record
  std::vector<std::string> missing_joints;
  if (!haveCompleteState(missing_joints))
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      std::set<std::string> mj;
      mj.insert(missing_joints.begin(), missing_joints.end());
      const std::vector<std::string>& names = jmg->getJointModelNames();
      bool ok = true;
      for (std::size_t i = 0; ok && i < names.size(); ++i)
        if (mj.find(names[i]) != mj.end())
          ok = false;
    }
    else
      ok = false;
  }
  return ok;
}

void CurrentStateMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if (joint_state->name.size() != joint_state->position.size())
  {
    ROS_ERROR_THROTTLE_NAMED(
        1, LOGNAME,
        "State monitor received invalid joint state (number of joint names does not match number of "
        "positions)");
    return;
  }
  bool update = false;

  {
    boost::mutex::scoped_lock _(state_update_lock_);
    // read the received values, and update their time stamps
    std::size_t n = joint_state->name.size();
    current_state_time_ = joint_state->header.stamp;
    for (std::size_t i = 0; i < n; ++i)
    {
      const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_state->name[i]);
      if (!jm)
        continue;
      // ignore fixed joints, multi-dof joints (they should not even be in the message)
      if (jm->getVariableCount() != 1)
        continue;

      joint_time_[jm] = joint_state->header.stamp;

      if (robot_state_.getJointPositions(jm)[0] != joint_state->position[i])
      {
        update = true;
        robot_state_.setJointPositions(jm, &(joint_state->position[i]));

        // continuous joints wrap, so we don't modify them (even if they are outside bounds!)
        if (jm->getType() == moveit::core::JointModel::REVOLUTE)
          if (static_cast<const moveit::core::RevoluteJointModel*>(jm)->isContinuous())
            continue;

        const moveit::core::VariableBounds& b =
            jm->getVariableBounds()[0];  // only one variable in the joint, so we get its bounds

        // if the read variable is 'almost' within bounds (up to error_ difference), then consider it to be within
        // bounds
        if (joint_state->position[i] < b.min_position_ && joint_state->position[i] >= b.min_position_ - error_)
          robot_state_.setJointPositions(jm, &b.min_position_);
        else if (joint_state->position[i] > b.max_position_ && joint_state->position[i] <= b.max_position_ + error_)
          robot_state_.setJointPositions(jm, &b.max_position_);
      }

      // optionally copy velocities and effort
      if (copy_dynamics_)
      {
        // update joint velocities
        if (joint_state->name.size() == joint_state->velocity.size() &&
            (!robot_state_.hasVelocities() || robot_state_.getJointVelocities(jm)[0] != joint_state->velocity[i]))
        {
          update = true;
          robot_state_.setJointVelocities(jm, &(joint_state->velocity[i]));
        }

        // update joint efforts
        if (joint_state->name.size() == joint_state->effort.size() &&
            (!robot_state_.hasEffort() || robot_state_.getJointEffort(jm)[0] != joint_state->effort[i]))
        {
          update = true;
          robot_state_.setJointEfforts(jm, &(joint_state->effort[i]));
        }
      }
    }
  }

  // callbacks, if needed
  if (update)
    for (JointStateUpdateCallback& update_callback : update_callbacks_)
      update_callback(joint_state);

  // notify waitForCurrentState() *after* potential update callbacks
  state_update_condition_.notify_all();
}

void CurrentStateMonitor::tfCallback()
{
  // read multi-dof joint states from TF, if needed
  const std::vector<const moveit::core::JointModel*>& multi_dof_joints = robot_model_->getMultiDOFJointModels();

  bool update = false;
  bool changes = false;
  {
    boost::mutex::scoped_lock _(state_update_lock_);

    for (const moveit::core::JointModel* joint : multi_dof_joints)
    {
      const std::string& child_frame = joint->getChildLinkModel()->getName();
      const std::string& parent_frame =
          joint->getParentLinkModel() ? joint->getParentLinkModel()->getName() : robot_model_->getModelFrame();

      ros::Time latest_common_time;
      geometry_msgs::TransformStamped transf;
      try
      {
        transf = tf_buffer_->lookupTransform(parent_frame, child_frame, ros::Time(0.0));
        latest_common_time = transf.header.stamp;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN_STREAM_ONCE_NAMED(LOGNAME, "Unable to update multi-DOF joint '"
                                                << joint->getName() << "': Failure to lookup transform between '"
                                                << parent_frame.c_str() << "' and '" << child_frame.c_str()
                                                << "' with TF exception: " << ex.what());
        continue;
      }

      // allow update if time is more recent or if it is a static transform (time = 0)
      if (latest_common_time <= joint_time_[joint] && latest_common_time > ros::Time(0))
        continue;
      joint_time_[joint] = latest_common_time;

      std::vector<double> new_values(joint->getStateSpaceDimension());
      const moveit::core::LinkModel* link = joint->getChildLinkModel();
      if (link->jointOriginTransformIsIdentity())
        joint->computeVariablePositions(tf2::transformToEigen(transf), new_values.data());
      else
        joint->computeVariablePositions(link->getJointOriginTransform().inverse() * tf2::transformToEigen(transf),
                                        new_values.data());

      if (joint->distance(new_values.data(), robot_state_.getJointPositions(joint)) > 1e-5)
      {
        changes = true;
      }

      robot_state_.setJointPositions(joint, new_values.data());
      update = true;
    }
  }

  // callbacks, if needed
  if (changes)
  {
    // stub joint state: multi-dof joints are not modelled in the message,
    // but we should still trigger the update callbacks
    sensor_msgs::JointStatePtr joint_state(new sensor_msgs::JointState);
    for (JointStateUpdateCallback& update_callback : update_callbacks_)
      update_callback(joint_state);
  }

  if (update)
  {
    // notify waitForCurrentState() *after* potential update callbacks
    state_update_condition_.notify_all();
  }
}

}  // namespace planning_scene_monitor
