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

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace planning_scene_monitor
{
using JointStateUpdateCallback = boost::function<void(const sensor_msgs::JointStateConstPtr&)>;

/** @class CurrentStateMonitor
    @brief Monitors the joint_states topic and tf to maintain the current state of the robot. */
class CurrentStateMonitor
{
  using TFConnection = boost::signals2::connection;

public:
  /**
   * @brief Constructor.
   * @param robot_model The current kinematic model to build on
   * @param tf_buffer A pointer to the tf2_ros Buffer to use
   */
  CurrentStateMonitor(const moveit::core::RobotModelConstPtr& robot_model,
                      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer);

  /** @brief Constructor.
   *  @param robot_model The current kinematic model to build on
   *  @param tf_buffer A pointer to the tf2_ros Buffer to use
   *  @param nh A ros::NodeHandle to pass node specific options
   */
  CurrentStateMonitor(const moveit::core::RobotModelConstPtr& robot_model,
                      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const ros::NodeHandle& nh);

  ~CurrentStateMonitor();

  /** @brief Start monitoring joint states on a particular topic
   *  @param joint_states_topic The topic name for joint states (defaults to "joint_states")
   */
  void startStateMonitor(const std::string& joint_states_topic = "joint_states");

  /** @brief Stop monitoring the "joint_states" topic
   */
  void stopStateMonitor();

  /** @brief Check if the state monitor is started */
  bool isActive() const;

  /** @brief Get the RobotModel for which we are monitoring state */
  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** @brief Get the name of the topic being monitored. Returns an empty string if the monitor is inactive. */
  std::string getMonitoredTopic() const;

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @return False if we have no joint state information for one or more of the joints
   */
  inline bool haveCompleteState() const
  {
    return haveCompleteStateHelper(ros::Time(0), nullptr);
  }

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @param oldest_allowed_update_time All joint information must be from this time or more current
   *  @return False if we have no joint state information for one of the joints or if our state
   *  information is more than \e age old*/
  inline bool haveCompleteState(const ros::Time& oldest_allowed_update_time) const
  {
    return haveCompleteStateHelper(oldest_allowed_update_time, nullptr);
  }

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @param age Joint information must be at most this old
   *  @return False if we have no joint state information for one of the joints or if our state
   *  information is more than \e age old
   */
  inline bool haveCompleteState(const ros::Duration& age) const
  {
    return haveCompleteStateHelper(ros::Time::now() - age, nullptr);
  }

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @param missing_joints Returns the list of joints that are missing
   *  @return False if we have no joint state information for one or more of the joints
   */
  inline bool haveCompleteState(std::vector<std::string>& missing_joints) const
  {
    return haveCompleteStateHelper(ros::Time(0), &missing_joints);
  }

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @param oldest_allowed_update_time All joint information must be from this time or more current
   *  @param missing_joints Returns the list of joints that are missing
   *  @return False if we have no joint state information for one of the joints or if our state
   *  information is more than \e age old*/
  inline bool haveCompleteState(const ros::Time& oldest_allowed_update_time,
                                std::vector<std::string>& missing_joints) const
  {
    return haveCompleteStateHelper(oldest_allowed_update_time, &missing_joints);
  }

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @return False if we have no joint state information for one of the joints or if our state
   *  information is more than \e age old
   */
  inline bool haveCompleteState(const ros::Duration& age, std::vector<std::string>& missing_joints) const
  {
    return haveCompleteStateHelper(ros::Time::now() - age, &missing_joints);
  }

  /** @brief Get the current state
   *  @return Returns the current state */
  moveit::core::RobotStatePtr getCurrentState() const;

  /** @brief Set the state \e upd to the current state maintained by this class. */
  void setToCurrentState(moveit::core::RobotState& upd) const;

  /** @brief Get the time stamp for the current state */
  ros::Time getCurrentStateTime() const;

  /** @brief Get the current state and its time stamp
   *  @return Returns a pair of the current state and its time stamp */
  std::pair<moveit::core::RobotStatePtr, ros::Time> getCurrentStateAndTime() const;

  /** @brief Get the current state values as a map from joint names to joint state values
   *  @return Returns the map from joint names to joint state values*/
  std::map<std::string, double> getCurrentStateValues() const;

  /** @brief Wait for at most \e wait_time seconds (default 1s) for a robot state more recent than t
   *  @return true on success, false if up-to-date robot state wasn't received within \e wait_time
   */
  bool waitForCurrentState(const ros::Time t = ros::Time::now(), double wait_time = 1.0) const;

  /** @brief Wait for at most \e wait_time seconds until the complete robot state is known.
      @return true if the full state is known */
  bool waitForCompleteState(double wait_time) const;

  /** @brief Wait for at most \e wait_time seconds until the joint values from the group \e group are known. Return true
   * if values for all joints in \e group are known */
  bool waitForCompleteState(const std::string& group, double wait_time) const;

  /** @brief Get the time point when the monitor was started */
  const ros::Time& getMonitorStartTime() const
  {
    return monitor_start_time_;
  }

  /** @brief Add a function that will be called whenever the joint state is updated*/
  void addUpdateCallback(const JointStateUpdateCallback& fn);

  /** @brief Clear the functions to be called when an update to the joint state is received */
  void clearUpdateCallbacks();

  /** @brief When a joint value is received to be out of bounds, it is changed slightly to fit within bounds,
   *  if the difference is less than a specified value (labeled the "allowed bounds error").
   *  This value can be set using this function.
   *  @param error The specified value for the "allowed bounds error". The default is machine precision. */
  void setBoundsError(double error)
  {
    error_ = (error > 0) ? error : -error;
  }

  /** @brief When a joint value is received to be out of bounds, it is changed slightly to fit within bounds,
   *  if the difference is less than a specified value (labeled the "allowed bounds error").
   *  @return The stored value for the "allowed bounds error"
   */
  double getBoundsError() const
  {
    return error_;
  }

  /** @brief Allow the joint_state arrrays velocity and effort to be copied into the robot state
   *  this is useful in some but not all applications
   */
  void enableCopyDynamics(bool enabled)
  {
    copy_dynamics_ = enabled;
  }

private:
  bool haveCompleteStateHelper(const ros::Time& oldest_allowed_update_time,
                               std::vector<std::string>* missing_joints) const;

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
  void tfCallback();

  ros::NodeHandle nh_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotState robot_state_;
  std::map<const moveit::core::JointModel*, ros::Time> joint_time_;
  bool state_monitor_started_;
  bool copy_dynamics_;  // Copy velocity and effort from joint_state
  ros::Time monitor_start_time_;
  double error_;
  ros::Subscriber joint_state_subscriber_;
  ros::Time current_state_time_;

  mutable boost::mutex state_update_lock_;
  mutable boost::condition_variable state_update_condition_;
  std::vector<JointStateUpdateCallback> update_callbacks_;

  std::shared_ptr<TFConnection> tf_connection_;
};

MOVEIT_CLASS_FORWARD(CurrentStateMonitor);  // Defines CurrentStateMonitorPtr, ConstPtr, WeakPtr... etc
}  // namespace planning_scene_monitor
