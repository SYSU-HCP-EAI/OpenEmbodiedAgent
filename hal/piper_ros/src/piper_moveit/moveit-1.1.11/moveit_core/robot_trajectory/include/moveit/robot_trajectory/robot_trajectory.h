/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Adam Leeper */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <deque>
#include <memory>

namespace robot_trajectory
{
MOVEIT_CLASS_FORWARD(RobotTrajectory);  // Defines RobotTrajectoryPtr, ConstPtr, WeakPtr... etc

/** \brief Maintain a sequence of waypoints and the time durations
    between these waypoints */
class RobotTrajectory
{
public:
  /** @brief construct a trajectory for the whole robot */
  explicit RobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model);

  /** @brief construct a trajectory for the named JointModelGroup
   * If group is an empty string, this is equivalent to the first constructor,
   * otherwise it is equivalent to `RobotTrajectory(robot_model, robot_model->getJointModelGroup(group))`.
   */
  RobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group);

  /** @brief construct a trajectory for the JointModelGroup
   *  Only joints from the specified group will be considered in this trajectory,
   *  even though all waypoints still consist of full RobotStates (representing all joints).
   *
   *  If group is nullptr this is equivalent to the first constructor.
   */
  RobotTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const moveit::core::JointModelGroup* group);

  /** Assignment operator, performing a shallow copy, i.e. copying waypoints by pointer */
  RobotTrajectory& operator=(const RobotTrajectory&) = default;

  /** @brief  Copy constructor allowing a shallow or deep copy of waypoints
   *  @param  other - RobotTrajectory to copy from
   *  @param  deepcopy - copy waypoints by value (true) or by pointer (false)?
   */
  RobotTrajectory(const RobotTrajectory& other, bool deepcopy = false);

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  const moveit::core::JointModelGroup* getGroup() const
  {
    return group_;
  }

  const std::string& getGroupName() const;

  RobotTrajectory& setGroupName(const std::string& group_name)
  {
    group_ = robot_model_->getJointModelGroup(group_name);
    return *this;
  }

  std::size_t getWayPointCount() const
  {
    return waypoints_.size();
  }

  const moveit::core::RobotState& getWayPoint(std::size_t index) const
  {
    return *waypoints_[index];
  }

  const moveit::core::RobotState& getLastWayPoint() const
  {
    return *waypoints_.back();
  }

  const moveit::core::RobotState& getFirstWayPoint() const
  {
    return *waypoints_.front();
  }

  moveit::core::RobotStatePtr& getWayPointPtr(std::size_t index)
  {
    return waypoints_[index];
  }

  moveit::core::RobotStatePtr& getLastWayPointPtr()
  {
    return waypoints_.back();
  }

  moveit::core::RobotStatePtr& getFirstWayPointPtr()
  {
    return waypoints_.front();
  }

  const std::deque<double>& getWayPointDurations() const
  {
    return duration_from_previous_;
  }

  /** @brief  Returns the duration after start that a waypoint will be reached.
   *  @param  The waypoint index.
   *  @return The duration from start; returns overall duration if index is out of range.
   */
  double getWayPointDurationFromStart(std::size_t index) const;

  [[deprecated]] double getWaypointDurationFromStart(std::size_t index) const;

  double getWayPointDurationFromPrevious(std::size_t index) const
  {
    if (duration_from_previous_.size() > index)
      return duration_from_previous_[index];
    else
      return 0.0;
  }

  RobotTrajectory& setWayPointDurationFromPrevious(std::size_t index, double value)
  {
    if (duration_from_previous_.size() <= index)
      duration_from_previous_.resize(index + 1, 0.0);
    duration_from_previous_[index] = value;
    return *this;
  }

  bool empty() const
  {
    return waypoints_.empty();
  }

  /**
   * \brief Add a point to the trajectory
   * \param state - current robot state
   * \param dt - duration from previous
   */
  RobotTrajectory& addSuffixWayPoint(const moveit::core::RobotState& state, double dt)
  {
    return addSuffixWayPoint(std::make_shared<moveit::core::RobotState>(state), dt);
  }

  /**
   * \brief Add a point to the trajectory
   * \param state - current robot state
   * \param dt - duration from previous
   */
  RobotTrajectory& addSuffixWayPoint(const moveit::core::RobotStatePtr& state, double dt)
  {
    state->update();
    waypoints_.push_back(state);
    duration_from_previous_.push_back(dt);
    return *this;
  }

  RobotTrajectory& addPrefixWayPoint(const moveit::core::RobotState& state, double dt)
  {
    return addPrefixWayPoint(std::make_shared<moveit::core::RobotState>(state), dt);
  }

  RobotTrajectory& addPrefixWayPoint(const moveit::core::RobotStatePtr& state, double dt)
  {
    state->update();
    waypoints_.push_front(state);
    duration_from_previous_.push_front(dt);
    return *this;
  }

  RobotTrajectory& insertWayPoint(std::size_t index, const moveit::core::RobotState& state, double dt)
  {
    return insertWayPoint(index, std::make_shared<moveit::core::RobotState>(state), dt);
  }

  RobotTrajectory& insertWayPoint(std::size_t index, const moveit::core::RobotStatePtr& state, double dt)
  {
    state->update();
    waypoints_.insert(waypoints_.begin() + index, state);
    duration_from_previous_.insert(duration_from_previous_.begin() + index, dt);
    return *this;
  }

  /**
   * \brief Add a specified part of a trajectory to the end of the current trajectory. The default (when \p start_index
   * and \p end_index are omitted) is to add the whole trajectory.
   * \param source - the trajectory containing the part to append to the end of current trajectory
   * \param dt - time step between last traj point in current traj and first traj point of append traj
   * \param start_index - index of first traj point of the part to append from the source traj, the default is to add
   * from the start of the source traj
   * \param end_index - index of last traj point of the part to append from the source traj, the default is to add until
   * the end of the source traj
   */
  RobotTrajectory& append(const RobotTrajectory& source, double dt, size_t start_index = 0,
                          size_t end_index = std::numeric_limits<std::size_t>::max());

  void swap(robot_trajectory::RobotTrajectory& other);

  RobotTrajectory& clear()
  {
    waypoints_.clear();
    duration_from_previous_.clear();
    return *this;
  }

  double getDuration() const;

  double getAverageSegmentDuration() const;

  void getRobotTrajectoryMsg(moveit_msgs::RobotTrajectory& trajectory,
                             const std::vector<std::string>& joint_filter = std::vector<std::string>()) const;

  /** \brief Copy the content of the trajectory message into this class. The trajectory message itself is not required
     to contain the values
      for all joints. For this reason a full starting state must be specified as reference (\e reference_state). Each
     point in the trajectory
      to be constructed internally is obtained by copying the reference state and overwriting the content from a
     trajectory point in \e trajectory. */
  RobotTrajectory& setRobotTrajectoryMsg(const moveit::core::RobotState& reference_state,
                                         const trajectory_msgs::JointTrajectory& trajectory);

  /** \brief Copy the content of the trajectory message into this class. The trajectory message itself is not required
     to contain the values
      for all joints. For this reason a full starting state must be specified as reference (\e reference_state). Each
     point in the trajectory
      to be constructed internally is obtained by copying the reference state and overwriting the content from a
     trajectory point in \e trajectory. */
  RobotTrajectory& setRobotTrajectoryMsg(const moveit::core::RobotState& reference_state,
                                         const moveit_msgs::RobotTrajectory& trajectory);

  /** \brief Copy the content of the trajectory message into this class. The trajectory message itself is not required
     to contain the values
      for all joints. For this reason a full starting state must be specified as reference (\e reference_state). Before
     use, the reference state is updated
      using \e state. Each point in the trajectory  to be constructed internally is obtained by copying the reference
     state and overwriting the content
      from a trajectory point in \e trajectory. */
  RobotTrajectory& setRobotTrajectoryMsg(const moveit::core::RobotState& reference_state,
                                         const moveit_msgs::RobotState& state,
                                         const moveit_msgs::RobotTrajectory& trajectory);

  RobotTrajectory& reverse();

  RobotTrajectory& unwind();
  RobotTrajectory& unwind(const moveit::core::RobotState& state);

  /** @brief Finds the waypoint indicies before and after a duration from start.
   *  @param The duration from start.
   *  @param The waypoint index before the supplied duration.
   *  @param The waypoint index after (or equal to) the supplied duration.
   *  @param The progress (0 to 1) between the two waypoints, based on time (not based on joint distances).
   */
  void findWayPointIndicesForDurationAfterStart(const double& duration, int& before, int& after, double& blend) const;

  // TODO support visitor function for interpolation, or at least different types.
  /** @brief Gets a robot state corresponding to a supplied duration from start for the trajectory, using linear time
   * interpolation.
   *  @param The duration from start.
   *  @param The resulting robot state.
   *  @return True if state is valid, false otherwise (trajectory is empty).
   */
  bool getStateAtDurationFromStart(const double request_duration, moveit::core::RobotStatePtr& output_state) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* group_;
  std::deque<moveit::core::RobotStatePtr> waypoints_;
  std::deque<double> duration_from_previous_;
};

/// \brief Calculate the path length of a given trajectory based on the
/// accumulated robot state distances.
/// The distance between two robot states is calculated based on the sum of
/// active joint distances between the two states (L1 norm).
/// \param[in] trajectory Given robot trajectory
/// \return Length of the robot trajectory [rad]
[[nodiscard]] double path_length(RobotTrajectory const& trajectory);
}  // namespace robot_trajectory
