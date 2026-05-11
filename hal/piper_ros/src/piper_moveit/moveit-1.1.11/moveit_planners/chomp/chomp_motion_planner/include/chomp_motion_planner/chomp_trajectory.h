/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/* Author: Mrinal Kalakrishnan */

#pragma once

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <chomp_motion_planner/chomp_utils.h>

#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <vector>
#include <eigen3/Eigen/Core>

namespace chomp
{
/**
 * \brief Represents a discretized joint-space trajectory for CHOMP
 */
class ChompTrajectory
{
public:
  /**
   * \brief Constructs a trajectory for a given robot model, trajectory duration, and discretization
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, double duration, double discretization,
                  const std::string& group_name);

  /**
   * \brief Constructs a trajectory for a given robot model, number of trajectory points, and discretization
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, size_t num_points, double discretization,
                  const std::string& group_name);

  /**
   * \brief Creates a new containing only the joints of interest, and adds padding to the start
   * and end if needed, to have enough trajectory points for the differentiation rules
   */
  ChompTrajectory(const ChompTrajectory& source_traj, const std::string& group_name, int diff_rule_length);

  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                  const trajectory_msgs::JointTrajectory& traj);

  /**
   * \brief Destructor
   */
  virtual ~ChompTrajectory() = default;

  double& operator()(size_t traj_point, size_t joint);

  double operator()(size_t traj_point, size_t joint) const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);

  /**
   * \brief Gets the number of points in the trajectory
   */
  size_t getNumPoints() const;

  /**
   * \brief Gets the number of points (that are free to be optimized) in the trajectory
   */
  size_t getNumFreePoints() const;

  /**
   * \brief Gets the number of joints in each trajectory point
   */
  size_t getNumJoints() const;

  /**
   * \brief Gets the discretization time interval of the trajectory
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive.
   */
  void fillInMinJerk();

  /**
   * \brief Generates a linearly interpolated trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive
   */
  void fillInLinearInterpolation();

  /**
   * \brief Generates a cubic interpolation of the trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive
   */
  void fillInCubicInterpolation();

  /**
   * \brief Receives the path obtained from a given MotionPlanDetailedResponse res object's trajectory (e.g., trajectory
   * produced by OMPL) and puts it into the appropriate trajectory format required for CHOMP
   * @param res
   */
  bool fillInFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory);

  /**
   * \brief This function assigns the given \a source RobotState to the row at index \a chomp_trajectory_point
   *
   * @param source The source RobotState
   * @param chomp_trajectory_point index of the chomp_trajectory's point (row)
   * @param group  JointModelGroup determining the joints to copy
   */
  void assignCHOMPTrajectoryPointFromRobotState(const moveit::core::RobotState& source, size_t chomp_trajectory_point,
                                                const moveit::core::JointModelGroup* group);

  /**
   * \brief Sets the start and end index for the modifiable part of the trajectory
   *
   * (Everything before the start and after the end index is considered fixed)
   * The values default to 1 and getNumPoints()-2
   */
  void setStartEndIndex(size_t start_index, size_t end_index);

  /**
   * \brief Gets the start index
   */
  size_t getStartIndex() const;

  /**
   * \brief Gets the end index
   */
  size_t getEndIndex() const;

  /**
   * \brief Gets the entire trajectory matrix
   */
  Eigen::MatrixXd& getTrajectory();

  /**
   * \brief Gets the block of the trajectory which can be optimized
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();

  /**
   * \brief Gets the block of free (optimizable) trajectory for a single joint
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(size_t joint);

  /**
   * \brief Updates the full trajectory (*this) from the group trajectory
   */
  void updateFromGroupTrajectory(const ChompTrajectory& group_trajectory);

  /**
   * \brief Gets the index in the full trajectory which was copied to this group trajectory
   */
  size_t getFullTrajectoryIndex(size_t i) const;

  /**
   * \brief Gets the joint velocities at the given trajectory point
   */
  template <typename Derived>
  void getJointVelocities(size_t traj_point, Eigen::MatrixBase<Derived>& velocities);

  double getDuration() const;

private:
  void init(); /**< \brief Allocates memory for the trajectory */

  std::string planning_group_name_;  //< Planning group that this trajectory corresponds to, if any
  size_t num_points_;                //< Number of points in the trajectory
  size_t num_joints_;                //< Number of joints in each trajectory point
  double discretization_;            //< Discretization of the trajectory
  double duration_;                  //< Duration of the trajectory
  Eigen::MatrixXd trajectory_;       //< Storage for the actual trajectory
  size_t start_index_;  // Start index (inclusive) of trajectory to be optimized (everything before will be ignored)
  size_t end_index_;    //< End index (inclusive) of trajectory to be optimized (everything after will be ignored)
  std::vector<size_t> full_trajectory_index_;  //< If this is a "group" trajectory, the indeces from the original traj
};

///////////////////////// inline functions follow //////////////////////

inline double& ChompTrajectory::operator()(size_t traj_point, size_t joint)
{
  return trajectory_(traj_point, joint);
}

inline double ChompTrajectory::operator()(size_t traj_point, size_t joint) const
{
  return trajectory_(traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr ChompTrajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::ColXpr ChompTrajectory::getJointTrajectory(int joint)
{
  return trajectory_.col(joint);
}

inline size_t ChompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline size_t ChompTrajectory::getNumFreePoints() const
{
  return (end_index_ - start_index_) + 1;
}

inline size_t ChompTrajectory::getNumJoints() const
{
  return num_joints_;
}

inline double ChompTrajectory::getDiscretization() const
{
  return discretization_;
}

inline void ChompTrajectory::setStartEndIndex(size_t start_index, size_t end_index)
{
  start_index_ = start_index;
  end_index_ = end_index;
}

inline size_t ChompTrajectory::getStartIndex() const
{
  return start_index_;
}

inline size_t ChompTrajectory::getEndIndex() const
{
  return end_index_;
}

inline Eigen::MatrixXd& ChompTrajectory::getTrajectory()
{
  return trajectory_;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ChompTrajectory::getFreeTrajectoryBlock()
{
  return trajectory_.block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
ChompTrajectory::getFreeJointTrajectoryBlock(size_t joint)
{
  return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline size_t ChompTrajectory::getFullTrajectoryIndex(size_t i) const
{
  return full_trajectory_index_[i];
}

template <typename Derived>
void ChompTrajectory::getJointVelocities(size_t traj_point, Eigen::MatrixBase<Derived>& velocities)
{
  velocities.setZero();
  double inv_time = 1.0 / discretization_;

  for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
  {
    velocities += (inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * trajectory_.row(traj_point + k).transpose();
  }
}

inline double ChompTrajectory::getDuration() const
{
  return duration_;
}
}  // namespace chomp
