/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <memory>
#include <string>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_interface.h>

#include "pilz_industrial_motion_planner/trajectory_blend_request.h"
#include "pilz_industrial_motion_planner/trajectory_blender.h"
#include "pilz_industrial_motion_planner/trajectory_functions.h"
#include "pilz_industrial_motion_planner/trajectory_generation_exceptions.h"

namespace pilz_industrial_motion_planner
{
using TipFrameFunc_t = std::function<const std::string&(const std::string&)>;

// List of exceptions which can be thrown by the PlanComponentsBuilder class.
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoBlenderSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoTipFrameFunctionSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoRobotModelSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(BlendingFailedException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @brief Helper class to encapsulate the merge and blend process of
 * trajectories.
 */
class PlanComponentsBuilder
{
public:
  /**
   * @brief Sets the blender used to blend two trajectories.
   */
  void setBlender(std::unique_ptr<pilz_industrial_motion_planner::TrajectoryBlender> blender);

  /**
   * @brief Sets the robot model needed to create new trajectory elements.
   */
  void setModel(const moveit::core::RobotModelConstPtr& model);

  /**
   * @brief Appends the specified trajectory to the trajectory container
   * under construction.
   *
   * The appending complies to the following rules:
   * - A trajectory is simply added/attached to the previous trajectory:
   *      - if they are from the same group and
   *      - if the specified blend_radius is zero.
   * - A trajectory is blended together with the previous trajectory:
   *      - if they are from the same group and
   *      - if the specified blend_radius is GREATER than zero.
   * - A new trajectory element is created and the given trajectory is
   * appended/attached to the newly created empty trajectory:
   *      - if the given and previous trajectory are from different groups.
   *
   * @param planning_scene The scene planning is occurring in.
   *
   * @param other Trajectories which has to be added to the trajectory container
   * under construction.
   *
   * @param blend_radius The blending radius between the previous and the
   * specified trajectory.
   */
  void append(const planning_scene::PlanningSceneConstPtr& planning_scene,
              const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius);

  /**
   * @brief Clears the trajectory container under construction.
   */
  void reset();

  /**
   * @return The final trajectory container which results from the append calls.
   */
  std::vector<robot_trajectory::RobotTrajectoryPtr> build() const;

private:
  void blend(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius);

private:
  /**
   * @brief Appends a trajectory to a result trajectory leaving out the
   * first point, if it matches the last point of the result trajectory.
   *
   * @note Controllers, so far at least the
   * ros_controllers::JointTrajectoryController require a timewise strictly
   * increasing trajectory. If through appending the last point of the
   * original trajectory gets repeated, it is removed here.
   */
  static void appendWithStrictTimeIncrease(robot_trajectory::RobotTrajectory& result,
                                           const robot_trajectory::RobotTrajectory& source);

private:
  //! Blender used to blend two trajectories.
  std::unique_ptr<pilz_industrial_motion_planner::TrajectoryBlender> blender_;

  //! Robot model needed to create new trajectory container elements.
  moveit::core::RobotModelConstPtr model_;

  //! The previously added trajectory.
  robot_trajectory::RobotTrajectoryPtr traj_tail_;

  //! The trajectory container under construction.
  std::vector<robot_trajectory::RobotTrajectoryPtr> traj_cont_;

private:
  //! Constant to check for equality of variables of two RobotState instances.
  static constexpr double ROBOT_STATE_EQUALITY_EPSILON = 1e-4;
};

inline void PlanComponentsBuilder::setBlender(std::unique_ptr<pilz_industrial_motion_planner::TrajectoryBlender> blender)
{
  blender_ = std::move(blender);
}

inline void PlanComponentsBuilder::setModel(const moveit::core::RobotModelConstPtr& model)
{
  model_ = model;
}

inline void PlanComponentsBuilder::reset()
{
  traj_tail_ = nullptr;
  traj_cont_.clear();
}

}  // namespace pilz_industrial_motion_planner
