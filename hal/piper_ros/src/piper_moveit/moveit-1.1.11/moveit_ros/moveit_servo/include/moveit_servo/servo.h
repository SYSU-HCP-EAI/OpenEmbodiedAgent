/*******************************************************************************
 *      Title     : servo.h
 *      Project   : moveit_servo
 *      Created   : 3/9/2017
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#pragma once

// System
#include <memory>

// MoveIt
#include <moveit_servo/collision_check.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo_calcs.h>

namespace moveit_servo
{
/**
 * Class Servo - Jacobian based robot control with collision avoidance.
 */
class Servo
{
public:
  Servo(ros::NodeHandle& nh, const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~Servo();

  /** \brief start servo node */
  void start();

  /** \brief Pause or unpause processing servo commands while keeping the timers alive */
  void setPaused(bool paused);

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getCommandFrameTransform(Eigen::Isometry3d& transform);
  bool getCommandFrameTransform(geometry_msgs::TransformStamped& transform);

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getEEFrameTransform(Eigen::Isometry3d& transform);
  bool getEEFrameTransform(geometry_msgs::TransformStamped& transform);

  /** \brief Get the parameters used by servo node. */
  const ServoParameters& getParameters() const;

  /** \brief Change the controlled link. Often, this is the end effector
   * This must be a link on the robot since MoveIt tracks the transform (not tf)
   */
  void changeRobotLinkCommandFrame(const std::string& new_command_frame)
  {
    servo_calcs_->changeRobotLinkCommandFrame(new_command_frame);
  }

  // Give test access to private/protected methods
  friend class ServoFixture;

private:
  bool readParameters();

  ros::NodeHandle nh_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Store the parameters that were read from ROS server
  ServoParameters parameters_;

  std::unique_ptr<ServoCalcs> servo_calcs_;
  std::unique_ptr<CollisionCheck> collision_checker_;
};

// ServoPtr using alias
using ServoPtr = std::shared_ptr<Servo>;

}  // namespace moveit_servo
