/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Sachin Chitta
 *********************************************************************/

#pragma once

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>

// MoveIt
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>

namespace kinematics_constraint_aware
{
/**
 * @class A kinematics request
 */
class KinematicsRequest
{
public:
  KinematicsRequest()
  {
  }

  virtual ~KinematicsRequest(){};

  std::vector<geometry_msgs::PoseStamped> pose_stamped_vector_;

  std::vector<std::string> ik_link_names_;

  moveit::core::RobotStatePtr robot_state_;

  kinematic_constraints::KinematicConstraintSetPtr constraints_;

  ros::Duration timeout_;

  std::string group_name_;

  bool check_for_collisions_;

  moveit::core::StateValidityCallbackFn constraint_callback_;
};

/**
 * @class A kinematics response
 */
class KinematicsResponse
{
public:
  KinematicsResponse()
  {
  }

  virtual ~KinematicsResponse(){};

  moveit::core::RobotStatePtr solution_;

  std::vector<kinematic_constraints::ConstraintEvaluationResult> constraint_eval_results_;

  collision_detection::CollisionResult collision_result_;

  moveit_msgs::MoveItErrorCodes error_code_;

  bool result_;
};
}  // namespace kinematics_constraint_aware
