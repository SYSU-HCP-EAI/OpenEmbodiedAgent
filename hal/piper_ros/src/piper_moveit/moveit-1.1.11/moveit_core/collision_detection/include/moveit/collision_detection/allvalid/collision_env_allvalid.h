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

/* Author: Ioan Sucan, Jia Pan, Jens Petit */

#pragma once

#include <moveit/collision_detection/collision_env.h>

namespace collision_detection
{
/** \brief Collision environment which always just returns no collisions.
 *
 *  This can be used to save resources if collision checking is not important. */
class CollisionEnvAllValid : public CollisionEnv
{
public:
  CollisionEnvAllValid(const moveit::core::RobotModelConstPtr& robot_model, double padding = 0.0, double scale = 1.0);
  CollisionEnvAllValid(const moveit::core::RobotModelConstPtr& robot_model, const WorldPtr& world, double padding = 0.0,
                       double scale = 1.0);
  CollisionEnvAllValid(const CollisionEnv& other, const WorldPtr& world);

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                           const moveit::core::RobotState& state) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                           const AllowedCollisionMatrix& acm) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;
  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  virtual double distanceRobot(const moveit::core::RobotState& state) const;
  virtual double distanceRobot(const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const;
  void distanceRobot(const DistanceRequest& req, DistanceResult& res,
                     const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const moveit::core::RobotState& state) const override;
  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const moveit::core::RobotState& state) const override;
};
}  // namespace collision_detection
