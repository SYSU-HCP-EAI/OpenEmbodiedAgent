/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Acorn Pooley, Ioan Sucan */

#pragma once

#include <moveit/collision_detection/collision_env.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(CollisionDetectorAllocator);  // Defines CollisionDetectorAllocatorPtr, ConstPtr, WeakPtr... etc

/** \brief An allocator for a compatible CollisionWorld/CollisionRobot pair. */
class CollisionDetectorAllocator
{
public:
  virtual ~CollisionDetectorAllocator()
  {
  }

  /** A unique name identifying the CollisionWorld/CollisionRobot pairing. */
  virtual const std::string& getName() const = 0;

  /** create a new CollisionWorld for checking collisions with the supplied world. */
  virtual CollisionEnvPtr allocateEnv(const WorldPtr& world,
                                      const moveit::core::RobotModelConstPtr& robot_model) const = 0;

  /** create a new CollisionWorld by copying an existing CollisionWorld of the same type.s
   * The world must be either the same world as used by \orig or a copy of that world which has not yet been modified.
   */
  virtual CollisionEnvPtr allocateEnv(const CollisionEnvConstPtr& orig, const WorldPtr& world) const = 0;

  /** create a new CollisionEnv given a robot_model with a new empty world */
  virtual CollisionEnvPtr allocateEnv(const moveit::core::RobotModelConstPtr& robot_model) const = 0;
};

/** \brief Template class to make it easy to create an allocator for a specific CollisionWorld/CollisionRobot pair. */
template <class CollisionEnvType, class CollisionDetectorAllocatorType>
class CollisionDetectorAllocatorTemplate : public CollisionDetectorAllocator
{
public:
  CollisionEnvPtr allocateEnv(const WorldPtr& world, const moveit::core::RobotModelConstPtr& robot_model) const override
  {
    return CollisionEnvPtr(new CollisionEnvType(robot_model, world));
  }

  CollisionEnvPtr allocateEnv(const CollisionEnvConstPtr& orig, const WorldPtr& world) const override
  {
    return CollisionEnvPtr(new CollisionEnvType(dynamic_cast<const CollisionEnvType&>(*orig), world));
  }

  CollisionEnvPtr allocateEnv(const moveit::core::RobotModelConstPtr& robot_model) const override
  {
    return CollisionEnvPtr(new CollisionEnvType(robot_model));
  }

  /** Create an allocator for collision detectors. */
  static CollisionDetectorAllocatorPtr create()
  {
    return CollisionDetectorAllocatorPtr(new CollisionDetectorAllocatorType());
  }
};
}  // namespace collision_detection
