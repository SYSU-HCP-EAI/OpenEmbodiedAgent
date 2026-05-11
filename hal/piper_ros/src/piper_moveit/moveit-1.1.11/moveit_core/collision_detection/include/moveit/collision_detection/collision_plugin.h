/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Fetch Robotics Inc.
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
 *   * Neither the name of Fetch Robotics nor the names of its
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

#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/planning_scene/planning_scene.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(CollisionPlugin);  // Defines CollisionPluginPtr, ConstPtr, WeakPtr... etc

/**
 * @brief Plugin API for loading a custom collision detection robot/world.
 *
 * Typical Usage:
 * <PRE>
 *   namespace my_collision_checker
 *   {
 *
 *   class MyCollisionDetectorAllocator :
 *     public collision_detection::CollisionDetectorAllocatorTemplate<MyCollisionEnv, MyCollisionDetectorAllocator>
 *   {
 *     public:
 *       const std::string& getName() const override {
 *         static const std::string NAME = "my_checker";
 *         return NAME;
 *       }
 *       static const std::string NAME_;
 *   };
 *
 *   }
 *
 *   namespace collision_detection
 *   {
 *
 *   class MyCollisionDetectionLoader : public CollisionPlugin
 *   {
 *   public:
 *     virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
 *     {
 *       scene->setActiveCollisionDetector(my_collision_checker::MyCollisionDetectorAllocator::create(), exclusive);
         return true;
 *     }
 *   };
 * </PRE>
 */
class CollisionPlugin
{
public:
  CollisionPlugin()
  {
  }
  virtual ~CollisionPlugin()
  {
  }

  /**
   * @brief This should be used to load your collision plugin.
   */
  virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const = 0;
};

}  // namespace collision_detection
