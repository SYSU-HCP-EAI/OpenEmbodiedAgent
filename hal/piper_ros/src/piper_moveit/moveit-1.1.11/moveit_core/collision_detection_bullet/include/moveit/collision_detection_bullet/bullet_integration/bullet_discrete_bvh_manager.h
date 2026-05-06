/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Levi Armstrong, Jens Petit */

#pragma once

#include <moveit/collision_detection_bullet/bullet_integration/bullet_utils.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_bvh_manager.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection_bullet
{
MOVEIT_CLASS_FORWARD(BulletDiscreteBVHManager);

/** @brief A bounding volume hierarchy (BVH) implementaiton of a discrete bullet manager */
class BulletDiscreteBVHManager : public BulletBVHManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor */
  BulletDiscreteBVHManager() = default;

  ~BulletDiscreteBVHManager() override = default;

  /**@brief Clone the manager
   *
   * This is to be used for multi threaded applications. A user should make a clone for each thread. */
  BulletDiscreteBVHManagerPtr clone() const;

  /**@brief Perform a contact test for all objects in the manager
   * @param collisions The Contact results data
   * @param acm The allowed collision matrix
   * @param req The contact request */
  void contactTest(collision_detection::CollisionResult& collisions, const collision_detection::CollisionRequest& req,
                   const collision_detection::AllowedCollisionMatrix* acm, bool self) override;

  /**@brief Add a bullet collision object to the manager
   *  @param cow The bullet collision object */
  void addCollisionObject(const CollisionObjectWrapperPtr& cow) override;
};
}  // namespace collision_detection_bullet
