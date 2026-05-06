/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#pragma once

#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_cast_bvh_manager.h>
#include <mutex>

namespace collision_detection
{
/** \brief  */
class CollisionEnvBullet : public CollisionEnv
{
public:
  CollisionEnvBullet() = delete;

  CollisionEnvBullet(const moveit::core::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);

  CollisionEnvBullet(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding = 0.0,
                     double scale = 1.0);

  CollisionEnvBullet(const CollisionEnvBullet& other, const WorldPtr& world);

  ~CollisionEnvBullet() override;

  CollisionEnvBullet(CollisionEnvBullet&) = delete;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                           const moveit::core::RobotState& state) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                           const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const moveit::core::RobotState& state) const override;

  void distanceRobot(const DistanceRequest& req, DistanceResult& res,
                     const moveit::core::RobotState& state) const override;

  void setWorld(const WorldPtr& world) override;

protected:
  /** \brief Updates the poses of the objects in the manager according to given robot state */
  void updateTransformsFromState(const moveit::core::RobotState& state,
                                 const collision_detection_bullet::BulletDiscreteBVHManagerPtr& manager) const;

  /** \brief Updates the collision objects saved in the manager to reflect a new padding or scaling of the robot links
   */
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;

  /** \brief All of the attached objects in the robot state are wrapped into bullet collision objects */
  void addAttachedOjects(const moveit::core::RobotState& state,
                         std::vector<collision_detection_bullet::CollisionObjectWrapperPtr>& cows) const;

  /** \brief Bundles the different checkSelfCollision functions into a single function */
  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

  void checkRobotCollisionHelperCCD(const CollisionRequest& req, CollisionResult& res,
                                    const moveit::core::RobotState& state1, const moveit::core::RobotState& state2,
                                    const AllowedCollisionMatrix* acm) const;

  void checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                 const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

  /** \brief Construts a bullet collision object out of a robot link */
  void addLinkAsCollisionObject(const urdf::LinkSharedPtr& link);

  /** \brief Handles self collision checks */
  collision_detection_bullet::BulletDiscreteBVHManagerPtr manager_{
    new collision_detection_bullet::BulletDiscreteBVHManager()
  };

  /** \brief Handles continuous robot world collision checks */
  collision_detection_bullet::BulletCastBVHManagerPtr manager_CCD_{
    new collision_detection_bullet::BulletCastBVHManager()
  };

  // Lock manager_ and manager_CCD_, for thread-safe collision tests
  mutable std::mutex collision_env_mutex_;

  /** \brief Adds a world object to the collision managers */
  void addToManager(const World::Object* obj);

  /** \brief Updates a managed collision object with its world representation.
   *
   * We have three cases: 1) the object is part of the manager and not of world --> delete it
   *                      2) the object is not in the manager, therefore register to manager,
   *                      3) the object is in the manager then delete and add the modified */
  void updateManagedObject(const std::string& id);

  /** \brief The active links where active refers to the group which can collide with everything */
  std::vector<std::string> active_;

private:
  /** \brief Callback function executed for each change to the world environment */
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);

  World::ObserverHandle observer_handle_;
};
}  // namespace collision_detection
