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

#include <moveit/collision_detection_bullet/bullet_integration/bullet_cast_bvh_manager.h>
#include <map>
#include <utility>

namespace collision_detection_bullet
{
BulletCastBVHManagerPtr BulletCastBVHManager::clone() const
{
  BulletCastBVHManagerPtr manager(new BulletCastBVHManager());

  for (const std::pair<const std::string, collision_detection_bullet::CollisionObjectWrapperPtr>& cow : link2cow_)
  {
    CollisionObjectWrapperPtr new_cow = cow.second->clone();

    assert(new_cow->getCollisionShape());
    assert(new_cow->getCollisionShape()->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    new_cow->setWorldTransform(cow.second->getWorldTransform());
    new_cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance_));
    manager->addCollisionObject(new_cow);
  }

  manager->setActiveCollisionObjects(active_);
  manager->setContactDistanceThreshold(contact_distance_);

  return manager;
}

void BulletCastBVHManager::setCastCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose1,
                                                            const Eigen::Isometry3d& pose2)
{
  // TODO: Find a way to remove this check. Need to store information in Tesseract EnvState indicating transforms with
  // geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    CollisionObjectWrapperPtr& cow = it->second;
    assert(cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter);

    btTransform tf1 = convertEigenToBt(pose1);
    btTransform tf2 = convertEigenToBt(pose2);

    cow->setWorldTransform(tf1);
    link2cow_[name]->setWorldTransform(tf1);

    // If collision object is disabled dont proceed
    if (cow->m_enabled)
    {
      if (btBroadphaseProxy::isConvex(cow->getCollisionShape()->getShapeType()))
      {
        static_cast<CastHullShape*>(cow->getCollisionShape())->updateCastTransform(tf1.inverseTimes(tf2));
      }
      else if (btBroadphaseProxy::isCompound(cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(cow->getCollisionShape());

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
          {
            const btTransform& local_tf = compound->getChildTransform(i);

            btTransform delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);
            static_cast<CastHullShape*>(compound->getChildShape(i))->updateCastTransform(delta_tf);
            compound->updateChildTransform(i, local_tf, false);  // This is required to update the BVH tree
          }
          else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
          {
            btCompoundShape* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));

            for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
            {
              assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));
              const btTransform& local_tf = second_compound->getChildTransform(j);

              btTransform delta_tf = (tf1 * local_tf).inverseTimes(tf2 * local_tf);
              static_cast<CastHullShape*>(second_compound->getChildShape(j))->updateCastTransform(delta_tf);
              second_compound->updateChildTransform(j, local_tf, false);  // This is required to update the BVH tree
            }
            second_compound->recalculateLocalAabb();
          }
        }
        compound->recalculateLocalAabb();
      }
      else
      {
        ROS_ERROR_NAMED("collision_detection.bullet", "I can only continuous collision check convex shapes and "
                                                      "compound shapes made of convex shapes");
        throw std::runtime_error(
            "I can only continuous collision check convex shapes and compound shapes made of convex shapes");
      }

      // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
    }
  }
}

void BulletCastBVHManager::contactTest(collision_detection::CollisionResult& collisions,
                                       const collision_detection::CollisionRequest& req,
                                       const collision_detection::AllowedCollisionMatrix* acm, bool /*self*/ = false)
{
  ContactTestData cdata(active_, contact_distance_, collisions, req);
  broadphase_->calculateOverlappingPairs(dispatcher_.get());
  btOverlappingPairCache* pair_cache = broadphase_->getOverlappingPairCache();

  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                         "Number overlapping candidates " << pair_cache->getNumOverlappingPairs());

  BroadphaseContactResultCallback cc(cdata, contact_distance_, acm, false, true);
  TesseractCollisionPairCallback collision_callback(dispatch_info_, dispatcher_.get(), cc);
  pair_cache->processAllOverlappingPairs(&collision_callback, dispatcher_.get());
}

void BulletCastBVHManager::addCollisionObject(const CollisionObjectWrapperPtr& cow)
{
  std::string name = cow->getName();
  if (cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)
  {
    CollisionObjectWrapperPtr cast_cow = makeCastCollisionObject(cow);
    link2cow_[name] = cast_cow;
  }
  else
  {
    link2cow_[name] = cow;
  }

  btVector3 aabb_min, aabb_max;
  link2cow_[name]->getAABB(aabb_min, aabb_max);

  int type = link2cow_[name]->getCollisionShape()->getShapeType();
  link2cow_[name]->setBroadphaseHandle(
      broadphase_->createProxy(aabb_min, aabb_max, type, link2cow_[name].get(), link2cow_[name]->m_collisionFilterGroup,
                               link2cow_[name]->m_collisionFilterMask, dispatcher_.get()));
}

}  // namespace collision_detection_bullet
