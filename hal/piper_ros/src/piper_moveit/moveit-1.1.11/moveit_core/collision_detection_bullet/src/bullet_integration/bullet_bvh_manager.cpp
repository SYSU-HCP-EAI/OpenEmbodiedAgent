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

/* Authors: Levi Armstrong, Jens Petit */

#include <moveit/collision_detection_bullet/bullet_integration/bullet_bvh_manager.h>
#include <map>
#include <utility>

namespace collision_detection_bullet
{
BulletBVHManager::BulletBVHManager()
{
  dispatcher_ = std::make_unique<btCollisionDispatcher>(&coll_config_);

  dispatcher_->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE,
                                           coll_config_.getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE,
                                                                                        CONVEX_SHAPE_PROXYTYPE));

  dispatcher_->setDispatcherFlags(dispatcher_->getDispatcherFlags() &
                                  ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);

  broadphase_ = std::make_unique<btDbvtBroadphase>();

  broadphase_->getOverlappingPairCache()->setOverlapFilterCallback(&filter_callback_);

  contact_distance_ = BULLET_DEFAULT_CONTACT_DISTANCE;
}

BulletBVHManager::~BulletBVHManager()
{
  // clean up remaining objects
  for (const std::pair<const std::string, collision_detection_bullet::CollisionObjectWrapperPtr>& cow : link2cow_)
    removeCollisionObjectFromBroadphase(cow.second, broadphase_, dispatcher_);
}

bool BulletBVHManager::hasCollisionObject(const std::string& name) const
{
  return (link2cow_.find(name) != link2cow_.end());
}

bool BulletBVHManager::removeCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    CollisionObjectWrapperPtr& cow = it->second;
    removeCollisionObjectFromBroadphase(cow, broadphase_, dispatcher_);
    link2cow_.erase(name);
    return true;
  }

  return false;
}

bool BulletBVHManager::enableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = true;
    return true;
  }

  return false;
}

bool BulletBVHManager::disableCollisionObject(const std::string& name)
{
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    it->second->m_enabled = false;
    return true;
  }

  return false;
}

void BulletBVHManager::setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose)
{
  // TODO(j-petit): Find a way to remove this check. Need to store information in CollisionEnv transforms with geometry
  auto it = link2cow_.find(name);
  if (it != link2cow_.end())
  {
    CollisionObjectWrapperPtr& cow = it->second;
    btTransform tf = convertEigenToBt(pose);
    cow->setWorldTransform(tf);

    // Now update Broadphase AABB (See BulletWorld updateSingleAabb function)
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

void BulletBVHManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  active_ = names;

  // Now need to update the broadphase with correct aabb
  for (std::pair<const std::string, CollisionObjectWrapperPtr>& co : link2cow_)
  {
    CollisionObjectWrapperPtr& cow = co.second;
    updateCollisionObjectFilters(active_, *cow);

    // The broadphase tree structure has to be updated, therefore remove and add is necessary
    removeCollisionObjectFromBroadphase(cow, broadphase_, dispatcher_);
    addCollisionObjectToBroadphase(cow, broadphase_, dispatcher_);
  }
}

const std::vector<std::string>& BulletBVHManager::getActiveCollisionObjects() const
{
  return active_;
}

void BulletBVHManager::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;

  for (std::pair<const std::string, CollisionObjectWrapperPtr>& co : link2cow_)
  {
    CollisionObjectWrapperPtr& cow = co.second;
    cow->setContactProcessingThreshold(static_cast<btScalar>(contact_distance));
    if (cow->getBroadphaseHandle())
      updateBroadphaseAABB(cow, broadphase_, dispatcher_);
  }
}

double BulletBVHManager::getContactDistanceThreshold() const
{
  return contact_distance_;
}

const std::map<std::string, CollisionObjectWrapperPtr>& BulletBVHManager::getCollisionObjects() const
{
  return link2cow_;
}

}  // namespace collision_detection_bullet
