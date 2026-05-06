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

#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_bullet/bullet_integration/ros_bullet_utils.h>
#include <moveit/collision_detection_bullet/bullet_integration/contact_checker_common.h>
#include <functional>
#include <bullet/btBulletCollisionCommon.h>

namespace collision_detection
{
namespace
{
static const std::string NAME = "Bullet";
const double MAX_DISTANCE_MARGIN = 99;
constexpr char LOGNAME[] = "collision_detection.bullet";
}  // namespace

CollisionEnvBullet::CollisionEnvBullet(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });

  for (const std::pair<const std::string, urdf::LinkSharedPtr>& link : robot_model_->getURDF()->links_)
  {
    addLinkAsCollisionObject(link.second);
  }
}

CollisionEnvBullet::CollisionEnvBullet(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world,
                                       double padding, double scale)
  : CollisionEnv(model, world, padding, scale)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });

  for (const std::pair<const std::string, urdf::LinkSharedPtr>& link : robot_model_->getURDF()->links_)
  {
    addLinkAsCollisionObject(link.second);
  }

  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvBullet::CollisionEnvBullet(const CollisionEnvBullet& other, const WorldPtr& world)
  : CollisionEnv(other, world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });

  for (const std::pair<const std::string, urdf::LinkSharedPtr>& link : other.robot_model_->getURDF()->links_)
  {
    addLinkAsCollisionObject(link.second);
  }

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvBullet::~CollisionEnvBullet()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionEnvBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvBullet::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionEnvBullet::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const moveit::core::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_bullet::CollisionObjectWrapperPtr> cows;
  addAttachedOjects(state, cows);

  if (req.distance)
  {
    manager_->setContactDistanceThreshold(MAX_DISTANCE_MARGIN);
  }

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->addCollisionObject(cow);
    manager_->setCollisionObjectsTransform(
        cow->getName(), state.getAttachedBody(cow->getName())->getGlobalCollisionBodyTransforms()[0]);
  }

  // updating link positions with the current robot state
  for (const std::string& link : active_)
  {
    manager_->setCollisionObjectsTransform(link, state.getCollisionBodyTransform(link, 0));
  }

  manager_->contactTest(res, req, acm, true);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state,
                                             const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, state, &acm);
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2) const
{
  checkRobotCollisionHelperCCD(req, res, state1, state2, nullptr);
}

void CollisionEnvBullet::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2,
                                             const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelperCCD(req, res, state1, state2, &acm);
}

void CollisionEnvBullet::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   const AllowedCollisionMatrix* acm) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  if (req.distance)
  {
    manager_->setContactDistanceThreshold(MAX_DISTANCE_MARGIN);
  }

  std::vector<collision_detection_bullet::CollisionObjectWrapperPtr> attached_cows;
  addAttachedOjects(state, attached_cows);
  updateTransformsFromState(state, manager_);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : attached_cows)
  {
    manager_->addCollisionObject(cow);
    manager_->setCollisionObjectsTransform(
        cow->getName(), state.getAttachedBody(cow->getName())->getGlobalCollisionBodyTransforms()[0]);
  }

  manager_->contactTest(res, req, acm, false);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : attached_cows)
  {
    manager_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvBullet::checkRobotCollisionHelperCCD(const CollisionRequest& req, CollisionResult& res,
                                                      const moveit::core::RobotState& state1,
                                                      const moveit::core::RobotState& state2,
                                                      const AllowedCollisionMatrix* acm) const
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);

  std::vector<collision_detection_bullet::CollisionObjectWrapperPtr> attached_cows;
  addAttachedOjects(state1, attached_cows);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : attached_cows)
  {
    manager_CCD_->addCollisionObject(cow);
    manager_CCD_->setCastCollisionObjectsTransform(
        cow->getName(), state1.getAttachedBody(cow->getName())->getGlobalCollisionBodyTransforms()[0],
        state2.getAttachedBody(cow->getName())->getGlobalCollisionBodyTransforms()[0]);
  }

  for (const std::string& link : active_)
  {
    manager_CCD_->setCastCollisionObjectsTransform(link, state1.getCollisionBodyTransform(link, 0),
                                                   state2.getCollisionBodyTransform(link, 0));
  }

  manager_CCD_->contactTest(res, req, acm, false);

  for (const collision_detection_bullet::CollisionObjectWrapperPtr& cow : attached_cows)
  {
    manager_CCD_->removeCollisionObject(cow->getName());
  }
}

void CollisionEnvBullet::distanceSelf(const DistanceRequest& /*req*/, DistanceResult& /*res*/,
                                      const moveit::core::RobotState& /*state*/) const
{
  ROS_INFO_NAMED(LOGNAME, "distanceSelf is not implemented for Bullet.");
}

void CollisionEnvBullet::distanceRobot(const DistanceRequest& /*req*/, DistanceResult& /*res*/,
                                       const moveit::core::RobotState& /*state*/) const
{
  ROS_INFO_NAMED(LOGNAME, "distanceRobot is not implemented for Bullet.");
}

void CollisionEnvBullet::addToManager(const World::Object* obj)
{
  std::vector<collision_detection_bullet::CollisionObjectType> collision_object_types;

  for (const shapes::ShapeConstPtr& shape : obj->shapes_)
  {
    if (shape->type == shapes::MESH)
      collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::CONVEX_HULL);
    else
      collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);
  }

  auto cow = std::make_shared<collision_detection_bullet::CollisionObjectWrapper>(
      obj->id_, collision_detection::BodyType::WORLD_OBJECT, obj->shapes_, obj->global_shape_poses_,
      collision_object_types, false);

  manager_->addCollisionObject(cow);
  manager_CCD_->addCollisionObject(cow->clone());
}

void CollisionEnvBullet::updateManagedObject(const std::string& id)
{
  if (getWorld()->hasObject(id))
  {
    auto it = getWorld()->find(id);
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
      manager_CCD_->removeCollisionObject(id);
      addToManager(it->second.get());
    }
    else
    {
      addToManager(it->second.get());
    }
  }
  else
  {
    if (manager_->hasCollisionObject(id))
    {
      manager_->removeCollisionObject(id);
      manager_CCD_->removeCollisionObject(id);
    }
  }
}

void CollisionEnvBullet::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  CollisionEnv::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionEnvBullet::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  std::lock_guard<std::mutex> guard(collision_env_mutex_);
  if (action == World::DESTROY)
  {
    manager_->removeCollisionObject(obj->id_);
    manager_CCD_->removeCollisionObject(obj->id_);
  }
  else
  {
    updateManagedObject(obj->id_);
  }
}

void CollisionEnvBullet::addAttachedOjects(const moveit::core::RobotState& state,
                                           std::vector<collision_detection_bullet::CollisionObjectWrapperPtr>& cows) const
{
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  state.getAttachedBodies(attached_bodies);

  for (const moveit::core::AttachedBody*& body : attached_bodies)
  {
    const EigenSTL::vector_Isometry3d& attached_body_transform = body->getGlobalCollisionBodyTransforms();

    std::vector<collision_detection_bullet::CollisionObjectType> collision_object_types(
        attached_body_transform.size(), collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);

    try
    {
      collision_detection_bullet::CollisionObjectWrapperPtr cow(new collision_detection_bullet::CollisionObjectWrapper(
          body->getName(), collision_detection::BodyType::ROBOT_ATTACHED, body->getShapes(), attached_body_transform,
          collision_object_types, body->getTouchLinks()));
      cows.push_back(cow);
    }
    catch (std::exception&)
    {
      ROS_ERROR_STREAM_NAMED("collision_detetction.bullet",
                             "Not adding " << body->getName() << " due to bad arguments.");
    }
  }
}

void CollisionEnvBullet::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  for (const std::string& link : links)
  {
    if (robot_model_->getURDF()->links_.find(link) != robot_model_->getURDF()->links_.end())
    {
      addLinkAsCollisionObject(robot_model_->getURDF()->links_[link]);
    }
    else
    {
      ROS_ERROR_NAMED("collision_detection.bullet", "Updating padding or scaling for unknown link: '%s'", link.c_str());
    }
  }
}

void CollisionEnvBullet::updateTransformsFromState(
    const moveit::core::RobotState& state, const collision_detection_bullet::BulletDiscreteBVHManagerPtr& manager) const
{
  // updating link positions with the current robot state
  for (const std::string& link : active_)
  {
    // select the first of the transformations for each link (composed of multiple shapes...)
    manager->setCollisionObjectsTransform(link, state.getCollisionBodyTransform(link, 0));
  }
}

void CollisionEnvBullet::addLinkAsCollisionObject(const urdf::LinkSharedPtr& link)
{
  if (!link->collision_array.empty())
  {
    const std::vector<urdf::CollisionSharedPtr>& col_array =
        link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, link->collision) :
                                        link->collision_array;

    std::vector<shapes::ShapeConstPtr> shapes;
    collision_detection_bullet::AlignedVector<Eigen::Isometry3d> shape_poses;
    std::vector<collision_detection_bullet::CollisionObjectType> collision_object_types;

    for (const auto& i : col_array)
    {
      if (i && i->geometry)
      {
        shapes::ShapePtr shape = collision_detection_bullet::constructShape(i->geometry.get());

        if (shape)
        {
          if (fabs(getLinkScale(link->name) - 1.0) >= std::numeric_limits<double>::epsilon() ||
              fabs(getLinkPadding(link->name)) >= std::numeric_limits<double>::epsilon())
          {
            shape->scaleAndPadd(getLinkScale(link->name), getLinkPadding(link->name));
          }

          shapes.push_back(shape);
          shape_poses.push_back(collision_detection_bullet::urdfPose2Eigen(i->origin));

          if (shape->type == shapes::MESH)
          {
            collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::CONVEX_HULL);
          }
          else
          {
            collision_object_types.push_back(collision_detection_bullet::CollisionObjectType::USE_SHAPE_TYPE);
          }
        }
      }
    }

    if (manager_->hasCollisionObject(link->name))
    {
      manager_->removeCollisionObject(link->name);
      manager_CCD_->removeCollisionObject(link->name);
    }

    try
    {
      collision_detection_bullet::CollisionObjectWrapperPtr cow(new collision_detection_bullet::CollisionObjectWrapper(
          link->name, collision_detection::BodyType::ROBOT_LINK, shapes, shape_poses, collision_object_types, true));
      manager_->addCollisionObject(cow);
      manager_CCD_->addCollisionObject(cow->clone());
      active_.push_back(cow->getName());
    }
    catch (std::exception&)
    {
      ROS_ERROR_STREAM_NAMED("collision_detetction.bullet", "Not adding " << link->name << " due to bad arguments.");
    }
  }
}

const std::string& CollisionDetectorAllocatorBullet::getName() const
{
  return NAME;
}

}  // namespace collision_detection
