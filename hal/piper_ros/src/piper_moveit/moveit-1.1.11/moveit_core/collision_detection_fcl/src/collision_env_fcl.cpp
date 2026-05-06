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

/* Author: Ioan Sucan, Jens Petit */

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection_fcl/collision_common.h>

#include <moveit/collision_detection_fcl/fcl_compat.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#endif

namespace collision_detection
{
static const std::string NAME = "FCL";
constexpr char LOGNAME[] = "collision_detection.fcl";

namespace
{
// Check whether this FCL version supports the requested computations
void checkFCLCapabilities(const DistanceRequest& req)
{
#if MOVEIT_FCL_VERSION < FCL_VERSION_CHECK(0, 6, 0)
  if (req.enable_nearest_points)
  {
    // Known issues:
    //   https://github.com/flexible-collision-library/fcl/issues/171,
    //   https://github.com/flexible-collision-library/fcl/pull/288
    ROS_ERROR_THROTTLE_NAMED(2.0, LOGNAME,
                             "You requested a distance check with enable_nearest_points=true, "
                             "but the FCL version MoveIt was compiled against (%d.%d.%d) "
                             "is known to return bogus nearest points. Please update your FCL "
                             "to at least 0.6.0.",
                             FCL_MAJOR_VERSION, FCL_MINOR_VERSION, FCL_PATCH_VERSION);
  }
#else
  (void)(req);  // silent -Wunused-parameter
#endif
}
}  // namespace

CollisionEnvFCL::CollisionEnvFCL(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
{
  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  std::size_t index;
  robot_geoms_.resize(robot_model_->getLinkGeometryCount());
  robot_fcl_objs_.resize(robot_model_->getLinkGeometryCount());
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (auto link : links)
    for (std::size_t j = 0; j < link->getShapes().size(); ++j)
    {
      FCLGeometryConstPtr link_geometry = createCollisionGeometry(link->getShapes()[j], getLinkScale(link->getName()),
                                                                  getLinkPadding(link->getName()), link, j);
      if (link_geometry)
      {
        index = link->getFirstCollisionBodyTransformIndex() + j;
        robot_geoms_[index] = link_geometry;

        // Need to store the FCL object so the AABB does not get recreated every time.
        // Every time this object is created, g->computeLocalAABB() is called  which is
        // very expensive and should only be calculated once. To update the AABB, use the
        // collObj->setTransform and then call collObj->computeAABB() to transform the AABB.
        robot_fcl_objs_[index] =
            FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(link_geometry->collision_geometry_));
      }
      else
        ROS_ERROR_NAMED(LOGNAME, "Unable to construct collision geometry for link '%s'", link->getName().c_str());
    }

  manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}

CollisionEnvFCL::CollisionEnvFCL(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding,
                                 double scale)
  : CollisionEnv(model, world, padding, scale)
{
  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  std::size_t index;
  robot_geoms_.resize(robot_model_->getLinkGeometryCount());
  robot_fcl_objs_.resize(robot_model_->getLinkGeometryCount());
  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (auto link : links)
    for (std::size_t j = 0; j < link->getShapes().size(); ++j)
    {
      FCLGeometryConstPtr g = createCollisionGeometry(link->getShapes()[j], getLinkScale(link->getName()),
                                                      getLinkPadding(link->getName()), link, j);
      if (g)
      {
        index = link->getFirstCollisionBodyTransformIndex() + j;
        robot_geoms_[index] = g;

        // Need to store the FCL object so the AABB does not get recreated every time.
        // Every time this object is created, g->computeLocalAABB() is called  which is
        // very expensive and should only be calculated once. To update the AABB, use the
        // collObj->setTransform and then call collObj->computeAABB() to transform the AABB.
        robot_fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
      }
      else
        ROS_ERROR_NAMED(LOGNAME, "Unable to construct collision geometry for link '%s'", link->getName().c_str());
    }

  manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvFCL::~CollisionEnvFCL()
{
  getWorld()->removeObserver(observer_handle_);
}

CollisionEnvFCL::CollisionEnvFCL(const CollisionEnvFCL& other, const WorldPtr& world) : CollisionEnv(other, world)
{
  robot_geoms_ = other.robot_geoms_;
  robot_fcl_objs_ = other.robot_fcl_objs_;

  manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();

  fcl_objs_ = other.fcl_objs_;
  for (auto& fcl_obj : fcl_objs_)
    fcl_obj.second.registerTo(manager_.get());
  // manager_->update();

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}

void CollisionEnvFCL::getAttachedBodyObjects(const moveit::core::AttachedBody* ab,
                                             std::vector<FCLGeometryConstPtr>& geoms) const
{
  const std::vector<shapes::ShapeConstPtr>& shapes = ab->getShapes();
  const size_t num_shapes = shapes.size();
  geoms.reserve(num_shapes);
  for (std::size_t i = 0; i < num_shapes; ++i)
  {
    FCLGeometryConstPtr co = createCollisionGeometry(shapes[i], getLinkScale(ab->getAttachedLinkName()),
                                                     getLinkPadding(ab->getAttachedLinkName()), ab, i);
    if (co)
      geoms.push_back(co);
  }
}

void CollisionEnvFCL::constructFCLObjectWorld(const World::Object* obj, FCLObject& fcl_obj) const
{
  for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
  {
    FCLGeometryConstPtr g = createCollisionGeometry(obj->shapes_[i], obj);
    if (g)
    {
      auto co = new fcl::CollisionObjectd(g->collision_geometry_, transform2fcl(obj->global_shape_poses_[i]));
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(co));
      fcl_obj.collision_geometry_.push_back(g);
    }
  }
}

void CollisionEnvFCL::constructFCLObjectRobot(const moveit::core::RobotState& state, FCLObject& fcl_obj) const
{
  fcl_obj.collision_objects_.reserve(robot_geoms_.size());
  fcl::Transform3d fcl_tf;

  for (std::size_t i = 0; i < robot_geoms_.size(); ++i)
    if (robot_geoms_[i] && robot_geoms_[i]->collision_geometry_)
    {
      transform2fcl(state.getCollisionBodyTransform(robot_geoms_[i]->collision_geometry_data_->ptr.link,
                                                    robot_geoms_[i]->collision_geometry_data_->shape_index),
                    fcl_tf);
      auto coll_obj = new fcl::CollisionObjectd(*robot_fcl_objs_[i]);
      coll_obj->setTransform(fcl_tf);
      coll_obj->computeAABB();
      fcl_obj.collision_objects_.push_back(FCLCollisionObjectPtr(coll_obj));
    }

  // TODO: Implement a method for caching fcl::CollisionObject's for moveit::core::AttachedBody's
  std::vector<const moveit::core::AttachedBody*> ab;
  state.getAttachedBodies(ab);
  for (auto& body : ab)
  {
    std::vector<FCLGeometryConstPtr> objs;
    getAttachedBodyObjects(body, objs);
    const EigenSTL::vector_Isometry3d& ab_t = body->getGlobalCollisionBodyTransforms();
    for (std::size_t k = 0; k < objs.size(); ++k)
      if (objs[k]->collision_geometry_)
      {
        transform2fcl(ab_t[k], fcl_tf);
        fcl_obj.collision_objects_.push_back(
            std::make_shared<fcl::CollisionObjectd>(objs[k]->collision_geometry_, fcl_tf));
        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
        // and would be destroyed when objs goes out of scope.
        fcl_obj.collision_geometry_.push_back(objs[k]);
      }
  }
}

void CollisionEnvFCL::allocSelfCollisionBroadPhase(const moveit::core::RobotState& state, FCLManager& manager) const
{
  manager.manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManagerd>();

  constructFCLObjectRobot(state, manager.object_);
  manager.object_.registerTo(manager.manager_.get());
}

void CollisionEnvFCL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state) const
{
  checkSelfCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvFCL::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                         const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void CollisionEnvFCL::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state,
                                               const AllowedCollisionMatrix* acm) const
{
  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  manager.manager_->collide(&cd, &collisionCallback);
  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceSelf(dreq, dres, state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionEnvFCL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state) const
{
  checkRobotCollisionHelper(req, res, state, nullptr);
}

void CollisionEnvFCL::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state,
                                          const AllowedCollisionMatrix& acm) const
{
  checkRobotCollisionHelper(req, res, state, &acm);
}

void CollisionEnvFCL::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                          const moveit::core::RobotState& /*state1*/,
                                          const moveit::core::RobotState& /*state2*/) const
{
  ROS_ERROR_NAMED(LOGNAME, "Continuous collision not implemented");
}

void CollisionEnvFCL::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                          const moveit::core::RobotState& /*state1*/,
                                          const moveit::core::RobotState& /*state2*/,
                                          const AllowedCollisionMatrix& /*acm*/) const
{
  ROS_ERROR_NAMED(LOGNAME, "Not implemented");
}

void CollisionEnvFCL::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                const moveit::core::RobotState& state,
                                                const AllowedCollisionMatrix* acm) const
{
  FCLObject fcl_obj;
  constructFCLObjectRobot(state, fcl_obj);

  CollisionData cd(&req, &res, acm);
  cd.enableGroup(getRobotModel());
  for (std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->collide(fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);

  if (req.distance)
  {
    DistanceRequest dreq;
    DistanceResult dres;

    dreq.group_name = req.group_name;
    dreq.acm = acm;
    dreq.enableGroup(getRobotModel());
    distanceRobot(dreq, dres, state);
    res.distance = dres.minimum_distance.distance;
  }
}

void CollisionEnvFCL::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                   const moveit::core::RobotState& state) const
{
  checkFCLCapabilities(req);

  FCLManager manager;
  allocSelfCollisionBroadPhase(state, manager);
  DistanceData drd(&req, &res);

  manager.manager_->distance(&drd, &distanceCallback);
}

void CollisionEnvFCL::distanceRobot(const DistanceRequest& req, DistanceResult& res,
                                    const moveit::core::RobotState& state) const
{
  checkFCLCapabilities(req);

  FCLObject fcl_obj;
  constructFCLObjectRobot(state, fcl_obj);

  DistanceData drd(&req, &res);
  for (std::size_t i = 0; !drd.done && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->distance(fcl_obj.collision_objects_[i].get(), &drd, &distanceCallback);
}

void CollisionEnvFCL::updateFCLObject(const std::string& id)
{
  // remove FCL objects that correspond to this object
  auto jt = fcl_objs_.find(id);
  if (jt != fcl_objs_.end())
  {
    jt->second.unregisterFrom(manager_.get());
    jt->second.clear();
  }

  // check to see if we have this object
  auto it = getWorld()->find(id);
  if (it != getWorld()->end())
  {
    // construct FCL objects that correspond to this object
    if (jt != fcl_objs_.end())
    {
      constructFCLObjectWorld(it->second.get(), jt->second);
      jt->second.registerTo(manager_.get());
    }
    else
    {
      constructFCLObjectWorld(it->second.get(), fcl_objs_[id]);
      fcl_objs_[id].registerTo(manager_.get());
    }
  }
  else
  {
    if (jt != fcl_objs_.end())
      fcl_objs_.erase(jt);
  }

  // manager_->update();
}

void CollisionEnvFCL::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  manager_->clear();
  fcl_objs_.clear();
  cleanCollisionGeometryCache();

  CollisionEnv::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionEnvFCL::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    auto it = fcl_objs_.find(obj->id_);
    if (it != fcl_objs_.end())
    {
      it->second.unregisterFrom(manager_.get());
      it->second.clear();
      fcl_objs_.erase(it);
    }
    cleanCollisionGeometryCache();
  }
  else
  {
    updateFCLObject(obj->id_);
    if (action & (World::DESTROY | World::REMOVE_SHAPE))
      cleanCollisionGeometryCache();
  }
}

void CollisionEnvFCL::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  std::size_t index;
  for (const auto& link : links)
  {
    const moveit::core::LinkModel* lmodel = robot_model_->getLinkModel(link);
    if (lmodel)
    {
      for (std::size_t j = 0; j < lmodel->getShapes().size(); ++j)
      {
        FCLGeometryConstPtr g = createCollisionGeometry(lmodel->getShapes()[j], getLinkScale(lmodel->getName()),
                                                        getLinkPadding(lmodel->getName()), lmodel, j);
        if (g)
        {
          index = lmodel->getFirstCollisionBodyTransformIndex() + j;
          robot_geoms_[index] = g;
          robot_fcl_objs_[index] = FCLCollisionObjectConstPtr(new fcl::CollisionObjectd(g->collision_geometry_));
        }
      }
    }
    else
      ROS_ERROR_NAMED(LOGNAME, "Updating padding or scaling for unknown link: '%s'", link.c_str());
  }
}

const std::string& CollisionDetectorAllocatorFCL::getName() const
{
  return NAME;
}

}  // namespace collision_detection
