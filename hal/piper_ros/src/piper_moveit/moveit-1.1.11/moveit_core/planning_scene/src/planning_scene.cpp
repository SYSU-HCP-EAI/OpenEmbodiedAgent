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

/* Author: Ioan Sucan */

#include <boost/algorithm/string.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/occupancy_map.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/exceptions/exceptions.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/utils/message_checks.h>
#include <octomap_msgs/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <memory>
#include <set>

namespace planning_scene
{
const std::string PlanningScene::OCTOMAP_NS = "<octomap>";
const std::string PlanningScene::DEFAULT_SCENE_NAME = "(noname)";

const std::string LOGNAME = "planning_scene";

class SceneTransforms : public moveit::core::Transforms
{
public:
  SceneTransforms(const PlanningScene* scene) : Transforms(scene->getRobotModel()->getModelFrame()), scene_(scene)
  {
  }

  bool canTransform(const std::string& from_frame) const override
  {
    return scene_->knowsFrameTransform(from_frame);
  }

  bool isFixedFrame(const std::string& frame) const override
  {
    if (frame.empty())
      return false;
    if (Transforms::isFixedFrame(frame))
      return true;
    if (frame[0] == '/')
      return knowsObjectFrame(frame.substr(1));
    else
      return knowsObjectFrame(frame);
  }

  const Eigen::Isometry3d& getTransform(const std::string& from_frame) const override
  {  // the call below also calls Transforms::getTransform() too
    return scene_->getFrameTransform(from_frame);
  }

private:
  // Returns true if frame_id is the name of an object or the name of a subframe on an object
  bool knowsObjectFrame(const std::string& frame_id) const
  {
    return scene_->getWorld()->knowsTransform(frame_id);
  }

  const PlanningScene* scene_;
};

bool PlanningScene::isEmpty(const moveit_msgs::PlanningScene& msg)
{
  return moveit::core::isEmpty(msg);
}

bool PlanningScene::isEmpty(const moveit_msgs::RobotState& msg)
{
  return moveit::core::isEmpty(msg);
}

bool PlanningScene::isEmpty(const moveit_msgs::PlanningSceneWorld& msg)
{
  return moveit::core::isEmpty(msg);
}

PlanningScene::PlanningScene(const moveit::core::RobotModelConstPtr& robot_model,
                             const collision_detection::WorldPtr& world)
  : robot_model_(robot_model), world_(world), world_const_(world)
{
  initialize();
}

PlanningScene::PlanningScene(const urdf::ModelInterfaceSharedPtr& urdf_model,
                             const srdf::ModelConstSharedPtr& srdf_model, const collision_detection::WorldPtr& world)
  : world_(world), world_const_(world)
{
  if (!urdf_model)
    throw moveit::ConstructException("The URDF model cannot be NULL");

  if (!srdf_model)
    throw moveit::ConstructException("The SRDF model cannot be NULL");

  robot_model_ = createRobotModel(urdf_model, srdf_model);
  if (!robot_model_)
    throw moveit::ConstructException("Could not create RobotModel");

  initialize();
}

PlanningScene::~PlanningScene()
{
  if (current_world_object_update_callback_)
    world_->removeObserver(current_world_object_update_observer_handle_);
}

void PlanningScene::initialize()
{
  name_ = DEFAULT_SCENE_NAME;

  scene_transforms_ = std::make_shared<SceneTransforms>(this);

  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  robot_state_->update();

  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(*getRobotModel()->getSRDF());

  setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
}

/* return NULL on failure */
moveit::core::RobotModelPtr PlanningScene::createRobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model,
                                                            const srdf::ModelConstSharedPtr& srdf_model)
{
  moveit::core::RobotModelPtr robot_model(new moveit::core::RobotModel(urdf_model, srdf_model));
  if (!robot_model || !robot_model->getRootJoint())
    return moveit::core::RobotModelPtr();

  return robot_model;
}

PlanningScene::PlanningScene(const PlanningSceneConstPtr& parent) : parent_(parent)
{
  if (!parent_)
    throw moveit::ConstructException("NULL parent pointer for planning scene");

  if (!parent_->getName().empty())
    name_ = parent_->getName() + "+";

  robot_model_ = parent_->robot_model_;

  // maintain a separate world.  Copy on write ensures that most of the object
  // info is shared until it is modified.
  world_ = std::make_shared<collision_detection::World>(*parent_->world_);
  world_const_ = world_;

  // record changes to the world
  world_diff_ = std::make_shared<collision_detection::WorldDiff>(world_);

  // Set up the same collision detectors as the parent
  for (const std::pair<const std::string, CollisionDetectorPtr>& it : parent_->collision_)
  {
    const CollisionDetectorPtr& parent_detector = it.second;
    CollisionDetectorPtr& detector = collision_[it.first];
    detector = std::make_shared<CollisionDetector>();
    detector->alloc_ = parent_detector->alloc_;
    detector->parent_ = parent_detector;

    detector->cenv_ = detector->alloc_->allocateEnv(parent_detector->cenv_, world_);
    detector->cenv_const_ = detector->cenv_;

    detector->cenv_unpadded_ = detector->alloc_->allocateEnv(parent_detector->cenv_unpadded_, world_);
    detector->cenv_unpadded_const_ = detector->cenv_unpadded_;
  }
  setActiveCollisionDetector(parent_->getActiveCollisionDetectorName());
}

PlanningScenePtr PlanningScene::clone(const PlanningSceneConstPtr& scene)
{
  PlanningScenePtr result = scene->diff();
  result->decoupleParent();
  result->setName(scene->getName());
  return result;
}

PlanningScenePtr PlanningScene::diff() const
{
  return PlanningScenePtr(new PlanningScene(shared_from_this()));
}

PlanningScenePtr PlanningScene::diff(const moveit_msgs::PlanningScene& msg) const
{
  PlanningScenePtr result = diff();
  result->setPlanningSceneDiffMsg(msg);
  return result;
}

void PlanningScene::CollisionDetector::copyPadding(const PlanningScene::CollisionDetector& src)
{
  cenv_->setLinkPadding(src.getCollisionEnv()->getLinkPadding());
  cenv_->setLinkScale(src.getCollisionEnv()->getLinkScale());
}

void PlanningScene::propogateRobotPadding()
{
  for (std::pair<const std::string, CollisionDetectorPtr>& it : collision_)
  {
    if (it.second != active_collision_)
      it.second->copyPadding(*active_collision_);
  }
}

void PlanningScene::CollisionDetector::findParent(const PlanningScene& scene)
{
  if (parent_ || !scene.parent_)
    return;

  CollisionDetectorConstIterator it = scene.parent_->collision_.find(alloc_->getName());
  if (it != scene.parent_->collision_.end())
    parent_ = it->second->parent_;
}

void PlanningScene::addCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator)
{
  const std::string& name = allocator->getName();
  CollisionDetectorPtr& detector = collision_[name];

  if (detector)  // already added this one
    return;

  detector = std::make_shared<CollisionDetector>();

  detector->alloc_ = allocator;

  if (!active_collision_)
    active_collision_ = detector;

  detector->findParent(*this);

  detector->cenv_ = detector->alloc_->allocateEnv(world_, getRobotModel());
  detector->cenv_const_ = detector->cenv_;

  // if the current active detector is not the added one, copy its padding to the new one and allocate unpadded
  if (detector != active_collision_)
  {
    detector->cenv_unpadded_ = detector->alloc_->allocateEnv(world_, getRobotModel());
    detector->cenv_unpadded_const_ = detector->cenv_unpadded_;

    detector->copyPadding(*active_collision_);
  }

  detector->cenv_unpadded_ = detector->alloc_->allocateEnv(world_, getRobotModel());
  detector->cenv_unpadded_const_ = detector->cenv_unpadded_;
}

void PlanningScene::setActiveCollisionDetector(const collision_detection::CollisionDetectorAllocatorPtr& allocator,
                                               bool exclusive)
{
  if (exclusive)
  {
    CollisionDetectorPtr p;
    CollisionDetectorIterator it = collision_.find(allocator->getName());
    if (it != collision_.end())
      p = it->second;

    collision_.clear();
    active_collision_.reset();

    if (p)
    {
      collision_[allocator->getName()] = p;
      active_collision_ = p;
      return;
    }
  }

  addCollisionDetector(allocator);
  setActiveCollisionDetector(allocator->getName());
}

bool PlanningScene::setActiveCollisionDetector(const std::string& collision_detector_name)
{
  CollisionDetectorIterator it = collision_.find(collision_detector_name);
  if (it != collision_.end())
  {
    active_collision_ = it->second;
    return true;
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME,
                    "Cannot setActiveCollisionDetector to '%s' -- it has been added to PlanningScene. "
                    "Keeping existing active collision detector '%s'",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return false;
  }
}

void PlanningScene::getCollisionDetectorNames(std::vector<std::string>& names) const
{
  names.clear();
  names.reserve(collision_.size());
  for (const std::pair<const std::string, CollisionDetectorPtr>& it : collision_)
    names.push_back(it.first);
}

const collision_detection::CollisionEnvConstPtr&
PlanningScene::getCollisionEnv(const std::string& collision_detector_name) const
{
  CollisionDetectorConstIterator it = collision_.find(collision_detector_name);
  if (it == collision_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "Could not get CollisionRobot named '%s'.  Returning active CollisionRobot '%s' instead",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return active_collision_->getCollisionEnv();
  }

  return it->second->getCollisionEnv();
}

const collision_detection::CollisionEnvConstPtr&
PlanningScene::getCollisionEnvUnpadded(const std::string& collision_detector_name) const
{
  CollisionDetectorConstIterator it = collision_.find(collision_detector_name);
  if (it == collision_.end())
  {
    ROS_ERROR_NAMED(LOGNAME,
                    "Could not get CollisionRobotUnpadded named '%s'. "
                    "Returning active CollisionRobotUnpadded '%s' instead",
                    collision_detector_name.c_str(), active_collision_->alloc_->getName().c_str());
    return active_collision_->getCollisionEnvUnpadded();
  }

  return it->second->getCollisionEnvUnpadded();
}

void PlanningScene::clearDiffs()
{
  if (!parent_)
    return;

  // clear everything, reset the world, record diffs
  world_ = std::make_shared<collision_detection::World>(*parent_->world_);
  world_const_ = world_;
  world_diff_ = std::make_shared<collision_detection::WorldDiff>(world_);
  if (current_world_object_update_callback_)
    current_world_object_update_observer_handle_ = world_->addObserver(current_world_object_update_callback_);

  // use parent crobot_ if it exists.  Otherwise copy padding from parent.
  for (std::pair<const std::string, CollisionDetectorPtr>& it : collision_)
  {
    if (!it.second->parent_)
      it.second->findParent(*this);

    if (it.second->parent_)
    {
      it.second->cenv_unpadded_ = it.second->alloc_->allocateEnv(it.second->parent_->cenv_unpadded_, world_);
      it.second->cenv_unpadded_const_ = it.second->cenv_unpadded_;

      it.second->cenv_ = it.second->alloc_->allocateEnv(it.second->parent_->cenv_, world_);
      it.second->cenv_const_ = it.second->cenv_;
    }
    else  // This is the parent CollisionDetector
    {
      it.second->copyPadding(*parent_->active_collision_);
      it.second->cenv_const_ = it.second->cenv_;
    }
  }

  scene_transforms_.reset();
  robot_state_.reset();
  acm_.reset();
  object_colors_.reset();
  object_types_.reset();
}

void PlanningScene::pushDiffs(const PlanningScenePtr& scene)
{
  if (!parent_)
    return;

  if (scene_transforms_)
    scene->getTransformsNonConst().setAllTransforms(scene_transforms_->getAllTransforms());

  if (robot_state_)
  {
    scene->getCurrentStateNonConst() = *robot_state_;
    // push colors and types for attached objects
    std::vector<const moveit::core::AttachedBody*> attached_objs;
    robot_state_->getAttachedBodies(attached_objs);
    for (const moveit::core::AttachedBody* attached_obj : attached_objs)
    {
      if (hasObjectType(attached_obj->getName()))
        scene->setObjectType(attached_obj->getName(), getObjectType(attached_obj->getName()));
      if (hasObjectColor(attached_obj->getName()))
        scene->setObjectColor(attached_obj->getName(), getObjectColor(attached_obj->getName()));
    }
  }

  if (acm_)
    scene->getAllowedCollisionMatrixNonConst() = *acm_;

  collision_detection::CollisionEnvPtr active_cenv = scene->getCollisionEnvNonConst();
  active_cenv->setLinkPadding(active_collision_->cenv_->getLinkPadding());
  active_cenv->setLinkScale(active_collision_->cenv_->getLinkScale());
  scene->propogateRobotPadding();

  if (world_diff_)
  {
    for (const std::pair<const std::string, collision_detection::World::Action>& it : *world_diff_)
    {
      if (it.second == collision_detection::World::DESTROY)
      {
        scene->world_->removeObject(it.first);
        scene->removeObjectColor(it.first);
        scene->removeObjectType(it.first);
      }
      else
      {
        const collision_detection::World::Object& obj = *world_->getObject(it.first);
        scene->world_->removeObject(obj.id_);
        scene->world_->addToObject(obj.id_, obj.pose_, obj.shapes_, obj.shape_poses_);
        if (hasObjectColor(it.first))
          scene->setObjectColor(it.first, getObjectColor(it.first));
        if (hasObjectType(it.first))
          scene->setObjectType(it.first, getObjectType(it.first));

        scene->world_->setSubframesOfObject(obj.id_, obj.subframe_poses_);
      }
    }
  }
}

void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    checkCollision(req, res, getCurrentStateNonConst());
  else
    checkCollision(req, res, getCurrentState());
}

void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res,
                                   const moveit::core::RobotState& robot_state) const
{
  checkCollision(req, res, robot_state, getAllowedCollisionMatrix());
}

void PlanningScene::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    checkSelfCollision(req, res, getCurrentStateNonConst());
  else
    checkSelfCollision(req, res, getCurrentState());
}

void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res,
                                   const moveit::core::RobotState& robot_state,
                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the padded version
  getCollisionEnv()->checkRobotCollision(req, res, robot_state, acm);

  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    getCollisionEnvUnpadded()->checkSelfCollision(req, res, robot_state, acm);
}

void PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                                           collision_detection::CollisionResult& res)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    checkCollisionUnpadded(req, res, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    checkCollisionUnpadded(req, res, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::checkCollisionUnpadded(const collision_detection::CollisionRequest& req,
                                           collision_detection::CollisionResult& res,
                                           const moveit::core::RobotState& robot_state,
                                           const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the unpadded version
  getCollisionEnvUnpadded()->checkRobotCollision(req, res, robot_state, acm);

  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
  {
    getCollisionEnvUnpadded()->checkSelfCollision(req, res, robot_state, acm);
  }
}

void PlanningScene::getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    getCollidingPairs(contacts, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    getCollidingPairs(contacts, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                                      const moveit::core::RobotState& robot_state,
                                      const collision_detection::AllowedCollisionMatrix& acm,
                                      const std::string& group_name) const
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = getRobotModel()->getLinkModelsWithCollisionGeometry().size() + 1;
  req.max_contacts_per_pair = 1;
  req.group_name = group_name;
  collision_detection::CollisionResult res;
  checkCollision(req, res, robot_state, acm);
  res.contacts.swap(contacts);
}

void PlanningScene::getCollidingLinks(std::vector<std::string>& links)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    getCollidingLinks(links, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    getCollidingLinks(links, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::getCollidingLinks(std::vector<std::string>& links, const moveit::core::RobotState& robot_state,
                                      const collision_detection::AllowedCollisionMatrix& acm) const
{
  collision_detection::CollisionResult::ContactMap contacts;
  getCollidingPairs(contacts, robot_state, acm);
  links.clear();
  for (collision_detection::CollisionResult::ContactMap::const_iterator it = contacts.begin(); it != contacts.end();
       ++it)
    for (const collision_detection::Contact& contact : it->second)
    {
      if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(contact.body_name_1);
      if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_LINK)
        links.push_back(contact.body_name_2);
    }
}

const collision_detection::CollisionEnvPtr& PlanningScene::getCollisionEnvNonConst()
{
  return active_collision_->cenv_;
}

moveit::core::RobotState& PlanningScene::getCurrentStateNonConst()
{
  if (!robot_state_)
  {
    robot_state_ = std::make_shared<moveit::core::RobotState>(parent_->getCurrentState());
    robot_state_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
  }
  robot_state_->update();
  return *robot_state_;
}

moveit::core::RobotStatePtr PlanningScene::getCurrentStateUpdated(const moveit_msgs::RobotState& update) const
{
  moveit::core::RobotStatePtr state(new moveit::core::RobotState(getCurrentState()));
  moveit::core::robotStateMsgToRobotState(getTransforms(), update, *state);
  return state;
}

void PlanningScene::setAttachedBodyUpdateCallback(const moveit::core::AttachedBodyCallback& callback)
{
  current_state_attached_body_callback_ = callback;
  if (robot_state_)
    robot_state_->setAttachedBodyUpdateCallback(callback);
}

void PlanningScene::setCollisionObjectUpdateCallback(const collision_detection::World::ObserverCallbackFn& callback)
{
  if (current_world_object_update_callback_)
    world_->removeObserver(current_world_object_update_observer_handle_);
  if (callback)
    current_world_object_update_observer_handle_ = world_->addObserver(callback);
  current_world_object_update_callback_ = callback;
}

collision_detection::AllowedCollisionMatrix& PlanningScene::getAllowedCollisionMatrixNonConst()
{
  if (!acm_)
    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(parent_->getAllowedCollisionMatrix());
  return *acm_;
}

const moveit::core::Transforms& PlanningScene::getTransforms()
{
  // Trigger an update of the robot transforms
  getCurrentStateNonConst().update();
  return static_cast<const PlanningScene*>(this)->getTransforms();
}

moveit::core::Transforms& PlanningScene::getTransformsNonConst()
{
  // Trigger an update of the robot transforms
  getCurrentStateNonConst().update();
  if (!scene_transforms_)
  {
    // The only case when there are no transforms is if this planning scene has a parent. When a non-const version of
    // the planning scene is requested, a copy of the parent's transforms is forced
    scene_transforms_ = std::make_shared<SceneTransforms>(this);
    scene_transforms_->setAllTransforms(parent_->getTransforms().getAllTransforms());
  }
  return *scene_transforms_;
}

void PlanningScene::getPlanningSceneDiffMsg(moveit_msgs::PlanningScene& scene_msg) const
{
  scene_msg.name = name_;
  scene_msg.robot_model_name = getRobotModel()->getName();
  scene_msg.is_diff = true;

  if (scene_transforms_)
    scene_transforms_->copyTransforms(scene_msg.fixed_frame_transforms);
  else
    scene_msg.fixed_frame_transforms.clear();

  if (robot_state_)
    moveit::core::robotStateToRobotStateMsg(*robot_state_, scene_msg.robot_state);
  else
    scene_msg.robot_state = moveit_msgs::RobotState();
  scene_msg.robot_state.is_diff = true;

  if (acm_)
    acm_->getMessage(scene_msg.allowed_collision_matrix);
  else
    scene_msg.allowed_collision_matrix = moveit_msgs::AllowedCollisionMatrix();

  active_collision_->cenv_->getPadding(scene_msg.link_padding);
  active_collision_->cenv_->getScale(scene_msg.link_scale);

  scene_msg.object_colors.clear();
  if (object_colors_)
  {
    unsigned int i = 0;
    scene_msg.object_colors.resize(object_colors_->size());
    for (ObjectColorMap::const_iterator it = object_colors_->begin(); it != object_colors_->end(); ++it, ++i)
    {
      scene_msg.object_colors[i].id = it->first;
      scene_msg.object_colors[i].color = it->second;
    }
  }

  scene_msg.world.collision_objects.clear();
  scene_msg.world.octomap = octomap_msgs::OctomapWithPose();

  if (world_diff_)
  {
    bool do_omap = false;
    for (const std::pair<const std::string, collision_detection::World::Action>& it : *world_diff_)
    {
      if (it.first == OCTOMAP_NS)
        do_omap = true;
      else if (it.second == collision_detection::World::DESTROY)
      {
        // if object became attached, it should not be recorded as removed here
        if (!std::count_if(scene_msg.robot_state.attached_collision_objects.cbegin(),
                           scene_msg.robot_state.attached_collision_objects.cend(),
                           [&it](const moveit_msgs::AttachedCollisionObject& aco) {
                             return aco.object.id == it.first &&
                                    aco.object.operation == moveit_msgs::CollisionObject::ADD;
                           }))
        {
          moveit_msgs::CollisionObject co;
          co.header.frame_id = getPlanningFrame();
          co.id = it.first;
          co.operation = moveit_msgs::CollisionObject::REMOVE;
          scene_msg.world.collision_objects.push_back(co);
        }
      }
      else
      {
        scene_msg.world.collision_objects.emplace_back();
        getCollisionObjectMsg(scene_msg.world.collision_objects.back(), it.first);
      }
    }
    if (do_omap)
      getOctomapMsg(scene_msg.world.octomap);
  }

  // Ensure all detached collision objects actually get removed when applying the diff
  // Because RobotState doesn't handle diffs (yet), we explicitly declare attached objects
  // as removed, if they show up as "normal" collision objects but were attached in parent
  for (const auto& collision_object : scene_msg.world.collision_objects)
  {
    if (parent_ && parent_->getCurrentState().hasAttachedBody(collision_object.id))
    {
      moveit_msgs::AttachedCollisionObject aco;
      aco.object.id = collision_object.id;
      aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
      scene_msg.robot_state.attached_collision_objects.push_back(aco);
    }
  }
}

namespace
{
class ShapeVisitorAddToCollisionObject : public boost::static_visitor<void>
{
public:
  ShapeVisitorAddToCollisionObject(moveit_msgs::CollisionObject* obj) : boost::static_visitor<void>(), obj_(obj)
  {
  }

  void setPoseMessage(const geometry_msgs::Pose* pose)
  {
    pose_ = pose;
  }

  void operator()(const shape_msgs::Plane& shape_msg) const
  {
    obj_->planes.push_back(shape_msg);
    obj_->plane_poses.push_back(*pose_);
  }

  void operator()(const shape_msgs::Mesh& shape_msg) const
  {
    obj_->meshes.push_back(shape_msg);
    obj_->mesh_poses.push_back(*pose_);
  }

  void operator()(const shape_msgs::SolidPrimitive& shape_msg) const
  {
    obj_->primitives.push_back(shape_msg);
    obj_->primitive_poses.push_back(*pose_);
  }

private:
  moveit_msgs::CollisionObject* obj_;
  const geometry_msgs::Pose* pose_;
};
}  // namespace

bool PlanningScene::getCollisionObjectMsg(moveit_msgs::CollisionObject& collision_obj, const std::string& ns) const
{
  collision_detection::CollisionEnv::ObjectConstPtr obj = world_->getObject(ns);
  if (!obj)
    return false;
  collision_obj.header.frame_id = getPlanningFrame();
  collision_obj.pose = tf2::toMsg(obj->pose_);
  collision_obj.id = ns;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  ShapeVisitorAddToCollisionObject sv(&collision_obj);
  for (std::size_t j = 0; j < obj->shapes_.size(); ++j)
  {
    shapes::ShapeMsg sm;
    if (constructMsgFromShape(obj->shapes_[j].get(), sm))
    {
      geometry_msgs::Pose p = tf2::toMsg(obj->shape_poses_[j]);
      sv.setPoseMessage(&p);
      boost::apply_visitor(sv, sm);
    }
  }

  if (!collision_obj.primitives.empty() || !collision_obj.meshes.empty() || !collision_obj.planes.empty())
  {
    if (hasObjectType(collision_obj.id))
      collision_obj.type = getObjectType(collision_obj.id);
  }
  for (const auto& frame_pair : obj->subframe_poses_)
  {
    collision_obj.subframe_names.push_back(frame_pair.first);
    geometry_msgs::Pose p;
    p = tf2::toMsg(frame_pair.second);
    collision_obj.subframe_poses.push_back(p);
  }

  return true;
}

void PlanningScene::getCollisionObjectMsgs(std::vector<moveit_msgs::CollisionObject>& collision_objs) const
{
  collision_objs.clear();
  const std::vector<std::string>& ids = world_->getObjectIds();
  for (const std::string& id : ids)
    if (id != OCTOMAP_NS)
    {
      collision_objs.emplace_back();
      getCollisionObjectMsg(collision_objs.back(), id);
    }
}

bool PlanningScene::getAttachedCollisionObjectMsg(moveit_msgs::AttachedCollisionObject& attached_collision_obj,
                                                  const std::string& ns) const
{
  std::vector<moveit_msgs::AttachedCollisionObject> attached_collision_objs;
  getAttachedCollisionObjectMsgs(attached_collision_objs);
  for (moveit_msgs::AttachedCollisionObject& it : attached_collision_objs)
  {
    if (it.object.id == ns)
    {
      attached_collision_obj = it;
      return true;
    }
  }
  return false;
}

void PlanningScene::getAttachedCollisionObjectMsgs(
    std::vector<moveit_msgs::AttachedCollisionObject>& attached_collision_objs) const
{
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  getCurrentState().getAttachedBodies(attached_bodies);
  attachedBodiesToAttachedCollisionObjectMsgs(attached_bodies, attached_collision_objs);
}

bool PlanningScene::getOctomapMsg(octomap_msgs::OctomapWithPose& octomap) const
{
  octomap.header.frame_id = getPlanningFrame();
  octomap.octomap = octomap_msgs::Octomap();

  collision_detection::CollisionEnv::ObjectConstPtr map = world_->getObject(OCTOMAP_NS);
  if (map)
  {
    if (map->shapes_.size() == 1)
    {
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      octomap_msgs::fullMapToMsg(*o->octree, octomap.octomap);
      octomap.origin = tf2::toMsg(map->shape_poses_[0]);
      return true;
    }
    ROS_ERROR_NAMED(LOGNAME, "Unexpected number of shapes in octomap collision object. Not including '%s' object",
                    OCTOMAP_NS.c_str());
  }
  return false;
}

void PlanningScene::getObjectColorMsgs(std::vector<moveit_msgs::ObjectColor>& object_colors) const
{
  object_colors.clear();

  unsigned int i = 0;
  ObjectColorMap cmap;
  getKnownObjectColors(cmap);
  object_colors.resize(cmap.size());
  for (ObjectColorMap::const_iterator it = cmap.begin(); it != cmap.end(); ++it, ++i)
  {
    object_colors[i].id = it->first;
    object_colors[i].color = it->second;
  }
}

void PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene& scene_msg) const
{
  scene_msg.name = name_;
  scene_msg.is_diff = false;
  scene_msg.robot_model_name = getRobotModel()->getName();
  getTransforms().copyTransforms(scene_msg.fixed_frame_transforms);

  moveit::core::robotStateToRobotStateMsg(getCurrentState(), scene_msg.robot_state);
  getAllowedCollisionMatrix().getMessage(scene_msg.allowed_collision_matrix);
  getCollisionEnv()->getPadding(scene_msg.link_padding);
  getCollisionEnv()->getScale(scene_msg.link_scale);

  getObjectColorMsgs(scene_msg.object_colors);

  // add collision objects
  getCollisionObjectMsgs(scene_msg.world.collision_objects);

  // get the octomap
  getOctomapMsg(scene_msg.world.octomap);
}

void PlanningScene::getPlanningSceneMsg(moveit_msgs::PlanningScene& scene_msg,
                                        const moveit_msgs::PlanningSceneComponents& comp) const
{
  scene_msg.is_diff = false;
  if (comp.components & moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS)
  {
    scene_msg.name = name_;
    scene_msg.robot_model_name = getRobotModel()->getName();
  }

  if (comp.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
    getTransforms().copyTransforms(scene_msg.fixed_frame_transforms);

  if (comp.components & moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS)
  {
    moveit::core::robotStateToRobotStateMsg(getCurrentState(), scene_msg.robot_state, true);
    for (moveit_msgs::AttachedCollisionObject& attached_collision_object :
         scene_msg.robot_state.attached_collision_objects)
    {
      if (hasObjectType(attached_collision_object.object.id))
      {
        attached_collision_object.object.type = getObjectType(attached_collision_object.object.id);
      }
    }
  }
  else if (comp.components & moveit_msgs::PlanningSceneComponents::ROBOT_STATE)
  {
    moveit::core::robotStateToRobotStateMsg(getCurrentState(), scene_msg.robot_state, false);
  }

  if (comp.components & moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX)
    getAllowedCollisionMatrix().getMessage(scene_msg.allowed_collision_matrix);

  if (comp.components & moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING)
  {
    getCollisionEnv()->getPadding(scene_msg.link_padding);
    getCollisionEnv()->getScale(scene_msg.link_scale);
  }

  if (comp.components & moveit_msgs::PlanningSceneComponents::OBJECT_COLORS)
    getObjectColorMsgs(scene_msg.object_colors);

  // add collision objects
  if (comp.components & moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY)
    getCollisionObjectMsgs(scene_msg.world.collision_objects);
  else if (comp.components & moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES)
  {
    const std::vector<std::string>& ids = world_->getObjectIds();
    scene_msg.world.collision_objects.clear();
    scene_msg.world.collision_objects.reserve(ids.size());
    for (const std::string& id : ids)
      if (id != OCTOMAP_NS)
      {
        moveit_msgs::CollisionObject co;
        co.id = id;
        if (hasObjectType(co.id))
          co.type = getObjectType(co.id);
        scene_msg.world.collision_objects.push_back(co);
      }
  }

  // get the octomap
  if (comp.components & moveit_msgs::PlanningSceneComponents::OCTOMAP)
    getOctomapMsg(scene_msg.world.octomap);
}

void PlanningScene::saveGeometryToStream(std::ostream& out) const
{
  out << name_ << std::endl;
  const std::vector<std::string>& ids = world_->getObjectIds();
  for (const std::string& id : ids)
    if (id != OCTOMAP_NS)
    {
      collision_detection::CollisionEnv::ObjectConstPtr obj = world_->getObject(id);
      if (obj)
      {
        out << "* " << id << std::endl;  // New object start
        // Write object pose
        writePoseToText(out, obj->pose_);

        // Write shapes and shape poses
        out << obj->shapes_.size() << std::endl;  // Number of shapes
        for (std::size_t j = 0; j < obj->shapes_.size(); ++j)
        {
          shapes::saveAsText(obj->shapes_[j].get(), out);
          // shape_poses_ is valid isometry by contract
          writePoseToText(out, obj->shape_poses_[j]);
          if (hasObjectColor(id))
          {
            const std_msgs::ColorRGBA& c = getObjectColor(id);
            out << c.r << " " << c.g << " " << c.b << " " << c.a << std::endl;
          }
          else
            out << "0 0 0 0" << std::endl;
        }

        // Write subframes
        out << obj->subframe_poses_.size() << std::endl;  // Number of subframes
        for (auto& pose_pair : obj->subframe_poses_)
        {
          out << pose_pair.first << std::endl;     // Subframe name
          writePoseToText(out, pose_pair.second);  // Subframe pose
        }
      }
    }
  out << "." << std::endl;
}

bool PlanningScene::loadGeometryFromStream(std::istream& in)
{
  return loadGeometryFromStream(in, Eigen::Isometry3d::Identity());  // Use no offset
}

bool PlanningScene::loadGeometryFromStream(std::istream& in, const Eigen::Isometry3d& offset)
{
  if (!in.good() || in.eof())
  {
    ROS_ERROR_NAMED(LOGNAME, "Bad input stream when loading scene geometry");
    return false;
  }
  // Read scene name
  std::getline(in, name_);

  // Identify scene format version for backwards compatibility of parser
  auto pos = in.tellg();  // remember current stream position
  std::string line;
  do
  {
    std::getline(in, line);
  } while (in.good() && !in.eof() && (line.empty() || line[0] != '*'));  // read * marker
  std::getline(in, line);                                                // next line determines format
  boost::algorithm::trim(line);
  // new format: line specifies position of object, with spaces as delimiter -> spaces indicate new format
  // old format: line specifies number of shapes
  bool uses_new_scene_format = line.find(' ') != std::string::npos;
  in.seekg(pos);

  Eigen::Isometry3d pose;  // Transient
  do
  {
    std::string marker;
    in >> marker;
    if (!in.good() || in.eof())
    {
      ROS_ERROR_NAMED(LOGNAME, "Bad input stream when loading marker in scene geometry");
      return false;
    }
    if (marker == "*")  // Start of new object
    {
      std::string object_id;
      std::getline(in, object_id);
      if (!in.good() || in.eof())
      {
        ROS_ERROR_NAMED(LOGNAME, "Bad input stream when loading object_id in scene geometry");
        return false;
      }
      boost::algorithm::trim(object_id);

      // Read in object pose (added in the new scene format)
      pose.setIdentity();
      if (uses_new_scene_format && !readPoseFromText(in, pose))
      {
        ROS_ERROR_NAMED(LOGNAME, "Failed to read object pose from scene file");
        return false;
      }
      Eigen::Isometry3d object_pose = offset * pose;  // Transform pose by input pose offset

      // Read in shapes
      unsigned int shape_count;
      in >> shape_count;
      if (shape_count)  // If there are any shapes to be loaded, clear any existing object first
        world_->removeObject(object_id);
      for (std::size_t i = 0; i < shape_count && in.good() && !in.eof(); ++i)
      {
        const auto shape = shapes::ShapeConstPtr(shapes::constructShapeFromText(in));
        if (!shape)
        {
          ROS_ERROR_NAMED(LOGNAME, "Failed to load shape from scene file");
          return false;
        }
        if (!readPoseFromText(in, pose))
        {
          ROS_ERROR_NAMED(LOGNAME, "Failed to read pose from scene file");
          return false;
        }
        float r, g, b, a;
        if (!(in >> r >> g >> b >> a))
        {
          ROS_ERROR_NAMED(LOGNAME, "Improperly formatted color in scene geometry file");
          return false;
        }
        if (shape)
        {
          world_->addToObject(object_id, shape, pose);
          if (r > 0.0f || g > 0.0f || b > 0.0f || a > 0.0f)
          {
            std_msgs::ColorRGBA color;
            color.r = r;
            color.g = g;
            color.b = b;
            color.a = a;
            setObjectColor(object_id, color);
          }
        }
      }

      // Finally set object's pose once
      world_->setObjectPose(object_id, object_pose);

      // Read in subframes (added in the new scene format)
      if (uses_new_scene_format)
      {
        moveit::core::FixedTransformsMap subframes;
        unsigned int subframe_count;
        in >> subframe_count;
        for (std::size_t i = 0; i < subframe_count && in.good() && !in.eof(); ++i)
        {
          std::string subframe_name;
          in >> subframe_name;
          if (!readPoseFromText(in, pose))
          {
            ROS_ERROR_NAMED(LOGNAME, "Failed to read subframe pose from scene file");
            return false;
          }
          subframes[subframe_name] = pose;
        }
        world_->setSubframesOfObject(object_id, subframes);
      }
    }
    else if (marker == ".")
    {
      // Marks the end of the scene geometry;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown marker in scene geometry file: " << marker);
      return false;
    }
  } while (true);
}

bool PlanningScene::readPoseFromText(std::istream& in, Eigen::Isometry3d& pose) const
{
  double x, y, z, rx, ry, rz, rw;
  if (!(in >> x >> y >> z))
  {
    ROS_ERROR_NAMED(LOGNAME, "Improperly formatted translation in scene geometry file");
    return false;
  }
  if (!(in >> rx >> ry >> rz >> rw))
  {
    ROS_ERROR_NAMED(LOGNAME, "Improperly formatted rotation in scene geometry file");
    return false;
  }
  pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(rw, rx, ry, rz);
  return true;
}

void PlanningScene::writePoseToText(std::ostream& out, const Eigen::Isometry3d& pose) const
{
  out << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << std::endl;
  Eigen::Quaterniond r(pose.linear());
  out << r.x() << " " << r.y() << " " << r.z() << " " << r.w() << std::endl;
}

void PlanningScene::setCurrentState(const moveit_msgs::RobotState& state)
{
  // The attached bodies will be processed separately by processAttachedCollisionObjectMsgs
  // after robot_state_ has been updated
  moveit_msgs::RobotState state_no_attached(state);
  state_no_attached.attached_collision_objects.clear();

  if (parent_)
  {
    if (!robot_state_)
    {
      robot_state_ = std::make_shared<moveit::core::RobotState>(parent_->getCurrentState());
      robot_state_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
    }
    moveit::core::robotStateMsgToRobotState(getTransforms(), state_no_attached, *robot_state_);
  }
  else
    moveit::core::robotStateMsgToRobotState(*scene_transforms_, state_no_attached, *robot_state_);

  for (std::size_t i = 0; i < state.attached_collision_objects.size(); ++i)
  {
    if (!state.is_diff && state.attached_collision_objects[i].object.operation != moveit_msgs::CollisionObject::ADD)
    {
      ROS_ERROR_NAMED(LOGNAME,
                      "The specified RobotState is not marked as is_diff. "
                      "The request to modify the object '%s' is not supported. Object is ignored.",
                      state.attached_collision_objects[i].object.id.c_str());
      continue;
    }
    processAttachedCollisionObjectMsg(state.attached_collision_objects[i]);
  }
}

void PlanningScene::setCurrentState(const moveit::core::RobotState& state)
{
  getCurrentStateNonConst() = state;
}

void PlanningScene::decoupleParent()
{
  if (!parent_)
    return;

  // This child planning scene did not have its own copy of frame transforms
  if (!scene_transforms_)
  {
    scene_transforms_ = std::make_shared<SceneTransforms>(this);
    scene_transforms_->setAllTransforms(parent_->getTransforms().getAllTransforms());
  }

  if (!robot_state_)
  {
    robot_state_ = std::make_shared<moveit::core::RobotState>(parent_->getCurrentState());
    robot_state_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
  }

  if (!acm_)
    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(parent_->getAllowedCollisionMatrix());

  for (std::pair<const std::string, CollisionDetectorPtr>& it : collision_)
  {
    it.second->parent_.reset();
  }
  world_diff_.reset();

  if (!object_colors_)
  {
    ObjectColorMap kc;
    parent_->getKnownObjectColors(kc);
    object_colors_ = std::make_unique<ObjectColorMap>(kc);
  }
  else
  {
    ObjectColorMap kc;
    parent_->getKnownObjectColors(kc);
    for (ObjectColorMap::const_iterator it = kc.begin(); it != kc.end(); ++it)
      if (object_colors_->find(it->first) == object_colors_->end())
        (*object_colors_)[it->first] = it->second;
  }

  if (!object_types_)
  {
    ObjectTypeMap kc;
    parent_->getKnownObjectTypes(kc);
    object_types_ = std::make_unique<ObjectTypeMap>(kc);
  }
  else
  {
    ObjectTypeMap kc;
    parent_->getKnownObjectTypes(kc);
    for (ObjectTypeMap::const_iterator it = kc.begin(); it != kc.end(); ++it)
      if (object_types_->find(it->first) == object_types_->end())
        (*object_types_)[it->first] = it->second;
  }

  parent_.reset();
}

bool PlanningScene::setPlanningSceneDiffMsg(const moveit_msgs::PlanningScene& scene_msg)
{
  bool result = true;

  ROS_DEBUG_NAMED(LOGNAME, "Adding planning scene diff");
  if (!scene_msg.name.empty())
    name_ = scene_msg.name;

  if (!scene_msg.robot_model_name.empty() && scene_msg.robot_model_name != getRobotModel()->getName())
    ROS_WARN_NAMED(LOGNAME, "Setting the scene for model '%s' but model '%s' is loaded.",
                   scene_msg.robot_model_name.c_str(), getRobotModel()->getName().c_str());

  // there is at least one transform in the list of fixed transform: from model frame to itself;
  // if the list is empty, then nothing has been set
  if (!scene_msg.fixed_frame_transforms.empty())
  {
    if (!scene_transforms_)
      scene_transforms_ = std::make_shared<SceneTransforms>(this);
    scene_transforms_->setTransforms(scene_msg.fixed_frame_transforms);
  }

  // if at least some joints have been specified, we set them
  if (!scene_msg.robot_state.multi_dof_joint_state.joint_names.empty() ||
      !scene_msg.robot_state.joint_state.name.empty() || !scene_msg.robot_state.attached_collision_objects.empty())
    setCurrentState(scene_msg.robot_state);

  // if at least some links are mentioned in the allowed collision matrix, then we have an update
  if (!scene_msg.allowed_collision_matrix.entry_names.empty())
    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(scene_msg.allowed_collision_matrix);

  if (!scene_msg.link_padding.empty() || !scene_msg.link_scale.empty())
  {
    for (std::pair<const std::string, CollisionDetectorPtr>& it : collision_)
    {
      it.second->cenv_->setPadding(scene_msg.link_padding);
      it.second->cenv_->setScale(scene_msg.link_scale);
    }
  }

  // if any colors have been specified, replace the ones we have with the specified ones
  for (const moveit_msgs::ObjectColor& object_color : scene_msg.object_colors)
    setObjectColor(object_color.id, object_color.color);

  // process collision object updates
  for (const moveit_msgs::CollisionObject& collision_object : scene_msg.world.collision_objects)
    result &= processCollisionObjectMsg(collision_object);

  // if an octomap was specified, replace the one we have with that one
  if (!scene_msg.world.octomap.octomap.data.empty())
    processOctomapMsg(scene_msg.world.octomap);

  return result;
}

bool PlanningScene::setPlanningSceneMsg(const moveit_msgs::PlanningScene& scene_msg)
{
  ROS_DEBUG_NAMED(LOGNAME, "Setting new planning scene: '%s'", scene_msg.name.c_str());
  name_ = scene_msg.name;

  if (!scene_msg.robot_model_name.empty() && scene_msg.robot_model_name != getRobotModel()->getName())
    ROS_WARN_NAMED(LOGNAME, "Setting the scene for model '%s' but model '%s' is loaded.",
                   scene_msg.robot_model_name.c_str(), getRobotModel()->getName().c_str());

  if (parent_)
    decoupleParent();

  object_types_.reset();
  scene_transforms_->setTransforms(scene_msg.fixed_frame_transforms);
  setCurrentState(scene_msg.robot_state);
  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(scene_msg.allowed_collision_matrix);
  for (std::pair<const std::string, CollisionDetectorPtr>& it : collision_)
  {
    it.second->cenv_->setPadding(scene_msg.link_padding);
    it.second->cenv_->setScale(scene_msg.link_scale);
  }
  object_colors_ = std::make_unique<ObjectColorMap>();
  for (const moveit_msgs::ObjectColor& object_color : scene_msg.object_colors)
    setObjectColor(object_color.id, object_color.color);
  world_->clearObjects();
  return processPlanningSceneWorldMsg(scene_msg.world);
}

bool PlanningScene::processPlanningSceneWorldMsg(const moveit_msgs::PlanningSceneWorld& world)
{
  bool result = true;
  for (const moveit_msgs::CollisionObject& collision_object : world.collision_objects)
    result &= processCollisionObjectMsg(collision_object);
  processOctomapMsg(world.octomap);
  return result;
}

bool PlanningScene::usePlanningSceneMsg(const moveit_msgs::PlanningScene& scene_msg)
{
  if (scene_msg.is_diff)
    return setPlanningSceneDiffMsg(scene_msg);
  else
    return setPlanningSceneMsg(scene_msg);
}

collision_detection::OccMapTreePtr createOctomap(const octomap_msgs::Octomap& map)
{
  std::shared_ptr<collision_detection::OccMapTree> om =
      std::make_shared<collision_detection::OccMapTree>(map.resolution);
  if (map.binary)
  {
    octomap_msgs::readTree(om.get(), map);
  }
  else
  {
    std::stringstream datastream;
    if (!map.data.empty())
    {
      datastream.write((const char*)&map.data[0], map.data.size());
      om->readData(datastream);
    }
  }
  return om;
}

void PlanningScene::processOctomapMsg(const octomap_msgs::Octomap& map)
{
  // each octomap replaces any previous one
  world_->removeObject(OCTOMAP_NS);

  if (map.data.empty())
    return;

  if (map.id != "OcTree")
  {
    ROS_ERROR_NAMED(LOGNAME, "Received octomap is of type '%s' but type 'OcTree' is expected.", map.id.c_str());
    return;
  }

  std::shared_ptr<collision_detection::OccMapTree> om = createOctomap(map);
  if (!map.header.frame_id.empty())
  {
    const Eigen::Isometry3d& t = getFrameTransform(map.header.frame_id);
    world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), t);
  }
  else
  {
    world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), Eigen::Isometry3d::Identity());
  }
}

void PlanningScene::removeAllCollisionObjects()
{
  const std::vector<std::string>& object_ids = world_->getObjectIds();
  for (const std::string& object_id : object_ids)
    if (object_id != OCTOMAP_NS)
    {
      world_->removeObject(object_id);
      removeObjectColor(object_id);
      removeObjectType(object_id);
    }
}

void PlanningScene::processOctomapMsg(const octomap_msgs::OctomapWithPose& map)
{
  // each octomap replaces any previous one
  world_->removeObject(OCTOMAP_NS);

  if (map.octomap.data.empty())
    return;

  if (map.octomap.id != "OcTree")
  {
    ROS_ERROR_NAMED(LOGNAME, "Received octomap is of type '%s' but type 'OcTree' is expected.", map.octomap.id.c_str());
    return;
  }

  std::shared_ptr<collision_detection::OccMapTree> om = createOctomap(map.octomap);

  const Eigen::Isometry3d& t = getFrameTransform(map.header.frame_id);
  Eigen::Isometry3d p;
  PlanningScene::poseMsgToEigen(map.origin, p);
  p = t * p;
  world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), p);
}

void PlanningScene::processOctomapPtr(const std::shared_ptr<const octomap::OcTree>& octree, const Eigen::Isometry3d& t)
{
  collision_detection::CollisionEnv::ObjectConstPtr map = world_->getObject(OCTOMAP_NS);
  if (map)
  {
    if (map->shapes_.size() == 1)
    {
      // check to see if we have the same octree pointer & pose.
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      if (o->octree == octree)
      {
        // if the pose changed, we update it
        if (map->shape_poses_[0].isApprox(t, std::numeric_limits<double>::epsilon() * 100.0))
        {
          if (world_diff_)
            world_diff_->set(OCTOMAP_NS, collision_detection::World::DESTROY | collision_detection::World::CREATE |
                                             collision_detection::World::ADD_SHAPE);
        }
        else
        {
          shapes::ShapeConstPtr shape = map->shapes_[0];
          map.reset();  // reset this pointer first so that caching optimizations can be used in CollisionWorld
          world_->moveShapeInObject(OCTOMAP_NS, shape, t);
        }
        return;
      }
    }
  }
  // if the octree pointer changed, update the structure
  world_->removeObject(OCTOMAP_NS);
  world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(octree)), t);
}

bool PlanningScene::processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject& object)
{
  if (object.object.operation == moveit_msgs::CollisionObject::ADD && !getRobotModel()->hasLinkModel(object.link_name))
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to attach a body to link '%s' (link not found)", object.link_name.c_str());
    return false;
  }

  if (object.object.id == OCTOMAP_NS)
  {
    ROS_ERROR_NAMED(LOGNAME, "The ID '%s' cannot be used for collision objects (name reserved)", OCTOMAP_NS.c_str());
    return false;
  }

  if (!robot_state_)  // there must be a parent in this case
  {
    robot_state_ = std::make_shared<moveit::core::RobotState>(parent_->getCurrentState());
    robot_state_->setAttachedBodyUpdateCallback(current_state_attached_body_callback_);
  }
  robot_state_->update();

  // The ADD/REMOVE operations follow this order:
  // STEP 1: Get info about the object from either the message or the world/RobotState
  // STEP 2: Remove the object from the world/RobotState if necessary
  // STEP 3: Put the object in the RobotState/world

  if (object.object.operation == moveit_msgs::CollisionObject::ADD ||
      object.object.operation == moveit_msgs::CollisionObject::APPEND)
  {
    const moveit::core::LinkModel* link_model = getRobotModel()->getLinkModel(object.link_name);
    if (link_model)
    {
      // items to build the attached object from (filled from existing world object or message)
      Eigen::Isometry3d object_pose_in_link;
      std::vector<shapes::ShapeConstPtr> shapes;
      EigenSTL::vector_Isometry3d shape_poses;
      moveit::core::FixedTransformsMap subframe_poses;

      // STEP 1: Obtain info about object to be attached.
      //         If it is in the world, message contents are ignored.

      collision_detection::CollisionEnv::ObjectConstPtr obj_in_world = world_->getObject(object.object.id);
      if (object.object.operation == moveit_msgs::CollisionObject::ADD && object.object.primitives.empty() &&
          object.object.meshes.empty() && object.object.planes.empty())
      {
        if (obj_in_world)
        {
          ROS_DEBUG_NAMED(LOGNAME, "Attaching world object '%s' to link '%s'", object.object.id.c_str(),
                          object.link_name.c_str());
          object_pose_in_link = robot_state_->getGlobalLinkTransform(link_model).inverse() * obj_in_world->pose_;
          shapes = obj_in_world->shapes_;
          shape_poses = obj_in_world->shape_poses_;
          subframe_poses = obj_in_world->subframe_poses_;
        }
        else
        {
          ROS_ERROR_NAMED(LOGNAME,
                          "Attempting to attach object '%s' to link '%s' but no geometry specified "
                          "and such an object does not exist in the collision world",
                          object.object.id.c_str(), object.link_name.c_str());
          return false;
        }
      }
      else  // If object is not in the world, use the message contents
      {
        Eigen::Isometry3d header_frame_to_object_pose;
        if (!shapesAndPosesFromCollisionObjectMessage(object.object, header_frame_to_object_pose, shapes, shape_poses))
          return false;
        const Eigen::Isometry3d world_to_header_frame = getFrameTransform(object.object.header.frame_id);
        const Eigen::Isometry3d link_to_header_frame =
            robot_state_->getGlobalLinkTransform(link_model).inverse() * world_to_header_frame;
        object_pose_in_link = link_to_header_frame * header_frame_to_object_pose;

        Eigen::Isometry3d subframe_pose;
        for (std::size_t i = 0; i < object.object.subframe_poses.size(); ++i)
        {
          PlanningScene::poseMsgToEigen(object.object.subframe_poses[i], subframe_pose);
          std::string name = object.object.subframe_names[i];
          subframe_poses[name] = subframe_pose;
        }
      }

      if (shapes.empty())
      {
        ROS_ERROR_NAMED(LOGNAME, "There is no geometry to attach to link '%s' as part of attached body '%s'",
                        object.link_name.c_str(), object.object.id.c_str());
        return false;
      }

      if (!object.object.type.db.empty() || !object.object.type.key.empty())
        setObjectType(object.object.id, object.object.type);

      // STEP 2: Remove the object from the world (if it existed)
      if (obj_in_world && world_->removeObject(object.object.id))
      {
        if (object.object.operation == moveit_msgs::CollisionObject::ADD)
          ROS_DEBUG_NAMED(LOGNAME, "Removing world object with the same name as newly attached object: '%s'",
                          object.object.id.c_str());
        else
          ROS_WARN_NAMED(LOGNAME,
                         "You tried to append geometry to an attached object "
                         "that is actually a world object ('%s'). World geometry is ignored.",
                         object.object.id.c_str());
      }

      // STEP 3: Attach the object to the robot
      if (object.object.operation == moveit_msgs::CollisionObject::ADD ||
          !robot_state_->hasAttachedBody(object.object.id))
      {
        if (robot_state_->clearAttachedBody(object.object.id))
          ROS_DEBUG_NAMED(LOGNAME,
                          "The robot state already had an object named '%s' attached to link '%s'. "
                          "The object was replaced.",
                          object.object.id.c_str(), object.link_name.c_str());

        robot_state_->attachBody(object.object.id, object_pose_in_link, shapes, shape_poses, object.touch_links,
                                 object.link_name, object.detach_posture, subframe_poses);
        ROS_DEBUG_NAMED(LOGNAME, "Attached object '%s' to link '%s'", object.object.id.c_str(),
                        object.link_name.c_str());
      }
      else  // APPEND: augment to existing attached object
      {
        const moveit::core::AttachedBody* ab = robot_state_->getAttachedBody(object.object.id);

        // Allow overriding the body's pose if provided, otherwise keep the old one
        if (moveit::core::isEmpty(object.object.pose))
          object_pose_in_link = ab->getPose();  // Keep old pose

        shapes.insert(shapes.end(), ab->getShapes().begin(), ab->getShapes().end());
        shape_poses.insert(shape_poses.end(), ab->getShapePoses().begin(), ab->getShapePoses().end());
        subframe_poses.insert(ab->getSubframes().begin(), ab->getSubframes().end());
        trajectory_msgs::JointTrajectory detach_posture =
            object.detach_posture.joint_names.empty() ? ab->getDetachPosture() : object.detach_posture;

        std::set<std::string> touch_links = ab->getTouchLinks();
        touch_links.insert(std::make_move_iterator(object.touch_links.begin()),
                           std::make_move_iterator(object.touch_links.end()));

        robot_state_->clearAttachedBody(object.object.id);
        robot_state_->attachBody(object.object.id, object_pose_in_link, shapes, shape_poses, touch_links,
                                 object.link_name, detach_posture, subframe_poses);
        ROS_DEBUG_NAMED(LOGNAME, "Appended things to object '%s' attached to link '%s'", object.object.id.c_str(),
                        object.link_name.c_str());
      }
      return true;
    }
    else
      ROS_ERROR_NAMED(LOGNAME, "Robot state is not compatible with robot model. This could be fatal.");
  }
  else if (object.object.operation == moveit_msgs::CollisionObject::REMOVE)  // == DETACH
  {
    // STEP 1: Get info about the object from the RobotState
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    if (object.object.id.empty())
    {
      const moveit::core::LinkModel* link_model =
          object.link_name.empty() ? nullptr : getRobotModel()->getLinkModel(object.link_name);
      if (link_model)  // if we have a link model specified, only fetch bodies attached to this link
        robot_state_->getAttachedBodies(attached_bodies, link_model);
      else
        robot_state_->getAttachedBodies(attached_bodies);
    }
    else  // A specific object id will be removed.
    {
      const moveit::core::AttachedBody* body = robot_state_->getAttachedBody(object.object.id);
      if (body)
      {
        if (!object.link_name.empty() && (body->getAttachedLinkName() != object.link_name))
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "The AttachedCollisionObject message states the object is attached to "
                                              << object.link_name << ", but it is actually attached to "
                                              << body->getAttachedLinkName()
                                              << ". Leave the link_name empty or specify the correct link.");
          return false;
        }
        attached_bodies.push_back(body);
      }
    }

    // STEP 2+3: Remove the attached object(s) from the RobotState and put them in the world
    for (const moveit::core::AttachedBody* attached_body : attached_bodies)
    {
      const std::string& name = attached_body->getName();
      if (world_->hasObject(name))
        ROS_WARN_NAMED(LOGNAME,
                       "The collision world already has an object with the same name as the body about to be detached. "
                       "NOT adding the detached body '%s' to the collision world.",
                       object.object.id.c_str());
      else
      {
        const Eigen::Isometry3d& pose = attached_body->getGlobalPose();
        world_->addToObject(name, pose, attached_body->getShapes(), attached_body->getShapePoses());
        world_->setSubframesOfObject(name, attached_body->getSubframes());
        ROS_DEBUG_NAMED(LOGNAME, "Detached object '%s' from link '%s' and added it back in the collision world",
                        name.c_str(), object.link_name.c_str());
      }

      robot_state_->clearAttachedBody(name);
    }
    if (!attached_bodies.empty() || object.object.id.empty())
      return true;
  }
  else if (object.object.operation == moveit_msgs::CollisionObject::MOVE)
  {
    ROS_ERROR_NAMED(LOGNAME, "Move for attached objects not yet implemented");
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Unknown collision object operation: %d", object.object.operation);
  }

  return false;
}

bool PlanningScene::processCollisionObjectMsg(const moveit_msgs::CollisionObject& object)
{
  if (object.id == OCTOMAP_NS)
  {
    ROS_ERROR_NAMED(LOGNAME, "The ID '%s' cannot be used for collision objects (name reserved)", OCTOMAP_NS.c_str());
    return false;
  }

  if (object.operation == moveit_msgs::CollisionObject::ADD || object.operation == moveit_msgs::CollisionObject::APPEND)
  {
    return processCollisionObjectAdd(object);
  }
  else if (object.operation == moveit_msgs::CollisionObject::REMOVE)
  {
    return processCollisionObjectRemove(object);
  }
  else if (object.operation == moveit_msgs::CollisionObject::MOVE)
  {
    return processCollisionObjectMove(object);
  }

  ROS_ERROR_NAMED(LOGNAME, "Unknown collision object operation: %d", object.operation);
  return false;
}

void PlanningScene::poseMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Isometry3d& out)
{
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if ((quaternion.x() == 0) && (quaternion.y() == 0) && (quaternion.z() == 0) && (quaternion.w() == 0))
  {
    ROS_WARN_NAMED(LOGNAME, "Empty quaternion found in pose message. Setting to neutral orientation.");
    quaternion.setIdentity();
  }
  else
  {
    quaternion.normalize();
  }
  out = translation * quaternion;
}

bool PlanningScene::shapesAndPosesFromCollisionObjectMessage(const moveit_msgs::CollisionObject& object,
                                                             Eigen::Isometry3d& object_pose,
                                                             std::vector<shapes::ShapeConstPtr>& shapes,
                                                             EigenSTL::vector_Isometry3d& shape_poses)
{
  if (object.primitives.size() < object.primitive_poses.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "More primitive shape poses than shapes in collision object message.");
    return false;
  }
  if (object.meshes.size() < object.mesh_poses.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "More mesh poses than meshes in collision object message.");
    return false;
  }
  if (object.planes.size() < object.plane_poses.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "More plane poses than planes in collision object message.");
    return false;
  }

  const int num_shapes = object.primitives.size() + object.meshes.size() + object.planes.size();
  shapes.reserve(num_shapes);
  shape_poses.reserve(num_shapes);

  PlanningScene::poseMsgToEigen(object.pose, object_pose);

  bool switch_object_pose_and_shape_pose = false;
  if (num_shapes == 1)
    if (moveit::core::isEmpty(object.pose))
    {
      switch_object_pose_and_shape_pose = true;  // If the object pose is not set but the shape pose is,
                                                 // use the shape's pose as the object pose.
    }

  auto append = [&object_pose, &shapes, &shape_poses,
                 &switch_object_pose_and_shape_pose](shapes::Shape* s, const geometry_msgs::Pose& pose_msg) {
    if (!s)
      return;
    Eigen::Isometry3d pose;
    PlanningScene::poseMsgToEigen(pose_msg, pose);
    if (!switch_object_pose_and_shape_pose)
      shape_poses.emplace_back(std::move(pose));
    else
    {
      shape_poses.emplace_back(std::move(object_pose));
      object_pose = pose;
    }
    shapes.emplace_back(shapes::ShapeConstPtr(s));
  };

  auto treat_shape_vectors = [&append](const auto& shape_vector,        // the shape_msgs of each type
                                       const auto& shape_poses_vector,  // std::vector<const geometry_msgs::Pose>
                                       const std::string& shape_type) {
    if (shape_vector.size() > shape_poses_vector.size())
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Number of " << shape_type
                                                   << " does not match number of poses "
                                                      "in collision object message. Assuming identity.");
      for (std::size_t i = 0; i < shape_vector.size(); ++i)
      {
        if (i >= shape_poses_vector.size())
        {
          // Empty shape pose => Identity
          geometry_msgs::Pose identity;
          identity.orientation.w = 1.0;
          append(shapes::constructShapeFromMsg(shape_vector[i]), identity);
        }
        else
          append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_vector[i]);
      }
    }
    else
      for (std::size_t i = 0; i < shape_vector.size(); ++i)
        append(shapes::constructShapeFromMsg(shape_vector[i]), shape_poses_vector[i]);
  };

  treat_shape_vectors(object.primitives, object.primitive_poses, std::string("primitive_poses"));
  treat_shape_vectors(object.meshes, object.mesh_poses, std::string("meshes"));
  treat_shape_vectors(object.planes, object.plane_poses, std::string("planes"));
  return true;
}

bool PlanningScene::processCollisionObjectAdd(const moveit_msgs::CollisionObject& object)
{
  if (!knowsFrameTransform(object.header.frame_id))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown frame: " << object.header.frame_id);
    return false;
  }

  if (object.primitives.empty() && object.meshes.empty() && object.planes.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "There are no shapes specified in the collision object message");
    return false;
  }

  // replace the object if ADD is specified instead of APPEND
  if (object.operation == moveit_msgs::CollisionObject::ADD && world_->hasObject(object.id))
    world_->removeObject(object.id);

  const Eigen::Isometry3d& world_to_object_header_transform = getFrameTransform(object.header.frame_id);
  Eigen::Isometry3d header_to_pose_transform;
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Isometry3d shape_poses;
  if (!shapesAndPosesFromCollisionObjectMessage(object, header_to_pose_transform, shapes, shape_poses))
    return false;
  const Eigen::Isometry3d object_frame_transform = world_to_object_header_transform * header_to_pose_transform;

  world_->addToObject(object.id, object_frame_transform, shapes, shape_poses);

  if (!object.type.key.empty() || !object.type.db.empty())
    setObjectType(object.id, object.type);

  // Add subframes
  moveit::core::FixedTransformsMap subframes;
  Eigen::Isometry3d subframe_pose;
  for (std::size_t i = 0; i < object.subframe_poses.size(); ++i)
  {
    PlanningScene::poseMsgToEigen(object.subframe_poses[i], subframe_pose);
    std::string name = object.subframe_names[i];
    subframes[name] = subframe_pose;
  }
  world_->setSubframesOfObject(object.id, subframes);
  return true;
}

bool PlanningScene::processCollisionObjectRemove(const moveit_msgs::CollisionObject& object)
{
  if (object.id.empty())
  {
    removeAllCollisionObjects();
  }
  else
  {
    if (!world_->removeObject(object.id))
    {
      ROS_WARN_STREAM_NAMED(LOGNAME,
                            "Tried to remove world object '" << object.id << "', but it does not exist in this scene.");
      return false;
    }

    removeObjectColor(object.id);
    removeObjectType(object.id);
  }
  return true;
}

bool PlanningScene::processCollisionObjectMove(const moveit_msgs::CollisionObject& object)
{
  if (world_->hasObject(object.id))
  {
    if (!object.primitives.empty() || !object.meshes.empty() || !object.planes.empty())
      ROS_WARN_NAMED(LOGNAME, "Move operation for object '%s' ignores the geometry specified in the message.",
                     object.id.c_str());

    const Eigen::Isometry3d& world_to_object_header_transform = getFrameTransform(object.header.frame_id);
    Eigen::Isometry3d header_to_pose_transform;

    PlanningScene::poseMsgToEigen(object.pose, header_to_pose_transform);

    const Eigen::Isometry3d object_frame_transform = world_to_object_header_transform * header_to_pose_transform;
    world_->setObjectPose(object.id, object_frame_transform);
    return true;
  }

  ROS_ERROR_NAMED(LOGNAME, "World object '%s' does not exist. Cannot move.", object.id.c_str());
  return false;
}

const Eigen::Isometry3d& PlanningScene::getFrameTransform(const std::string& frame_id) const
{
  return getFrameTransform(getCurrentState(), frame_id);
}

const Eigen::Isometry3d& PlanningScene::getFrameTransform(const std::string& frame_id)
{
  if (getCurrentState().dirtyLinkTransforms())
    return getFrameTransform(getCurrentStateNonConst(), frame_id);
  else
    return getFrameTransform(getCurrentState(), frame_id);
}

const Eigen::Isometry3d& PlanningScene::getFrameTransform(const moveit::core::RobotState& state,
                                                          const std::string& frame_id) const
{
  if (!frame_id.empty() && frame_id[0] == '/')
    // Recursively call itself without the slash in front of frame name
    return getFrameTransform(frame_id.substr(1));

  bool frame_found;
  const Eigen::Isometry3d& t1 = state.getFrameTransform(frame_id, &frame_found);
  if (frame_found)
    return t1;

  const Eigen::Isometry3d& t2 = getWorld()->getTransform(frame_id, frame_found);
  if (frame_found)
    return t2;
  return getTransforms().Transforms::getTransform(frame_id);
}

bool PlanningScene::knowsFrameTransform(const std::string& frame_id) const
{
  return knowsFrameTransform(getCurrentState(), frame_id);
}

bool PlanningScene::knowsFrameTransform(const moveit::core::RobotState& state, const std::string& frame_id) const
{
  if (!frame_id.empty() && frame_id[0] == '/')
    return knowsFrameTransform(frame_id.substr(1));

  if (state.knowsFrameTransform(frame_id))
    return true;
  if (getWorld()->knowsTransform(frame_id))
    return true;
  return getTransforms().Transforms::canTransform(frame_id);
}

bool PlanningScene::hasObjectType(const std::string& object_id) const
{
  if (object_types_)
    if (object_types_->find(object_id) != object_types_->end())
      return true;
  if (parent_)
    return parent_->hasObjectType(object_id);
  return false;
}

const object_recognition_msgs::ObjectType& PlanningScene::getObjectType(const std::string& object_id) const
{
  if (object_types_)
  {
    ObjectTypeMap::const_iterator it = object_types_->find(object_id);
    if (it != object_types_->end())
      return it->second;
  }
  if (parent_)
    return parent_->getObjectType(object_id);
  static const object_recognition_msgs::ObjectType EMPTY;
  return EMPTY;
}

void PlanningScene::setObjectType(const std::string& object_id, const object_recognition_msgs::ObjectType& type)
{
  if (!object_types_)
    object_types_ = std::make_unique<ObjectTypeMap>();
  (*object_types_)[object_id] = type;
}

void PlanningScene::removeObjectType(const std::string& object_id)
{
  if (object_types_)
    object_types_->erase(object_id);
}

void PlanningScene::getKnownObjectTypes(ObjectTypeMap& kc) const
{
  kc.clear();
  if (parent_)
    parent_->getKnownObjectTypes(kc);
  if (object_types_)
    for (ObjectTypeMap::const_iterator it = object_types_->begin(); it != object_types_->end(); ++it)
      kc[it->first] = it->second;
}

bool PlanningScene::hasObjectColor(const std::string& object_id) const
{
  if (object_colors_)
    if (object_colors_->find(object_id) != object_colors_->end())
      return true;
  if (parent_)
    return parent_->hasObjectColor(object_id);
  return false;
}

const std_msgs::ColorRGBA& PlanningScene::getObjectColor(const std::string& object_id) const
{
  if (object_colors_)
  {
    ObjectColorMap::const_iterator it = object_colors_->find(object_id);
    if (it != object_colors_->end())
      return it->second;
  }
  if (parent_)
    return parent_->getObjectColor(object_id);
  static const std_msgs::ColorRGBA EMPTY;
  return EMPTY;
}

void PlanningScene::getKnownObjectColors(ObjectColorMap& kc) const
{
  kc.clear();
  if (parent_)
    parent_->getKnownObjectColors(kc);
  if (object_colors_)
    for (ObjectColorMap::const_iterator it = object_colors_->begin(); it != object_colors_->end(); ++it)
      kc[it->first] = it->second;
}

void PlanningScene::setObjectColor(const std::string& object_id, const std_msgs::ColorRGBA& color)
{
  if (object_id.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot set color of object with empty object_id.");
    return;
  }
  if (!object_colors_)
    object_colors_ = std::make_unique<ObjectColorMap>();
  (*object_colors_)[object_id] = color;
}

void PlanningScene::removeObjectColor(const std::string& object_id)
{
  if (object_colors_)
    object_colors_->erase(object_id);
}

bool PlanningScene::isStateColliding(const moveit_msgs::RobotState& state, const std::string& group, bool verbose) const
{
  moveit::core::RobotState s(getCurrentState());
  moveit::core::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateColliding(s, group, verbose);
}

bool PlanningScene::isStateColliding(const std::string& group, bool verbose)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
    return isStateColliding(getCurrentStateNonConst(), group, verbose);
  else
    return isStateColliding(getCurrentState(), group, verbose);
}

bool PlanningScene::isStateColliding(const moveit::core::RobotState& state, const std::string& group, bool verbose) const
{
  collision_detection::CollisionRequest req;
  req.verbose = verbose;
  req.group_name = group;
  collision_detection::CollisionResult res;
  checkCollision(req, res, state);
  return res.collision;
}

bool PlanningScene::isStateFeasible(const moveit_msgs::RobotState& state, bool verbose) const
{
  if (state_feasibility_)
  {
    moveit::core::RobotState s(getCurrentState());
    moveit::core::robotStateMsgToRobotState(getTransforms(), state, s);
    return state_feasibility_(s, verbose);
  }
  return true;
}

bool PlanningScene::isStateFeasible(const moveit::core::RobotState& state, bool verbose) const
{
  if (state_feasibility_)
    return state_feasibility_(state, verbose);
  return true;
}

bool PlanningScene::isStateConstrained(const moveit_msgs::RobotState& state, const moveit_msgs::Constraints& constr,
                                       bool verbose) const
{
  moveit::core::RobotState s(getCurrentState());
  moveit::core::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateConstrained(s, constr, verbose);
}

bool PlanningScene::isStateConstrained(const moveit::core::RobotState& state, const moveit_msgs::Constraints& constr,
                                       bool verbose) const
{
  kinematic_constraints::KinematicConstraintSetPtr ks(
      new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
  ks->add(constr, getTransforms());
  if (ks->empty())
    return true;
  else
    return isStateConstrained(state, *ks, verbose);
}

bool PlanningScene::isStateConstrained(const moveit_msgs::RobotState& state,
                                       const kinematic_constraints::KinematicConstraintSet& constr, bool verbose) const
{
  moveit::core::RobotState s(getCurrentState());
  moveit::core::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateConstrained(s, constr, verbose);
}

bool PlanningScene::isStateConstrained(const moveit::core::RobotState& state,
                                       const kinematic_constraints::KinematicConstraintSet& constr, bool verbose) const
{
  return constr.decide(state, verbose).satisfied;
}

bool PlanningScene::isStateValid(const moveit::core::RobotState& state, const std::string& group, bool verbose) const
{
  static const moveit_msgs::Constraints EMP_CONSTRAINTS;
  return isStateValid(state, EMP_CONSTRAINTS, group, verbose);
}

bool PlanningScene::isStateValid(const moveit_msgs::RobotState& state, const std::string& group, bool verbose) const
{
  static const moveit_msgs::Constraints EMP_CONSTRAINTS;
  return isStateValid(state, EMP_CONSTRAINTS, group, verbose);
}

bool PlanningScene::isStateValid(const moveit_msgs::RobotState& state, const moveit_msgs::Constraints& constr,
                                 const std::string& group, bool verbose) const
{
  moveit::core::RobotState s(getCurrentState());
  moveit::core::robotStateMsgToRobotState(getTransforms(), state, s);
  return isStateValid(s, constr, group, verbose);
}

bool PlanningScene::isStateValid(const moveit::core::RobotState& state, const moveit_msgs::Constraints& constr,
                                 const std::string& group, bool verbose) const
{
  if (isStateColliding(state, group, verbose))
    return false;
  if (!isStateFeasible(state, verbose))
    return false;
  return isStateConstrained(state, constr, verbose);
}

bool PlanningScene::isStateValid(const moveit::core::RobotState& state,
                                 const kinematic_constraints::KinematicConstraintSet& constr, const std::string& group,
                                 bool verbose) const
{
  if (isStateColliding(state, group, verbose))
    return false;
  if (!isStateFeasible(state, verbose))
    return false;
  return isStateConstrained(state, constr, verbose);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory, const std::string& group, bool verbose,
                                std::vector<std::size_t>* invalid_index) const
{
  static const moveit_msgs::Constraints EMP_CONSTRAINTS;
  static const std::vector<moveit_msgs::Constraints> EMP_CONSTRAINTS_VECTOR;
  return isPathValid(start_state, trajectory, EMP_CONSTRAINTS, EMP_CONSTRAINTS_VECTOR, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  static const std::vector<moveit_msgs::Constraints> EMP_CONSTRAINTS_VECTOR;
  return isPathValid(start_state, trajectory, path_constraints, EMP_CONSTRAINTS_VECTOR, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const moveit_msgs::Constraints& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(start_state, trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const moveit_msgs::RobotState& start_state,
                                const moveit_msgs::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  robot_trajectory::RobotTrajectory t(getRobotModel(), group);
  moveit::core::RobotState start(getCurrentState());
  moveit::core::robotStateMsgToRobotState(getTransforms(), start_state, start);
  t.setRobotTrajectoryMsg(start, trajectory);
  return isPathValid(t, path_constraints, goal_constraints, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  bool result = true;
  if (invalid_index)
    invalid_index->clear();
  kinematic_constraints::KinematicConstraintSet ks_p(getRobotModel());
  ks_p.add(path_constraints, getTransforms());
  std::size_t n_wp = trajectory.getWayPointCount();
  for (std::size_t i = 0; i < n_wp; ++i)
  {
    const moveit::core::RobotState& st = trajectory.getWayPoint(i);

    bool this_state_valid = true;
    if (isStateColliding(st, group, verbose))
      this_state_valid = false;
    if (!isStateFeasible(st, verbose))
      this_state_valid = false;
    if (!ks_p.empty() && !ks_p.decide(st, verbose).satisfied)
      this_state_valid = false;

    if (!this_state_valid)
    {
      if (invalid_index)
        invalid_index->push_back(i);
      else
        return false;
      result = false;
    }

    // check goal for last state
    if (i + 1 == n_wp && !goal_constraints.empty())
    {
      bool found = false;
      for (const moveit_msgs::Constraints& goal_constraint : goal_constraints)
      {
        if (isStateConstrained(st, goal_constraint))
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        if (verbose)
          ROS_INFO_NAMED(LOGNAME, "Goal not satisfied");
        if (invalid_index)
          invalid_index->push_back(i);
        result = false;
      }
    }
  }
  return result;
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const moveit_msgs::Constraints& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  std::vector<moveit_msgs::Constraints> goal_constraints_vector(1, goal_constraints);
  return isPathValid(trajectory, path_constraints, goal_constraints_vector, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  static const std::vector<moveit_msgs::Constraints> EMP_CONSTRAINTS_VECTOR;
  return isPathValid(trajectory, path_constraints, EMP_CONSTRAINTS_VECTOR, group, verbose, invalid_index);
}

bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  static const moveit_msgs::Constraints EMP_CONSTRAINTS;
  static const std::vector<moveit_msgs::Constraints> EMP_CONSTRAINTS_VECTOR;
  return isPathValid(trajectory, EMP_CONSTRAINTS, EMP_CONSTRAINTS_VECTOR, group, verbose, invalid_index);
}

void PlanningScene::getCostSources(const robot_trajectory::RobotTrajectory& trajectory, std::size_t max_costs,
                                   std::set<collision_detection::CostSource>& costs, double overlap_fraction) const
{
  getCostSources(trajectory, max_costs, std::string(), costs, overlap_fraction);
}

void PlanningScene::getCostSources(const robot_trajectory::RobotTrajectory& trajectory, std::size_t max_costs,
                                   const std::string& group_name, std::set<collision_detection::CostSource>& costs,
                                   double overlap_fraction) const
{
  collision_detection::CollisionRequest creq;
  creq.max_cost_sources = max_costs;
  creq.group_name = group_name;
  creq.cost = true;
  std::set<collision_detection::CostSource> cs;
  std::set<collision_detection::CostSource> cs_start;
  std::size_t n_wp = trajectory.getWayPointCount();
  for (std::size_t i = 0; i < n_wp; ++i)
  {
    collision_detection::CollisionResult cres;
    checkCollision(creq, cres, trajectory.getWayPoint(i));
    cs.insert(cres.cost_sources.begin(), cres.cost_sources.end());
    if (i == 0)
      cs_start.swap(cres.cost_sources);
  }

  if (cs.size() <= max_costs)
    costs.swap(cs);
  else
  {
    costs.clear();
    std::size_t i = 0;
    for (std::set<collision_detection::CostSource>::iterator it = cs.begin(); i < max_costs; ++it, ++i)
      costs.insert(*it);
  }

  collision_detection::removeCostSources(costs, cs_start, overlap_fraction);
  collision_detection::removeOverlapping(costs, overlap_fraction);
}

void PlanningScene::getCostSources(const moveit::core::RobotState& state, std::size_t max_costs,
                                   std::set<collision_detection::CostSource>& costs) const
{
  getCostSources(state, max_costs, std::string(), costs);
}

void PlanningScene::getCostSources(const moveit::core::RobotState& state, std::size_t max_costs,
                                   const std::string& group_name,
                                   std::set<collision_detection::CostSource>& costs) const
{
  collision_detection::CollisionRequest creq;
  creq.max_cost_sources = max_costs;
  creq.group_name = group_name;
  creq.cost = true;
  collision_detection::CollisionResult cres;
  checkCollision(creq, cres, state);
  cres.cost_sources.swap(costs);
}

void PlanningScene::printKnownObjects(std::ostream& out) const
{
  const std::vector<std::string>& objects = getWorld()->getObjectIds();
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  getCurrentState().getAttachedBodies(attached_bodies);

  // Output
  out << "-----------------------------------------\n";
  out << "PlanningScene Known Objects:\n";
  out << "  - Collision World Objects:\n ";
  for (const std::string& object : objects)
  {
    out << "\t- " << object << "\n";
  }

  out << "  - Attached Bodies:\n";
  for (const moveit::core::AttachedBody* attached_body : attached_bodies)
  {
    out << "\t- " << attached_body->getName() << "\n";
  }
  out << "-----------------------------------------\n";
}

}  // end of namespace planning_scene
