/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#include <moveit/robot_model/robot_model.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/collision_distance_field/collision_detector_allocator_distance_field.h>
#include <functional>
#include <memory>
#include <utility>

namespace collision_detection
{
namespace
{
static const std::string NAME = "DISTANCE_FIELD";
const double EPSILON = 0.001f;
}  // namespace

CollisionEnvDistanceField::CollisionEnvDistanceField(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution,
    double collision_tolerance, double max_propogation_distance, double /*padding*/, double /*scale*/)
  : CollisionEnv(robot_model)
{
  initialize(link_body_decompositions, Eigen::Vector3d(size_x, size_y, size_z), origin, use_signed_distance_field,
             resolution, collision_tolerance, max_propogation_distance);

  setPadding(0.0);

  distance_field_cache_entry_world_ = generateDistanceFieldCacheEntryWorld();

  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { return notifyObjectChange(object, action); });
}

CollisionEnvDistanceField::CollisionEnvDistanceField(
    const moveit::core::RobotModelConstPtr& robot_model, const WorldPtr& world,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution,
    double collision_tolerance, double max_propogation_distance, double padding, double scale)
  : CollisionEnv(robot_model, world, padding, scale)
{
  initialize(link_body_decompositions, Eigen::Vector3d(size_x, size_y, size_z), origin, use_signed_distance_field,
             resolution, collision_tolerance, max_propogation_distance);

  distance_field_cache_entry_world_ = generateDistanceFieldCacheEntryWorld();

  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { return notifyObjectChange(object, action); });

  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvDistanceField::CollisionEnvDistanceField(const CollisionEnvDistanceField& other, const WorldPtr& world)
  : CollisionEnv(other, world)
{
  size_ = other.size_;
  origin_ = other.origin_;

  use_signed_distance_field_ = other.use_signed_distance_field_;
  resolution_ = other.resolution_;
  collision_tolerance_ = other.collision_tolerance_;
  max_propogation_distance_ = other.max_propogation_distance_;
  link_body_decomposition_vector_ = other.link_body_decomposition_vector_;
  link_body_decomposition_index_map_ = other.link_body_decomposition_index_map_;
  in_group_update_map_ = other.in_group_update_map_;
  distance_field_cache_entry_world_ = generateDistanceFieldCacheEntryWorld();
  pregenerated_group_state_representation_map_ = other.pregenerated_group_state_representation_map_;
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { return notifyObjectChange(object, action); });
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionEnvDistanceField::~CollisionEnvDistanceField()
{
  getWorld()->removeObserver(observer_handle_);
}

void CollisionEnvDistanceField::initialize(
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, const Eigen::Vector3d& size,
    const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution, double collision_tolerance,
    double max_propogation_distance)
{
  size_ = size;
  origin_ = origin;
  use_signed_distance_field_ = use_signed_distance_field;
  resolution_ = resolution;
  collision_tolerance_ = collision_tolerance;
  max_propogation_distance_ = max_propogation_distance;
  addLinkBodyDecompositions(resolution_, link_body_decompositions);
  moveit::core::RobotState state(robot_model_);
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  const std::vector<const moveit::core::JointModelGroup*>& jmg = robot_model_->getJointModelGroups();
  for (const moveit::core::JointModelGroup* jm : jmg)
  {
    std::map<std::string, bool> updated_group_entry;
    std::vector<std::string> links = jm->getUpdatedLinkModelsWithGeometryNames();
    for (const std::string& link : links)
    {
      updated_group_entry[link] = true;
    }
    in_group_update_map_[jm->getName()] = updated_group_entry;
    state.updateLinkTransforms();
    DistanceFieldCacheEntryPtr dfce =
        generateDistanceFieldCacheEntry(jm->getName(), state, &planning_scene_->getAllowedCollisionMatrix(), false);
    getGroupStateRepresentation(dfce, state, pregenerated_group_state_representation_map_[jm->getName()]);
  }
}

void CollisionEnvDistanceField::generateCollisionCheckingStructures(
    const std::string& group_name, const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix* acm, GroupStateRepresentationPtr& gsr,
    bool generate_distance_field) const
{
  DistanceFieldCacheEntryConstPtr dfce = getDistanceFieldCacheEntry(group_name, state, acm);
  if (!dfce || (generate_distance_field && !dfce->distance_field_))
  {
    // ROS_DEBUG_STREAM_NAMED("collision_distance_field", "Generating new
    // DistanceFieldCacheEntry for CollisionRobot");
    DistanceFieldCacheEntryPtr new_dfce =
        generateDistanceFieldCacheEntry(group_name, state, acm, generate_distance_field);
    boost::mutex::scoped_lock slock(update_cache_lock_);
    (const_cast<CollisionEnvDistanceField*>(this))->distance_field_cache_entry_ = new_dfce;
    dfce = new_dfce;
  }
  getGroupStateRepresentation(dfce, state, gsr);
}

void CollisionEnvDistanceField::checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state,
                                                         const collision_detection::AllowedCollisionMatrix* acm,
                                                         GroupStateRepresentationPtr& gsr) const
{
  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, acm, gsr, true);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }
  // ros::WallTime n = ros::WallTime::now();
  bool done = getSelfCollisions(req, res, gsr);

  if (!done)
  {
    getIntraGroupCollisions(req, res, gsr);
    ROS_DEBUG_COND(res.collision, "Intra Group Collision found");
  }
}

DistanceFieldCacheEntryConstPtr
CollisionEnvDistanceField::getDistanceFieldCacheEntry(const std::string& group_name,
                                                      const moveit::core::RobotState& state,
                                                      const collision_detection::AllowedCollisionMatrix* acm) const
{
  DistanceFieldCacheEntryConstPtr ret;
  if (!distance_field_cache_entry_)
  {
    ROS_DEBUG_STREAM("No current Distance field cache entry");
    return ret;
  }
  DistanceFieldCacheEntryConstPtr cur = distance_field_cache_entry_;
  if (group_name != cur->group_name_)
  {
    ROS_DEBUG("No cache entry as group name changed from %s to %s", cur->group_name_.c_str(), group_name.c_str());
    return ret;
  }
  else if (!compareCacheEntryToState(cur, state))
  {
    // Regenerating distance field as state has changed from last time
    // ROS_DEBUG_STREAM_NAMED("collision_distance_field", "Regenerating distance field as
    // state has changed from last time");
    return ret;
  }
  else if (acm && !compareCacheEntryToAllowedCollisionMatrix(cur, *acm))
  {
    ROS_DEBUG("Regenerating distance field as some relevant part of the acm changed");
    return ret;
  }
  return cur;
}

void CollisionEnvDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult& res,
                                                   const moveit::core::RobotState& state) const
{
  GroupStateRepresentationPtr gsr;
  checkSelfCollisionHelper(req, res, state, nullptr, gsr);
}

void CollisionEnvDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   GroupStateRepresentationPtr& gsr) const
{
  checkSelfCollisionHelper(req, res, state, nullptr, gsr);
}

void CollisionEnvDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
  GroupStateRepresentationPtr gsr;
  checkSelfCollisionHelper(req, res, state, &acm, gsr);
}

void CollisionEnvDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                   collision_detection::CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   const collision_detection::AllowedCollisionMatrix& acm,
                                                   GroupStateRepresentationPtr& gsr) const
{
  if (gsr)
  {
    ROS_WARN("Shouldn't be calling this function with initialized gsr - ACM "
             "will be ignored");
  }
  checkSelfCollisionHelper(req, res, state, &acm, gsr);
}

bool CollisionEnvDistanceField::getSelfCollisions(const collision_detection::CollisionRequest& req,
                                                  collision_detection::CollisionResult& res,
                                                  GroupStateRepresentationPtr& gsr) const
{
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size() + gsr->dfce_->attached_body_names_.size(); i++)
  {
    bool is_link = i < gsr->dfce_->link_names_.size();
    if ((is_link && !gsr->dfce_->link_has_geometry_[i]) || !gsr->dfce_->self_collision_enabled_[i])
      continue;
    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;

    if (is_link)
    {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    }
    else
    {
      collision_spheres_1 =
          &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters());
    }

    if (req.contacts)
    {
      std::vector<unsigned int> colls;
      bool coll =
          getCollisionSphereCollision(gsr->dfce_->distance_field_.get(), *collision_spheres_1, *sphere_centers_1,
                                      max_propogation_distance_, collision_tolerance_,
                                      std::min(req.max_contacts_per_pair, req.max_contacts - res.contact_count), colls);
      if (coll)
      {
        res.collision = true;
        for (unsigned int col : colls)
        {
          collision_detection::Contact con;
          if (is_link)
          {
            con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[col];
            con.body_type_1 = collision_detection::BodyTypes::ROBOT_LINK;
            con.body_name_1 = gsr->dfce_->link_names_[i];
          }
          else
          {
            con.pos = gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters()[col];
            con.body_type_1 = collision_detection::BodyTypes::ROBOT_ATTACHED;
            con.body_name_1 = gsr->dfce_->attached_body_names_[i - gsr->dfce_->link_names_.size()];
          }

          ROS_DEBUG_STREAM("Self collision detected for link " << con.body_name_1);

          con.body_type_2 = collision_detection::BodyTypes::ROBOT_LINK;
          con.body_name_2 = "self";
          res.contact_count++;
          res.contacts[std::pair<std::string, std::string>(con.body_name_1, con.body_name_2)].push_back(con);
          gsr->gradients_[i].types[col] = SELF;
        }
        gsr->gradients_[i].collision = true;

        if (res.contact_count >= req.max_contacts)
        {
          return true;
        }
      }
    }
    else
    {
      bool coll = getCollisionSphereCollision(gsr->dfce_->distance_field_.get(), *collision_spheres_1,
                                              *sphere_centers_1, max_propogation_distance_, collision_tolerance_);
      if (coll)
      {
        ROS_DEBUG("Link %s in self collision", gsr->dfce_->link_names_[i].c_str());
        res.collision = true;
        return true;
      }
    }
  }
  return (res.contact_count >= req.max_contacts);
}

bool CollisionEnvDistanceField::getSelfProximityGradients(GroupStateRepresentationPtr& gsr) const
{
  bool in_collision = false;

  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++)
  {
    const std::string& link_name = gsr->dfce_->link_names_[i];
    bool is_link = i < gsr->dfce_->link_names_.size();
    if ((is_link && !gsr->dfce_->link_has_geometry_[i]) || !gsr->dfce_->self_collision_enabled_[i])
    {
      continue;
    }

    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;
    if (is_link)
    {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    }
    else
    {
      collision_spheres_1 =
          &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters());
    }

    // computing distance gradients by checking collisions against other mobile
    // links as indicated by the AllowedCollisionMatrix
    bool coll = false;
    if (gsr->dfce_->acm_.getSize() > 0)
    {
      AllowedCollision::Type col_type;
      for (unsigned int j = 0; j < gsr->dfce_->link_names_.size(); j++)
      {
        // on self collisions skip
        if (link_name == gsr->dfce_->link_names_[j])
        {
          continue;
        }

        // on collision exceptions skip
        if (gsr->dfce_->acm_.getEntry(link_name, gsr->dfce_->link_names_[j], col_type) &&
            col_type != AllowedCollision::NEVER)
        {
          continue;
        }

        if (gsr->link_distance_fields_[j])
        {
          coll = gsr->link_distance_fields_[j]->getCollisionSphereGradients(
              *collision_spheres_1, *sphere_centers_1, gsr->gradients_[i], collision_detection::SELF,
              collision_tolerance_, false, max_propogation_distance_, false);

          if (coll)
          {
            in_collision = true;
          }
        }
      }
    }

    coll = getCollisionSphereGradients(gsr->dfce_->distance_field_.get(), *collision_spheres_1, *sphere_centers_1,
                                       gsr->gradients_[i], collision_detection::SELF, collision_tolerance_, false,
                                       max_propogation_distance_, false);

    if (coll)
    {
      in_collision = true;
    }
  }

  return in_collision;
}

bool CollisionEnvDistanceField::getIntraGroupCollisions(const collision_detection::CollisionRequest& req,
                                                        collision_detection::CollisionResult& res,
                                                        GroupStateRepresentationPtr& gsr) const
{
  unsigned int num_links = gsr->dfce_->link_names_.size();
  unsigned int num_attached_bodies = gsr->dfce_->attached_body_names_.size();

  for (unsigned int i = 0; i < num_links + num_attached_bodies; i++)
  {
    for (unsigned int j = i + 1; j < num_links + num_attached_bodies; j++)
    {
      if (i == j)
        continue;
      bool i_is_link = i < num_links;
      bool j_is_link = j < num_links;

      if ((i_is_link && !gsr->dfce_->link_has_geometry_[i]) || (j_is_link && !gsr->dfce_->link_has_geometry_[j]))
        continue;

      if (!gsr->dfce_->intra_group_collision_enabled_[i][j])
        continue;

      if (i_is_link && j_is_link &&
          !doBoundingSpheresIntersect(gsr->link_body_decompositions_[i], gsr->link_body_decompositions_[j]))
      {
        // ROS_DEBUG_STREAM("Bounding spheres for " <<
        // gsr->dfce_->link_names_[i] << " and " << gsr->dfce_->link_names_[j]
        //<< " don't intersect");
        continue;
      }
      else if (!i_is_link || !j_is_link)
      {
        bool all_ok = true;
        if (!i_is_link && j_is_link)
        {
          for (unsigned int k = 0; k < gsr->attached_body_decompositions_[i - num_links]->getSize(); k++)
          {
            if (doBoundingSpheresIntersect(
                    gsr->link_body_decompositions_[j],
                    gsr->attached_body_decompositions_[i - num_links]->getPosedBodySphereDecomposition(k)))
            {
              all_ok = false;
              break;
            }
          }
        }
        else if (i_is_link && !j_is_link)
        {
          for (unsigned int k = 0; k < gsr->attached_body_decompositions_[j - num_links]->getSize(); k++)
          {
            if (doBoundingSpheresIntersect(
                    gsr->link_body_decompositions_[i],
                    gsr->attached_body_decompositions_[j - num_links]->getPosedBodySphereDecomposition(k)))
            {
              all_ok = false;
              break;
            }
          }
        }
        else
        {
          for (unsigned int k = 0; k < gsr->attached_body_decompositions_[i - num_links]->getSize() && all_ok; k++)
          {
            for (unsigned int l = 0; l < gsr->attached_body_decompositions_[j - num_links]->getSize(); l++)
            {
              if (doBoundingSpheresIntersect(
                      gsr->attached_body_decompositions_[i - num_links]->getPosedBodySphereDecomposition(k),
                      gsr->attached_body_decompositions_[j - num_links]->getPosedBodySphereDecomposition(l)))
              {
                all_ok = false;
                break;
              }
            }
          }
        }
        if (all_ok)
        {
          continue;
        }
        // std::cerr << "Bounding spheres for " << gsr->dfce_->link_names_[i] <<
        // " and " << gsr->dfce_->link_names_[j]
        //           << " intersect" << std::endl;
      }
      int num_pair = -1;
      std::string name_1;
      std::string name_2;
      if (i_is_link)
      {
        name_1 = gsr->dfce_->link_names_[i];
      }
      else
      {
        name_1 = gsr->dfce_->attached_body_names_[i - num_links];
      }

      if (j_is_link)
      {
        name_2 = gsr->dfce_->link_names_[j];
      }
      else
      {
        name_2 = gsr->dfce_->attached_body_names_[j - num_links];
      }
      if (req.contacts)
      {
        collision_detection::CollisionResult::ContactMap::iterator it =
            res.contacts.find(std::pair<std::string, std::string>(name_1, name_2));
        if (it == res.contacts.end())
        {
          num_pair = 0;
        }
        else
        {
          num_pair = it->second.size();
        }
      }
      const std::vector<CollisionSphere>* collision_spheres_1;
      const std::vector<CollisionSphere>* collision_spheres_2;
      const EigenSTL::vector_Vector3d* sphere_centers_1;
      const EigenSTL::vector_Vector3d* sphere_centers_2;
      if (i_is_link)
      {
        collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
      }
      else
      {
        collision_spheres_1 = &(gsr->attached_body_decompositions_[i - num_links]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->attached_body_decompositions_[i - num_links]->getSphereCenters());
      }
      if (j_is_link)
      {
        collision_spheres_2 = &(gsr->link_body_decompositions_[j]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->link_body_decompositions_[j]->getSphereCenters());
      }
      else
      {
        collision_spheres_2 = &(gsr->attached_body_decompositions_[j - num_links]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->attached_body_decompositions_[j - num_links]->getSphereCenters());
      }

      for (unsigned int k = 0; k < collision_spheres_1->size() && num_pair < (int)req.max_contacts_per_pair; k++)
      {
        for (unsigned int l = 0; l < collision_spheres_2->size() && num_pair < (int)req.max_contacts_per_pair; l++)
        {
          Eigen::Vector3d gradient = (*sphere_centers_1)[k] - (*sphere_centers_2)[l];
          double dist = gradient.norm();
          // std::cerr << "Dist is " << dist << " rad " <<
          // (*collision_spheres_1)[k].radius_+(*collision_spheres_2)[l].radius_
          // << std::endl;

          if (dist < (*collision_spheres_1)[k].radius_ + (*collision_spheres_2)[l].radius_)
          {
            //            ROS_DEBUG("Intra-group contact between %s and %s, d =
            //            %f <  r1 = %f + r2 = %f", name_1.c_str(),
            //            name_2.c_str(),
            //                      dist ,(*collision_spheres_1)[k].radius_
            //                      ,(*collision_spheres_2)[l].radius_);
            //            Eigen::Vector3d sc1 = (*sphere_centers_1)[k];
            //            Eigen::Vector3d sc2 = (*sphere_centers_2)[l];
            //            ROS_DEBUG("sphere center 1:[ %f, %f, %f ], sphere
            //            center 2: [%f, %f,%f ], lbdc size =
            //            %i",sc1[0],sc1[1],sc1[2],
            //                      sc2[0],sc2[1],sc2[2],int(gsr->link_body_decompositions_.size()));
            res.collision = true;

            if (req.contacts)
            {
              collision_detection::Contact con;
              con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[k];
              con.body_name_1 = name_1;
              con.body_name_2 = name_2;
              if (i_is_link)
              {
                con.body_type_1 = collision_detection::BodyTypes::ROBOT_LINK;
              }
              else
              {
                con.body_type_1 = collision_detection::BodyTypes::ROBOT_ATTACHED;
              }
              if (j_is_link)
              {
                con.body_type_2 = collision_detection::BodyTypes::ROBOT_LINK;
              }
              else
              {
                con.body_type_2 = collision_detection::BodyTypes::ROBOT_ATTACHED;
              }
              res.contact_count++;
              res.contacts[std::pair<std::string, std::string>(con.body_name_1, con.body_name_2)].push_back(con);
              num_pair++;
              // std::cerr << "Pushing back intra " << con.body_name_1 << " and
              // " << con.body_name_2 << std::endl;
              gsr->gradients_[i].types[k] = INTRA;
              gsr->gradients_[i].collision = true;
              gsr->gradients_[j].types[l] = INTRA;
              gsr->gradients_[j].collision = true;
              // ROS_INFO_STREAM("Sphere 1 " << (*sphere_centers_1)[k]);
              // ROS_INFO_STREAM("Sphere 2 " << (*sphere_centers_2)[l]);
              // ROS_INFO_STREAM("Norm " << gradient.norm());
              // ROS_INFO_STREAM("Dist is " << dist
              //                 << " radius 1 " <<
              //                 (*collision_spheres_1)[k].radius_
              //                 << " radius 2 " <<
              //                 (*collision_spheres_2)[l].radius_);
              // ROS_INFO_STREAM("Gradient " << gradient);
              // ROS_INFO_STREAM("Spheres intersect for " <<
              // gsr->dfce_->link_names_[i] << " and " <<
              // gsr->dfce_->link_names_[j]);
              // std::cerr << "Spheres intersect for " <<
              // gsr->dfce_->link_names_[i] << " and " <<
              // gsr->dfce_->link_names_[j] << std::cerr;
              if (res.contact_count >= req.max_contacts)
              {
                return true;
              }
            }
            else
            {
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

bool CollisionEnvDistanceField::getIntraGroupProximityGradients(GroupStateRepresentationPtr& gsr) const
{
  bool in_collision = false;
  unsigned int num_links = gsr->dfce_->link_names_.size();
  unsigned int num_attached_bodies = gsr->dfce_->attached_body_names_.size();
  // TODO - deal with attached bodies
  for (unsigned int i = 0; i < num_links + num_attached_bodies; i++)
  {
    for (unsigned int j = i + 1; j < num_links + num_attached_bodies; j++)
    {
      if (i == j)
        continue;
      bool i_is_link = i < num_links;
      bool j_is_link = j < num_links;
      if ((i_is_link && !gsr->dfce_->link_has_geometry_[i]) || (j_is_link && !gsr->dfce_->link_has_geometry_[j]))
        continue;
      if (!gsr->dfce_->intra_group_collision_enabled_[i][j])
        continue;
      const std::vector<CollisionSphere>* collision_spheres_1;
      const std::vector<CollisionSphere>* collision_spheres_2;
      const EigenSTL::vector_Vector3d* sphere_centers_1;
      const EigenSTL::vector_Vector3d* sphere_centers_2;
      if (i_is_link)
      {
        collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
      }
      else
      {
        collision_spheres_1 = &(gsr->attached_body_decompositions_[i - num_links]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->attached_body_decompositions_[i - num_links]->getSphereCenters());
      }
      if (j_is_link)
      {
        collision_spheres_2 = &(gsr->link_body_decompositions_[j]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->link_body_decompositions_[j]->getSphereCenters());
      }
      else
      {
        collision_spheres_2 = &(gsr->attached_body_decompositions_[j - num_links]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->attached_body_decompositions_[j - num_links]->getSphereCenters());
      }
      for (unsigned int k = 0; k < collision_spheres_1->size(); k++)
      {
        for (unsigned int l = 0; l < collision_spheres_2->size(); l++)
        {
          Eigen::Vector3d gradient = (*sphere_centers_1)[k] - (*sphere_centers_2)[l];
          double dist = gradient.norm();
          if (dist < gsr->gradients_[i].distances[k])
          {
            gsr->gradients_[i].distances[k] = dist;
            gsr->gradients_[i].gradients[k] = gradient;
            gsr->gradients_[i].types[k] = INTRA;
          }
          if (dist < gsr->gradients_[j].distances[l])
          {
            gsr->gradients_[j].distances[l] = dist;
            gsr->gradients_[j].gradients[l] = -gradient;
            gsr->gradients_[j].types[l] = INTRA;
          }
        }
      }
    }
  }
  return in_collision;
}
DistanceFieldCacheEntryPtr CollisionEnvDistanceField::generateDistanceFieldCacheEntry(
    const std::string& group_name, const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix* acm, bool generate_distance_field) const
{
  DistanceFieldCacheEntryPtr dfce(new DistanceFieldCacheEntry());

  if (robot_model_->getJointModelGroup(group_name) == nullptr)
  {
    ROS_WARN("No group %s", group_name.c_str());
    return dfce;
  }

  dfce->group_name_ = group_name;
  dfce->state_ = std::make_shared<moveit::core::RobotState>(state);
  if (acm)
  {
    dfce->acm_ = *acm;
  }
  else
  {
    ROS_WARN_STREAM("Allowed Collision Matrix is null, enabling all collisions "
                    "in the DistanceFieldCacheEntry");
  }

  // generateAllowedCollisionInformation(dfce);
  dfce->link_names_ = robot_model_->getJointModelGroup(group_name)->getUpdatedLinkModelNames();
  std::vector<const moveit::core::AttachedBody*> all_attached_bodies;
  dfce->state_->getAttachedBodies(all_attached_bodies);
  unsigned int att_count = 0;

  // may be bigger than necessary
  std::vector<bool> all_true(dfce->link_names_.size() + all_attached_bodies.size(), true);
  std::vector<bool> all_false(dfce->link_names_.size() + all_attached_bodies.size(), false);

  const std::vector<const moveit::core::LinkModel*>& lsv = state.getJointModelGroup(group_name)->getUpdatedLinkModels();
  dfce->self_collision_enabled_.resize(dfce->link_names_.size() + all_attached_bodies.size(), true);
  dfce->intra_group_collision_enabled_.resize(dfce->link_names_.size() + all_attached_bodies.size());

  for (unsigned int i = 0; i < dfce->link_names_.size(); i++)
  {
    std::string link_name = dfce->link_names_[i];
    const moveit::core::LinkModel* link_state = dfce->state_->getLinkModel(link_name);
    bool found = false;

    for (unsigned int j = 0; j < lsv.size(); j++)
    {
      if (lsv[j]->getName() == link_name)
      {
        dfce->link_state_indices_.push_back(j);
        found = true;
        break;
      }
    }

    if (!found)
    {
      ROS_DEBUG("No link state found for link %s", dfce->link_names_[i].c_str());
      continue;
    }

    if (!link_state->getShapes().empty())
    {
      dfce->link_has_geometry_.push_back(true);
      dfce->link_body_indices_.push_back(link_body_decomposition_index_map_.find(link_name)->second);

      if (acm)
      {
        collision_detection::AllowedCollision::Type t;
        if (acm->getEntry(link_name, link_name, t) && t == collision_detection::AllowedCollision::ALWAYS)
        {
          dfce->self_collision_enabled_[i] = false;
        }

        dfce->intra_group_collision_enabled_[i] = all_true;
        for (unsigned int j = i + 1; j < dfce->link_names_.size(); j++)
        {
          if (link_name == dfce->link_names_[j])
          {
            dfce->intra_group_collision_enabled_[i][j] = false;
            continue;
          }
          if (acm->getEntry(link_name, dfce->link_names_[j], t) && t == collision_detection::AllowedCollision::ALWAYS)
          {
            dfce->intra_group_collision_enabled_[i][j] = false;
          }
        }

        std::vector<const moveit::core::AttachedBody*> link_attached_bodies;
        state.getAttachedBodies(link_attached_bodies, link_state);
        for (unsigned int j = 0; j < link_attached_bodies.size(); j++, att_count++)
        {
          dfce->attached_body_names_.push_back(link_attached_bodies[j]->getName());
          dfce->attached_body_link_state_indices_.push_back(dfce->link_state_indices_[i]);

          if (acm->getEntry(link_name, link_attached_bodies[j]->getName(), t))
          {
            if (t == collision_detection::AllowedCollision::ALWAYS)
            {
              dfce->intra_group_collision_enabled_[i][att_count + dfce->link_names_.size()] = false;
            }
          }
          // std::cerr << "Checking touch links for " << link_name << " and " <<
          // attached_bodies[j]->getName()
          //           << " num " << attached_bodies[j]->getTouchLinks().size()
          //           << std::endl;
          // touch links take priority
          if (link_attached_bodies[j]->getTouchLinks().find(link_name) != link_attached_bodies[j]->getTouchLinks().end())
          {
            dfce->intra_group_collision_enabled_[i][att_count + dfce->link_names_.size()] = false;
            // std::cerr << "Setting intra group for " << link_name << " and
            // attached body " << link_attached_bodies[j]->getName() << " to
            // false" << std::endl;
          }
        }
      }
      else
      {
        dfce->self_collision_enabled_[i] = true;
        dfce->intra_group_collision_enabled_[i] = all_true;
      }
    }
    else
    {
      dfce->link_has_geometry_.push_back(false);
      dfce->link_body_indices_.push_back(0);
      dfce->self_collision_enabled_[i] = false;
      dfce->intra_group_collision_enabled_[i] = all_false;
    }
  }

  for (unsigned int i = 0; i < dfce->attached_body_names_.size(); i++)
  {
    dfce->intra_group_collision_enabled_[i + dfce->link_names_.size()] = all_true;
    if (acm)
    {
      collision_detection::AllowedCollision::Type t;
      if (acm->getEntry(dfce->attached_body_names_[i], dfce->attached_body_names_[i], t) &&
          t == collision_detection::AllowedCollision::ALWAYS)
      {
        dfce->self_collision_enabled_[i + dfce->link_names_.size()] = false;
      }
      for (unsigned int j = i + 1; j < dfce->attached_body_names_.size(); j++)
      {
        if (acm->getEntry(dfce->attached_body_names_[i], dfce->attached_body_names_[j], t) &&
            t == collision_detection::AllowedCollision::ALWAYS)
        {
          dfce->intra_group_collision_enabled_[i + dfce->link_names_.size()][j + dfce->link_names_.size()] = false;
        }
        // TODO - allow for touch links to be attached bodies?
        // else {
        // std::cerr << "Setting not allowed for " << link_name << " and " <<
        // dfce->link_names_[j] << std::endl;
        //}
      }
    }
  }

  std::map<std::string, GroupStateRepresentationPtr>::const_iterator it =
      pregenerated_group_state_representation_map_.find(dfce->group_name_);
  if (it != pregenerated_group_state_representation_map_.end())
  {
    dfce->pregenerated_group_state_representation_ = it->second;
  }

  std::map<std::string, bool> updated_map;
  if (!dfce->link_names_.empty())
  {
    const std::vector<const moveit::core::JointModel*>& child_joint_models =
        dfce->state_->getJointModelGroup(dfce->group_name_)->getActiveJointModels();
    for (const moveit::core::JointModel* child_joint_model : child_joint_models)
    {
      updated_map[child_joint_model->getName()] = true;
      ROS_DEBUG_STREAM("Joint " << child_joint_model->getName() << " has been added to updated list");
    }
  }

  const std::vector<std::string>& state_variable_names = state.getVariableNames();
  for (const std::string& state_variable_name : state_variable_names)
  {
    double val = state.getVariablePosition(state_variable_name);
    dfce->state_values_.push_back(val);
    if (updated_map.count(state_variable_name) == 0)
    {
      dfce->state_check_indices_.push_back(dfce->state_values_.size() - 1);
      ROS_DEBUG_STREAM("Non-group joint " << state_variable_name << " will be checked for state changes");
    }
  }

  if (generate_distance_field)
  {
    if (dfce->distance_field_)
    {
      ROS_DEBUG_STREAM("CollisionRobot skipping distance field generation, "
                       "will use existing one");
    }
    else
    {
      std::vector<PosedBodyPointDecompositionPtr> non_group_link_decompositions;
      std::vector<PosedBodyPointDecompositionVectorPtr> non_group_attached_body_decompositions;
      const std::map<std::string, bool>& updated_group_map = in_group_update_map_.find(group_name)->second;
      for (const moveit::core::LinkModel* link_model : robot_model_->getLinkModelsWithCollisionGeometry())
      {
        const std::string& link_name = link_model->getName();
        const moveit::core::LinkModel* link_state = dfce->state_->getLinkModel(link_name);
        if (updated_group_map.find(link_name) != updated_group_map.end())
        {
          continue;
        }

        // populating array with link that are not part of the planning group
        non_group_link_decompositions.push_back(getPosedLinkBodyPointDecomposition(link_state));
        non_group_link_decompositions.back()->updatePose(dfce->state_->getFrameTransform(link_state->getName()));

        std::vector<const moveit::core::AttachedBody*> attached_bodies;
        dfce->state_->getAttachedBodies(attached_bodies, link_state);
        for (const moveit::core::AttachedBody* attached_body : attached_bodies)
        {
          non_group_attached_body_decompositions.push_back(
              getAttachedBodyPointDecomposition(attached_body, resolution_));
        }
      }
      dfce->distance_field_ = std::make_shared<distance_field::PropagationDistanceField>(
          size_.x(), size_.y(), size_.z(), resolution_, origin_.x() - 0.5 * size_.x(), origin_.y() - 0.5 * size_.y(),
          origin_.z() - 0.5 * size_.z(), max_propogation_distance_, use_signed_distance_field_);

      // ROS_INFO_STREAM("Creation took " <<
      // (ros::WallTime::now()-before_create).toSec());
      // TODO - deal with AllowedCollisionMatrix
      // now we need to actually set the points
      // TODO - deal with shifted robot
      EigenSTL::vector_Vector3d all_points;
      for (collision_detection::PosedBodyPointDecompositionPtr& non_group_link_decomposition :
           non_group_link_decompositions)
      {
        all_points.insert(all_points.end(), non_group_link_decomposition->getCollisionPoints().begin(),
                          non_group_link_decomposition->getCollisionPoints().end());
      }

      for (collision_detection::PosedBodyPointDecompositionVectorPtr& non_group_attached_body_decomposition :
           non_group_attached_body_decompositions)
      {
        const EigenSTL::vector_Vector3d collision_points = non_group_attached_body_decomposition->getCollisionPoints();
        all_points.insert(all_points.end(), collision_points.begin(), collision_points.end());
      }

      dfce->distance_field_->addPointsToField(all_points);
      ROS_DEBUG_STREAM("CollisionRobot distance field has been initialized with " << all_points.size() << " points.");
    }
  }
  return dfce;
}

void CollisionEnvDistanceField::addLinkBodyDecompositions(double resolution)
{
  const std::vector<const moveit::core::LinkModel*>& link_models = robot_model_->getLinkModelsWithCollisionGeometry();
  for (const moveit::core::LinkModel* link_model : link_models)
  {
    if (link_model->getShapes().empty())
    {
      ROS_WARN("No collision geometry for link model %s though there should be", link_model->getName().c_str());
      continue;
    }

    ROS_DEBUG("Generating model for %s", link_model->getName().c_str());
    BodyDecompositionConstPtr bd(new BodyDecomposition(link_model->getShapes(),
                                                       link_model->getCollisionOriginTransforms(), resolution,
                                                       getLinkPadding(link_model->getName())));
    link_body_decomposition_vector_.push_back(bd);
    link_body_decomposition_index_map_[link_model->getName()] = link_body_decomposition_vector_.size() - 1;
  }
}

void CollisionEnvDistanceField::createCollisionModelMarker(const moveit::core::RobotState& state,
                                                           visualization_msgs::MarkerArray& model_markers) const
{
  // creating colors
  std_msgs::ColorRGBA robot_color;
  robot_color.r = 0;
  robot_color.b = 0.8f;
  robot_color.g = 0;
  robot_color.a = 0.5;

  std_msgs::ColorRGBA world_links_color;
  world_links_color.r = 1;
  world_links_color.g = 1;
  world_links_color.b = 0;
  world_links_color.a = 0.5;

  // creating sphere marker
  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = robot_model_->getRootLinkName();
  sphere_marker.header.stamp = ros::Time(0);
  sphere_marker.ns = distance_field_cache_entry_->group_name_ + "_sphere_decomposition";
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::Marker::ADD;
  sphere_marker.pose.orientation.x = 0;
  sphere_marker.pose.orientation.y = 0;
  sphere_marker.pose.orientation.z = 0;
  sphere_marker.pose.orientation.w = 1;
  sphere_marker.color = robot_color;
  sphere_marker.lifetime = ros::Duration(0);

  unsigned int id = 0;
  const moveit::core::JointModelGroup* joint_group = state.getJointModelGroup(distance_field_cache_entry_->group_name_);
  const std::vector<std::string>& group_link_names = joint_group->getUpdatedLinkModelNames();

  std::map<std::string, unsigned int>::const_iterator map_iter;
  for (map_iter = link_body_decomposition_index_map_.begin(); map_iter != link_body_decomposition_index_map_.end();
       map_iter++)
  {
    const std::string& link_name = map_iter->first;
    unsigned int link_index = map_iter->second;

    // selecting color
    if (std::find(group_link_names.begin(), group_link_names.end(), link_name) != group_link_names.end())
    {
      sphere_marker.color = robot_color;
    }
    else
    {
      sphere_marker.color = world_links_color;
    }

    collision_detection::PosedBodySphereDecompositionPtr sphere_representation(
        new PosedBodySphereDecomposition(link_body_decomposition_vector_[link_index]));
    sphere_representation->updatePose(state.getGlobalLinkTransform(link_name));
    for (unsigned int j = 0; j < sphere_representation->getCollisionSpheres().size(); j++)
    {
      sphere_marker.pose.position = tf2::toMsg(sphere_representation->getSphereCenters()[j]);
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z =
          2 * sphere_representation->getCollisionSpheres()[j].radius_;
      sphere_marker.id = id;
      id++;

      model_markers.markers.push_back(sphere_marker);
    }
  }
}

void CollisionEnvDistanceField::addLinkBodyDecompositions(
    double resolution, const std::map<std::string, std::vector<CollisionSphere>>& link_spheres)
{
  ROS_ASSERT_MSG(robot_model_, "RobotModelPtr is invalid");
  const std::vector<const moveit::core::LinkModel*>& link_models = robot_model_->getLinkModelsWithCollisionGeometry();

  for (const moveit::core::LinkModel* link_model : link_models)
  {
    if (link_model->getShapes().empty())
    {
      ROS_WARN_STREAM("Skipping model generation for link " << link_model->getName()
                                                            << " since it contains no geometries");
      continue;
    }

    BodyDecompositionPtr bd(new BodyDecomposition(link_model->getShapes(), link_model->getCollisionOriginTransforms(),
                                                  resolution, getLinkPadding(link_model->getName())));

    ROS_DEBUG("Generated model for %s", link_model->getName().c_str());

    if (link_spheres.find(link_model->getName()) != link_spheres.end())
    {
      bd->replaceCollisionSpheres(link_spheres.find(link_model->getName())->second, Eigen::Isometry3d::Identity());
    }
    link_body_decomposition_vector_.push_back(bd);
    link_body_decomposition_index_map_[link_model->getName()] = link_body_decomposition_vector_.size() - 1;
  }
  ROS_DEBUG_STREAM(__FUNCTION__ << " Finished ");
}

PosedBodySphereDecompositionPtr
CollisionEnvDistanceField::getPosedLinkBodySphereDecomposition(const moveit::core::LinkModel* /*ls*/,
                                                               unsigned int ind) const
{
  PosedBodySphereDecompositionPtr ret;
  ret = std::make_shared<PosedBodySphereDecomposition>(link_body_decomposition_vector_.at(ind));
  return ret;
}

PosedBodyPointDecompositionPtr
CollisionEnvDistanceField::getPosedLinkBodyPointDecomposition(const moveit::core::LinkModel* ls) const
{
  PosedBodyPointDecompositionPtr ret;
  std::map<std::string, unsigned int>::const_iterator it = link_body_decomposition_index_map_.find(ls->getName());
  if (it == link_body_decomposition_index_map_.end())
  {
    ROS_ERROR_NAMED("collision_distance_field", "No link body decomposition for link %s.", ls->getName().c_str());
    return ret;
  }
  ret = std::make_shared<PosedBodyPointDecomposition>(link_body_decomposition_vector_[it->second]);
  return ret;
}

void CollisionEnvDistanceField::updateGroupStateRepresentationState(const moveit::core::RobotState& state,
                                                                    GroupStateRepresentationPtr& gsr) const
{
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++)
  {
    const moveit::core::LinkModel* ls = state.getLinkModel(gsr->dfce_->link_names_[i]);
    if (gsr->dfce_->link_has_geometry_[i])
    {
      gsr->link_body_decompositions_[i]->updatePose(state.getFrameTransform(ls->getName()));
      gsr->link_distance_fields_[i]->updatePose(state.getFrameTransform(ls->getName()));
      gsr->gradients_[i].closest_distance = DBL_MAX;
      gsr->gradients_[i].collision = false;
      gsr->gradients_[i].types.assign(gsr->link_body_decompositions_[i]->getCollisionSpheres().size(), NONE);
      gsr->gradients_[i].distances.assign(gsr->link_body_decompositions_[i]->getCollisionSpheres().size(), DBL_MAX);
      gsr->gradients_[i].gradients.assign(gsr->link_body_decompositions_[i]->getCollisionSpheres().size(),
                                          Eigen::Vector3d(0.0, 0.0, 0.0));
      gsr->gradients_[i].sphere_locations = gsr->link_body_decompositions_[i]->getSphereCenters();
    }
  }

  for (unsigned int i = 0; i < gsr->dfce_->attached_body_names_.size(); i++)
  {
    const moveit::core::AttachedBody* att = state.getAttachedBody(gsr->dfce_->attached_body_names_[i]);
    if (!att)
    {
      ROS_WARN("Attached body discrepancy");
      continue;
    }

    // TODO: This logic for checking attached body count might be incorrect
    if (gsr->attached_body_decompositions_.size() != att->getShapes().size())
    {
      ROS_WARN("Attached body size discrepancy");
      continue;
    }

    for (unsigned int j = 0; j < att->getShapes().size(); j++)
    {
      gsr->attached_body_decompositions_[i]->updatePose(j, att->getGlobalCollisionBodyTransforms()[j]);
    }

    gsr->gradients_[i + gsr->dfce_->link_names_.size()].closest_distance = DBL_MAX;
    gsr->gradients_[i + gsr->dfce_->link_names_.size()].collision = false;
    gsr->gradients_[i + gsr->dfce_->link_names_.size()].types.assign(
        gsr->attached_body_decompositions_[i]->getCollisionSpheres().size(), NONE);
    gsr->gradients_[i + gsr->dfce_->link_names_.size()].distances.assign(
        gsr->attached_body_decompositions_[i]->getCollisionSpheres().size(), DBL_MAX);
    gsr->gradients_[i + gsr->dfce_->link_names_.size()].gradients.assign(
        gsr->attached_body_decompositions_[i]->getCollisionSpheres().size(), Eigen::Vector3d(0.0, 0.0, 0.0));
    gsr->gradients_[i + gsr->dfce_->link_names_.size()].sphere_locations =
        gsr->attached_body_decompositions_[i]->getSphereCenters();
  }
}

void CollisionEnvDistanceField::getGroupStateRepresentation(const DistanceFieldCacheEntryConstPtr& dfce,
                                                            const moveit::core::RobotState& state,
                                                            GroupStateRepresentationPtr& gsr) const
{
  if (!dfce->pregenerated_group_state_representation_)
  {
    ROS_DEBUG_STREAM("Creating GroupStateRepresentation");

    // unsigned int count = 0;
    gsr = std::make_shared<GroupStateRepresentation>();
    gsr->dfce_ = dfce;
    gsr->gradients_.resize(dfce->link_names_.size() + dfce->attached_body_names_.size());

    Eigen::Vector3d link_size;
    Eigen::Vector3d link_origin;
    for (unsigned int i = 0; i < dfce->link_names_.size(); i++)
    {
      const moveit::core::LinkModel* ls = state.getLinkModel(dfce->link_names_[i]);
      if (dfce->link_has_geometry_[i])
      {
        // create link body geometric decomposition
        gsr->link_body_decompositions_.push_back(getPosedLinkBodySphereDecomposition(ls, dfce->link_body_indices_[i]));

        // create and fill link distance field
        PosedBodySphereDecompositionPtr& link_bd = gsr->link_body_decompositions_.back();
        double diameter = 2 * link_bd->getBoundingSphereRadius();
        link_size = Eigen::Vector3d(diameter, diameter, diameter);
        link_origin = link_bd->getBoundingSphereCenter() - 0.5 * link_size;

        ROS_DEBUG_STREAM("Creating PosedDistanceField for link "
                         << dfce->link_names_[i] << " with size [" << link_size.x() << ", " << link_size.y() << ", "
                         << link_size.z() << "] and origin " << link_origin.x() << ", " << link_origin.y() << ", "
                         << link_origin.z());

        gsr->link_distance_fields_.push_back(std::make_shared<PosedDistanceField>(
            link_size, link_origin, resolution_, max_propogation_distance_, use_signed_distance_field_));
        gsr->link_distance_fields_.back()->addPointsToField(link_bd->getCollisionPoints());
        ROS_DEBUG_STREAM("Created PosedDistanceField for link " << dfce->link_names_[i] << " with "
                                                                << link_bd->getCollisionPoints().size() << " points");

        gsr->link_body_decompositions_.back()->updatePose(state.getFrameTransform(ls->getName()));
        gsr->link_distance_fields_.back()->updatePose(state.getFrameTransform(ls->getName()));
        gsr->gradients_[i].types.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size(), NONE);
        gsr->gradients_[i].distances.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size(),
                                            DBL_MAX);
        gsr->gradients_[i].gradients.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size());
        gsr->gradients_[i].sphere_radii = gsr->link_body_decompositions_.back()->getSphereRadii();
        gsr->gradients_[i].joint_name = ls->getParentJointModel()->getName();
      }
      else
      {
        gsr->link_body_decompositions_.push_back(PosedBodySphereDecompositionPtr());
        gsr->link_distance_fields_.push_back(PosedDistanceFieldPtr());
      }
    }
  }
  else
  {
    gsr = std::make_shared<GroupStateRepresentation>(*(dfce->pregenerated_group_state_representation_));
    gsr->dfce_ = dfce;
    gsr->gradients_.resize(dfce->link_names_.size() + dfce->attached_body_names_.size());
    for (unsigned int i = 0; i < dfce->link_names_.size(); i++)
    {
      const moveit::core::LinkModel* ls = state.getLinkModel(dfce->link_names_[i]);
      if (dfce->link_has_geometry_[i])
      {
        gsr->link_body_decompositions_[i]->updatePose(state.getFrameTransform(ls->getName()));
        gsr->link_distance_fields_[i]->updatePose(state.getFrameTransform(ls->getName()));
        gsr->gradients_[i].sphere_locations = gsr->link_body_decompositions_[i]->getSphereCenters();
      }
    }
  }

  for (unsigned int i = 0; i < dfce->attached_body_names_.size(); i++)
  {
    int link_index = dfce->attached_body_link_state_indices_[i];
    const moveit::core::LinkModel* ls =
        state.getJointModelGroup(gsr->dfce_->group_name_)->getUpdatedLinkModels()[link_index];
    // const moveit::core::LinkModel* ls =
    // state.getLinkStateVector()[dfce->attached_body_link_state_indices_[i]];
    /// std::cerr << "Attached " << dfce->attached_body_names_[i] << " index "
    /// << dfce->attached_body_link_state_indices_[i] << std::endl;
    gsr->attached_body_decompositions_.push_back(
        getAttachedBodySphereDecomposition(state.getAttachedBody(dfce->attached_body_names_[i]), resolution_));
    gsr->gradients_[i + dfce->link_names_.size()].types.resize(
        gsr->attached_body_decompositions_.back()->getCollisionSpheres().size(), NONE);
    gsr->gradients_[i + dfce->link_names_.size()].distances.resize(
        gsr->attached_body_decompositions_.back()->getCollisionSpheres().size(), DBL_MAX);
    gsr->gradients_[i + dfce->link_names_.size()].gradients.resize(
        gsr->attached_body_decompositions_.back()->getCollisionSpheres().size());
    gsr->gradients_[i + dfce->link_names_.size()].sphere_locations =
        gsr->attached_body_decompositions_.back()->getSphereCenters();
    gsr->gradients_[i + dfce->link_names_.size()].sphere_radii =
        gsr->attached_body_decompositions_.back()->getSphereRadii();
    gsr->gradients_[i + dfce->link_names_.size()].joint_name = ls->getParentJointModel()->getName();
  }
}

bool CollisionEnvDistanceField::compareCacheEntryToState(const DistanceFieldCacheEntryConstPtr& dfce,
                                                         const moveit::core::RobotState& state) const
{
  std::vector<double> new_state_values(state.getVariableCount());
  for (unsigned int i = 0; i < new_state_values.size(); i++)
  {
    new_state_values[i] = state.getVariablePosition(i);
  }

  if (dfce->state_values_.size() != new_state_values.size())
  {
    ROS_ERROR("State value size mismatch");
    return false;
  }

  for (unsigned int i = 0; i < dfce->state_check_indices_.size(); i++)
  {
    double diff =
        fabs(dfce->state_values_[dfce->state_check_indices_[i]] - new_state_values[dfce->state_check_indices_[i]]);
    if (diff > EPSILON)
    {
      ROS_WARN_STREAM("State for Variable " << state.getVariableNames()[dfce->state_check_indices_[i]]
                                            << " has changed by " << diff << " radians");
      return false;
    }
  }
  std::vector<const moveit::core::AttachedBody*> attached_bodies_dfce;
  std::vector<const moveit::core::AttachedBody*> attached_bodies_state;
  dfce->state_->getAttachedBodies(attached_bodies_dfce);
  state.getAttachedBodies(attached_bodies_state);
  if (attached_bodies_dfce.size() != attached_bodies_state.size())
  {
    return false;
  }
  // TODO - figure all the things that can change
  for (unsigned int i = 0; i < attached_bodies_dfce.size(); i++)
  {
    if (attached_bodies_dfce[i]->getName() != attached_bodies_state[i]->getName())
    {
      return false;
    }
    if (attached_bodies_dfce[i]->getTouchLinks() != attached_bodies_state[i]->getTouchLinks())
    {
      return false;
    }
    if (attached_bodies_dfce[i]->getShapes().size() != attached_bodies_state[i]->getShapes().size())
    {
      return false;
    }
    for (unsigned int j = 0; j < attached_bodies_dfce[i]->getShapes().size(); j++)
    {
      if (attached_bodies_dfce[i]->getShapes()[j] != attached_bodies_state[i]->getShapes()[j])
      {
        return false;
      }
    }
  }
  return true;
}

bool CollisionEnvDistanceField::compareCacheEntryToAllowedCollisionMatrix(
    const DistanceFieldCacheEntryConstPtr& dfce, const collision_detection::AllowedCollisionMatrix& acm) const
{
  if (dfce->acm_.getSize() != acm.getSize())
  {
    ROS_DEBUG("Allowed collision matrix size mismatch");
    return false;
  }
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  dfce->state_->getAttachedBodies(attached_bodies);
  for (unsigned int i = 0; i < dfce->link_names_.size(); i++)
  {
    std::string link_name = dfce->link_names_[i];
    if (dfce->link_has_geometry_[i])
    {
      bool self_collision_enabled = true;
      collision_detection::AllowedCollision::Type t;
      if (acm.getEntry(link_name, link_name, t))
      {
        if (t == collision_detection::AllowedCollision::ALWAYS)
        {
          self_collision_enabled = false;
        }
      }
      if (self_collision_enabled != dfce->self_collision_enabled_[i])
      {
        // ROS_INFO_STREAM("Self collision for " << link_name << " went from "
        // << dfce->self_collision_enabled_[i] << " to " <<
        // self_collision_enabled);
        return false;
      }
      for (unsigned int j = i; j < dfce->link_names_.size(); j++)
      {
        if (i == j)
          continue;
        if (dfce->link_has_geometry_[j])
        {
          bool intra_collision_enabled = true;
          if (acm.getEntry(link_name, dfce->link_names_[j], t))
          {
            if (t == collision_detection::AllowedCollision::ALWAYS)
            {
              intra_collision_enabled = false;
            }
          }
          if (dfce->intra_group_collision_enabled_[i][j] != intra_collision_enabled)
          {
            // std::cerr << "Intra collision for " << dfce->link_names_[i] << "
            // " << dfce->link_names_[j]
            //           << " went from " <<
            //           dfce->intra_group_collision_enabled_[i][j] << " to " <<
            //           intra_collision_enabled << std::endl;
            return false;
          }
        }
      }
    }
  }
  return true;
}

// void
// CollisionEnvDistanceField::generateAllowedCollisionInformation(CollisionEnvDistanceField::DistanceFieldCacheEntryPtr&
// dfce)
// {
//   for(unsigned int i = 0; i < dfce.link_names_.size(); i++) {
//     for(unsigned int j = 0; j <
//     if(dfce->acm.find
//   }
// }

void CollisionEnvDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state) const
{
  GroupStateRepresentationPtr gsr;
  checkCollision(req, res, state, gsr);
}

void CollisionEnvDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state,
                                               GroupStateRepresentationPtr& gsr) const
{
  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, nullptr, gsr, true);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }
  bool done = getSelfCollisions(req, res, gsr);
  if (!done)
  {
    done = getIntraGroupCollisions(req, res, gsr);
  }
  if (!done)
  {
    getEnvironmentCollisions(req, res, distance_field_cache_entry_world_->distance_field_, gsr);
  }

  (const_cast<CollisionEnvDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionEnvDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state,
                                               const AllowedCollisionMatrix& acm) const
{
  GroupStateRepresentationPtr gsr;
  checkCollision(req, res, state, acm, gsr);
}

void CollisionEnvDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm,
                                               GroupStateRepresentationPtr& gsr) const
{
  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, &acm, gsr, true);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }
  bool done = getSelfCollisions(req, res, gsr);
  if (!done)
  {
    done = getIntraGroupCollisions(req, res, gsr);
  }
  if (!done)
  {
    getEnvironmentCollisions(req, res, distance_field_cache_entry_world_->distance_field_, gsr);
  }

  (const_cast<CollisionEnvDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionEnvDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                    const moveit::core::RobotState& state) const
{
  GroupStateRepresentationPtr gsr;
  checkRobotCollision(req, res, state, gsr);
}

void CollisionEnvDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                    const moveit::core::RobotState& state,
                                                    GroupStateRepresentationPtr& gsr) const
{
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_world_->distance_field_;
  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, nullptr, gsr, false);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }
  getEnvironmentCollisions(req, res, env_distance_field, gsr);
  (const_cast<CollisionEnvDistanceField*>(this))->last_gsr_ = gsr;

  // checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionEnvDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                    const moveit::core::RobotState& state,
                                                    const AllowedCollisionMatrix& acm) const
{
  GroupStateRepresentationPtr gsr;
  checkRobotCollision(req, res, state, acm, gsr);
}

void CollisionEnvDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                    const moveit::core::RobotState& state,
                                                    const AllowedCollisionMatrix& acm,
                                                    GroupStateRepresentationPtr& gsr) const
{
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_world_->distance_field_;

  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, &acm, gsr, true);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }
  getEnvironmentCollisions(req, res, env_distance_field, gsr);
  (const_cast<CollisionEnvDistanceField*>(this))->last_gsr_ = gsr;

  // checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void CollisionEnvDistanceField::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                                    const moveit::core::RobotState& /*state1*/,
                                                    const moveit::core::RobotState& /*state2*/,
                                                    const AllowedCollisionMatrix& /*acm*/) const
{
  ROS_ERROR_NAMED("collision_detection.distance", "Continuous collision checking not implemented");
}

void CollisionEnvDistanceField::checkRobotCollision(const CollisionRequest& /*req*/, CollisionResult& /*res*/,
                                                    const moveit::core::RobotState& /*state1*/,
                                                    const moveit::core::RobotState& /*state2*/) const
{
  ROS_ERROR_NAMED("collision_detection.distance", "Continuous collision checking not implemented");
}

void CollisionEnvDistanceField::getCollisionGradients(const CollisionRequest& req, CollisionResult& /*res*/,
                                                      const moveit::core::RobotState& state,
                                                      const AllowedCollisionMatrix* acm,
                                                      GroupStateRepresentationPtr& gsr) const
{
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_world_->distance_field_;

  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, acm, gsr, true);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }

  getSelfProximityGradients(gsr);
  getIntraGroupProximityGradients(gsr);
  getEnvironmentProximityGradients(env_distance_field, gsr);

  (const_cast<CollisionEnvDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionEnvDistanceField::getAllCollisions(const CollisionRequest& req, CollisionResult& res,
                                                 const moveit::core::RobotState& state,
                                                 const AllowedCollisionMatrix* acm,
                                                 GroupStateRepresentationPtr& gsr) const
{
  if (!gsr)
  {
    generateCollisionCheckingStructures(req.group_name, state, acm, gsr, true);
  }
  else
  {
    updateGroupStateRepresentationState(state, gsr);
  }
  getSelfCollisions(req, res, gsr);
  getIntraGroupCollisions(req, res, gsr);
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_world_->distance_field_;
  getEnvironmentCollisions(req, res, env_distance_field, gsr);

  (const_cast<CollisionEnvDistanceField*>(this))->last_gsr_ = gsr;
}

bool CollisionEnvDistanceField::getEnvironmentCollisions(const CollisionRequest& req, CollisionResult& res,
                                                         const distance_field::DistanceFieldConstPtr& env_distance_field,
                                                         GroupStateRepresentationPtr& gsr) const
{
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size() + gsr->dfce_->attached_body_names_.size(); i++)
  {
    bool is_link = i < gsr->dfce_->link_names_.size();
    std::string link_name = i < gsr->dfce_->link_names_.size() ? gsr->dfce_->link_names_[i] : "attached";
    if (is_link && !gsr->dfce_->link_has_geometry_[i])
    {
      continue;
    }

    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;

    if (is_link)
    {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    }
    else
    {
      collision_spheres_1 =
          &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters());
    }

    if (req.contacts)
    {
      std::vector<unsigned int> colls;
      bool coll =
          getCollisionSphereCollision(env_distance_field.get(), *collision_spheres_1, *sphere_centers_1,
                                      max_propogation_distance_, collision_tolerance_,
                                      std::min(req.max_contacts_per_pair, req.max_contacts - res.contact_count), colls);
      if (coll)
      {
        res.collision = true;
        for (unsigned int col : colls)
        {
          Contact con;
          if (is_link)
          {
            con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[col];
            con.body_type_1 = BodyTypes::ROBOT_LINK;
            con.body_name_1 = gsr->dfce_->link_names_[i];
          }
          else
          {
            con.pos = gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters()[col];
            con.body_type_1 = BodyTypes::ROBOT_ATTACHED;
            con.body_name_1 = gsr->dfce_->attached_body_names_[i - gsr->dfce_->link_names_.size()];
          }

          con.body_type_2 = BodyTypes::WORLD_OBJECT;
          con.body_name_2 = "environment";
          res.contact_count++;
          res.contacts[std::pair<std::string, std::string>(con.body_name_1, con.body_name_2)].push_back(con);
          gsr->gradients_[i].types[col] = ENVIRONMENT;
          // ROS_DEBUG_STREAM("Link " << dfce->link_names_[i] << " sphere " <<
          // colls[j] << " in env collision");
        }

        gsr->gradients_[i].collision = true;
        if (res.contact_count >= req.max_contacts)
        {
          return true;
        }
      }
    }
    else
    {
      bool coll = getCollisionSphereCollision(env_distance_field.get(), *collision_spheres_1, *sphere_centers_1,
                                              max_propogation_distance_, collision_tolerance_);
      if (coll)
      {
        res.collision = true;
        return true;
      }
    }
  }
  return (res.contact_count >= req.max_contacts);
}

bool CollisionEnvDistanceField::getEnvironmentProximityGradients(
    const distance_field::DistanceFieldConstPtr& env_distance_field, GroupStateRepresentationPtr& gsr) const
{
  bool in_collision = false;
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++)
  {
    bool is_link = i < gsr->dfce_->link_names_.size();

    if (is_link && !gsr->dfce_->link_has_geometry_[i])
    {
      continue;
    }

    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;
    if (is_link)
    {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    }
    else
    {
      collision_spheres_1 =
          &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters());
    }

    bool coll = getCollisionSphereGradients(env_distance_field.get(), *collision_spheres_1, *sphere_centers_1,
                                            gsr->gradients_[i], ENVIRONMENT, collision_tolerance_, false,
                                            max_propogation_distance_, false);
    if (coll)
    {
      in_collision = true;
    }
  }
  return in_collision;
}

void CollisionEnvDistanceField::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  distance_field_cache_entry_world_->distance_field_->reset();

  CollisionEnv::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { return notifyObjectChange(object, action); });

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionEnvDistanceField::notifyObjectChange(const ObjectConstPtr& obj, World::Action action)
{
  ros::WallTime n = ros::WallTime::now();

  EigenSTL::vector_Vector3d add_points;
  EigenSTL::vector_Vector3d subtract_points;
  updateDistanceObject(obj->id_, distance_field_cache_entry_world_, add_points, subtract_points);

  if (action == World::DESTROY)
  {
    distance_field_cache_entry_world_->distance_field_->removePointsFromField(subtract_points);
  }
  else if (action & (World::MOVE_SHAPE | World::REMOVE_SHAPE))
  {
    distance_field_cache_entry_world_->distance_field_->removePointsFromField(subtract_points);
    distance_field_cache_entry_world_->distance_field_->addPointsToField(add_points);
  }
  else
  {
    distance_field_cache_entry_world_->distance_field_->addPointsToField(add_points);
  }

  ROS_DEBUG_NAMED("collision_distance_field", "Modifying object %s took %lf s", obj->id_.c_str(),
                  (ros::WallTime::now() - n).toSec());
}

void CollisionEnvDistanceField::updateDistanceObject(const std::string& id, DistanceFieldCacheEntryWorldPtr& dfce,
                                                     EigenSTL::vector_Vector3d& add_points,
                                                     EigenSTL::vector_Vector3d& subtract_points)
{
  std::map<std::string, std::vector<PosedBodyPointDecompositionPtr>>::iterator cur_it =
      dfce->posed_body_point_decompositions_.find(id);
  if (cur_it != dfce->posed_body_point_decompositions_.end())
  {
    for (collision_detection::PosedBodyPointDecompositionPtr& posed_body_point_decomposition : cur_it->second)
    {
      subtract_points.insert(subtract_points.end(), posed_body_point_decomposition->getCollisionPoints().begin(),
                             posed_body_point_decomposition->getCollisionPoints().end());
    }
  }

  World::ObjectConstPtr object = getWorld()->getObject(id);
  if (object)
  {
    ROS_DEBUG_STREAM("Updating/Adding Object '" << object->id_ << "' with " << object->shapes_.size()
                                                << " shapes  to CollisionEnvDistanceField");
    std::vector<PosedBodyPointDecompositionPtr> shape_points;
    for (unsigned int i = 0; i < object->shapes_.size(); i++)
    {
      shapes::ShapeConstPtr shape = object->shapes_[i];
      if (shape->type == shapes::OCTREE)
      {
        const shapes::OcTree* octree_shape = static_cast<const shapes::OcTree*>(shape.get());
        std::shared_ptr<const octomap::OcTree> octree = octree_shape->octree;

        shape_points.push_back(std::make_shared<PosedBodyPointDecomposition>(octree));
      }
      else
      {
        BodyDecompositionConstPtr bd = getBodyDecompositionCacheEntry(shape, resolution_);
        shape_points.push_back(
            std::make_shared<PosedBodyPointDecomposition>(bd, getWorld()->getGlobalShapeTransform(id, i)));
      }

      add_points.insert(add_points.end(), shape_points.back()->getCollisionPoints().begin(),
                        shape_points.back()->getCollisionPoints().end());
    }

    dfce->posed_body_point_decompositions_[id] = shape_points;
  }
  else
  {
    ROS_DEBUG_STREAM("Removing Object '" << id << "' from CollisionEnvDistanceField");
    dfce->posed_body_point_decompositions_.erase(id);
  }
}

CollisionEnvDistanceField::DistanceFieldCacheEntryWorldPtr
CollisionEnvDistanceField::generateDistanceFieldCacheEntryWorld()
{
  DistanceFieldCacheEntryWorldPtr dfce(new DistanceFieldCacheEntryWorld());
  dfce->distance_field_ = std::make_shared<distance_field::PropagationDistanceField>(
      size_.x(), size_.y(), size_.z(), resolution_, origin_.x() - 0.5 * size_.x(), origin_.y() - 0.5 * size_.y(),
      origin_.z() - 0.5 * size_.z(), max_propogation_distance_, use_signed_distance_field_);

  EigenSTL::vector_Vector3d add_points;
  EigenSTL::vector_Vector3d subtract_points;
  for (const std::pair<const std::string, ObjectPtr>& object : *getWorld())
  {
    updateDistanceObject(object.first, dfce, add_points, subtract_points);
  }
  dfce->distance_field_->addPointsToField(add_points);
  return dfce;
}

const std::string& CollisionDetectorAllocatorDistanceField::getName() const
{
  return NAME;
}

}  // namespace collision_detection
