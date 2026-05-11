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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/thread/mutex.hpp>

namespace collision_detection
{
static const double DEFAULT_SIZE_X = 3.0;
static const double DEFAULT_SIZE_Y = 3.0;
static const double DEFAULT_SIZE_Z = 4.0;
static const bool DEFAULT_USE_SIGNED_DISTANCE_FIELD = false;
static const double DEFAULT_RESOLUTION = .02;
static const double DEFAULT_COLLISION_TOLERANCE = 0.0;
static const double DEFAULT_MAX_PROPOGATION_DISTANCE = .25;

MOVEIT_CLASS_FORWARD(CollisionEnvDistanceField);  // Defines CollisionEnvDistanceFieldPtr, ConstPtr, WeakPtr... etc

class CollisionEnvDistanceField : public CollisionEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionEnvDistanceField(const moveit::core::RobotModelConstPtr& robot_model,
                            const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions =
                                std::map<std::string, std::vector<CollisionSphere>>(),
                            double size_x = DEFAULT_SIZE_X, double size_y = DEFAULT_SIZE_Y,
                            double size_z = DEFAULT_SIZE_Z, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                            bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                            double resolution = DEFAULT_RESOLUTION,
                            double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                            double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE, double padding = 0.0,
                            double scale = 1.0);

  CollisionEnvDistanceField(const moveit::core::RobotModelConstPtr& robot_model, const WorldPtr& world,
                            const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions =
                                std::map<std::string, std::vector<CollisionSphere>>(),
                            double size_x = DEFAULT_SIZE_X, double size_y = DEFAULT_SIZE_Y,
                            double size_z = DEFAULT_SIZE_Z, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                            bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                            double resolution = DEFAULT_RESOLUTION,
                            double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                            double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE, double padding = 0.0,
                            double scale = 1.0);

  CollisionEnvDistanceField(const CollisionEnvDistanceField& other, const WorldPtr& world);

  void initialize(const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions,
                  const Eigen::Vector3d& size, const Eigen::Vector3d& origin, bool use_signed_distance_field,
                  double resolution, double collision_tolerance, double max_propogation_distance);

  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state, GroupStateRepresentationPtr& gsr) const;

  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state,
                          const collision_detection::AllowedCollisionMatrix& acm) const override;

  void checkSelfCollision(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state, const collision_detection::AllowedCollisionMatrix& acm,
                          GroupStateRepresentationPtr& gsr) const;

  void createCollisionModelMarker(const moveit::core::RobotState& state,
                                  visualization_msgs::MarkerArray& model_markers) const;

  virtual double distanceSelf(const moveit::core::RobotState& /* state */) const
  {
    return 0.0;
  }

  virtual double distanceSelf(const moveit::core::RobotState& /* state */,
                              const collision_detection::AllowedCollisionMatrix& /* acm */) const
  {
    return 0.0;
  }

  void distanceSelf(const DistanceRequest& /* req */, DistanceResult& /* res */,
                    const moveit::core::RobotState& /* state */) const override
  {
    ROS_ERROR_NAMED("collision_distance_field", "Not implemented");
  }

  DistanceFieldCacheEntryConstPtr getLastDistanceFieldEntry() const
  {
    return distance_field_cache_entry_;
  }

  // void getSelfCollisionsGradients(const collision_detection::CollisionRequest
  // &req,
  //                                 collision_detection::CollisionResult &res,
  //                                 const moveit::core::RobotState &state,
  //                                 const
  //                                 collision_detection::AllowedCollisionMatrix
  //                                 &acm) const;

  MOVEIT_STRUCT_FORWARD(DistanceFieldCacheEntryWorld);
  struct DistanceFieldCacheEntryWorld
  {
    std::map<std::string, std::vector<PosedBodyPointDecompositionPtr>> posed_body_point_decompositions_;
    distance_field::DistanceFieldPtr distance_field_;
  };

  ~CollisionEnvDistanceField() override;

  void checkCollision(const CollisionRequest& req, CollisionResult& res,
                      const moveit::core::RobotState& state) const override;

  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                              GroupStateRepresentationPtr& gsr) const;

  void checkCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                      const AllowedCollisionMatrix& acm) const override;

  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                              const AllowedCollisionMatrix& acm, GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                           const moveit::core::RobotState& state) const override;

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state, GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                           const AllowedCollisionMatrix& acm) const override;

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm,
                                   GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;

  virtual double distanceRobot(const moveit::core::RobotState& state, bool verbose = false) const
  {
    (void)state;
    (void)verbose;
    return 0.0;
  }

  virtual double distanceRobot(const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm,
                               bool verbose = false) const
  {
    (void)state;
    (void)acm;
    (void)verbose;
    return 0.0;
  }

  void distanceRobot(const DistanceRequest& /* req */, DistanceResult& /* res */,
                     const moveit::core::RobotState& /* state */) const override
  {
    ROS_ERROR_NAMED("collision_distance_field", "Not implemented");
  }

  void setWorld(const WorldPtr& world) override;

  distance_field::DistanceFieldConstPtr getDistanceField() const
  {
    return distance_field_cache_entry_->distance_field_;
  }

  collision_detection::GroupStateRepresentationConstPtr getLastGroupStateRepresentation() const
  {
    return last_gsr_;
  }

  void getCollisionGradients(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                             const AllowedCollisionMatrix* acm, GroupStateRepresentationPtr& gsr) const;

  void getAllCollisions(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                        const AllowedCollisionMatrix* acm, GroupStateRepresentationPtr& gsr) const;

protected:
  bool getSelfProximityGradients(GroupStateRepresentationPtr& gsr) const;

  bool getIntraGroupProximityGradients(GroupStateRepresentationPtr& gsr) const;

  bool getSelfCollisions(const collision_detection::CollisionRequest& req, collision_detection::CollisionResult& res,
                         GroupStateRepresentationPtr& gsr) const;

  bool getIntraGroupCollisions(const collision_detection::CollisionRequest& req,
                               collision_detection::CollisionResult& res, GroupStateRepresentationPtr& gsr) const;

  void checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
                                const collision_detection::AllowedCollisionMatrix* acm,
                                GroupStateRepresentationPtr& gsr) const;

  void updateGroupStateRepresentationState(const moveit::core::RobotState& state,
                                           GroupStateRepresentationPtr& gsr) const;

  void generateCollisionCheckingStructures(const std::string& group_name, const moveit::core::RobotState& state,
                                           const collision_detection::AllowedCollisionMatrix* acm,
                                           GroupStateRepresentationPtr& gsr, bool generate_distance_field) const;

  DistanceFieldCacheEntryConstPtr
  getDistanceFieldCacheEntry(const std::string& group_name, const moveit::core::RobotState& state,
                             const collision_detection::AllowedCollisionMatrix* acm) const;

  DistanceFieldCacheEntryPtr generateDistanceFieldCacheEntry(const std::string& group_name,
                                                             const moveit::core::RobotState& state,
                                                             const collision_detection::AllowedCollisionMatrix* acm,
                                                             bool generate_distance_field) const;

  void addLinkBodyDecompositions(double resolution);

  void addLinkBodyDecompositions(double resolution,
                                 const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions);

  PosedBodySphereDecompositionPtr getPosedLinkBodySphereDecomposition(const moveit::core::LinkModel* ls,
                                                                      unsigned int ind) const;

  PosedBodyPointDecompositionPtr getPosedLinkBodyPointDecomposition(const moveit::core::LinkModel* ls) const;

  void getGroupStateRepresentation(const DistanceFieldCacheEntryConstPtr& dfce, const moveit::core::RobotState& state,
                                   GroupStateRepresentationPtr& gsr) const;

  bool compareCacheEntryToState(const DistanceFieldCacheEntryConstPtr& dfce,
                                const moveit::core::RobotState& state) const;

  bool compareCacheEntryToAllowedCollisionMatrix(const DistanceFieldCacheEntryConstPtr& dfce,
                                                 const collision_detection::AllowedCollisionMatrix& acm) const;

  void updatedPaddingOrScaling(const std::vector<std::string>& /*links*/) override{};

  DistanceFieldCacheEntryWorldPtr generateDistanceFieldCacheEntryWorld();

  void updateDistanceObject(const std::string& id, CollisionEnvDistanceField::DistanceFieldCacheEntryWorldPtr& dfce,
                            EigenSTL::vector_Vector3d& add_points, EigenSTL::vector_Vector3d& subtract_points);

  bool getEnvironmentCollisions(const CollisionRequest& req, CollisionResult& res,
                                const distance_field::DistanceFieldConstPtr& env_distance_field,
                                GroupStateRepresentationPtr& gsr) const;

  bool getEnvironmentProximityGradients(const distance_field::DistanceFieldConstPtr& env_distance_field,
                                        GroupStateRepresentationPtr& gsr) const;

  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);

  Eigen::Vector3d size_;
  Eigen::Vector3d origin_;
  bool use_signed_distance_field_;
  double resolution_;
  double collision_tolerance_;
  double max_propogation_distance_;

  std::vector<BodyDecompositionConstPtr> link_body_decomposition_vector_;
  std::map<std::string, unsigned int> link_body_decomposition_index_map_;

  mutable boost::mutex update_cache_lock_;
  DistanceFieldCacheEntryPtr distance_field_cache_entry_;
  std::map<std::string, std::map<std::string, bool>> in_group_update_map_;
  std::map<std::string, GroupStateRepresentationPtr> pregenerated_group_state_representation_map_;

  planning_scene::PlanningScenePtr planning_scene_;

  mutable boost::mutex update_cache_lock_world_;
  DistanceFieldCacheEntryWorldPtr distance_field_cache_entry_world_;
  GroupStateRepresentationPtr last_gsr_;
  World::ObserverHandle observer_handle_;
};
}  // namespace collision_detection
