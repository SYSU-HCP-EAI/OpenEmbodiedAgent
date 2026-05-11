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

/* Author: E. Gil Jones, Jens Petit */

#pragma once

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>

#include <boost/thread/mutex.hpp>

namespace collision_detection
{
/** \brief This hybrid collision environment combines FCL and a distance field. Both can be used to calculate
 *  collisions. */
class CollisionEnvHybrid : public collision_detection::CollisionEnvFCL
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionEnvHybrid(const moveit::core::RobotModelConstPtr& robot_model,
                     const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions =
                         std::map<std::string, std::vector<CollisionSphere>>(),
                     double size_x = DEFAULT_SIZE_X, double size_y = DEFAULT_SIZE_Y, double size_z = DEFAULT_SIZE_Z,
                     const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                     bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                     double resolution = DEFAULT_RESOLUTION, double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                     double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE, double padding = 0.0,
                     double scale = 1.0);

  CollisionEnvHybrid(const moveit::core::RobotModelConstPtr& robot_model, const WorldPtr& world,
                     const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions =
                         std::map<std::string, std::vector<CollisionSphere>>(),
                     double size_x = DEFAULT_SIZE_X, double size_y = DEFAULT_SIZE_Y, double size_z = DEFAULT_SIZE_Z,
                     const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                     bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                     double resolution = DEFAULT_RESOLUTION, double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                     double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE, double padding = 0.0,
                     double scale = 1.0);

  CollisionEnvHybrid(const CollisionEnvHybrid& other, const WorldPtr& world);

  ~CollisionEnvHybrid() override
  {
  }

  void initializeRobotDistanceField(const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions,
                                    double size_x, double size_y, double size_z, bool use_signed_distance_field,
                                    double resolution, double collision_tolerance, double max_propogation_distance)
  {
    cenv_distance_->initialize(link_body_decompositions, Eigen::Vector3d(size_x, size_y, size_z),
                               Eigen::Vector3d(0, 0, 0), use_signed_distance_field, resolution, collision_tolerance,
                               max_propogation_distance);
  }

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res,
                                       const moveit::core::RobotState& state) const;

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
                                       GroupStateRepresentationPtr& gsr) const;

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
                                       const collision_detection::AllowedCollisionMatrix& acm) const;

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
                                       const collision_detection::AllowedCollisionMatrix& acm,
                                       GroupStateRepresentationPtr& gsr) const;
  const CollisionEnvDistanceFieldConstPtr getCollisionRobotDistanceField() const
  {
    return cenv_distance_;
  }

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state) const;

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state, GroupStateRepresentationPtr& gsr) const;

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const;

  void checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm,
                                   GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                        const moveit::core::RobotState& state) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                        const moveit::core::RobotState& state, GroupStateRepresentationPtr& gsr) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                        const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const;

  void checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                        const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm,
                                        GroupStateRepresentationPtr& gsr) const;

  void setWorld(const WorldPtr& world) override;

  void getCollisionGradients(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                             const AllowedCollisionMatrix* acm, GroupStateRepresentationPtr& gsr) const;

  void getAllCollisions(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                        const AllowedCollisionMatrix* acm, GroupStateRepresentationPtr& gsr) const;

  const CollisionEnvDistanceFieldConstPtr getCollisionWorldDistanceField() const
  {
    return cenv_distance_;
  }

protected:
  CollisionEnvDistanceFieldPtr cenv_distance_;
};
}  // namespace collision_detection
