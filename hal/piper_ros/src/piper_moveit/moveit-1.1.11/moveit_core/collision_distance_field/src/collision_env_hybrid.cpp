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

#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/collision_distance_field/collision_env_hybrid.h>

namespace collision_detection
{
namespace
{
static const std::string NAME = "HYBRID";
}  // namespace

CollisionEnvHybrid::CollisionEnvHybrid(
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution,
    double collision_tolerance, double max_propogation_distance, double padding, double scale)
  : CollisionEnvFCL(robot_model)
  , cenv_distance_(new collision_detection::CollisionEnvDistanceField(
        robot_model, getWorld(), link_body_decompositions, size_x, size_y, size_z, origin, use_signed_distance_field,
        resolution, collision_tolerance, max_propogation_distance, padding, scale))
{
}

CollisionEnvHybrid::CollisionEnvHybrid(
    const moveit::core::RobotModelConstPtr& robot_model, const WorldPtr& world,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, const Eigen::Vector3d& origin, bool use_signed_distance_field, double resolution,
    double collision_tolerance, double max_propogation_distance, double padding, double scale)
  : CollisionEnvFCL(robot_model, world, padding, scale)
  , cenv_distance_(new collision_detection::CollisionEnvDistanceField(
        robot_model, getWorld(), link_body_decompositions, size_x, size_y, size_z, origin, use_signed_distance_field,
        resolution, collision_tolerance, max_propogation_distance, padding, scale))
{
}

CollisionEnvHybrid::CollisionEnvHybrid(const CollisionEnvHybrid& other, const WorldPtr& world)
  : CollisionEnvFCL(other, world)
  , cenv_distance_(new collision_detection::CollisionEnvDistanceField(*other.getCollisionWorldDistanceField(), world))
{
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state) const
{
  cenv_distance_->checkSelfCollision(req, res, state);
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state,
                                                         GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkSelfCollision(req, res, state, gsr);
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state,
                                                         const collision_detection::AllowedCollisionMatrix& acm) const
{
  cenv_distance_->checkSelfCollision(req, res, state, acm);
}

void CollisionEnvHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                         collision_detection::CollisionResult& res,
                                                         const moveit::core::RobotState& state,
                                                         const collision_detection::AllowedCollisionMatrix& acm,
                                                         GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkSelfCollision(req, res, state, acm, gsr);
}

void CollisionEnvHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                     const moveit::core::RobotState& state) const
{
  cenv_distance_->checkCollision(req, res, state);
}

void CollisionEnvHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                     const moveit::core::RobotState& state,
                                                     GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkCollision(req, res, state, gsr);
}

void CollisionEnvHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                     const moveit::core::RobotState& state,
                                                     const AllowedCollisionMatrix& acm) const
{
  cenv_distance_->checkCollision(req, res, state, acm);
}

void CollisionEnvHybrid::checkCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                     const moveit::core::RobotState& state,
                                                     const AllowedCollisionMatrix& acm,
                                                     GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkCollision(req, res, state, acm, gsr);
}

void CollisionEnvHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                          const moveit::core::RobotState& state) const
{
  cenv_distance_->checkRobotCollision(req, res, state);
}

void CollisionEnvHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                          const moveit::core::RobotState& state,
                                                          GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkRobotCollision(req, res, state, gsr);
}

void CollisionEnvHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                          const moveit::core::RobotState& state,
                                                          const AllowedCollisionMatrix& acm) const
{
  cenv_distance_->checkRobotCollision(req, res, state, acm);
}

void CollisionEnvHybrid::checkRobotCollisionDistanceField(const CollisionRequest& req, CollisionResult& res,
                                                          const moveit::core::RobotState& state,
                                                          const AllowedCollisionMatrix& acm,
                                                          GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->checkRobotCollision(req, res, state, acm, gsr);
}

void CollisionEnvHybrid::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  cenv_distance_->setWorld(world);
  CollisionEnvFCL::setWorld(world);
}

void CollisionEnvHybrid::getCollisionGradients(const CollisionRequest& req, CollisionResult& res,
                                               const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm,
                                               GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->getCollisionGradients(req, res, state, acm, gsr);
}

void CollisionEnvHybrid::getAllCollisions(const CollisionRequest& req, CollisionResult& res,
                                          const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm,
                                          GroupStateRepresentationPtr& gsr) const
{
  cenv_distance_->getAllCollisions(req, res, state, acm, gsr);
}

const std::string& CollisionDetectorAllocatorHybrid::getName() const
{
  return NAME;
}

}  // namespace collision_detection
