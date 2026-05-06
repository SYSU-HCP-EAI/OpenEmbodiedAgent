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

#pragma once

#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection_fcl/collision_common.h>

#if (MOVEIT_FCL_VERSION >= FCL_VERSION_CHECK(0, 6, 0))
#include <fcl/broadphase/broadphase_collision_manager.h>
#else
#include <fcl/broadphase/broadphase.h>
#endif

#include <memory>

namespace collision_detection
{
/** \brief FCL implementation of the CollisionEnv */
class CollisionEnvFCL : public CollisionEnv
{
public:
  CollisionEnvFCL() = delete;

  CollisionEnvFCL(const moveit::core::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);

  CollisionEnvFCL(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding = 0.0,
                  double scale = 1.0);

  CollisionEnvFCL(const CollisionEnvFCL& other, const WorldPtr& world);

  ~CollisionEnvFCL() override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                           const moveit::core::RobotState& state) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                           const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
                           const moveit::core::RobotState& state2) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const moveit::core::RobotState& state) const override;

  void distanceRobot(const DistanceRequest& req, DistanceResult& res,
                     const moveit::core::RobotState& state) const override;

  void setWorld(const WorldPtr& world) override;

protected:
  /** \brief Updates the FCL collision geometry and objects saved in the CollisionRobotFCL members to reflect a new
   *   padding or scaling of the robot links.
   *
   *   It searches for the link through the pointed-to robot model of the CollisionRobot and then constructs new FCL
   *   collision objects and geometries depending on the changed robot model.
   *
   *   \param links The names of the links which have been updated in the robot model */
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;

  /** \brief Bundles the different checkSelfCollision functions into a single function */
  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

  /** \brief Bundles the different checkRobotCollision functions into a single function */
  void checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                 const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

  /** \brief Construct an FCL collision object from MoveIt's World::Object. */
  void constructFCLObjectWorld(const World::Object* obj, FCLObject& fcl_obj) const;

  /** \brief Updates the specified object in \m fcl_objs_ and in the manager from new data available in the World.
   *
   *  If it does not exist in world, it is deleted. If it's not existing in \m fcl_objs_ yet, it's added there. */
  void updateFCLObject(const std::string& id);

  /** \brief Out of the current robot state and its attached bodies construct an FCLObject which can then be used to
   *   check for collision.
   *
   *   The current state is used to recalculate the AABB of the FCL collision objects. However they are not computed from
   *   scratch (which would require call to computeLocalAABB()) but are only transformed according to the joint states.
   *
   *   \param state The current robot state
   *   \param fcl_obj The newly filled object */
  void constructFCLObjectRobot(const moveit::core::RobotState& state, FCLObject& fcl_obj) const;

  /** \brief Prepares for the collision check through constructing an FCL collision object out of the current robot
   *   state and specifying a broadphase collision manager of FCL where the constructed object is registered to. */
  void allocSelfCollisionBroadPhase(const moveit::core::RobotState& state, FCLManager& manager) const;

  /** \brief Converts all shapes which make up an atttached body into a vector of FCLGeometryConstPtr.
   *
   *   When they are converted, they can be added to the FCL representation of the robot for collision checking.
   *
   *   \param ab Pointer to the attached body
   *   \param geoms Output vector of geometries
   */
  void getAttachedBodyObjects(const moveit::core::AttachedBody* ab, std::vector<FCLGeometryConstPtr>& geoms) const;

  /** \brief Vector of shared pointers to the FCL geometry for the objects in fcl_objs_. */
  std::vector<FCLGeometryConstPtr> robot_geoms_;

  /** \brief Vector of shared pointers to the FCL collision objects which make up the robot */
  std::vector<FCLCollisionObjectConstPtr> robot_fcl_objs_;

  /// FCL collision manager which handles the collision checking process
  std::unique_ptr<fcl::BroadPhaseCollisionManagerd> manager_;

  std::map<std::string, FCLObject> fcl_objs_;

private:
  /** \brief Callback function executed for each change to the world environment */
  void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);

  World::ObserverHandle observer_handle_;
};
}  // namespace collision_detection
