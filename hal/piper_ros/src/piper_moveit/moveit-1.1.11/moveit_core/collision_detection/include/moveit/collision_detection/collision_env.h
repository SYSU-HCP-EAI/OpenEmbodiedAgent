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

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/LinkPadding.h>
#include <moveit_msgs/LinkScale.h>
#include <moveit/collision_detection/world.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(CollisionEnv);  // Defines CollisionEnvPtr, ConstPtr, WeakPtr... etc

/** \brief Provides the interface to the individual collision checking libraries. */
class CollisionEnv
{
public:
  CollisionEnv() = delete;

  /** @brief Constructor
   *  @param model A robot model to construct the collision robot from
   *  @param padding The padding to use for all objects/links on the robot
   *  @scale scale A common scaling to use for all objects/links on the robot
   */
  CollisionEnv(const moveit::core::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);

  /** @brief Constructor
   *  @param model A robot model to construct the collision robot from
   *  @param padding The padding to use for all objects/links on the robot
   *  @scale scale A common scaling to use for all objects/links on the robot
   */
  CollisionEnv(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding = 0.0,
               double scale = 1.0);

  /** \brief Copy constructor */
  CollisionEnv(const CollisionEnv& other, const WorldPtr& world);

  virtual ~CollisionEnv()
  {
  }

  /** @brief Check for robot self collision. Any collision between any pair of links is checked for, NO collisions are
   *   ignored.
   *
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made */
  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                  const moveit::core::RobotState& state) const = 0;

  /** \brief Check for self collision. Allowed collisions specified by the allowed collision matrix are
   *   taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made
   *  @param acm The allowed collision matrix. */
  virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                  const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check whether the robot model is in collision with itself or the world at a particular state.
   *  Any collision between any pair of links is checked for, NO collisions are ignored.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made         */
  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res,
                              const moveit::core::RobotState& state) const;

  /** \brief Check whether the robot model is in collision with itself or the world at a particular state.
   *  Allowed collisions specified by the allowed collision matrix are taken into account.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @param state The kinematic state for which checks are being made
   *  @param acm The allowed collision matrix. */
  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                              const AllowedCollisionMatrix& acm) const;

  /** \brief Check whether the robot model is in collision with the world. Any collisions between a robot link
   *  and the world are considered. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state The kinematic state for which checks are being made
   */
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state) const = 0;

  /** \brief Check whether the robot model is in collision with the world.
   *  Allowed collisions are ignored. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state The kinematic state for which checks are being made
   *  @param acm The allowed collision matrix.*/
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check whether the robot model is in collision with the world in a continuous manner (between two robot
   * states).
   *  Allowed collisions are ignored. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made
   *  @param acm The allowed collision matrix.*/
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state1, const moveit::core::RobotState& state2,
                                   const AllowedCollisionMatrix& acm) const = 0;

  /** \brief Check whether the robot model is in collision with the world in a continuous manner (between two robot
   * states).
   *  Allowed collisions are ignored. Self collisions are not checked.
   *  @param req A CollisionRequest object that encapsulates the collision request
   *  @param res A CollisionResult object that encapsulates the collision result
   *  @robot robot The collision model for the robot
   *  @param state1 The kinematic state at the start of the segment for which checks are being made
   *  @param state2 The kinematic state at the end of the segment for which checks are being made
   *  @param acm The allowed collision matrix.*/
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                   const moveit::core::RobotState& state1,
                                   const moveit::core::RobotState& state2) const = 0;

  /** \brief The distance to self-collision given the robot is at state \e state.
      @param req A DistanceRequest object that encapsulates the distance request
      @param res A DistanceResult object that encapsulates the distance result
      @param state The state of this robot to consider */
  virtual void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                            const moveit::core::RobotState& state) const = 0;

  /** \brief The distance to self-collision given the robot is at state \e state. */
  inline double distanceSelf(const moveit::core::RobotState& state) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.enableGroup(getRobotModel());
    distanceSelf(req, res, state);
    return res.minimum_distance.distance;
  }

  /** \brief The distance to self-collision given the robot is at state \e state, ignoring
      the distances between links that are allowed to always collide (as specified by \e acm) */
  inline double distanceSelf(const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.enableGroup(getRobotModel());
    req.acm = &acm;
    distanceSelf(req, res, state);
    return res.minimum_distance.distance;
  }

  /** \brief Compute the distance between a robot and the world
   *  @param req A DistanceRequest object that encapsulates the distance request
   *  @param res A DistanceResult object that encapsulates the distance result
   *  @param robot The robot to check distance for
   *  @param state The state for the robot to check distances from */
  virtual void distanceRobot(const DistanceRequest& req, DistanceResult& res,
                             const moveit::core::RobotState& state) const = 0;

  /** \brief Compute the shortest distance between a robot and the world
   *  @param robot The robot to check distance for
   *  @param state The state for the robot to check distances from
   *  @param verbose Output debug information about distance checks */
  inline double distanceRobot(const moveit::core::RobotState& state, bool verbose = false) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.verbose = verbose;
    req.enableGroup(getRobotModel());

    distanceRobot(req, res, state);
    return res.minimum_distance.distance;
  }

  /** \brief Compute the shortest distance between a robot and the world
   *  @param robot The robot to check distance for
   *  @param state The state for the robot to check distances from
   *  @param acm Using an allowed collision matrix has the effect of ignoring distances from links that are always
   * allowed to be in collision.
   *  @param verbose Output debug information about distance checks */
  inline double distanceRobot(const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm,
                              bool verbose = false) const
  {
    DistanceRequest req;
    DistanceResult res;

    req.acm = &acm;
    req.verbose = verbose;
    req.enableGroup(getRobotModel());

    distanceRobot(req, res, state);
    return res.minimum_distance.distance;
  }

  /** set the world to use.
   * This can be expensive unless the new and old world are empty.
   * Passing NULL will result in a new empty world being created. */
  virtual void setWorld(const WorldPtr& world);

  /** access the world geometry */
  const WorldPtr& getWorld()
  {
    return world_;
  }

  /** access the world geometry */
  const WorldConstPtr& getWorld() const
  {
    return world_const_;
  }

  using ObjectPtr = World::ObjectPtr;
  using ObjectConstPtr = World::ObjectConstPtr;

  /** @brief The kinematic model corresponding to this collision model*/
  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** @brief Set the link padding for a particular link.
   *  @param link_name The link name to set padding for
   *  @param padding The padding to set (in meters)
   */
  void setLinkPadding(const std::string& link_name, double padding);

  /** @brief Get the link padding for a particular link*/
  double getLinkPadding(const std::string& link_name) const;

  /** @brief Set the link paddings using a map (from link names to padding value) */
  void setLinkPadding(const std::map<std::string, double>& padding);

  /** @brief Get the link paddings as a map (from link names to padding value) */
  const std::map<std::string, double>& getLinkPadding() const;

  /** @brief Set the scaling for a particular link*/
  void setLinkScale(const std::string& link_name, double scale);

  /** @brief Set the scaling for a particular link*/
  double getLinkScale(const std::string& link_name) const;

  /** @brief Set the link scaling using a map (from link names to scale value) */
  void setLinkScale(const std::map<std::string, double>& scale);

  /** @brief Get the link scaling as a map (from link names to scale value)*/
  const std::map<std::string, double>& getLinkScale() const;

  /** @brief Set the link padding (for every link)*/
  void setPadding(double padding);

  /** @brief Set the link scaling (for every link)*/
  void setScale(double scale);

  /** @brief Set the link padding from a vector of messages*/
  void setPadding(const std::vector<moveit_msgs::LinkPadding>& padding);

  /** @brief Get the link padding as a vector of messages*/
  void getPadding(std::vector<moveit_msgs::LinkPadding>& padding) const;

  /** @brief Set the link scaling from a vector of messages*/
  void setScale(const std::vector<moveit_msgs::LinkScale>& scale);

  /** @brief Get the link scaling as a vector of messages*/
  void getScale(std::vector<moveit_msgs::LinkScale>& scale) const;

protected:
  /** @brief When the scale or padding is changed for a set of links by any of the functions in this class,
     updatedPaddingOrScaling() function is called.
      This function has an empty default implementation. The intention is to override this function in a derived class
     to allow for updating
      additional structures that may need such updating when link scale or padding changes.
      @param links the names of the links whose padding or scaling were updated */
  virtual void updatedPaddingOrScaling(const std::vector<std::string>& links);

  /** @brief The kinematic model corresponding to this collision model*/
  moveit::core::RobotModelConstPtr robot_model_;

  /** @brief The internally maintained map (from link names to padding)*/
  std::map<std::string, double> link_padding_;

  /** @brief The internally maintained map (from link names to scaling)*/
  std::map<std::string, double> link_scale_;

private:
  WorldPtr world_;             // The world always valid, never nullptr.
  WorldConstPtr world_const_;  // always same as world_
};
}  // namespace collision_detection
