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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

namespace moveit
{
namespace planning_interface
{
MOVEIT_CLASS_FORWARD(PlanningSceneInterface);  // Defines PlanningSceneInterfacePtr, ConstPtr, WeakPtr... etc

class PlanningSceneInterface
{
public:
  /**
    \param ns. Namespace in which all MoveIt related topics and services are discovered
    \param wait. Wait for services if they are not announced in ROS.
    If this is false, the constructor throws std::runtime_error instead.
  */
  explicit PlanningSceneInterface(const std::string& ns = "", bool wait = true);
  ~PlanningSceneInterface();

  /**
   * \name Manage the world
   */
  /**@{*/

  /** \brief Get the names of all known objects in the world. If \e with_type is set to true, only return objects that
   * have a known type. */
  std::vector<std::string> getKnownObjectNames(bool with_type = false);

  /** \brief Get the names of known objects in the world that are located within a bounding region (specified in the
     frame reported by getPlanningFrame()).
       +      If \e with_type is set to true, only return objects that have a known type. */
  std::vector<std::string> getKnownObjectNamesInROI(double minx, double miny, double minz, double maxx, double maxy,
                                                    double maxz, bool with_type, std::vector<std::string>& types);

  /** \brief Get the names of known objects in the world that are located within a bounding region (specified in the
     frame reported by getPlanningFrame()).
      If \e with_type is set to true, only return objects that have a known type. */
  std::vector<std::string> getKnownObjectNamesInROI(double minx, double miny, double minz, double maxx, double maxy,
                                                    double maxz, bool with_type = false)
  {
    std::vector<std::string> empty_vector_string;
    return getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type, empty_vector_string);
  };

  /** \brief Get the poses from the objects identified by the given object ids list. */
  std::map<std::string, geometry_msgs::Pose> getObjectPoses(const std::vector<std::string>& object_ids);

  /** \brief Get the objects identified by the given object ids list. If no ids are provided, return all the known
   * objects. */
  std::map<std::string, moveit_msgs::CollisionObject>
  getObjects(const std::vector<std::string>& object_ids = std::vector<std::string>());

  /** \brief Get the attached objects identified by the given object ids list. If no ids are provided, return all the
   * attached objects. */
  std::map<std::string, moveit_msgs::AttachedCollisionObject>
  getAttachedObjects(const std::vector<std::string>& object_ids = std::vector<std::string>());

  /** \brief Apply collision object to the planning scene of the move_group node synchronously.
      Other PlanningSceneMonitors will NOT receive the update unless they subscribe to move_group's monitored scene */
  bool applyCollisionObject(const moveit_msgs::CollisionObject& collision_object);

  /** \brief Apply collision object to the planning scene of the move_group node synchronously.
      Other PlanningSceneMonitors will NOT receive the update unless they subscribe to move_group's monitored scene */
  bool applyCollisionObject(const moveit_msgs::CollisionObject& collision_object,
                            const std_msgs::ColorRGBA& object_color);

  /** \brief Apply collision objects to the planning scene of the move_group node synchronously.
      Other PlanningSceneMonitors will NOT receive the update unless they subscribe to move_group's monitored scene.
      If object_colors do not specify an id, the corresponding object id from collision_objects is used. */
  bool applyCollisionObjects(
      const std::vector<moveit_msgs::CollisionObject>& collision_objects,
      const std::vector<moveit_msgs::ObjectColor>& object_colors = std::vector<moveit_msgs::ObjectColor>());

  /** \brief Apply attached collision object to the planning scene of the move_group node synchronously.
      Other PlanningSceneMonitors will NOT receive the update unless they subscribe to move_group's monitored scene */
  bool applyAttachedCollisionObject(const moveit_msgs::AttachedCollisionObject& attached_collision_object);

  /** \brief Apply attached collision objects to the planning scene of the move_group node synchronously.
      Other PlanningSceneMonitors will NOT receive the update unless they subscribe to move_group's monitored scene */
  bool
  applyAttachedCollisionObjects(const std::vector<moveit_msgs::AttachedCollisionObject>& attached_collision_objects);

  /** \brief Update the planning_scene of the move_group node with the given ps synchronously.
      Other PlanningSceneMonitors will NOT receive the update unless they subscribe to move_group's monitored scene */
  bool applyPlanningScene(const moveit_msgs::PlanningScene& ps);

  /** \brief Add collision objects to the world via /planning_scene.
      Make sure object.operation is set to object.ADD.

      The update runs asynchronously. If you need the objects to be available *directly* after you called this function,
      consider using `applyCollisionObjects` instead. */
  void addCollisionObjects(
      const std::vector<moveit_msgs::CollisionObject>& collision_objects,
      const std::vector<moveit_msgs::ObjectColor>& object_colors = std::vector<moveit_msgs::ObjectColor>()) const;

  /** \brief Remove collision objects from the world via /planning_scene.

      The update runs asynchronously. If you need the objects to be removed *directly* after you called this function,
      consider using `applyCollisionObjects` instead. */
  void removeCollisionObjects(const std::vector<std::string>& object_ids) const;

  /**@}*/

private:
  class PlanningSceneInterfaceImpl;
  PlanningSceneInterfaceImpl* impl_;
};
}  // namespace planning_interface
}  // namespace moveit
