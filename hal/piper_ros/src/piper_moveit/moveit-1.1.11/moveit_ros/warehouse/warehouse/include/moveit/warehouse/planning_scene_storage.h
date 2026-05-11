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

#include "moveit/warehouse/moveit_message_storage.h"
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace moveit_warehouse
{
typedef warehouse_ros::MessageWithMetadata<moveit_msgs::PlanningScene>::ConstPtr PlanningSceneWithMetadata;
typedef warehouse_ros::MessageWithMetadata<moveit_msgs::MotionPlanRequest>::ConstPtr MotionPlanRequestWithMetadata;
typedef warehouse_ros::MessageWithMetadata<moveit_msgs::RobotTrajectory>::ConstPtr RobotTrajectoryWithMetadata;

typedef warehouse_ros::MessageCollection<moveit_msgs::PlanningScene>::Ptr PlanningSceneCollection;
typedef warehouse_ros::MessageCollection<moveit_msgs::MotionPlanRequest>::Ptr MotionPlanRequestCollection;
typedef warehouse_ros::MessageCollection<moveit_msgs::RobotTrajectory>::Ptr RobotTrajectoryCollection;

MOVEIT_CLASS_FORWARD(PlanningSceneStorage);  // Defines PlanningSceneStoragePtr, ConstPtr, WeakPtr... etc

class PlanningSceneStorage : public MoveItMessageStorage
{
public:
  static const std::string DATABASE_NAME;

  static const std::string PLANNING_SCENE_ID_NAME;
  static const std::string MOTION_PLAN_REQUEST_ID_NAME;

  PlanningSceneStorage(warehouse_ros::DatabaseConnection::Ptr conn);

  void addPlanningScene(const moveit_msgs::PlanningScene& scene);
  void addPlanningQuery(const moveit_msgs::MotionPlanRequest& planning_query, const std::string& scene_name,
                        const std::string& query_name = "");
  void addPlanningResult(const moveit_msgs::MotionPlanRequest& planning_query,
                         const moveit_msgs::RobotTrajectory& result, const std::string& scene_name);

  bool hasPlanningScene(const std::string& name) const;
  void getPlanningSceneNames(std::vector<std::string>& names) const;
  void getPlanningSceneNames(const std::string& regex, std::vector<std::string>& names) const;

  /** \brief Get the latest planning scene named \e scene_name */
  bool getPlanningScene(PlanningSceneWithMetadata& scene_m, const std::string& scene_name) const;
  bool getPlanningSceneWorld(moveit_msgs::PlanningSceneWorld& world, const std::string& scene_name) const;

  bool hasPlanningQuery(const std::string& scene_name, const std::string& query_name) const;
  bool getPlanningQuery(MotionPlanRequestWithMetadata& query_m, const std::string& scene_name,
                        const std::string& query_name);
  void getPlanningQueries(std::vector<MotionPlanRequestWithMetadata>& planning_queries,
                          const std::string& scene_name) const;
  void getPlanningQueriesNames(std::vector<std::string>& query_names, const std::string& scene_name) const;
  void getPlanningQueriesNames(const std::string& regex, std::vector<std::string>& query_names,
                               const std::string& scene_name) const;
  void getPlanningQueries(std::vector<MotionPlanRequestWithMetadata>& planning_queries,
                          std::vector<std::string>& query_names, const std::string& scene_name) const;

  void getPlanningResults(std::vector<RobotTrajectoryWithMetadata>& planning_results, const std::string& scene_name,
                          const moveit_msgs::MotionPlanRequest& planning_query) const;
  void getPlanningResults(std::vector<RobotTrajectoryWithMetadata>& planning_results, const std::string& scene_name,
                          const std::string& query_name) const;

  void renamePlanningScene(const std::string& old_scene_name, const std::string& new_scene_name);
  void renamePlanningQuery(const std::string& scene_name, const std::string& old_query_name,
                           const std::string& new_query_name);

  void removePlanningScene(const std::string& scene_name);
  void removePlanningQuery(const std::string& scene_name, const std::string& query_name);
  void removePlanningQueries(const std::string& scene_name);
  void removePlanningResults(const std::string& scene_name);
  void removePlanningResults(const std::string& scene_name, const std::string& query_name);

  void reset();

private:
  void createCollections();

  std::string getMotionPlanRequestName(const moveit_msgs::MotionPlanRequest& planning_query,
                                       const std::string& scene_name) const;
  std::string addNewPlanningRequest(const moveit_msgs::MotionPlanRequest& planning_query, const std::string& scene_name,
                                    const std::string& query_name);

  PlanningSceneCollection planning_scene_collection_;
  MotionPlanRequestCollection motion_plan_request_collection_;
  RobotTrajectoryCollection robot_trajectory_collection_;
};
}  // namespace moveit_warehouse
