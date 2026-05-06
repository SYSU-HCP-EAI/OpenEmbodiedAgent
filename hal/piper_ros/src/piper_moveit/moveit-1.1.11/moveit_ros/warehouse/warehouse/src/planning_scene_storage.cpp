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

#include <moveit/warehouse/planning_scene_storage.h>
#include <boost/regex.hpp>
#include <utility>

const std::string moveit_warehouse::PlanningSceneStorage::DATABASE_NAME = "moveit_planning_scenes";

const std::string moveit_warehouse::PlanningSceneStorage::PLANNING_SCENE_ID_NAME = "planning_scene_id";
const std::string moveit_warehouse::PlanningSceneStorage::MOTION_PLAN_REQUEST_ID_NAME = "motion_request_id";

using warehouse_ros::Metadata;
using warehouse_ros::Query;

moveit_warehouse::PlanningSceneStorage::PlanningSceneStorage(warehouse_ros::DatabaseConnection::Ptr conn)
  : MoveItMessageStorage(std::move(conn))
{
  createCollections();
}

void moveit_warehouse::PlanningSceneStorage::createCollections()
{
  planning_scene_collection_ = conn_->openCollectionPtr<moveit_msgs::PlanningScene>(DATABASE_NAME, "planning_scene");
  motion_plan_request_collection_ =
      conn_->openCollectionPtr<moveit_msgs::MotionPlanRequest>(DATABASE_NAME, "motion_plan_request");
  robot_trajectory_collection_ =
      conn_->openCollectionPtr<moveit_msgs::RobotTrajectory>(DATABASE_NAME, "robot_trajectory");
}

void moveit_warehouse::PlanningSceneStorage::reset()
{
  planning_scene_collection_.reset();
  motion_plan_request_collection_.reset();
  robot_trajectory_collection_.reset();
  conn_->dropDatabase(DATABASE_NAME);
  createCollections();
}

void moveit_warehouse::PlanningSceneStorage::addPlanningScene(const moveit_msgs::PlanningScene& scene)
{
  bool replace = false;
  if (hasPlanningScene(scene.name))
  {
    removePlanningScene(scene.name);
    replace = true;
  }
  Metadata::Ptr metadata = planning_scene_collection_->createMetadata();
  metadata->append(PLANNING_SCENE_ID_NAME, scene.name);
  planning_scene_collection_->insert(scene, metadata);
  ROS_DEBUG("%s scene '%s'", replace ? "Replaced" : "Added", scene.name.c_str());
}

bool moveit_warehouse::PlanningSceneStorage::hasPlanningScene(const std::string& name) const
{
  Query::Ptr q = planning_scene_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, name);
  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_->queryList(q, true);
  return !planning_scenes.empty();
}

std::string
moveit_warehouse::PlanningSceneStorage::getMotionPlanRequestName(const moveit_msgs::MotionPlanRequest& planning_query,
                                                                 const std::string& scene_name) const
{
  // get all existing motion planning requests for this planning scene
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  std::vector<MotionPlanRequestWithMetadata> existing_requests = motion_plan_request_collection_->queryList(q, false);

  // if there are no requests stored, we are done
  if (existing_requests.empty())
    return "";

  // compute the serialization of the message passed as argument
  const size_t serial_size_arg = ros::serialization::serializationLength(planning_query);
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  ros::serialization::OStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::serialize(stream_arg, planning_query);
  const void* data_arg = buffer_arg.get();

  for (MotionPlanRequestWithMetadata& existing_request : existing_requests)
  {
    const size_t serial_size =
        ros::serialization::serializationLength(static_cast<const moveit_msgs::MotionPlanRequest&>(*existing_request));
    if (serial_size != serial_size_arg)
      continue;
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, static_cast<const moveit_msgs::MotionPlanRequest&>(*existing_request));
    const void* data = buffer.get();
    if (memcmp(data_arg, data, serial_size) == 0)
      // we found the same message twice
      return existing_request->lookupString(MOTION_PLAN_REQUEST_ID_NAME);
  }
  return "";
}

void moveit_warehouse::PlanningSceneStorage::addPlanningQuery(const moveit_msgs::MotionPlanRequest& planning_query,
                                                              const std::string& scene_name,
                                                              const std::string& query_name)
{
  std::string id = getMotionPlanRequestName(planning_query, scene_name);

  // if we are trying to overwrite, we remove the old query first (if it exists).
  if (!query_name.empty() && id.empty())
    removePlanningQuery(scene_name, query_name);

  if (id != query_name || id.empty())
    addNewPlanningRequest(planning_query, scene_name, query_name);
}

std::string moveit_warehouse::PlanningSceneStorage::addNewPlanningRequest(
    const moveit_msgs::MotionPlanRequest& planning_query, const std::string& scene_name, const std::string& query_name)
{
  std::string id = query_name;
  if (id.empty())
  {
    std::set<std::string> used;
    Query::Ptr q = motion_plan_request_collection_->createQuery();
    q->append(PLANNING_SCENE_ID_NAME, scene_name);
    std::vector<MotionPlanRequestWithMetadata> existing_requests = motion_plan_request_collection_->queryList(q, true);
    for (MotionPlanRequestWithMetadata& existing_request : existing_requests)
      used.insert(existing_request->lookupString(MOTION_PLAN_REQUEST_ID_NAME));
    std::size_t index = existing_requests.size();
    do
    {
      id = "Motion Plan Request " + boost::lexical_cast<std::string>(index);
      index++;
    } while (used.find(id) != used.end());
  }
  Metadata::Ptr metadata = motion_plan_request_collection_->createMetadata();
  metadata->append(PLANNING_SCENE_ID_NAME, scene_name);
  metadata->append(MOTION_PLAN_REQUEST_ID_NAME, id);
  motion_plan_request_collection_->insert(planning_query, metadata);
  ROS_DEBUG("Saved planning query '%s' for scene '%s'", id.c_str(), scene_name.c_str());
  return id;
}

void moveit_warehouse::PlanningSceneStorage::addPlanningResult(const moveit_msgs::MotionPlanRequest& planning_query,
                                                               const moveit_msgs::RobotTrajectory& result,
                                                               const std::string& scene_name)
{
  std::string id = getMotionPlanRequestName(planning_query, scene_name);
  if (id.empty())
    id = addNewPlanningRequest(planning_query, scene_name, "");
  Metadata::Ptr metadata = robot_trajectory_collection_->createMetadata();
  metadata->append(PLANNING_SCENE_ID_NAME, scene_name);
  metadata->append(MOTION_PLAN_REQUEST_ID_NAME, id);
  robot_trajectory_collection_->insert(result, metadata);
}

void moveit_warehouse::PlanningSceneStorage::getPlanningSceneNames(std::vector<std::string>& names) const
{
  names.clear();
  Query::Ptr q = planning_scene_collection_->createQuery();
  std::vector<PlanningSceneWithMetadata> planning_scenes =
      planning_scene_collection_->queryList(q, true, PLANNING_SCENE_ID_NAME, true);
  for (PlanningSceneWithMetadata& planning_scene : planning_scenes)
    if (planning_scene->lookupField(PLANNING_SCENE_ID_NAME))
      names.push_back(planning_scene->lookupString(PLANNING_SCENE_ID_NAME));
}

void moveit_warehouse::PlanningSceneStorage::getPlanningSceneNames(const std::string& regex,
                                                                   std::vector<std::string>& names) const
{
  getPlanningSceneNames(names);
  filterNames(regex, names);
}

bool moveit_warehouse::PlanningSceneStorage::getPlanningSceneWorld(moveit_msgs::PlanningSceneWorld& world,
                                                                   const std::string& scene_name) const
{
  PlanningSceneWithMetadata scene_m;
  if (getPlanningScene(scene_m, scene_name))
  {
    world = scene_m->world;
    return true;
  }
  else
    return false;
}

bool moveit_warehouse::PlanningSceneStorage::getPlanningScene(PlanningSceneWithMetadata& scene_m,
                                                              const std::string& scene_name) const
{
  Query::Ptr q = planning_scene_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  std::vector<PlanningSceneWithMetadata> planning_scenes = planning_scene_collection_->queryList(q, false);
  if (planning_scenes.empty())
  {
    ROS_WARN("Planning scene '%s' was not found in the database", scene_name.c_str());
    return false;
  }
  scene_m = planning_scenes.back();
  // in case the scene was renamed, the name in the message may be out of date
  const_cast<moveit_msgs::PlanningScene*>(static_cast<const moveit_msgs::PlanningScene*>(scene_m.get()))->name =
      scene_name;
  return true;
}

bool moveit_warehouse::PlanningSceneStorage::getPlanningQuery(MotionPlanRequestWithMetadata& query_m,
                                                              const std::string& scene_name,
                                                              const std::string& query_name)
{
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  q->append(MOTION_PLAN_REQUEST_ID_NAME, query_name);
  std::vector<MotionPlanRequestWithMetadata> planning_queries = motion_plan_request_collection_->queryList(q, false);
  if (planning_queries.empty())
  {
    ROS_ERROR("Planning query '%s' not found for scene '%s'", query_name.c_str(), scene_name.c_str());
    return false;
  }
  else
  {
    query_m = planning_queries.front();
    return true;
  }
}

void moveit_warehouse::PlanningSceneStorage::getPlanningQueries(
    std::vector<MotionPlanRequestWithMetadata>& planning_queries, const std::string& scene_name) const
{
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  planning_queries = motion_plan_request_collection_->queryList(q, false);
}

void moveit_warehouse::PlanningSceneStorage::getPlanningQueriesNames(std::vector<std::string>& query_names,
                                                                     const std::string& scene_name) const
{
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  std::vector<MotionPlanRequestWithMetadata> planning_queries = motion_plan_request_collection_->queryList(q, true);
  query_names.clear();
  for (MotionPlanRequestWithMetadata& planning_query : planning_queries)
    if (planning_query->lookupField(MOTION_PLAN_REQUEST_ID_NAME))
      query_names.push_back(planning_query->lookupString(MOTION_PLAN_REQUEST_ID_NAME));
}

void moveit_warehouse::PlanningSceneStorage::getPlanningQueriesNames(const std::string& regex,
                                                                     std::vector<std::string>& query_names,
                                                                     const std::string& scene_name) const
{
  getPlanningQueriesNames(query_names, scene_name);

  if (!regex.empty())
  {
    std::vector<std::string> fnames;
    boost::regex r(regex);
    for (const std::string& query_name : query_names)
    {
      boost::cmatch match;
      if (boost::regex_match(query_name.c_str(), match, r))
      {
        fnames.push_back(query_name);
      }
    }
    query_names.swap(fnames);
  }
}

void moveit_warehouse::PlanningSceneStorage::getPlanningQueries(
    std::vector<MotionPlanRequestWithMetadata>& planning_queries, std::vector<std::string>& query_names,
    const std::string& scene_name) const
{
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  planning_queries = motion_plan_request_collection_->queryList(q, false);
  query_names.resize(planning_queries.size());
  for (std::size_t i = 0; i < planning_queries.size(); ++i)
    if (planning_queries[i]->lookupField(MOTION_PLAN_REQUEST_ID_NAME))
      query_names[i] = planning_queries[i]->lookupString(MOTION_PLAN_REQUEST_ID_NAME);
    else
      query_names[i].clear();
}

void moveit_warehouse::PlanningSceneStorage::getPlanningResults(
    std::vector<RobotTrajectoryWithMetadata>& planning_results, const std::string& scene_name,
    const moveit_msgs::MotionPlanRequest& planning_query) const
{
  std::string id = getMotionPlanRequestName(planning_query, scene_name);
  if (id.empty())
    planning_results.clear();
  else
    getPlanningResults(planning_results, id, scene_name);
}

void moveit_warehouse::PlanningSceneStorage::getPlanningResults(
    std::vector<RobotTrajectoryWithMetadata>& planning_results, const std::string& scene_name,
    const std::string& planning_query) const
{
  Query::Ptr q = robot_trajectory_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  q->append(MOTION_PLAN_REQUEST_ID_NAME, planning_query);
  planning_results = robot_trajectory_collection_->queryList(q, false);
}

bool moveit_warehouse::PlanningSceneStorage::hasPlanningQuery(const std::string& scene_name,
                                                              const std::string& query_name) const
{
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  q->append(MOTION_PLAN_REQUEST_ID_NAME, query_name);
  std::vector<MotionPlanRequestWithMetadata> queries = motion_plan_request_collection_->queryList(q, true);
  return !queries.empty();
}

void moveit_warehouse::PlanningSceneStorage::renamePlanningScene(const std::string& old_scene_name,
                                                                 const std::string& new_scene_name)
{
  Query::Ptr q = planning_scene_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, old_scene_name);
  Metadata::Ptr m = planning_scene_collection_->createMetadata();
  m->append(PLANNING_SCENE_ID_NAME, new_scene_name);
  planning_scene_collection_->modifyMetadata(q, m);
  ROS_DEBUG("Renamed planning scene from '%s' to '%s'", old_scene_name.c_str(), new_scene_name.c_str());
}

void moveit_warehouse::PlanningSceneStorage::renamePlanningQuery(const std::string& scene_name,
                                                                 const std::string& old_query_name,
                                                                 const std::string& new_query_name)
{
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  q->append(MOTION_PLAN_REQUEST_ID_NAME, old_query_name);
  Metadata::Ptr m = motion_plan_request_collection_->createMetadata();
  m->append(MOTION_PLAN_REQUEST_ID_NAME, new_query_name);
  motion_plan_request_collection_->modifyMetadata(q, m);
  ROS_DEBUG("Renamed planning query for scene '%s' from '%s' to '%s'", scene_name.c_str(), old_query_name.c_str(),
            new_query_name.c_str());
}

void moveit_warehouse::PlanningSceneStorage::removePlanningScene(const std::string& scene_name)
{
  removePlanningQueries(scene_name);
  Query::Ptr q = planning_scene_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  unsigned int rem = planning_scene_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u PlanningScene messages (named '%s')", rem, scene_name.c_str());
}

void moveit_warehouse::PlanningSceneStorage::removePlanningQueries(const std::string& scene_name)
{
  removePlanningResults(scene_name);
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  unsigned int rem = motion_plan_request_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u MotionPlanRequest messages for scene '%s'", rem, scene_name.c_str());
}

void moveit_warehouse::PlanningSceneStorage::removePlanningQuery(const std::string& scene_name,
                                                                 const std::string& query_name)
{
  removePlanningResults(scene_name, query_name);
  Query::Ptr q = motion_plan_request_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  q->append(MOTION_PLAN_REQUEST_ID_NAME, query_name);
  unsigned int rem = motion_plan_request_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u MotionPlanRequest messages for scene '%s', query '%s'", rem, scene_name.c_str(),
            query_name.c_str());
}

void moveit_warehouse::PlanningSceneStorage::removePlanningResults(const std::string& scene_name)
{
  Query::Ptr q = robot_trajectory_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  unsigned int rem = robot_trajectory_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u RobotTrajectory messages for scene '%s'", rem, scene_name.c_str());
}

void moveit_warehouse::PlanningSceneStorage::removePlanningResults(const std::string& scene_name,
                                                                   const std::string& query_name)
{
  Query::Ptr q = robot_trajectory_collection_->createQuery();
  q->append(PLANNING_SCENE_ID_NAME, scene_name);
  q->append(MOTION_PLAN_REQUEST_ID_NAME, query_name);
  unsigned int rem = robot_trajectory_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u RobotTrajectory messages for scene '%s', query '%s'", rem, scene_name.c_str(),
            query_name.c_str());
}
