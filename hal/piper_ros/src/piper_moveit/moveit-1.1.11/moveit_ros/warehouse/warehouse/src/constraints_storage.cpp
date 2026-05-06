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

#include <moveit/warehouse/constraints_storage.h>

#include <utility>

const std::string moveit_warehouse::ConstraintsStorage::DATABASE_NAME = "moveit_constraints";

const std::string moveit_warehouse::ConstraintsStorage::CONSTRAINTS_ID_NAME = "constraints_id";
const std::string moveit_warehouse::ConstraintsStorage::CONSTRAINTS_GROUP_NAME = "group_id";
const std::string moveit_warehouse::ConstraintsStorage::ROBOT_NAME = "robot_id";

using warehouse_ros::Metadata;
using warehouse_ros::Query;

moveit_warehouse::ConstraintsStorage::ConstraintsStorage(warehouse_ros::DatabaseConnection::Ptr conn)
  : MoveItMessageStorage(std::move(conn))
{
  createCollections();
}

void moveit_warehouse::ConstraintsStorage::createCollections()
{
  constraints_collection_ = conn_->openCollectionPtr<moveit_msgs::Constraints>(DATABASE_NAME, "constraints");
}

void moveit_warehouse::ConstraintsStorage::reset()
{
  constraints_collection_.reset();
  conn_->dropDatabase(DATABASE_NAME);
  createCollections();
}

void moveit_warehouse::ConstraintsStorage::addConstraints(const moveit_msgs::Constraints& msg, const std::string& robot,
                                                          const std::string& group)
{
  bool replace = false;
  if (hasConstraints(msg.name, robot, group))
  {
    removeConstraints(msg.name, robot, group);
    replace = true;
  }
  Metadata::Ptr metadata = constraints_collection_->createMetadata();
  metadata->append(CONSTRAINTS_ID_NAME, msg.name);
  metadata->append(ROBOT_NAME, robot);
  metadata->append(CONSTRAINTS_GROUP_NAME, group);
  constraints_collection_->insert(msg, metadata);
  ROS_DEBUG("%s constraints '%s'", replace ? "Replaced" : "Added", msg.name.c_str());
}

bool moveit_warehouse::ConstraintsStorage::hasConstraints(const std::string& name, const std::string& robot,
                                                          const std::string& group) const
{
  Query::Ptr q = constraints_collection_->createQuery();
  q->append(CONSTRAINTS_ID_NAME, name);
  if (!robot.empty())
    q->append(ROBOT_NAME, robot);
  if (!group.empty())
    q->append(CONSTRAINTS_GROUP_NAME, group);
  std::vector<ConstraintsWithMetadata> constr = constraints_collection_->queryList(q, true);
  return !constr.empty();
}

void moveit_warehouse::ConstraintsStorage::getKnownConstraints(const std::string& regex,
                                                               std::vector<std::string>& names,
                                                               const std::string& robot, const std::string& group) const
{
  getKnownConstraints(names, robot, group);
  filterNames(regex, names);
}

void moveit_warehouse::ConstraintsStorage::getKnownConstraints(std::vector<std::string>& names,
                                                               const std::string& robot, const std::string& group) const
{
  names.clear();
  Query::Ptr q = constraints_collection_->createQuery();
  if (!robot.empty())
    q->append(ROBOT_NAME, robot);
  if (!group.empty())
    q->append(CONSTRAINTS_GROUP_NAME, group);
  std::vector<ConstraintsWithMetadata> constr = constraints_collection_->queryList(q, true, CONSTRAINTS_ID_NAME, true);
  for (const ConstraintsWithMetadata& it : constr)
    if (it->lookupField(CONSTRAINTS_ID_NAME))
      names.push_back(it->lookupString(CONSTRAINTS_ID_NAME));
}

bool moveit_warehouse::ConstraintsStorage::getConstraints(ConstraintsWithMetadata& msg_m, const std::string& name,
                                                          const std::string& robot, const std::string& group) const
{
  Query::Ptr q = constraints_collection_->createQuery();
  q->append(CONSTRAINTS_ID_NAME, name);
  if (!robot.empty())
    q->append(ROBOT_NAME, robot);
  if (!group.empty())
    q->append(CONSTRAINTS_GROUP_NAME, group);
  std::vector<ConstraintsWithMetadata> constr = constraints_collection_->queryList(q, false);
  if (constr.empty())
    return false;
  else
  {
    msg_m = constr.back();
    // in case the constraints were renamed, the name in the message may be out of date
    const_cast<moveit_msgs::Constraints*>(static_cast<const moveit_msgs::Constraints*>(msg_m.get()))->name = name;
    return true;
  }
}

void moveit_warehouse::ConstraintsStorage::renameConstraints(const std::string& old_name, const std::string& new_name,
                                                             const std::string& robot, const std::string& group)
{
  Query::Ptr q = constraints_collection_->createQuery();
  q->append(CONSTRAINTS_ID_NAME, old_name);
  if (!robot.empty())
    q->append(ROBOT_NAME, robot);
  if (!group.empty())
    q->append(CONSTRAINTS_GROUP_NAME, group);
  Metadata::Ptr m = constraints_collection_->createMetadata();
  m->append(CONSTRAINTS_ID_NAME, new_name);
  constraints_collection_->modifyMetadata(q, m);
  ROS_DEBUG("Renamed constraints from '%s' to '%s'", old_name.c_str(), new_name.c_str());
}

void moveit_warehouse::ConstraintsStorage::removeConstraints(const std::string& name, const std::string& robot,
                                                             const std::string& group)
{
  Query::Ptr q = constraints_collection_->createQuery();
  q->append(CONSTRAINTS_ID_NAME, name);
  if (!robot.empty())
    q->append(ROBOT_NAME, robot);
  if (!group.empty())
    q->append(CONSTRAINTS_GROUP_NAME, group);
  unsigned int rem = constraints_collection_->removeMessages(q);
  ROS_DEBUG("Removed %u Constraints messages (named '%s')", rem, name.c_str());
}
