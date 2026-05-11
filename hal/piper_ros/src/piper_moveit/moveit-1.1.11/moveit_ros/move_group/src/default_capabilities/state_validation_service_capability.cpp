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

#include "state_validation_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupStateValidationService::MoveGroupStateValidationService() : MoveGroupCapability("StateValidationService")
{
}

void MoveGroupStateValidationService::initialize()
{
  validity_service_ = root_node_handle_.advertiseService(STATE_VALIDITY_SERVICE_NAME,
                                                         &MoveGroupStateValidationService::computeService, this);
}

bool MoveGroupStateValidationService::computeService(moveit_msgs::GetStateValidity::Request& req,
                                                     moveit_msgs::GetStateValidity::Response& res)
{
  planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
  moveit::core::RobotState rs = ls->getCurrentState();
  moveit::core::robotStateMsgToRobotState(req.robot_state, rs);

  res.valid = true;

  // configure collision request
  collision_detection::CollisionRequest creq;
  creq.group_name = req.group_name;
  creq.cost = true;
  creq.contacts = true;
  creq.max_contacts = ls->getWorld()->size() + ls->getRobotModel()->getLinkModelsWithCollisionGeometry().size();
  creq.max_cost_sources = creq.max_contacts;
  creq.max_contacts *= creq.max_contacts;
  collision_detection::CollisionResult cres;

  // check collision
  ls->checkCollision(creq, cres, rs);

  // copy contacts if any
  if (cres.collision)
  {
    ros::Time time_now = ros::Time::now();
    res.contacts.reserve(cres.contact_count);
    res.valid = false;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = cres.contacts.begin();
         it != cres.contacts.end(); ++it)
      for (const collision_detection::Contact& contact : it->second)
      {
        res.contacts.resize(res.contacts.size() + 1);
        collision_detection::contactToMsg(contact, res.contacts.back());
        res.contacts.back().header.frame_id = ls->getPlanningFrame();
        res.contacts.back().header.stamp = time_now;
      }
  }

  // copy cost sources
  res.cost_sources.reserve(cres.cost_sources.size());
  for (const collision_detection::CostSource& cost_source : cres.cost_sources)
  {
    res.cost_sources.resize(res.cost_sources.size() + 1);
    collision_detection::costSourceToMsg(cost_source, res.cost_sources.back());
  }

  // evaluate constraints
  if (!moveit::core::isEmpty(req.constraints))
  {
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    kset.add(req.constraints, ls->getTransforms());
    std::vector<kinematic_constraints::ConstraintEvaluationResult> kres;
    kinematic_constraints::ConstraintEvaluationResult total_result = kset.decide(rs, kres);
    if (!total_result.satisfied)
      res.valid = false;

    // copy constraint results
    res.constraint_result.resize(kres.size());
    for (std::size_t k = 0; k < kres.size(); ++k)
    {
      res.constraint_result[k].result = kres[k].satisfied;
      res.constraint_result[k].distance = kres[k].distance;
    }
  }

  return true;
}
}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupStateValidationService, move_group::MoveGroupCapability)
