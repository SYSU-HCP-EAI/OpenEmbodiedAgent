/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include "pilz_industrial_motion_planner/pilz_industrial_motion_planner.h"

#include "pilz_industrial_motion_planner/planning_context_loader.h"
#include "pilz_industrial_motion_planner/planning_context_loader_ptp.h"
#include "pilz_industrial_motion_planner/planning_exceptions.h"

#include "pilz_industrial_motion_planner/cartesian_limits_aggregator.h"
#include "pilz_industrial_motion_planner/joint_limits_aggregator.h"

// Boost includes
#include <boost/scoped_ptr.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <pluginlib/class_loader.hpp>

namespace pilz_industrial_motion_planner
{
static const std::string PARAM_NAMESPACE_LIMTS = "robot_description_planning";

bool CommandPlanner::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{
  // Call parent class initialize
  planning_interface::PlannerManager::initialize(model, ns);

  // Store the model and the namespace
  model_ = model;
  namespace_ = ns;

  // Obtain the aggregated joint limits
  aggregated_limit_active_joints_ = pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
      ros::NodeHandle(PARAM_NAMESPACE_LIMTS), model->getActiveJointModels());

  // Obtain cartesian limits
  cartesian_limit_ = pilz_industrial_motion_planner::CartesianLimitsAggregator::getAggregatedLimits(
      ros::NodeHandle(PARAM_NAMESPACE_LIMTS));

  // Load the planning context loader
  planner_context_loader = std::make_unique<pluginlib::ClassLoader<PlanningContextLoader>>(
      "pilz_industrial_motion_planner", "pilz_industrial_motion_planner::PlanningContextLoader");

  // List available plugins
  const std::vector<std::string>& factories = planner_context_loader->getDeclaredClasses();
  std::stringstream ss;
  for (const auto& factory : factories)
  {
    ss << factory << " ";
  }

  ROS_INFO_STREAM("Available plugins: " << ss.str());

  // Load each factory
  for (const auto& factory : factories)
  {
    ROS_INFO_STREAM("About to load: " << factory);
    PlanningContextLoaderPtr loader_pointer(planner_context_loader->createInstance(factory));

    pilz_industrial_motion_planner::LimitsContainer limits;
    limits.setJointLimits(aggregated_limit_active_joints_);
    limits.setCartesianLimits(cartesian_limit_);

    loader_pointer->setLimits(limits);
    loader_pointer->setModel(model_);

    registerContextLoader(loader_pointer);
  }

  return true;
}

std::string CommandPlanner::getDescription() const
{
  return "Pilz Industrial Motion Planner";
}

void CommandPlanner::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();

  for (const auto& context_loader : context_loader_map_)
  {
    algs.push_back(context_loader.first);
  }
}

planning_interface::PlanningContextPtr
CommandPlanner::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const moveit_msgs::MotionPlanRequest& req,
                                   moveit_msgs::MoveItErrorCodes& error_code) const
{
  ROS_DEBUG_STREAM("Loading PlanningContext for request\n<request>\n" << req << "\n</request>");

  // Check that a loaded for this request exists
  if (!canServiceRequest(req))
  {
    ROS_ERROR_STREAM("No ContextLoader for planner_id '" << req.planner_id.c_str() << "' found. Planning not possible.");
    return nullptr;
  }

  planning_interface::PlanningContextPtr planning_context;

  if (context_loader_map_.at(req.planner_id)->loadContext(planning_context, req.planner_id, req.group_name))
  {
    ROS_DEBUG_STREAM("Found planning context loader for " << req.planner_id << " group:" << req.group_name);
    planning_context->setMotionPlanRequest(req);
    planning_context->setPlanningScene(planning_scene);
    return planning_context;
  }
  else
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return nullptr;
  }
}

bool CommandPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
{
  return context_loader_map_.find(req.planner_id) != context_loader_map_.end();
}

void CommandPlanner::registerContextLoader(
    const pilz_industrial_motion_planner::PlanningContextLoaderPtr& planning_context_loader)
{
  // Only add if command is not already in list, throw exception if not
  if (context_loader_map_.find(planning_context_loader->getAlgorithm()) == context_loader_map_.end())
  {
    context_loader_map_[planning_context_loader->getAlgorithm()] = planning_context_loader;
    ROS_INFO_STREAM("Registered Algorithm [" << planning_context_loader->getAlgorithm() << "]");
  }
  else
  {
    throw ContextLoaderRegistrationException("The command [" + planning_context_loader->getAlgorithm() +
                                             "] is already registered");
  }
}

}  // namespace pilz_industrial_motion_planner

PLUGINLIB_EXPORT_CLASS(pilz_industrial_motion_planner::CommandPlanner, planning_interface::PlannerManager)
