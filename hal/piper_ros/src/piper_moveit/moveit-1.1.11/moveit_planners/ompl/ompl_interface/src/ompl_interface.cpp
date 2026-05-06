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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/profiler/profiler.h>
#include <moveit/utils/lexical_casts.h>
#include <fstream>

namespace ompl_interface
{
constexpr char LOGNAME[] = "ompl_interface";
}  // namespace ompl_interface

ompl_interface::OMPLInterface::OMPLInterface(const moveit::core::RobotModelConstPtr& robot_model,
                                             const ros::NodeHandle& nh)
  : nh_(nh)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  ROS_DEBUG_NAMED(LOGNAME, "Initializing OMPL interface using ROS parameters");
  loadPlannerConfigurations();
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::OMPLInterface(const moveit::core::RobotModelConstPtr& robot_model,
                                             const planning_interface::PlannerConfigurationMap& pconfig,
                                             const ros::NodeHandle& nh)
  : nh_(nh)
  , robot_model_(robot_model)
  , constraint_sampler_manager_(new constraint_samplers::ConstraintSamplerManager())
  , context_manager_(robot_model, constraint_sampler_manager_)
  , use_constraints_approximations_(true)
  , simplify_solutions_(true)
{
  ROS_DEBUG_NAMED(LOGNAME, "Initializing OMPL interface using specified configuration");
  setPlannerConfigurations(pconfig);
  loadConstraintSamplers();
}

ompl_interface::OMPLInterface::~OMPLInterface() = default;

void ompl_interface::OMPLInterface::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  for (const moveit::core::JointModelGroup* group : robot_model_->getJointModelGroups())
  {
    if (pconfig.find(group->getName()) == pconfig.end())
    {
      planning_interface::PlannerConfigurationSettings empty;
      empty.name = empty.group = group->getName();
      pconfig2[empty.name] = empty;
    }
  }

  context_manager_.setPlannerConfigurations(pconfig2);
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::OMPLInterface::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req) const
{
  moveit_msgs::MoveItErrorCodes dummy;
  return getPlanningContext(planning_scene, req, dummy);
}

ompl_interface::ModelBasedPlanningContextPtr
ompl_interface::OMPLInterface::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req,
                                                  moveit_msgs::MoveItErrorCodes& error_code) const
{
  ModelBasedPlanningContextPtr ctx =
      context_manager_.getPlanningContext(planning_scene, req, error_code, nh_, use_constraints_approximations_);
  if (ctx)
    configureContext(ctx);
  return ctx;
}

void ompl_interface::OMPLInterface::configureContext(const ModelBasedPlanningContextPtr& context) const
{
  context->simplifySolutions(simplify_solutions_);
}

void ompl_interface::OMPLInterface::loadConstraintSamplers()
{
  constraint_sampler_manager_loader_ =
      std::make_shared<constraint_sampler_manager_loader::ConstraintSamplerManagerLoader>(constraint_sampler_manager_);
}

bool ompl_interface::OMPLInterface::loadPlannerConfiguration(
    const std::string& group_name, const std::string& planner_id,
    const std::map<std::string, std::string>& group_params,
    planning_interface::PlannerConfigurationSettings& planner_config)
{
  XmlRpc::XmlRpcValue xml_config;
  if (!nh_.getParam("planner_configs/" + planner_id, xml_config))
  {
    ROS_ERROR_NAMED(LOGNAME, "Could not find the planner configuration '%s' on the param server", planner_id.c_str());
    return false;
  }

  if (xml_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_NAMED(LOGNAME, "A planning configuration should be of type XmlRpc Struct type (for configuration '%s')",
                    planner_id.c_str());
    return false;
  }

  planner_config.name = group_name + "[" + planner_id + "]";
  planner_config.group = group_name;

  // default to specified parameters of the group (overridden by configuration specific parameters)
  planner_config.config = group_params;

  // read parameters specific for this configuration
  for (std::pair<const std::string, XmlRpc::XmlRpcValue>& element : xml_config)
  {
    if (element.second.getType() == XmlRpc::XmlRpcValue::TypeString)
      planner_config.config[element.first] = static_cast<std::string>(element.second);
    else if (element.second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      planner_config.config[element.first] = moveit::core::toString(static_cast<double>(element.second));
    else if (element.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
      planner_config.config[element.first] = std::to_string(static_cast<int>(element.second));
    else if (element.second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      planner_config.config[element.first] = std::to_string(static_cast<bool>(element.second));
  }

  return true;
}

void ompl_interface::OMPLInterface::loadPlannerConfigurations()
{
  // read the planning configuration for each group
  planning_interface::PlannerConfigurationMap pconfig;
  pconfig.clear();

  for (const std::string& group_name : robot_model_->getJointModelGroupNames())
  {
    // the set of planning parameters that can be specific for the group (inherited by configurations of that group)
    static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction",
                                                      "enforce_joint_model_state_space" };

    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    for (const std::string& k : KNOWN_GROUP_PARAMS)
    {
      std::string param_name{ group_name };
      param_name += "/";
      param_name += k;

      if (nh_.hasParam(param_name))
      {
        std::string value;
        if (nh_.getParam(param_name, value))
        {
          if (!value.empty())
            specific_group_params[k] = value;
          continue;
        }

        double value_d;
        if (nh_.getParam(param_name, value_d))
        {
          // convert to string using no locale
          specific_group_params[k] = moveit::core::toString(value_d);
          continue;
        }

        int value_i;
        if (nh_.getParam(param_name, value_i))
        {
          specific_group_params[k] = std::to_string(value_i);
          continue;
        }

        bool value_b;
        if (nh_.getParam(param_name, value_b))
        {
          specific_group_params[k] = std::to_string(value_b);
          continue;
        }
      }
    }

    // add default planner configuration
    planning_interface::PlannerConfigurationSettings default_pc;
    std::string default_planner_id;
    if (nh_.getParam(group_name + "/default_planner_config", default_planner_id))
    {
      if (!loadPlannerConfiguration(group_name, default_planner_id, specific_group_params, default_pc))
        default_planner_id = "";
    }

    if (default_planner_id.empty())
    {
      default_pc.group = group_name;
      default_pc.config = specific_group_params;
      default_pc.config["type"] = "geometric::RRTConnect";
    }

    default_pc.name = group_name;  // this is the name of the default config
    pconfig[default_pc.name] = default_pc;

    // get parameters specific to each planner type
    XmlRpc::XmlRpcValue config_names;
    if (nh_.getParam(group_name + "/planner_configs", config_names))
    {
      if (config_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_NAMED(LOGNAME,
                        "The planner_configs argument of a group configuration "
                        "should be an array of strings (for group '%s')",
                        group_name.c_str());
        continue;
      }

      for (int j = 0; j < config_names.size(); ++j)  // NOLINT(modernize-loop-convert)
      {
        if (config_names[j].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR_NAMED(LOGNAME, "Planner configuration names must be of type string (for group '%s')",
                          group_name.c_str());
          continue;
        }

        const std::string planner_id = static_cast<std::string>(config_names[j]);

        planning_interface::PlannerConfigurationSettings pc;
        if (loadPlannerConfiguration(group_name, planner_id, specific_group_params, pc))
          pconfig[pc.name] = pc;
      }
    }
  }

  for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Parameters for configuration '" << config.first << "'");

    for (const std::pair<const std::string, std::string>& parameters : config.second.config)
      ROS_DEBUG_STREAM_NAMED(LOGNAME, " - " << parameters.first << " = " << parameters.second);
  }

  setPlannerConfigurations(pconfig);
}

void ompl_interface::OMPLInterface::printStatus()
{
  ROS_INFO_NAMED(LOGNAME, "OMPL ROS interface is running.");
}
