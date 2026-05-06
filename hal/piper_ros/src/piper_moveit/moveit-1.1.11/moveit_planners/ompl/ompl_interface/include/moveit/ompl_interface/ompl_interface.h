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

#pragma once

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <string>
#include <map>
#include <ros/ros.h>

/** \brief The MoveIt interface to OMPL */
namespace ompl_interface
{
/** @class OMPLInterface
 *  This class defines the interface to the motion planners in OMPL*/
class OMPLInterface
{
public:
  /** \brief Initialize OMPL-based planning for a particular robot model. ROS configuration is read from the specified
   * NodeHandle */
  OMPLInterface(const moveit::core::RobotModelConstPtr& robot_model, const ros::NodeHandle& nh = ros::NodeHandle("~"));

  /** \brief Initialize OMPL-based planning for a particular robot model. ROS configuration is read from the specified
     NodeHandle. However,
      planner configurations are used as specified in \e pconfig instead of reading them from the ROS parameter server
     */
  OMPLInterface(const moveit::core::RobotModelConstPtr& robot_model,
                const planning_interface::PlannerConfigurationMap& pconfig,
                const ros::NodeHandle& nh = ros::NodeHandle("~"));

  virtual ~OMPLInterface();

  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig);

  /** @brief Get the configurations for the planners that are already loaded
      @param pconfig Configurations for the different planners */
  const planning_interface::PlannerConfigurationMap& getPlannerConfigurations() const
  {
    return context_manager_.getPlannerConfigurations();
  }

  ModelBasedPlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req) const;
  ModelBasedPlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req,
                                                  moveit_msgs::MoveItErrorCodes& error_code) const;

  const PlanningContextManager& getPlanningContextManager() const
  {
    return context_manager_;
  }

  PlanningContextManager& getPlanningContextManager()
  {
    return context_manager_;
  }

  constraint_samplers::ConstraintSamplerManager& getConstraintSamplerManager()
  {
    return *constraint_sampler_manager_;
  }

  const constraint_samplers::ConstraintSamplerManager& getConstraintSamplerManager() const
  {
    return *constraint_sampler_manager_;
  }

  void useConstraintsApproximations(bool flag)
  {
    use_constraints_approximations_ = flag;
  }

  bool isUsingConstraintsApproximations() const
  {
    return use_constraints_approximations_;
  }
  bool simplifySolutions() const
  {
    return simplify_solutions_;
  }

  void simplifySolutions(bool flag)
  {
    simplify_solutions_ = flag;
  }

  /** @brief Print the status of this node*/
  void printStatus();

protected:
  /** @brief Load planner configurations for specified group into planner_config */
  bool loadPlannerConfiguration(const std::string& group_name, const std::string& planner_id,
                                const std::map<std::string, std::string>& group_params,
                                planning_interface::PlannerConfigurationSettings& planner_config);

  /** @brief Configure the planners*/
  void loadPlannerConfigurations();

  /** @brief Load the additional plugins for sampling constraints */
  void loadConstraintSamplers();

  void configureContext(const ModelBasedPlanningContextPtr& context) const;

  /** \brief Configure the OMPL planning context for a new planning request */
  ModelBasedPlanningContextPtr prepareForSolve(const planning_interface::MotionPlanRequest& req,
                                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                                               moveit_msgs::MoveItErrorCodes* error_code, unsigned int* attempts,
                                               double* timeout) const;

  ros::NodeHandle nh_;  /// The ROS node handle

  /** \brief The kinematic model for which motion plans are computed */
  moveit::core::RobotModelConstPtr robot_model_;

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  PlanningContextManager context_manager_;

  bool use_constraints_approximations_;

  bool simplify_solutions_;

private:
  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
};
}  // namespace ompl_interface
