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

#pragma once

#include <ros/ros.h>

#include "pilz_industrial_motion_planner/joint_limits_extension.h"
#include "pilz_industrial_motion_planner/planning_context_loader.h"

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>

#include <pluginlib/class_loader.hpp>

// Boost includes
#include <boost/scoped_ptr.hpp>

namespace pilz_industrial_motion_planner
{
/**
 * @brief MoveIt Plugin for Planning with Standard Robot Commands
 * This planner is dedicated to return a instance of PlanningContext that
 * corresponds to the requested motion command
 * set as planner_id in the MotionPlanRequest).
 * It can be easily extended with additional commands by creating a class
 * inherting from PlanningContextLoader.
 */
class CommandPlanner : public planning_interface::PlannerManager
{
public:
  ~CommandPlanner() override
  {
  }

  /**
   * @brief Initializes the planner
   * Upon initialization this planner will look for plugins implementing
   * pilz_industrial_motion_planner::PlanningContextLoader.
   * @param model The robot model
   * @param ns The namespace
   * @return true on success, false otherwise
   */
  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;

  /// Description of the planner
  std::string getDescription() const override;

  /**
   * @brief Returns the available planning commands
   * @param list with the planning algorithms
   * @note behined each command is a
   * pilz_industrial_motion_planner::PlanningContextLoader loaded as plugin
   */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  /**
   * @brief Returns a PlanningContext that can be used to solve(calculate) the
   * trajectory that corresponds to command
   * given in motion request as planner_id.
   * @param planning_scene
   * @param req
   * @param error_code
   * @return
   */
  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override;

  /**
   * @brief Checks if the request can be handled
   * @param motion request containing the planning_id that corresponds to the
   * motion command
   * @return true if the request can be handled
   */
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

  /**
   * @brief Register a PlanningContextLoader to be used by the CommandPlanner
   * @param planning_context_loader
   * @throw ContextLoaderRegistrationException if a loader with the same
   * algorithm name is already registered
   */
  void registerContextLoader(const pilz_industrial_motion_planner::PlanningContextLoaderPtr& planning_context_loader);

private:
  /// Plugin loader
  std::unique_ptr<pluginlib::ClassLoader<PlanningContextLoader>> planner_context_loader;

  /// Mapping from command to loader
  std::map<std::string, pilz_industrial_motion_planner::PlanningContextLoaderPtr> context_loader_map_;

  /// Robot model obtained at initialize
  moveit::core::RobotModelConstPtr model_;

  /// Namespace where the parameters are stored, obtained at initialize
  std::string namespace_;

  /// aggregated limits of the active joints
  pilz_industrial_motion_planner::JointLimitsContainer aggregated_limit_active_joints_;

  /// cartesian limit
  pilz_industrial_motion_planner::CartesianLimit cartesian_limit_;
};

MOVEIT_CLASS_FORWARD(CommandPlanner);

}  // namespace pilz_industrial_motion_planner
