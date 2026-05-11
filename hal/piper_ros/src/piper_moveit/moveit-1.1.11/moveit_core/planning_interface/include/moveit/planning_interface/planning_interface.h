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
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <string>
#include <map>

namespace planning_scene
{
MOVEIT_CLASS_FORWARD(PlanningScene);  // Defines PlanningScenePtr, ConstPtr, WeakPtr... etc
}

/** \brief This namespace includes the base class for MoveIt planners */
namespace planning_interface
{
/**
   \brief Specify the settings for a particular planning algorithm, for a particular group. The Planner plugin uses
   these settings to configure the algorithm.
   \note Settings with unknown keys are ignored. Settings for unknown groups are ignored.
*/
struct PlannerConfigurationSettings
{
  /** \brief The group (as defined in the SRDF) this configuration is meant for */
  std::string group;

  /* \brief Name of the configuration.

     For a group's default configuration, this should be the same as the group name.
     Otherwise, the form "group_name[config_name]" is expected for the name. */
  std::string name;

  /** \brief Key-value pairs of settings that get passed to the planning algorithm */
  std::map<std::string, std::string> config;
};

/** \brief Map from PlannerConfigurationSettings.name to PlannerConfigurationSettings */
typedef std::map<std::string, PlannerConfigurationSettings> PlannerConfigurationMap;

MOVEIT_CLASS_FORWARD(PlanningContext);  // Defines PlanningContextPtr, ConstPtr, WeakPtr... etc

/** \brief Representation of a particular planning context -- the planning scene and the request are known,
    solution is not yet computed. */
class PlanningContext
{
public:
  /** \brief Construct a planning context named \e name for the group \e group */
  PlanningContext(const std::string& name, const std::string& group);

  virtual ~PlanningContext();

  /** \brief Get the name of the group this planning context is for */
  const std::string& getGroupName() const
  {
    return group_;
  }

  /** \brief Get the name of this planning context */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief Get the planning scene associated to this planning context */
  const planning_scene::PlanningSceneConstPtr& getPlanningScene() const
  {
    return planning_scene_;
  }

  /** \brief Get the motion plan request associated to this planning context */
  const MotionPlanRequest& getMotionPlanRequest() const
  {
    return request_;
  }

  /** \brief Set the planning scene for this context */
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

  /** \brief Set the planning request for this context */
  void setMotionPlanRequest(const MotionPlanRequest& request);

  /** \brief Solve the motion planning problem and store the result in \e res. This function should not clear data
   * structures before computing. The constructor and clear() do that. */
  virtual bool solve(MotionPlanResponse& res) = 0;

  /** \brief Solve the motion planning problem and store the detailed result in \e res. This function should not clear
   * data structures before computing. The constructor and clear() do that. */
  virtual bool solve(MotionPlanDetailedResponse& res) = 0;

  /** \brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).*/
  virtual bool terminate() = 0;

  /** \brief Clear the data structures used by the planner */
  virtual void clear() = 0;

protected:
  /// The name of this planning context
  std::string name_;

  /// The group (as in the SRDF) this planning context is for
  std::string group_;

  /// The planning scene for this context
  planning_scene::PlanningSceneConstPtr planning_scene_;

  /// The planning request for this context
  MotionPlanRequest request_;
};

MOVEIT_CLASS_FORWARD(PlannerManager);  // Defines PlannerManagerPtr, ConstPtr, WeakPtr... etc

/** \brief Base class for a MoveIt planner */
class PlannerManager
{
public:
  PlannerManager()
  {
  }

  virtual ~PlannerManager()
  {
  }

  /// Initialize a planner. This function will be called after the construction of the plugin, before any other call is
  /// made.
  /// It is assumed that motion plans will be computed for the robot described by \e model and that any exposed ROS
  /// functionality
  /// or required ROS parameters are namespaced by \e ns
  virtual bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns);

  /// Get \brief a short string that identifies the planning interface
  virtual std::string getDescription() const;

  /// \brief Get the names of the known planning algorithms (values that can be filled as planner_id in the planning
  /// request)
  virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const;

  /// \brief Construct a planning context given the current scene and a planning request. If a problem is encountered,
  /// error code is set and empty ptr is returned.
  /// The returned motion planner context is clean -- the motion planner will start from scratch every time a context is
  /// constructed.
  /// \param planning_scene A const planning scene to use for planning
  /// \param req The representation of the planning request
  /// \param error_code This is where the error is set if constructing the planning context fails
  virtual PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                const MotionPlanRequest& req,
                                                moveit_msgs::MoveItErrorCodes& error_code) const = 0;

  /// \brief Calls the function above but ignores the error_code
  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const MotionPlanRequest& req) const;

  /// \brief Determine whether this plugin instance is able to represent this planning request
  virtual bool canServiceRequest(const MotionPlanRequest& req) const = 0;

  /// \brief Specify the settings to be used for specific algorithms
  virtual void setPlannerConfigurations(const PlannerConfigurationMap& pcs);

  /// \brief Get the settings for a specific algorithm
  const PlannerConfigurationMap& getPlannerConfigurations() const
  {
    return config_settings_;
  }

  /// \brief Request termination, if a solve() function is currently computing plans
  void terminate() const;

protected:
  /** \brief All the existing planning configurations. The name
      of the configuration is the key of the map. This name can
      be of the form "group_name[config_name]" if there are
      particular configurations specified for a group, or of the
      form "group_name" if default settings are to be used. */
  PlannerConfigurationMap config_settings_;
};

}  // namespace planning_interface
