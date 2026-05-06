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

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space_factory.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/macros/class_forward.h>

#include <ompl/base/PlannerDataStorage.h>

#include <string>
#include <map>

namespace ompl_interface
{
class MultiQueryPlannerAllocator
{
public:
  MultiQueryPlannerAllocator() = default;
  ~MultiQueryPlannerAllocator();

  template <typename T>
  ob::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr& si, const std::string& new_name,
                                 const ModelBasedPlanningContextSpecification& spec);

private:
  template <typename T>
  ob::PlannerPtr allocatePlannerImpl(const ob::SpaceInformationPtr& si, const std::string& new_name,
                                     const ModelBasedPlanningContextSpecification& spec, bool load_planner_data = false,
                                     bool store_planner_data = false, const std::string& file_path = "");

  template <typename T>
  inline ob::Planner* allocatePersistentPlanner(const ob::PlannerData& data);

  // Storing multi-query planners
  std::map<std::string, ob::PlannerPtr> planners_;

  std::map<std::string, std::string> planner_data_storage_paths_;

  // Store and load planner data
  ob::PlannerDataStorage storage_;
};

class PlanningContextManager
{
public:
  PlanningContextManager(moveit::core::RobotModelConstPtr robot_model,
                         constraint_samplers::ConstraintSamplerManagerPtr csm);
  ~PlanningContextManager();

  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig);

  /** @brief Return the previously set planner configurations */
  const planning_interface::PlannerConfigurationMap& getPlannerConfigurations() const
  {
    return planner_configs_;
  }

  /* \brief Get the maximum number of sampling attempts allowed when sampling states is needed */
  unsigned int getMaximumStateSamplingAttempts() const
  {
    return max_state_sampling_attempts_;
  }

  /* \brief Set the maximum number of sampling attempts allowed when sampling states is needed */
  void setMaximumStateSamplingAttempts(unsigned int max_state_sampling_attempts)
  {
    max_state_sampling_attempts_ = max_state_sampling_attempts;
  }

  /* \brief Get the maximum number of sampling attempts allowed when sampling goals is needed */
  unsigned int getMaximumGoalSamplingAttempts() const
  {
    return max_goal_sampling_attempts_;
  }

  /* \brief Set the maximum number of sampling attempts allowed when sampling goals is needed */
  void setMaximumGoalSamplingAttempts(unsigned int max_goal_sampling_attempts)
  {
    max_goal_sampling_attempts_ = max_goal_sampling_attempts;
  }

  /* \brief Get the maximum number of goal samples */
  unsigned int getMaximumGoalSamples() const
  {
    return max_goal_samples_;
  }

  /* \brief Set the maximum number of goal samples */
  void setMaximumGoalSamples(unsigned int max_goal_samples)
  {
    max_goal_samples_ = max_goal_samples;
  }

  /* \brief Get the maximum number of planning threads allowed */
  unsigned int getMaximumPlanningThreads() const
  {
    return max_planning_threads_;
  }

  /* \brief Set the maximum number of planning threads */
  void setMaximumPlanningThreads(unsigned int max_planning_threads)
  {
    max_planning_threads_ = max_planning_threads;
  }

  /* \brief Get the maximum solution segment length */
  double getMaximumSolutionSegmentLength() const
  {
    return max_solution_segment_length_;
  }

  /* \brief Set the maximum solution segment length */
  void setMaximumSolutionSegmentLength(double mssl)
  {
    max_solution_segment_length_ = mssl;
  }

  unsigned int getMinimumWaypointCount() const
  {
    return minimum_waypoint_count_;
  }

  /** \brief Get the minimum number of waypoints along the solution path */
  void setMinimumWaypointCount(unsigned int mwc)
  {
    minimum_waypoint_count_ = mwc;
  }

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** \brief Returns a planning context to OMPLInterface, which in turn passes it to OMPLPlannerManager.
   *
   * This function checks the input and reads planner specific configurations.
   * Then it creates the planning context with PlanningContextManager::createPlanningContext.
   * Finally, it puts the context into a state appropriate for planning.
   * This last step involves setting the start, goal, and state validity checker using the method
   * ModelBasedPlanningContext::configure.
   *
   * */
  ModelBasedPlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                  const planning_interface::MotionPlanRequest& req,
                                                  moveit_msgs::MoveItErrorCodes& error_code, const ros::NodeHandle& nh,
                                                  bool use_constraints_approximations) const;

  void registerPlannerAllocator(const std::string& planner_id, const ConfiguredPlannerAllocator& pa)
  {
    known_planners_[planner_id] = pa;
  }

  void registerStateSpaceFactory(const ModelBasedStateSpaceFactoryPtr& factory)
  {
    state_space_factories_[factory->getType()] = factory;
  }

  const std::map<std::string, ConfiguredPlannerAllocator>& getRegisteredPlannerAllocators() const
  {
    return known_planners_;
  }

  const std::map<std::string, ModelBasedStateSpaceFactoryPtr>& getRegisteredStateSpaceFactories() const
  {
    return state_space_factories_;
  }

  ConfiguredPlannerSelector getPlannerSelector() const;

protected:
  ConfiguredPlannerAllocator plannerSelector(const std::string& planner) const;

  void registerDefaultPlanners();
  void registerDefaultStateSpaces();

  template <typename T>
  void registerPlannerAllocatorHelper(const std::string& planner_id);

  /** \brief This is the function that constructs new planning contexts if no previous ones exist that are suitable */
  ModelBasedPlanningContextPtr getPlanningContext(const planning_interface::PlannerConfigurationSettings& config,
                                                  const ModelBasedStateSpaceFactoryPtr& factory) const;

  const ModelBasedStateSpaceFactoryPtr& getStateSpaceFactory(const std::string& factory_type) const;
  const ModelBasedStateSpaceFactoryPtr& getStateSpaceFactory(const std::string& group_name,
                                                             const moveit_msgs::MotionPlanRequest& req) const;

  /** \brief The kinematic model for which motion plans are computed */
  moveit::core::RobotModelConstPtr robot_model_;

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  std::map<std::string, ConfiguredPlannerAllocator> known_planners_;
  std::map<std::string, ModelBasedStateSpaceFactoryPtr> state_space_factories_;

  /** \brief All the existing planning configurations. The name
      of the configuration is the key of the map. This name can
      be of the form "group_name[config_name]" if there are
      particular configurations specified for a group, or of the
      form "group_name" if default settings are to be used. */
  planning_interface::PlannerConfigurationMap planner_configs_;

  /// maximum number of states to sample in the goal region for any planning request (when such sampling is possible)
  unsigned int max_goal_samples_;

  /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some
  /// set of constraints
  unsigned int max_state_sampling_attempts_;

  /// maximum number of attempts to be made at sampling goals
  unsigned int max_goal_sampling_attempts_;

  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int max_planning_threads_;

  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the
  /// extent of the space
  double max_solution_segment_length_;

  /// the minimum number of points to include on the solution path (interpolation is used to reach this number, if
  /// needed)
  unsigned int minimum_waypoint_count_;

  /// Multi-query planner allocator
  MultiQueryPlannerAllocator planner_allocator_;

private:
  MOVEIT_STRUCT_FORWARD(CachedContexts);
  CachedContextsPtr cached_contexts_;
};
}  // namespace ompl_interface
