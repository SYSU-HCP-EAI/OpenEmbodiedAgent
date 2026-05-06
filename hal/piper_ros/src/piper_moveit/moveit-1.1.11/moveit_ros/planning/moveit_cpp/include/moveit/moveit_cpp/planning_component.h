/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Henning Kayser
   Desc: API for planning and execution capabilities of a JointModelGroup */

#pragma once

#include <ros/ros.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit/planning_interface/planning_response.h>
#include <mutex>

namespace moveit_cpp
{
MOVEIT_CLASS_FORWARD(PlanningComponent);  // Defines PlanningComponentPtr, ConstPtr, WeakPtr... etc

/// \brief A function to choose the solution with the shortest path from a vector of solutions
planning_interface::MotionPlanResponse
getShortestSolution(std::vector<planning_interface::MotionPlanResponse> const& solutions);

class PlanningComponent
{
public:
  using MoveItErrorCode [[deprecated("Use moveit::core::MoveItErrorCode")]] = moveit::core::MoveItErrorCode;

  class PlanSolutions
  {
  public:
    /// \brief Constructor
    PlanSolutions(const size_t expected_size = 0)
    {
      solutions_.reserve(expected_size);
    }

    /// \brief Thread safe method to add PlanSolutions to this data structure
    /// TODO(sjahr): Refactor this method to an insert method similar to
    /// https://github.com/ompl/ompl/blob/main/src/ompl/base/src/ProblemDefinition.cpp#L54-L161. This way, it is
    /// possible to create a sorted container e.g. according to a user specified criteria
    void pushBack(const planning_interface::MotionPlanResponse& plan_solution)
    {
      std::lock_guard<std::mutex> lock_guard(solutions_mutex_);
      solutions_.push_back(plan_solution);
    }

    /// \brief Get solutions
    std::vector<planning_interface::MotionPlanResponse> const& getSolutions() const
    {
      return solutions_;
    }

  private:
    std::vector<planning_interface::MotionPlanResponse> solutions_;
    std::mutex solutions_mutex_;
  };

  /// Planner parameters provided with the MotionPlanRequest
  struct PlanRequestParameters
  {
    std::string planner_id;
    std::string planning_pipeline;
    int planning_attempts;
    double planning_time;
    double max_velocity_scaling_factor;
    double max_acceleration_scaling_factor;

    void load(const ros::NodeHandle& nh, const std::string& param_namespace = "")
    {
      std::string ns = "plan_request_params/";
      if (!param_namespace.empty())
      {
        ns = param_namespace + "/plan_request_params/";
      }

      nh.param(ns + "planner_id", planner_id, std::string(""));
      nh.param(ns + "planning_pipeline", planning_pipeline, std::string(""));
      nh.param(ns + "planning_time", planning_time, 1.0);
      nh.param(ns + "planning_attempts", planning_attempts, 5);
      nh.param(ns + "max_velocity_scaling_factor", max_velocity_scaling_factor, 1.0);
      nh.param(ns + "max_acceleration_scaling_factor", max_acceleration_scaling_factor, 1.0);
    }
  };

  /// Planner parameters provided with the MotionPlanRequest
  struct MultiPipelinePlanRequestParameters
  {
    MultiPipelinePlanRequestParameters(const ros::NodeHandle& nh,
                                       const std::vector<std::string>& planning_pipeline_names)
    {
      multi_plan_request_parameters.reserve(planning_pipeline_names.size());

      for (auto const& planning_pipeline_name : planning_pipeline_names)
      {
        PlanRequestParameters parameters;
        parameters.load(nh, planning_pipeline_name);
        multi_plan_request_parameters.push_back(parameters);
      }
    }
    // Plan request parameters for the individual planning pipelines which run concurrently
    std::vector<PlanRequestParameters> multi_plan_request_parameters;
  };

  /// \brief A solution callback function type for the parallel planning API of planning component
  typedef std::function<planning_interface::MotionPlanResponse(
      std::vector<planning_interface::MotionPlanResponse> const& solutions)>
      SolutionCallbackFunction;
  /// \brief A stopping criterion callback function for the parallel planning API of planning component
  typedef std::function<bool(PlanSolutions const& solutions,
                             MultiPipelinePlanRequestParameters const& plan_request_parameters)>
      StoppingCriterionFunction;

  /** \brief Constructor */
  PlanningComponent(const std::string& group_name, const ros::NodeHandle& nh);
  PlanningComponent(const std::string& group_name, const MoveItCppPtr& moveit_cpp);

  /**
   * @brief This class owns unique resources (e.g. action clients, threads) and its not very
   * meaningful to copy. Pass by references, move it, or simply create multiple instances where
   * required.
   */
  PlanningComponent(const PlanningComponent&) = delete;
  PlanningComponent& operator=(const PlanningComponent&) = delete;

  PlanningComponent(PlanningComponent&& other) = default;
  PlanningComponent& operator=(PlanningComponent&& other) = delete;

  /** \brief Destructor */
  ~PlanningComponent();

  /** \brief Get the name of the planning group */
  const std::string& getPlanningGroupName() const;

  /** \brief Get the names of the named robot states available as targets */
  const std::vector<std::string> getNamedTargetStates();

  /** \brief Get the joint values for targets specified by name */
  std::map<std::string, double> getNamedTargetStateValues(const std::string& name);

  /** \brief Specify the workspace bounding box.
       The box is specified in the planning frame (i.e. relative to the robot root link start position).
       This is useful when the planning group contains the root joint of the robot -- i.e. when planning motion for the
     robot relative to the world. */
  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);

  /** \brief Remove the workspace bounding box from planning */
  void unsetWorkspace();

  /** \brief Get the considered start state if defined, otherwise get the current state */
  moveit::core::RobotStatePtr getStartState();

  /** \brief Set the robot state that should be considered as start state for planning */
  bool setStartState(const moveit::core::RobotState& start_state);
  /** \brief Set the named robot state that should be considered as start state for planning */
  bool setStartState(const std::string& named_state);

  /** \brief Set the start state for planning to be the current state of the robot */
  void setStartStateToCurrentState();

  /** \brief Set the goal constraints used for planning */
  bool setGoal(const std::vector<moveit_msgs::Constraints>& goal_constraints);
  /** \brief Set the goal constraints generated from a target state */
  bool setGoal(const moveit::core::RobotState& goal_state);
  /** \brief Set the goal constraints generated from target pose and robot link */
  bool setGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& link_name);
  /** \brief Set the goal constraints generated from a named target state */
  bool setGoal(const std::string& named_target);

  /** \brief Set the path constraints used for planning */
  bool setPathConstraints(const moveit_msgs::Constraints& path_constraints);

  /** \brief Set the trajectory constraints generated from a moveit msg Constraints */
  bool setTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints& trajectory_constraints);

  /** \brief Run a plan from start or current state to fulfill the last goal constraints provided by setGoal() using
   * default parameters. */
  planning_interface::MotionPlanResponse plan();
  /** \brief Run a plan from start or current state to fulfill the last goal constraints provided by setGoal() using the
   * provided PlanRequestParameters. */
  planning_interface::MotionPlanResponse plan(const PlanRequestParameters& parameters, const bool store_solution = true);

  /** \brief Run a plan from start or current state to fulfill the last goal constraints provided by setGoal() using the
   * provided PlanRequestParameters. */
  planning_interface::MotionPlanResponse
  plan(const MultiPipelinePlanRequestParameters& parameters,
       const SolutionCallbackFunction& solution_selection_callback = &getShortestSolution,
       const StoppingCriterionFunction& stopping_criterion_callback = StoppingCriterionFunction());

  /** \brief Execute the latest computed solution trajectory computed by plan(). By default this function terminates
   * after the execution is complete. The execution can be run in background by setting blocking to false. */
  bool execute(bool blocking = true);

  /** \brief Return the last plan solution*/
  planning_interface::MotionPlanResponse const& getLastMotionPlanResponse();

private:
  // Core properties and instances
  ros::NodeHandle nh_;
  MoveItCppPtr moveit_cpp_;
  const std::string group_name_;
  // The robot_model_ member variable of MoveItCpp class will manually free the joint_model_group_ resources
  const moveit::core::JointModelGroup* joint_model_group_;

  // Planning
  // The start state used in the planning motion request
  moveit::core::RobotStatePtr considered_start_state_;
  std::vector<moveit_msgs::Constraints> current_goal_constraints_;
  moveit_msgs::TrajectoryConstraints current_trajectory_constraints_;
  moveit_msgs::Constraints current_path_constraints_;
  PlanRequestParameters plan_request_parameters_;
  moveit_msgs::WorkspaceParameters workspace_parameters_;
  bool workspace_parameters_set_ = false;
  planning_interface::MotionPlanResponse last_plan_solution_;

  // common properties for goals
  // TODO(henningkayser): support goal tolerances
  // double goal_joint_tolerance_;
  // double goal_position_tolerance_;
  // double goal_orientation_tolerance_;
  // TODO(henningkayser): implment path/trajectory constraints
  // std::unique_ptr<moveit_msgs::Constraints> path_constraints_;
  // std::unique_ptr<moveit_msgs::TrajectoryConstraints> trajectory_constraints_;

  /** \brief Reset all member variables */
  void clearContents();
};
}  // namespace moveit_cpp

namespace moveit
{
namespace planning_interface
{
using PlanningComponent [[deprecated("use moveit_cpp")]] = moveit_cpp::PlanningComponent;
[[deprecated("use moveit_cpp")]] MOVEIT_DECLARE_PTR(PlanningComponent, moveit_cpp::PlanningComponent);
}  // namespace planning_interface
}  // namespace moveit
