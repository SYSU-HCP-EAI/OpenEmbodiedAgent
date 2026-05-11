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

/* Author: Henning Kayser */

#include <stdexcept>

#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <thread>

namespace moveit_cpp
{
constexpr char LOGNAME[] = "planning_component";

PlanningComponent::PlanningComponent(const std::string& group_name, const MoveItCppPtr& moveit_cpp)
  : nh_(moveit_cpp->getNodeHandle()), moveit_cpp_(moveit_cpp), group_name_(group_name)
{
  joint_model_group_ = moveit_cpp_->getRobotModel()->getJointModelGroup(group_name);
  if (!joint_model_group_)
  {
    std::string error = "Could not find joint model group '" + group_name + "'.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }
  plan_request_parameters_.load(nh_);
  ROS_DEBUG_STREAM_NAMED(
      LOGNAME, "Plan request parameters loaded with --"
                   << " planning_pipeline: " << plan_request_parameters_.planning_pipeline << ","
                   << " planner_id: " << plan_request_parameters_.planner_id << ","
                   << " planning_time: " << plan_request_parameters_.planning_time << ","
                   << " planning_attempts: " << plan_request_parameters_.planning_attempts << ","
                   << " max_velocity_scaling_factor: " << plan_request_parameters_.max_velocity_scaling_factor << ","
                   << " max_acceleration_scaling_factor: " << plan_request_parameters_.max_acceleration_scaling_factor);
}

PlanningComponent::PlanningComponent(const std::string& group_name, const ros::NodeHandle& nh)
  : PlanningComponent(group_name, std::make_shared<MoveItCpp>(nh))
{
}

PlanningComponent::~PlanningComponent()
{
  ROS_INFO_NAMED(LOGNAME, "Deleting PlanningComponent '%s'", group_name_.c_str());
  clearContents();
}

const std::vector<std::string> PlanningComponent::getNamedTargetStates()
{
  if (joint_model_group_)
  {
    return joint_model_group_->getDefaultStateNames();
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "Unable to find joint group with name '%s'.", group_name_.c_str());
  }

  std::vector<std::string> empty;
  return empty;
}

const std::string& PlanningComponent::getPlanningGroupName() const
{
  return group_name_;
}

bool PlanningComponent::setPathConstraints(const moveit_msgs::Constraints& path_constraints)
{
  current_path_constraints_ = path_constraints;
  return true;
}

bool PlanningComponent::setTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
  current_trajectory_constraints_ = trajectory_constraints;
  return true;
}

planning_interface::MotionPlanResponse PlanningComponent::plan(const PlanRequestParameters& parameters,
                                                               const bool store_solution)
{
  auto plan_solution = planning_interface::MotionPlanResponse();
  if (!joint_model_group_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrieve joint model group for name '%s'.", group_name_.c_str());
    plan_solution.error_code_ = moveit::core::MoveItErrorCode::INVALID_GROUP_NAME;
    if (store_solution)
    {
      last_plan_solution_ = plan_solution;
    }
    return plan_solution;
  }

  // Clone current planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      moveit_cpp_->getPlanningSceneMonitorNonConst();
  planning_scene_monitor->updateFrameTransforms();
  planning_scene_monitor->lockSceneRead();  // LOCK planning scene
  planning_scene::PlanningScenePtr planning_scene =
      planning_scene::PlanningScene::clone(planning_scene_monitor->getPlanningScene());
  planning_scene_monitor->unlockSceneRead();  // UNLOCK planning scene
  planning_scene_monitor.reset();             // release this pointer

  // Init MotionPlanRequest
  ::planning_interface::MotionPlanRequest req;
  req.group_name = group_name_;
  req.planner_id = parameters.planner_id;
  req.num_planning_attempts = std::max(1, parameters.planning_attempts);
  req.allowed_planning_time = parameters.planning_time;
  req.max_velocity_scaling_factor = parameters.max_velocity_scaling_factor;
  req.max_acceleration_scaling_factor = parameters.max_acceleration_scaling_factor;
  if (workspace_parameters_set_)
    req.workspace_parameters = workspace_parameters_;

  // Set start state
  moveit::core::RobotStatePtr start_state = considered_start_state_;
  if (!start_state)
    start_state = moveit_cpp_->getCurrentState();
  start_state->update();
  moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);
  planning_scene->setCurrentState(*start_state);

  // Set goal constraints
  if (current_goal_constraints_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No goal constraints set for planning request");
    plan_solution.error_code_ = moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
    if (store_solution)
    {
      last_plan_solution_ = plan_solution;
    }
    return plan_solution;
  }
  req.goal_constraints = current_goal_constraints_;

  // Set path constraints
  req.path_constraints = current_path_constraints_;

  // Set trajectory constraints
  req.trajectory_constraints = current_trajectory_constraints_;

  // Run planning attempt
  ::planning_interface::MotionPlanResponse res;
  const auto& pipelines = moveit_cpp_->getPlanningPipelines();
  auto it = pipelines.find(parameters.planning_pipeline);
  if (it == pipelines.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No planning pipeline available for name '%s'", parameters.planning_pipeline.c_str());
    plan_solution.error_code_ = moveit::core::MoveItErrorCode::FAILURE;
    if (store_solution)
    {
      last_plan_solution_ = plan_solution;
    }
    return plan_solution;
  }
  const planning_pipeline::PlanningPipelinePtr pipeline = it->second;
  pipeline->generatePlan(planning_scene, req, res);
  plan_solution.error_code_ = res.error_code_;
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    if (store_solution)
    {
      last_plan_solution_ = plan_solution;
    }
    return plan_solution;
  }
  plan_solution.trajectory_ = res.trajectory_;
  plan_solution.planning_time_ = res.planning_time_;
  plan_solution.start_state_ = req.start_state;
  plan_solution.error_code_ = res.error_code_.val;
  // TODO(henningkayser): Visualize trajectory
  // std::vector<const moveit::core::LinkModel*> eef_links;
  // if (joint_model_group->getEndEffectorTips(eef_links))
  //{
  //  for (const auto& eef_link : eef_links)
  //  {
  //    ROS_INFO_STREAM("Publishing trajectory for end effector " << eef_link->getName());
  //    visual_tools_->publishTrajectoryLine(last_solution_trajectory_, eef_link);
  //    visual_tools_->publishTrajectoryPath(last_solution_trajectory_, false);
  //    visual_tools_->publishRobotState(last_solution_trajectory_->getLastWayPoint(), rviz_visual_tools::TRANSLUCENT);
  //  }
  //}

  if (store_solution)
  {
    last_plan_solution_ = plan_solution;
  }
  return plan_solution;
}

planning_interface::MotionPlanResponse
PlanningComponent::plan(const MultiPipelinePlanRequestParameters& parameters,
                        const SolutionCallbackFunction& solution_selection_callback,
                        const StoppingCriterionFunction& stopping_criterion_callback)
{
  // Create solutions container
  PlanningComponent::PlanSolutions planning_solutions{ parameters.multi_plan_request_parameters.size() };
  std::vector<std::thread> planning_threads;
  planning_threads.reserve(parameters.multi_plan_request_parameters.size());

  // Print a warning if more parallel planning problems than available concurrent threads are defined. If
  // std::thread::hardware_concurrency() is not defined, the command returns 0 so the check does not work
  auto const hardware_concurrency = std::thread::hardware_concurrency();
  if (parameters.multi_plan_request_parameters.size() > hardware_concurrency && hardware_concurrency != 0)
  {
    ROS_WARN_NAMED(
        LOGNAME,
        "More parallel planning problems defined ('%ld') than possible to solve concurrently with the hardware ('%d')",
        parameters.multi_plan_request_parameters.size(), hardware_concurrency);
  }

  // Launch planning threads
  for (auto const& plan_request_parameter : parameters.multi_plan_request_parameters)
  {
    auto planning_thread = std::thread([&]() {
      auto plan_solution = planning_interface::MotionPlanResponse();
      try
      {
        plan_solution = plan(plan_request_parameter, false);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning pipeline '" << plan_request_parameter.planning_pipeline.c_str()
                                                              << "' threw exception '" << e.what() << "'");
        plan_solution.error_code_ = moveit::core::MoveItErrorCode::FAILURE;
      }
      plan_solution.planner_id_ = plan_request_parameter.planner_id;
      planning_solutions.pushBack(plan_solution);

      if (stopping_criterion_callback != nullptr)
      {
        if (stopping_criterion_callback(planning_solutions, parameters))
        {
          // Terminate planning pipelines
          ROS_INFO_STREAM_NAMED(LOGNAME,
                                "Stopping criterion met: Terminating planning pipelines that are still active");
          for (auto const& plan_request_parameter : parameters.multi_plan_request_parameters)
          {
            moveit_cpp_->terminatePlanningPipeline(plan_request_parameter.planning_pipeline);
          }
        }
      }
    });
    planning_threads.push_back(std::move(planning_thread));
  }

  // Wait for threads to finish
  for (auto& planning_thread : planning_threads)
  {
    if (planning_thread.joinable())
    {
      planning_thread.join();
    }
  }

  // Return best solution determined by user defined callback (Default: Shortest path)
  last_plan_solution_ = solution_selection_callback(planning_solutions.getSolutions());
  return last_plan_solution_;
}

planning_interface::MotionPlanResponse PlanningComponent::plan()
{
  return plan(plan_request_parameters_);
}

bool PlanningComponent::setStartState(const moveit::core::RobotState& start_state)
{
  considered_start_state_ = std::make_shared<moveit::core::RobotState>(start_state);
  return true;
}

moveit::core::RobotStatePtr PlanningComponent::getStartState()
{
  if (considered_start_state_)
    return considered_start_state_;
  else
  {
    moveit::core::RobotStatePtr s;
    moveit_cpp_->getCurrentState(s, 1.0);
    return s;
  }
}

bool PlanningComponent::setStartState(const std::string& start_state_name)
{
  const auto& named_targets = getNamedTargetStates();
  if (std::find(named_targets.begin(), named_targets.end(), start_state_name) == named_targets.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No predefined joint state found for target name '%s'", start_state_name.c_str());
    return false;
  }
  moveit::core::RobotState start_state(moveit_cpp_->getRobotModel());
  start_state.setToDefaultValues(joint_model_group_, start_state_name);
  return setStartState(start_state);
}

void PlanningComponent::setStartStateToCurrentState()
{
  considered_start_state_.reset();
}

std::map<std::string, double> PlanningComponent::getNamedTargetStateValues(const std::string& name)
{
  // TODO(henningkayser): verify result
  std::map<std::string, double> positions;
  joint_model_group_->getVariableDefaultPositions(name, positions);
  return positions;
}

void PlanningComponent::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  workspace_parameters_.header.frame_id = moveit_cpp_->getRobotModel()->getModelFrame();
  workspace_parameters_.header.stamp = ros::Time::now();
  workspace_parameters_.min_corner.x = minx;
  workspace_parameters_.min_corner.y = miny;
  workspace_parameters_.min_corner.z = minz;
  workspace_parameters_.max_corner.x = maxx;
  workspace_parameters_.max_corner.y = maxy;
  workspace_parameters_.max_corner.z = maxz;
  workspace_parameters_set_ = true;
}

void PlanningComponent::unsetWorkspace()
{
  workspace_parameters_set_ = false;
}

bool PlanningComponent::setGoal(const std::vector<moveit_msgs::Constraints>& goal_constraints)
{
  current_goal_constraints_ = goal_constraints;
  return true;
}

bool PlanningComponent::setGoal(const moveit::core::RobotState& goal_state)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_) };
  return true;
}

bool PlanningComponent::setGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& link_name)
{
  current_goal_constraints_ = { kinematic_constraints::constructGoalConstraints(link_name, goal_pose) };
  return true;
}

bool PlanningComponent::setGoal(const std::string& goal_state_name)
{
  const auto& named_targets = getNamedTargetStates();
  if (std::find(named_targets.begin(), named_targets.end(), goal_state_name) == named_targets.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "No predefined joint state found for target name '%s'", goal_state_name.c_str());
    return false;
  }
  moveit::core::RobotState goal_state(moveit_cpp_->getRobotModel());
  goal_state.setToDefaultValues(joint_model_group_, goal_state_name);
  return setGoal(goal_state);
}

bool PlanningComponent::execute(bool blocking)
{
  if (!last_plan_solution_)
  {
    ROS_ERROR_NAMED(LOGNAME, "There is no successful plan to execute");
    return false;
  }

  // TODO(henningkayser): parameterize timestamps if required
  // trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  // if (!totg.computeTimeStamps(*last_solution_trajectory_, max_velocity_scaling_factor_,
  // max_acceleration_scaling_factor_))
  //{
  //  ROS_ERROR("Failed to parameterize trajectory");
  //  return false;
  //}
  return moveit_cpp_->execute(group_name_, last_plan_solution_.trajectory_, blocking);
}

planning_interface::MotionPlanResponse const& PlanningComponent::getLastMotionPlanResponse()
{
  return last_plan_solution_;
}

planning_interface::MotionPlanResponse
getShortestSolution(std::vector<planning_interface::MotionPlanResponse> const& solutions)
{
  // Find trajectory with minimal path
  auto const shortest_trajectory = std::min_element(solutions.begin(), solutions.end(),
                                                    [](planning_interface::MotionPlanResponse const& solution_a,
                                                       planning_interface::MotionPlanResponse const& solution_b) {
                                                      // If both solutions were successful, check which path is shorter
                                                      if (solution_a && solution_b)
                                                      {
                                                        return robot_trajectory::path_length(*solution_a.trajectory_) <
                                                               robot_trajectory::path_length(*solution_b.trajectory_);
                                                      }
                                                      // If only solution a is successful, return a
                                                      else if (solution_a)
                                                      {
                                                        return true;
                                                      }
                                                      // Else return solution b, either because it is successful or not
                                                      return false;
                                                    });
  if (shortest_trajectory->trajectory_ != nullptr)
  {
    ROS_INFO_NAMED(LOGNAME, "Chosen solution with shortest path length: '%f'",
                   robot_trajectory::path_length(*shortest_trajectory->trajectory_));
  }
  else
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Could not determine shortest path");
  }
  return *shortest_trajectory;
}

void PlanningComponent::clearContents()
{
  considered_start_state_.reset();
  last_plan_solution_ = planning_interface::MotionPlanResponse();
  current_goal_constraints_.clear();
  moveit_cpp_.reset();
}
}  // namespace moveit_cpp
