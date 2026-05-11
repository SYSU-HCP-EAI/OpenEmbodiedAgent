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

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/detail/goal_union.h>
#include <moveit/ompl_interface/detail/projection_evaluators.h>
#include <moveit/ompl_interface/detail/constraints_library.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/profiler/profiler.h>
#include <moveit/utils/lexical_casts.h>

#include <ompl/config.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/PDF.h>
// TODO: remove when ROS Melodic and older are no longer supported
#if OMPL_VERSION_VALUE < 1005000
#include <ompl/base/PlannerTerminationCondition.h>
#else
// IterationTerminationCondition was moved to a separate file and
// CostConvergenceTerminationCondition was added in OMPL 1.5.0.
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#endif

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include <ompl/geometric/planners/prm/LazyPRM.h>

namespace ompl_interface
{
constexpr char LOGNAME[] = "model_based_planning_context";
}  // namespace ompl_interface

ompl_interface::ModelBasedPlanningContext::ModelBasedPlanningContext(const std::string& name,
                                                                     const ModelBasedPlanningContextSpecification& spec)
  : planning_interface::PlanningContext(name, spec.state_space_->getJointModelGroup()->getName())
  , spec_(spec)
  , complete_initial_robot_state_(spec.state_space_->getRobotModel())
  , ompl_simple_setup_(spec.ompl_simple_setup_)
  , ompl_benchmark_(*ompl_simple_setup_)
  , ompl_parallel_plan_(ompl_simple_setup_->getProblemDefinition())
  , ptc_(nullptr)
  , last_plan_time_(0.0)
  , last_simplify_time_(0.0)
  , max_goal_samples_(0)
  , max_state_sampling_attempts_(0)
  , max_goal_sampling_attempts_(0)
  , max_planning_threads_(0)
  , max_solution_segment_length_(0.0)
  , minimum_waypoint_count_(0)
  , multi_query_planning_enabled_(false)  // maintain "old" behavior by default
  , simplify_solutions_(true)
  , interpolate_(true)
  , hybridize_(true)
{
  complete_initial_robot_state_.update();

  constraints_library_ = std::make_shared<ConstraintsLibrary>(this);
}

void ompl_interface::ModelBasedPlanningContext::configure(const ros::NodeHandle& nh, bool use_constraints_approximations)
{
  loadConstraintApproximations(nh);
  if (!use_constraints_approximations)
  {
    setConstraintsApproximations(ConstraintsLibraryPtr());
  }
  complete_initial_robot_state_.update();
  ompl_simple_setup_->getStateSpace()->computeSignature(space_signature_);
  ompl_simple_setup_->getStateSpace()->setStateSamplerAllocator(
      [this](const ompl::base::StateSpace* ss) { return allocPathConstrainedSampler(ss); });

  // convert the input state to the corresponding OMPL state
  ompl::base::ScopedState<> ompl_start_state(spec_.state_space_);
  spec_.state_space_->copyToOMPLState(ompl_start_state.get(), getCompleteInitialRobotState());
  ompl_simple_setup_->setStartState(ompl_start_state);
  ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(this)));

  if (path_constraints_ && constraints_library_)
  {
    const ConstraintApproximationPtr& constraint_approx =
        constraints_library_->getConstraintApproximation(path_constraints_msg_);
    if (constraint_approx)
    {
      getOMPLStateSpace()->setInterpolationFunction(constraint_approx->getInterpolationFunction());
      ROS_INFO_NAMED(LOGNAME, "Using precomputed interpolation states");
    }
  }

  useConfig();
  if (ompl_simple_setup_->getGoal())
    ompl_simple_setup_->setup();
}

void ompl_interface::ModelBasedPlanningContext::setProjectionEvaluator(const std::string& peval)
{
  if (!spec_.state_space_)
  {
    ROS_ERROR_NAMED(LOGNAME, "No state space is configured yet");
    return;
  }
  ob::ProjectionEvaluatorPtr projection_eval = getProjectionEvaluator(peval);
  if (projection_eval)
    spec_.state_space_->registerDefaultProjection(projection_eval);
}

ompl::base::ProjectionEvaluatorPtr
ompl_interface::ModelBasedPlanningContext::getProjectionEvaluator(const std::string& peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getRobotModel()->hasLinkModel(link_name))
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
    else
      ROS_ERROR_NAMED(LOGNAME,
                      "Attempted to set projection evaluator with respect to position of link '%s', "
                      "but that link is not known to the kinematic model.",
                      link_name.c_str());
  }
  else if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string joints = peval.substr(7, peval.length() - 8);
    boost::replace_all(joints, ",", " ");
    std::vector<unsigned int> j;
    std::stringstream ss(joints);
    while (ss.good() && !ss.eof())
    {
      std::string joint;
      ss >> joint >> std::ws;
      if (getJointModelGroup()->hasJointModel(joint))
      {
        unsigned int variable_count = getJointModelGroup()->getJointModel(joint)->getVariableCount();
        if (variable_count > 0)
        {
          int idx = getJointModelGroup()->getVariableGroupIndex(joint);
          for (unsigned int q = 0; q < variable_count; ++q)
            j.push_back(idx + q);
        }
        else
          ROS_WARN_NAMED(LOGNAME, "%s: Ignoring joint '%s' in projection since it has 0 DOF", name_.c_str(),
                         joint.c_str());
      }
      else
        ROS_ERROR_NAMED(LOGNAME,
                        "%s: Attempted to set projection evaluator with respect to value of joint "
                        "'%s', but that joint is not known to the group '%s'.",
                        name_.c_str(), joint.c_str(), getGroupName().c_str());
    }
    if (j.empty())
      ROS_ERROR_NAMED(LOGNAME, "%s: No valid joints specified for joint projection", name_.c_str());
    else
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
  return ob::ProjectionEvaluatorPtr();
}

ompl::base::StateSamplerPtr
ompl_interface::ModelBasedPlanningContext::allocPathConstrainedSampler(const ompl::base::StateSpace* state_space) const
{
  if (spec_.state_space_.get() != state_space)
  {
    ROS_ERROR_NAMED(LOGNAME, "%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
    return ompl::base::StateSamplerPtr();
  }

  ROS_DEBUG_NAMED(LOGNAME, "%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());

  if (path_constraints_)
  {
    if (constraints_library_)
    {
      const ConstraintApproximationPtr& constraint_approx =
          constraints_library_->getConstraintApproximation(path_constraints_msg_);
      if (constraint_approx)
      {
        ompl::base::StateSamplerAllocator state_sampler_allocator =
            constraint_approx->getStateSamplerAllocator(path_constraints_msg_);
        if (state_sampler_allocator)
        {
          ompl::base::StateSamplerPtr state_sampler = state_sampler_allocator(state_space);
          if (state_sampler)
          {
            ROS_INFO_NAMED(LOGNAME,
                           "%s: Using precomputed state sampler (approximated constraint space) for constraint '%s'",
                           name_.c_str(), path_constraints_msg_.name.c_str());
            return state_sampler;
          }
        }
      }
    }

    constraint_samplers::ConstraintSamplerPtr constraint_sampler;
    if (spec_.constraint_sampler_manager_)
      constraint_sampler = spec_.constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(),
                                                                            path_constraints_->getAllConstraints());

    if (constraint_sampler)
    {
      ROS_INFO_NAMED(LOGNAME, "%s: Allocating specialized state sampler for state space", name_.c_str());
      return ob::StateSamplerPtr(new ConstrainedSampler(this, constraint_sampler));
    }
  }
  ROS_DEBUG_NAMED(LOGNAME, "%s: Allocating default state sampler for state space", name_.c_str());
  return state_space->allocDefaultStateSampler();
}

void ompl_interface::ModelBasedPlanningContext::useConfig()
{
  const std::map<std::string, std::string>& config = spec_.config_;
  if (config.empty())
    return;
  std::map<std::string, std::string> cfg = config;

  // set the distance between waypoints when interpolating and collision checking.
  auto it = cfg.find("longest_valid_segment_fraction");
  // If one of the two variables is set.
  if (it != cfg.end() || max_solution_segment_length_ != 0.0)
  {
    // clang-format off
    double longest_valid_segment_fraction_config = (it != cfg.end())
      ? moveit::core::toDouble(it->second)  // value from config file if there
      : 0.01;  // default value in OMPL.
    double longest_valid_segment_fraction_final = longest_valid_segment_fraction_config;
    if (max_solution_segment_length_ > 0.0)
    {
      // If this parameter is specified too, take the most conservative of the two variables,
      // i.e. the one that uses the shorter segment length.
      longest_valid_segment_fraction_final = std::min(
          longest_valid_segment_fraction_config,
          max_solution_segment_length_ / spec_.state_space_->getMaximumExtent()
      );
    }
    // clang-format on

    // convert to string using no locale
    cfg["longest_valid_segment_fraction"] = moveit::core::toString(longest_valid_segment_fraction_final);
  }

  // set the projection evaluator
  it = cfg.find("projection_evaluator");
  if (it != cfg.end())
  {
    setProjectionEvaluator(boost::trim_copy(it->second));
    cfg.erase(it);
  }

  if (cfg.empty())
    return;

  std::string optimizer;
  ompl::base::OptimizationObjectivePtr objective;
  it = cfg.find("optimization_objective");
  if (it != cfg.end())
  {
    optimizer = it->second;
    cfg.erase(it);

    if (optimizer == "PathLengthOptimizationObjective")
    {
      objective =
          std::make_shared<ompl::base::PathLengthOptimizationObjective>(ompl_simple_setup_->getSpaceInformation());
    }
    else if (optimizer == "MinimaxObjective")
    {
      objective = std::make_shared<ompl::base::MinimaxObjective>(ompl_simple_setup_->getSpaceInformation());
    }
    else if (optimizer == "StateCostIntegralObjective")
    {
      objective = std::make_shared<ompl::base::StateCostIntegralObjective>(ompl_simple_setup_->getSpaceInformation());
    }
    else if (optimizer == "MechanicalWorkOptimizationObjective")
    {
      objective =
          std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(ompl_simple_setup_->getSpaceInformation());
    }
    else if (optimizer == "MaximizeMinClearanceObjective")
    {
      objective =
          std::make_shared<ompl::base::MaximizeMinClearanceObjective>(ompl_simple_setup_->getSpaceInformation());
    }
    else
    {
      objective =
          std::make_shared<ompl::base::PathLengthOptimizationObjective>(ompl_simple_setup_->getSpaceInformation());
    }

    ompl_simple_setup_->setOptimizationObjective(objective);
  }

  // Don't clear planner data if multi-query planning is enabled
  it = cfg.find("multi_query_planning_enabled");
  if (it != cfg.end())
    multi_query_planning_enabled_ = boost::lexical_cast<bool>(it->second);

  // check whether the path returned by the planner should be interpolated
  it = cfg.find("interpolate");
  if (it != cfg.end())
  {
    interpolate_ = boost::lexical_cast<bool>(it->second);
    cfg.erase(it);
  }

  // check whether solution paths from parallel planning should be hybridized
  it = cfg.find("hybridize");
  if (it != cfg.end())
  {
    hybridize_ = boost::lexical_cast<bool>(it->second);
    cfg.erase(it);
  }

  // remove the 'type' parameter; the rest are parameters for the planner itself
  it = cfg.find("type");
  if (it == cfg.end())
  {
    if (name_ != getGroupName())
      ROS_WARN_NAMED(LOGNAME, "%s: Attribute 'type' not specified in planner configuration", name_.c_str());
  }
  else
  {
    std::string type = it->second;
    cfg.erase(it);
    const std::string planner_name = getGroupName() + "/" + name_;
    ompl_simple_setup_->setPlannerAllocator(
        [planner_name, &spec = this->spec_, allocator = spec_.planner_selector_(type)](
            const ompl::base::SpaceInformationPtr& si) { return allocator(si, planner_name, spec); });
    ROS_INFO_NAMED(LOGNAME,
                   "Planner configuration '%s' will use planner '%s'. "
                   "Additional configuration parameters will be set when the planner is constructed.",
                   name_.c_str(), type.c_str());
  }

  // call the setParams() after setup(), so we know what the params are
  ompl_simple_setup_->getSpaceInformation()->setup();
  ompl_simple_setup_->getSpaceInformation()->params().setParams(cfg, true);
  // call setup() again for possibly new param values
  ompl_simple_setup_->getSpaceInformation()->setup();
}

void ompl_interface::ModelBasedPlanningContext::setPlanningVolume(const moveit_msgs::WorkspaceParameters& wparams)
{
  if (wparams.min_corner.x == wparams.max_corner.x && wparams.min_corner.x == 0.0 &&
      wparams.min_corner.y == wparams.max_corner.y && wparams.min_corner.y == 0.0 &&
      wparams.min_corner.z == wparams.max_corner.z && wparams.min_corner.z == 0.0)
    ROS_WARN_NAMED(LOGNAME, "It looks like the planning volume was not specified.");

  ROS_DEBUG_NAMED(LOGNAME,
                  "%s: Setting planning volume (affects SE2 & SE3 joints only) to x = [%f, %f], y = "
                  "[%f, %f], z = [%f, %f]",
                  name_.c_str(), wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y,
                  wparams.min_corner.z, wparams.max_corner.z);

  spec_.state_space_->setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y,
                                        wparams.max_corner.y, wparams.min_corner.z, wparams.max_corner.z);
}

void ompl_interface::ModelBasedPlanningContext::simplifySolution(double timeout)
{
  ompl::time::point start = ompl::time::now();
  ob::PlannerTerminationCondition ptc = constructPlannerTerminationCondition(timeout, start);
  registerTerminationCondition(ptc);
  ompl_simple_setup_->simplifySolution(ptc);
  last_simplify_time_ = ompl_simple_setup_->getLastSimplificationTime();
  unregisterTerminationCondition();
}

void ompl_interface::ModelBasedPlanningContext::interpolateSolution()
{
  if (ompl_simple_setup_->haveSolutionPath())
  {
    og::PathGeometric& pg = ompl_simple_setup_->getSolutionPath();

    // Find the number of states that will be in the interpolated solution.
    // This is what interpolate() does internally.
    unsigned int eventual_states = 1;
    std::vector<ompl::base::State*> states = pg.getStates();
    for (size_t i = 0; i < states.size() - 1; i++)
    {
      eventual_states += ompl_simple_setup_->getStateSpace()->validSegmentCount(states[i], states[i + 1]);
    }

    if (eventual_states < minimum_waypoint_count_)
    {
      // If that's not enough states, use the minimum amount instead.
      pg.interpolate(minimum_waypoint_count_);
    }
    else
    {
      // Interpolate the path to have as the exact states that are checked when validating motions.
      pg.interpolate();
    }
  }
}

void ompl_interface::ModelBasedPlanningContext::convertPath(const ompl::geometric::PathGeometric& pg,
                                                            robot_trajectory::RobotTrajectory& traj) const
{
  moveit::core::RobotState ks = complete_initial_robot_state_;
  for (std::size_t i = 0; i < pg.getStateCount(); ++i)
  {
    spec_.state_space_->copyToRobotState(ks, pg.getState(i));
    traj.addSuffixWayPoint(ks, 0.0);
  }
}

bool ompl_interface::ModelBasedPlanningContext::getSolutionPath(robot_trajectory::RobotTrajectory& traj) const
{
  traj.clear();
  if (ompl_simple_setup_->haveSolutionPath())
    convertPath(ompl_simple_setup_->getSolutionPath(), traj);
  return ompl_simple_setup_->haveSolutionPath();
}

void ompl_interface::ModelBasedPlanningContext::setVerboseStateValidityChecks(bool flag)
{
  if (ompl_simple_setup_->getStateValidityChecker())
    static_cast<StateValidityChecker*>(ompl_simple_setup_->getStateValidityChecker().get())->setVerbose(flag);
}

ompl::base::GoalPtr ompl_interface::ModelBasedPlanningContext::constructGoal()
{
  // ******************* set up the goal representation, based on goal constraints

  std::vector<ob::GoalPtr> goals;
  for (kinematic_constraints::KinematicConstraintSetPtr& goal_constraint : goal_constraints_)
  {
    constraint_samplers::ConstraintSamplerPtr constraint_sampler;
    if (spec_.constraint_sampler_manager_)
      constraint_sampler = spec_.constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(),
                                                                            goal_constraint->getAllConstraints());
    if (constraint_sampler)
    {
      ob::GoalPtr goal = ob::GoalPtr(new ConstrainedGoalSampler(this, goal_constraint, constraint_sampler));
      goals.push_back(goal);
    }
  }

  if (!goals.empty())
    return goals.size() == 1 ? goals[0] : ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));
  else
    ROS_ERROR_NAMED(LOGNAME, "Unable to construct goal representation");

  return ob::GoalPtr();
}

ompl::base::PlannerTerminationCondition
ompl_interface::ModelBasedPlanningContext::constructPlannerTerminationCondition(double timeout,
                                                                                const ompl::time::point& start)
{
  auto it = spec_.config_.find("termination_condition");
  if (it == spec_.config_.end())
    return ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
  std::string termination_string = it->second;
  std::vector<std::string> termination_and_params;
  boost::split(termination_and_params, termination_string, boost::is_any_of("[ ,]"));

  if (termination_and_params.empty())
    ROS_ERROR_NAMED(LOGNAME, "Termination condition not specified");
  // Terminate if a maximum number of iterations is exceeded or a timeout occurs.
  // The semantics of "iterations" are planner-specific, but typically it corresponds to the number of times
  // an attempt was made to grow a roadmap/tree.
  else if (termination_and_params[0] == "Iteration")
  {
    if (termination_and_params.size() > 1)
      return ob::plannerOrTerminationCondition(
          ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start)),
          ob::IterationTerminationCondition(std::stoul(termination_and_params[1])));
    else
      ROS_ERROR_NAMED(LOGNAME, "Missing argument to Iteration termination condition");
  }
// TODO: remove when ROS Melodic and older are no longer supported
#if OMPL_VERSION_VALUE >= 1005000
  // Terminate if the cost has converged or a timeout occurs.
  // Only useful for anytime/optimizing planners.
  else if (termination_and_params[0] == "CostConvergence")
  {
    std::size_t solutions_window = 10u;
    double epsilon = 0.1;
    if (termination_and_params.size() > 1)
    {
      solutions_window = std::stoul(termination_and_params[1]);
      if (termination_and_params.size() > 2)
        epsilon = moveit::core::toDouble(termination_and_params[2]);
    }
    return ob::plannerOrTerminationCondition(
        ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start)),
        ob::CostConvergenceTerminationCondition(ompl_simple_setup_->getProblemDefinition(), solutions_window, epsilon));
  }
#endif
  // Terminate as soon as an exact solution is found or a timeout occurs.
  // This modifies the behavior of anytime/optimizing planners to terminate upon discovering
  // the first feasible solution.
  else if (termination_and_params[0] == "ExactSolution")
  {
    return ob::plannerOrTerminationCondition(
        ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start)),
        ob::exactSolnPlannerTerminationCondition(ompl_simple_setup_->getProblemDefinition()));
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "Unknown planner termination condition");

  // return a planner termination condition to suppress compiler warning
  return ob::plannerAlwaysTerminatingCondition();
}

void ompl_interface::ModelBasedPlanningContext::setCompleteInitialState(
    const moveit::core::RobotState& complete_initial_robot_state)
{
  complete_initial_robot_state_ = complete_initial_robot_state;
  complete_initial_robot_state_.update();
}

void ompl_interface::ModelBasedPlanningContext::clear()
{
  if (!multi_query_planning_enabled_)
    ompl_simple_setup_->clear();
// TODO: remove when ROS Melodic and older are no longer supported
#if OMPL_VERSION_VALUE >= 1005000
  else
  {
    // For LazyPRM and LazyPRMstar we assume that the environment *could* have changed
    // This means that we need to reset the validity flags for every node and edge in
    // the roadmap. For PRM and PRMstar we assume that the environment is static. If
    // this is not the case, then multi-query planning should not be enabled.
    auto planner = dynamic_cast<ompl::geometric::LazyPRM*>(ompl_simple_setup_->getPlanner().get());
    if (planner != nullptr)
      planner->clearValidity();
  }
#endif
  ompl_simple_setup_->clearStartStates();
  ompl_simple_setup_->setGoal(ob::GoalPtr());
  ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr());
  path_constraints_.reset();
  goal_constraints_.clear();
  getOMPLStateSpace()->setInterpolationFunction(InterpolationFunction());
}

bool ompl_interface::ModelBasedPlanningContext::setPathConstraints(const moveit_msgs::Constraints& path_constraints,
                                                                   moveit_msgs::MoveItErrorCodes* /*error*/)
{
  // ******************* set the path constraints to use
  path_constraints_ = std::make_shared<kinematic_constraints::KinematicConstraintSet>(getRobotModel());
  path_constraints_->add(path_constraints, getPlanningScene()->getTransforms());
  path_constraints_msg_ = path_constraints;

  return true;
}

bool ompl_interface::ModelBasedPlanningContext::setGoalConstraints(
    const std::vector<moveit_msgs::Constraints>& goal_constraints, const moveit_msgs::Constraints& path_constraints,
    moveit_msgs::MoveItErrorCodes* error)
{
  // ******************* check if the input is correct
  goal_constraints_.clear();
  for (const moveit_msgs::Constraints& goal_constraint : goal_constraints)
  {
    moveit_msgs::Constraints constr = kinematic_constraints::mergeConstraints(goal_constraint, path_constraints);
    kinematic_constraints::KinematicConstraintSetPtr kset(
        new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
    kset->add(constr, getPlanningScene()->getTransforms());
    if (!kset->empty())
      goal_constraints_.push_back(kset);
  }

  if (goal_constraints_.empty())
  {
    ROS_WARN_NAMED(LOGNAME, "%s: No goal constraints specified. There is no problem to solve.", name_.c_str());
    if (error)
      error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  ob::GoalPtr goal = constructGoal();
  ompl_simple_setup_->setGoal(goal);
  return static_cast<bool>(goal);
}

bool ompl_interface::ModelBasedPlanningContext::benchmark(double timeout, unsigned int count,
                                                          const std::string& filename)
{
  ompl_benchmark_.clearPlanners();
  ompl_simple_setup_->setup();
  ompl_benchmark_.addPlanner(ompl_simple_setup_->getPlanner());
  ompl_benchmark_.setExperimentName(getRobotModel()->getName() + "_" + getGroupName() + "_" +
                                    getPlanningScene()->getName() + "_" + name_);

  ot::Benchmark::Request req;
  req.maxTime = timeout;
  req.runCount = count;
  req.displayProgress = true;
  req.saveConsoleOutput = false;
  ompl_benchmark_.benchmark(req);
  return filename.empty() ? ompl_benchmark_.saveResultsToFile() : ompl_benchmark_.saveResultsToFile(filename.c_str());
}

void ompl_interface::ModelBasedPlanningContext::startSampling()
{
  bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->startSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux*>(ompl_simple_setup_->getGoal().get())->startSampling();
}

void ompl_interface::ModelBasedPlanningContext::stopSampling()
{
  bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ob::GoalLazySamples*>(ompl_simple_setup_->getGoal().get())->stopSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux*>(ompl_simple_setup_->getGoal().get())->stopSampling();
}

void ompl_interface::ModelBasedPlanningContext::preSolve()
{
  // clear previously computed solutions
  ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
  const ob::PlannerPtr planner = ompl_simple_setup_->getPlanner();
  if (planner && !multi_query_planning_enabled_)
    planner->clear();
  startSampling();
  ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
}

void ompl_interface::ModelBasedPlanningContext::postSolve()
{
  stopSampling();
  int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
  int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
  ROS_DEBUG_NAMED(LOGNAME, "There were %d valid motions and %d invalid motions.", v, iv);
}

bool ompl_interface::ModelBasedPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  res.error_code_ = solve(request_.allowed_planning_time, request_.num_planning_attempts);
  if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    double ptime = getLastPlanTime();
    if (simplify_solutions_)
    {
      simplifySolution(request_.allowed_planning_time - ptime);
      ptime += getLastSimplifyTime();
    }

    if (interpolate_)
      interpolateSolution();

    // fill the response
    ROS_DEBUG_NAMED(LOGNAME, "%s: Returning successful solution with %lu states", getName().c_str(),
                    getOMPLSimpleSetup()->getSolutionPath().getStateCount());

    res.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(getRobotModel(), getGroupName());
    getSolutionPath(*res.trajectory_);
    res.planning_time_ = ptime;
    return true;
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Unable to solve the planning problem");
    return false;
  }
}

bool ompl_interface::ModelBasedPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  res.error_code_ = solve(request_.allowed_planning_time, request_.num_planning_attempts);
  if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    res.trajectory_.reserve(3);

    // add info about planned solution
    double ptime = getLastPlanTime();
    res.processing_time_.push_back(ptime);
    res.description_.emplace_back("plan");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back() = std::make_shared<robot_trajectory::RobotTrajectory>(getRobotModel(), getGroupName());
    getSolutionPath(*res.trajectory_.back());

    // simplify solution if time remains
    if (simplify_solutions_)
    {
      simplifySolution(request_.allowed_planning_time - ptime);
      res.processing_time_.push_back(getLastSimplifyTime());
      res.description_.emplace_back("simplify");
      res.trajectory_.resize(res.trajectory_.size() + 1);
      res.trajectory_.back() = std::make_shared<robot_trajectory::RobotTrajectory>(getRobotModel(), getGroupName());
      getSolutionPath(*res.trajectory_.back());
    }

    if (interpolate_)
    {
      ompl::time::point start_interpolate = ompl::time::now();
      interpolateSolution();
      res.processing_time_.push_back(ompl::time::seconds(ompl::time::now() - start_interpolate));
      res.description_.emplace_back("interpolate");
      res.trajectory_.resize(res.trajectory_.size() + 1);
      res.trajectory_.back() = std::make_shared<robot_trajectory::RobotTrajectory>(getRobotModel(), getGroupName());
      getSolutionPath(*res.trajectory_.back());
    }

    ROS_DEBUG_NAMED(LOGNAME, "%s: Returning successful solution with %lu states", getName().c_str(),
                    getOMPLSimpleSetup()->getSolutionPath().getStateCount());
    return true;
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Unable to solve the planning problem");
    return false;
  }
}

const moveit_msgs::MoveItErrorCodes ompl_interface::ModelBasedPlanningContext::solve(double timeout, unsigned int count)
{
  moveit::tools::Profiler::ScopedBlock sblock("PlanningContext:Solve");
  ompl::time::point start = ompl::time::now();
  preSolve();

  moveit_msgs::MoveItErrorCodes result;
  result.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  ob::PlannerTerminationCondition ptc = constructPlannerTerminationCondition(timeout, start);
  registerTerminationCondition(ptc);
  if (count <= 1 || multi_query_planning_enabled_)  // multi-query planners should always run in single instances
  {
    ROS_DEBUG_NAMED(LOGNAME, "%s: Solving the planning problem once...", name_.c_str());
    result.val = errorCode(ompl_simple_setup_->solve(ptc));
    last_plan_time_ = ompl_simple_setup_->getLastPlanComputationTime();
  }
  else
  {
    ROS_DEBUG_NAMED(LOGNAME, "%s: Solving the planning problem %u times...", name_.c_str(), count);
    ompl_parallel_plan_.clearHybridizationPaths();

    auto plan_parallel = [this, &ptc](unsigned int num_planners) {
      ompl_parallel_plan_.clearPlanners();
      if (ompl_simple_setup_->getPlannerAllocator())
        for (unsigned int i = 0; i < num_planners; ++i)
          ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
      else
        for (unsigned int i = 0; i < num_planners; ++i)
          ompl_parallel_plan_.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));

      return errorCode(ompl_parallel_plan_.solve(ptc, 1, num_planners, hybridize_));
    };

    if (count <= max_planning_threads_)
    {
      result.val = plan_parallel(count);
    }
    else
    {
      int n = count / max_planning_threads_;
      for (int i = 0; i < n && result.val != moveit_msgs::MoveItErrorCodes::SUCCESS && !ptc(); ++i)
        result.val = plan_parallel(max_planning_threads_);
      if (result.val != moveit_msgs::MoveItErrorCodes::SUCCESS && !ptc())
        result.val = plan_parallel(count % max_planning_threads_);
    }
    last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
  }
  unregisterTerminationCondition();
  postSolve();
  return result;
}

void ompl_interface::ModelBasedPlanningContext::registerTerminationCondition(const ob::PlannerTerminationCondition& ptc)
{
  std::unique_lock<std::mutex> slock(ptc_lock_);
  ptc_ = &ptc;
}

void ompl_interface::ModelBasedPlanningContext::unregisterTerminationCondition()
{
  std::unique_lock<std::mutex> slock(ptc_lock_);
  ptc_ = nullptr;
}

int32_t ompl_interface::ModelBasedPlanningContext::errorCode(const ompl::base::PlannerStatus& status)
{
  auto result = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  switch (ompl::base::PlannerStatus::StatusType(status))
  {
    case ompl::base::PlannerStatus::UNKNOWN:
      ROS_WARN_NAMED(LOGNAME, "Motion planning failed for an unknown reason");
      result = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      break;
    case ompl::base::PlannerStatus::INVALID_START:
      ROS_WARN_NAMED(LOGNAME, "Invalid start state");
      result = moveit_msgs::MoveItErrorCodes::START_STATE_INVALID;
      break;
    case ompl::base::PlannerStatus::INVALID_GOAL:
      ROS_WARN_NAMED(LOGNAME, "Invalid goal state");
      result = moveit_msgs::MoveItErrorCodes::GOAL_STATE_INVALID;
      break;
    case ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
      ROS_WARN_NAMED(LOGNAME, "Unrecognized goal type");
      result = moveit_msgs::MoveItErrorCodes::UNRECOGNIZED_GOAL_TYPE;
      break;
    case ompl::base::PlannerStatus::TIMEOUT:
      ROS_WARN_NAMED(LOGNAME, "Timed out");
      result = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
      break;
    case ompl::base::PlannerStatus::APPROXIMATE_SOLUTION:
      ROS_WARN_NAMED(LOGNAME, "Solution is approximate. This usually indicates a failure.");
      result = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      break;
    case ompl::base::PlannerStatus::EXACT_SOLUTION:
      result = moveit_msgs::MoveItErrorCodes::SUCCESS;
      break;
    case ompl::base::PlannerStatus::CRASH:
      ROS_WARN_NAMED(LOGNAME, "OMPL crashed!");
      result = moveit_msgs::MoveItErrorCodes::CRASH;
      break;
    case ompl::base::PlannerStatus::ABORT:
      ROS_WARN_NAMED(LOGNAME, "OMPL was aborted");
      result = moveit_msgs::MoveItErrorCodes::ABORT;
      break;
    default:
      // This should never happen
      ROS_WARN_NAMED(LOGNAME, "Unexpected PlannerStatus code from OMPL.");
      result = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }
  return result;
}

bool ompl_interface::ModelBasedPlanningContext::terminate()
{
  std::unique_lock<std::mutex> slock(ptc_lock_);
  if (ptc_)
    ptc_->terminate();
  return true;
}

bool ompl_interface::ModelBasedPlanningContext::saveConstraintApproximations(const ros::NodeHandle& nh)
{
  std::string constraint_path;
  if (nh.getParam("constraint_approximations_path", constraint_path))
  {
    constraints_library_->saveConstraintApproximations(constraint_path);
    return true;
  }
  ROS_WARN_NAMED(LOGNAME, "ROS param 'constraint_approximations' not found. Unable to save constraint approximations");
  return false;
}

bool ompl_interface::ModelBasedPlanningContext::loadConstraintApproximations(const ros::NodeHandle& nh)
{
  std::string constraint_path;
  if (nh.getParam("constraint_approximations_path", constraint_path))
  {
    constraints_library_->loadConstraintApproximations(constraint_path);
    std::stringstream ss;
    constraints_library_->printConstraintApproximations(ss);
    ROS_INFO_STREAM(ss.str());
    return true;
  }
  return false;
}
