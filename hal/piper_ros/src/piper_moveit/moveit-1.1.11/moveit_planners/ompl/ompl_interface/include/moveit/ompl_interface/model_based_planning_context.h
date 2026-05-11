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

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_interface/planning_interface.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/StateStorage.h>

namespace ompl_interface
{
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

MOVEIT_CLASS_FORWARD(ModelBasedPlanningContext);  // Defines ModelBasedPlanningContextPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(ConstraintsLibrary);         // Defines ConstraintsLibraryPtr, ConstPtr, WeakPtr... etc

struct ModelBasedPlanningContextSpecification;
typedef std::function<ob::PlannerPtr(const ompl::base::SpaceInformationPtr& si, const std::string& name,
                                     const ModelBasedPlanningContextSpecification& spec)>
    ConfiguredPlannerAllocator;
typedef std::function<ConfiguredPlannerAllocator(const std::string& planner_type)> ConfiguredPlannerSelector;

struct ModelBasedPlanningContextSpecification
{
  std::map<std::string, std::string> config_;
  ConfiguredPlannerSelector planner_selector_;
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  ModelBasedStateSpacePtr state_space_;
  og::SimpleSetupPtr ompl_simple_setup_;  // pass in the correct simple setup type
};

class ModelBasedPlanningContext : public planning_interface::PlanningContext
{
public:
  ModelBasedPlanningContext(const std::string& name, const ModelBasedPlanningContextSpecification& spec);

  ~ModelBasedPlanningContext() override
  {
  }

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  void clear() override;
  bool terminate() override;

  const ModelBasedPlanningContextSpecification& getSpecification() const
  {
    return spec_;
  }

  const std::map<std::string, std::string>& getSpecificationConfig() const
  {
    return spec_.config_;
  }

  void setSpecificationConfig(const std::map<std::string, std::string>& config)
  {
    spec_.config_ = config;
  }

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return spec_.state_space_->getRobotModel();
  }

  const moveit::core::JointModelGroup* getJointModelGroup() const
  {
    return spec_.state_space_->getJointModelGroup();
  }

  const moveit::core::RobotState& getCompleteInitialRobotState() const
  {
    return complete_initial_robot_state_;
  }

  const ModelBasedStateSpacePtr& getOMPLStateSpace() const
  {
    return spec_.state_space_;
  }

  const og::SimpleSetupPtr& getOMPLSimpleSetup() const
  {
    return ompl_simple_setup_;
  }

  og::SimpleSetupPtr& getOMPLSimpleSetup()
  {
    return ompl_simple_setup_;
  }

  const ot::Benchmark& getOMPLBenchmark() const
  {
    return ompl_benchmark_;
  }

  ot::Benchmark& getOMPLBenchmark()
  {
    return ompl_benchmark_;
  }

  const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints() const
  {
    return path_constraints_;
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

  /* \brief Get the maximum number of valid goal samples to store */
  unsigned int getMaximumGoalSamples() const
  {
    return max_goal_samples_;
  }

  /* \brief Set the maximum number of valid goal samples to store */
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

  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintSamplerManager()
  {
    return spec_.constraint_sampler_manager_;
  }

  void setConstraintSamplerManager(const constraint_samplers::ConstraintSamplerManagerPtr& csm)
  {
    spec_.constraint_sampler_manager_ = csm;
  }

  void setVerboseStateValidityChecks(bool flag);

  void setProjectionEvaluator(const std::string& peval);

  void setPlanningVolume(const moveit_msgs::WorkspaceParameters& wparams);

  void setCompleteInitialState(const moveit::core::RobotState& complete_initial_robot_state);

  bool setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                          const moveit_msgs::Constraints& path_constraints, moveit_msgs::MoveItErrorCodes* error);
  bool setPathConstraints(const moveit_msgs::Constraints& path_constraints, moveit_msgs::MoveItErrorCodes* error);

  void setConstraintsApproximations(const ConstraintsLibraryPtr& constraints_library)
  {
    constraints_library_ = constraints_library;
  }

  ConstraintsLibraryPtr getConstraintsLibraryNonConst()
  {
    return constraints_library_;
  }

  const ConstraintsLibraryPtr& getConstraintsLibrary() const
  {
    return constraints_library_;
  }

  bool simplifySolutions() const
  {
    return simplify_solutions_;
  }

  void simplifySolutions(bool flag)
  {
    simplify_solutions_ = flag;
  }

  void setInterpolation(bool flag)
  {
    interpolate_ = flag;
  }

  void setHybridize(bool flag)
  {
    hybridize_ = flag;
  }

  /* @brief Solve the planning problem. Return true if the problem is solved
     @param timeout The time to spend on solving
     @param count The number of runs to combine the paths of, in an attempt to generate better quality paths
  */
  const moveit_msgs::MoveItErrorCodes solve(double timeout, unsigned int count);

  /* @brief Benchmark the planning problem. Return true on successful saving of benchmark results
     @param timeout The time to spend on solving
     @param count The number of runs to average in the computation of the benchmark
     @param filename The name of the file to which the benchmark results are to be saved (automatic names can be
     provided if a name is not specified)
  */
  bool benchmark(double timeout, unsigned int count, const std::string& filename = "");

  /* @brief Get the amount of time spent computing the last plan */
  double getLastPlanTime() const
  {
    return last_plan_time_;
  }

  /* @brief Get the amount of time spent simplifying the last plan */
  double getLastSimplifyTime() const
  {
    return last_simplify_time_;
  }

  /* @brief Apply smoothing and try to simplify the plan
     @param timeout The amount of time allowed to be spent on simplifying the plan*/
  void simplifySolution(double timeout);

  /* @brief Interpolate the solution*/
  void interpolateSolution();

  /* @brief Get the solution as a RobotTrajectory object*/
  bool getSolutionPath(robot_trajectory::RobotTrajectory& traj) const;

  void convertPath(const og::PathGeometric& pg, robot_trajectory::RobotTrajectory& traj) const;

  /** @brief Look up param server 'constraint_approximations' and use its value as the path to load constraint
   * approximations to */
  bool loadConstraintApproximations(const ros::NodeHandle& nh);

  /** @brief Look up param server 'constraint_approximations' and use its value as the path to save constraint
   * approximations to */
  bool saveConstraintApproximations(const ros::NodeHandle& nh);

  /** \brief Configure ompl_simple_setup_ and optionally the constraints_library_.
   *
   * ompl_simple_setup_ gets a start state, state sampler, and state validity checker.
   *
   * \param nh ROS node handle used to load the constraint approximations.
   * \param use_constraints_approximations Set to true if we want to load the constraint approximation.
   * */
  virtual void configure(const ros::NodeHandle& nh, bool use_constraints_approximations);

protected:
  void preSolve();
  void postSolve();

  void startSampling();
  void stopSampling();

  virtual ob::ProjectionEvaluatorPtr getProjectionEvaluator(const std::string& peval) const;
  virtual ob::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace* ss) const;
  virtual void useConfig();
  virtual ob::GoalPtr constructGoal();

  /* @brief Construct a planner termination condition, by default a simple time limit
     @param timeout The maximum time (in seconds) that can be used for planning
     @param start The point in time from which planning is considered to have started

     An additional planner termination condition can be specified per planner
     configuration in ompl_planning.yaml via the `termination_condition` parameter.
     Possible values are:

     * `Iteration[num]`: Terminate after `num` iterations. Here, `num` should be replaced
       with a positive integer.
     * `CostConvergence[solutions_window,epsilon]`: Terminate after the cost (as specified
       by an optimization objective) has converged. The parameter `solutions_window`
       specifies the minimum number of solutions to use in deciding whether a planner has
       converged. The parameter `epsilon`	is the threshold to consider for convergence.
       This should be a positive number close to 0. If the cumulative moving average does
       not change by a relative fraction of epsilon after a new better solution is found,
       convergence has been reached.
     * `ExactSolution`: Terminate as soon as an exact solution is found or a timeout
       occurs. This modifies the behavior of anytime/optimizing planners to terminate
       upon discovering the first feasible solution.

     In all cases, the planner will terminate when either the user-specified termination
     condition is satisfied or the time limit specified by `timeout` has been reached,
     whichever occurs first.
  */
  virtual ob::PlannerTerminationCondition constructPlannerTerminationCondition(double timeout,
                                                                               const ompl::time::point& start);

  void registerTerminationCondition(const ob::PlannerTerminationCondition& ptc);
  void unregisterTerminationCondition();

  /** \brief Convert OMPL PlannerStatus to moveit_msgs::msg::MoveItErrorCode */
  int32_t errorCode(const ompl::base::PlannerStatus& status);

  ModelBasedPlanningContextSpecification spec_;

  moveit::core::RobotState complete_initial_robot_state_;

  /// the OMPL planning context; this contains the problem definition and the planner used
  og::SimpleSetupPtr ompl_simple_setup_;

  /// the OMPL tool for benchmarking planners
  ot::Benchmark ompl_benchmark_;

  /// tool used to compute multiple plans in parallel; this uses the problem definition maintained by ompl_simple_setup_
  ot::ParallelPlan ompl_parallel_plan_;

  std::vector<int> space_signature_;

  kinematic_constraints::KinematicConstraintSetPtr path_constraints_;
  moveit_msgs::Constraints path_constraints_msg_;
  std::vector<kinematic_constraints::KinematicConstraintSetPtr> goal_constraints_;

  const ob::PlannerTerminationCondition* ptc_;
  std::mutex ptc_lock_;

  /// the time spent computing the last plan
  double last_plan_time_;

  /// the time spent simplifying the last plan
  double last_simplify_time_;

  /// maximum number of valid states to store in the goal region for any planning request (when such sampling is
  /// possible)
  unsigned int max_goal_samples_;

  /// maximum number of attempts to be made at sampling a state when attempting to find valid states that satisfy some
  /// set of constraints
  unsigned int max_state_sampling_attempts_;

  /// maximum number of attempts to be made at sampling a goal states
  unsigned int max_goal_sampling_attempts_;

  /// when planning in parallel, this is the maximum number of threads to use at one time
  unsigned int max_planning_threads_;

  /// the maximum length that is allowed for segments that make up the motion plan; by default this is 1% from the
  /// extent of the space
  double max_solution_segment_length_;

  /// the minimum number of points to include on the solution path (interpolation is used to reach this number, if
  /// needed)
  unsigned int minimum_waypoint_count_;

  /// when false, clears planners before running solve()
  bool multi_query_planning_enabled_;

  ConstraintsLibraryPtr constraints_library_;

  bool simplify_solutions_;

  // if false the final solution is not interpolated
  bool interpolate_;

  // if false parallel plan returns the first solution found
  bool hybridize_;
};
}  // namespace ompl_interface
