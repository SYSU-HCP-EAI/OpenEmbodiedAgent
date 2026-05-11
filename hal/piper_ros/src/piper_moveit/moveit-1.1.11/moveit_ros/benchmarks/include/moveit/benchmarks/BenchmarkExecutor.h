/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#pragma once

#include <moveit/benchmarks/BenchmarkOptions.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/planning_scene_world_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/trajectory_constraints_storage.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <warehouse_ros/database_loader.h>
#include <pluginlib/class_loader.hpp>

#include <map>
#include <vector>
#include <string>
#include <boost/function.hpp>

namespace moveit_ros_benchmarks
{
/// A class that executes motion plan requests and aggregates data across multiple runs
/// Note: This class operates outside of MoveGroup and does NOT use PlanningRequestAdapters
class BenchmarkExecutor
{
public:
  /// Structure to hold information for a single run of a planner
  typedef std::map<std::string, std::string> PlannerRunData;
  /// Structure to hold information for a single planner's benchmark data.
  typedef std::vector<PlannerRunData> PlannerBenchmarkData;

  /// Definition of a query-start benchmark event function.  Invoked before a new query is benchmarked.
  typedef boost::function<void(const moveit_msgs::MotionPlanRequest& request, planning_scene::PlanningScenePtr)>
      QueryStartEventFunction;

  /// Definition of a query-end benchmark event function.  Invoked after a query has finished benchmarking.
  typedef boost::function<void(const moveit_msgs::MotionPlanRequest& request, planning_scene::PlanningScenePtr)>
      QueryCompletionEventFunction;

  /// Definition of a planner-switch benchmark event function. Invoked before a planner starts any runs for a particular
  /// query.
  typedef boost::function<void(const moveit_msgs::MotionPlanRequest& request, PlannerBenchmarkData& benchmark_data)>
      PlannerStartEventFunction;

  /// Definition of a planner-switch benchmark event function. Invoked after a planner completes all runs for a
  /// particular query.
  typedef boost::function<void(const moveit_msgs::MotionPlanRequest& request, PlannerBenchmarkData& benchmark_data)>
      PlannerCompletionEventFunction;

  /// Definition of a pre-run benchmark event function.  Invoked immediately before each planner calls solve().
  typedef boost::function<void(moveit_msgs::MotionPlanRequest& request)> PreRunEventFunction;

  /// Definition of a post-run benchmark event function.  Invoked immediately after each planner calls solve().
  typedef boost::function<void(const moveit_msgs::MotionPlanRequest& request,
                               const planning_interface::MotionPlanDetailedResponse& response, PlannerRunData& run_data)>
      PostRunEventFunction;

  BenchmarkExecutor(const std::string& robot_description_param = "robot_description");
  virtual ~BenchmarkExecutor();

  // Initialize the benchmark executor by loading planning pipelines from the
  // given set of classes
  void initialize(const std::vector<std::string>& plugin_classes);

  void addPreRunEvent(const PreRunEventFunction& func);
  void addPostRunEvent(const PostRunEventFunction& func);
  void addPlannerStartEvent(const PlannerStartEventFunction& func);
  void addPlannerCompletionEvent(const PlannerCompletionEventFunction& func);
  void addQueryStartEvent(const QueryStartEventFunction& func);
  void addQueryCompletionEvent(const QueryCompletionEventFunction& func);

  virtual void clear();

  virtual bool runBenchmarks(const BenchmarkOptions& opts);

protected:
  struct BenchmarkRequest
  {
    std::string name;
    moveit_msgs::MotionPlanRequest request;
  };

  struct StartState
  {
    moveit_msgs::RobotState state;
    std::string name;
  };

  struct PathConstraints
  {
    std::vector<moveit_msgs::Constraints> constraints;
    std::string name;
  };

  struct TrajectoryConstraints
  {
    moveit_msgs::TrajectoryConstraints constraints;
    std::string name;
  };

  virtual bool initializeBenchmarks(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                                    std::vector<BenchmarkRequest>& queries);

  /// Initialize benchmark query data from start states and constraints
  virtual bool loadBenchmarkQueryData(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                                      std::vector<StartState>& start_states,
                                      std::vector<PathConstraints>& path_constraints,
                                      std::vector<PathConstraints>& goal_constraints,
                                      std::vector<TrajectoryConstraints>& traj_constraints,
                                      std::vector<BenchmarkRequest>& queries);

  virtual void collectMetrics(PlannerRunData& metrics, const planning_interface::MotionPlanDetailedResponse& mp_res,
                              bool solved, double total_time);

  /// Compute the similarity of each (final) trajectory to all other (final) trajectories in the experiment and write
  /// the results to planner_data metrics
  void computeAveragePathSimilarities(PlannerBenchmarkData& planner_data,
                                      const std::vector<planning_interface::MotionPlanDetailedResponse>& responses,
                                      const std::vector<bool>& solved);

  /// Helper function used by computeAveragePathSimilarities() for computing a heuristic distance metric between two
  /// robot trajectories. This function aligns both trajectories in a greedy fashion and computes the mean waypoint
  /// distance averaged over all aligned waypoints. Using a greedy approach is more efficient than dynamic time warping,
  /// and seems to be sufficient for similar trajectories.
  bool computeTrajectoryDistance(const robot_trajectory::RobotTrajectory& traj_first,
                                 const robot_trajectory::RobotTrajectory& traj_second, double& result_distance);

  virtual void writeOutput(const BenchmarkRequest& brequest, const std::string& start_time, double benchmark_duration);

  void shiftConstraintsByOffset(moveit_msgs::Constraints& constraints, const std::vector<double>& offset);

  /// Check that the desired planner plugins and algorithms exist for the given group
  bool plannerConfigurationsExist(const std::map<std::string, std::vector<std::string>>& planners,
                                  const std::string& group_name);

  /// Check that the given requests can be run on the set of planner plugins and algorithms
  bool queriesAndPlannersCompatible(const std::vector<BenchmarkRequest>& requests,
                                    const std::map<std::string, std::vector<std::string>>& planners);

  /// Load the planning scene with the given name from the warehouse
  bool loadPlanningScene(const std::string& scene_name, moveit_msgs::PlanningScene& scene_msg);

  /// Load all states matching the given regular expression from the warehouse
  bool loadStates(const std::string& regex, std::vector<StartState>& start_states);

  /// Load all constraints matching the given regular expression from the warehouse
  bool loadPathConstraints(const std::string& regex, std::vector<PathConstraints>& constraints);

  /// Load all trajectory constraints from the warehouse that match the given regular expression
  bool loadTrajectoryConstraints(const std::string& regex, std::vector<TrajectoryConstraints>& constraints);

  /// Load all motion plan requests matching the given regular expression from the warehouse
  bool loadQueries(const std::string& regex, const std::string& scene_name, std::vector<BenchmarkRequest>& queries);

  /// Duplicate the given benchmark request for all combinations of start states and path constraints
  void createRequestCombinations(const BenchmarkRequest& brequest, const std::vector<StartState>& start_states,
                                 const std::vector<PathConstraints>& path_constraints,
                                 std::vector<BenchmarkRequest>& combos);

  /// Execute the given motion plan request on the set of planners for the set number of runs
  void runBenchmark(moveit_msgs::MotionPlanRequest request,
                    const std::map<std::string, std::vector<std::string>>& planners, int runs);

  planning_scene_monitor::PlanningSceneMonitor* psm_;
  moveit_warehouse::PlanningSceneStorage* pss_;
  moveit_warehouse::PlanningSceneWorldStorage* psws_;
  moveit_warehouse::RobotStateStorage* rs_;
  moveit_warehouse::ConstraintsStorage* cs_;
  moveit_warehouse::TrajectoryConstraintsStorage* tcs_;

  warehouse_ros::DatabaseLoader dbloader;
  planning_scene::PlanningScenePtr planning_scene_;

  BenchmarkOptions options_;

  std::map<std::string, planning_pipeline::PlanningPipelinePtr> planning_pipelines_;

  std::vector<PlannerBenchmarkData> benchmark_data_;

  std::vector<PreRunEventFunction> pre_event_fns_;
  std::vector<PostRunEventFunction> post_event_fns_;
  std::vector<PlannerStartEventFunction> planner_start_fns_;
  std::vector<PlannerCompletionEventFunction> planner_completion_fns_;
  std::vector<QueryStartEventFunction> query_start_fns_;
  std::vector<QueryCompletionEventFunction> query_end_fns_;
};
}  // namespace moveit_ros_benchmarks
