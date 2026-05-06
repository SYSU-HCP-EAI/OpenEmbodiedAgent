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
 *   * Neither the name of the PickNik LLC nor the names of its
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
/* Description: A simple benchmark that plans trajectories for all combinations of specified predefined poses */

// MoveIt Benchmark
#include <moveit/benchmarks/BenchmarkOptions.h>
#include <moveit/benchmarks/BenchmarkExecutor.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

namespace moveit_ros_benchmarks
{
constexpr char LOGNAME[] = "combine_predefined_poses_benchmark";
class CombinePredefinedPosesBenchmark : public BenchmarkExecutor
{
public:
  bool loadBenchmarkQueryData(const BenchmarkOptions& opts, moveit_msgs::PlanningScene& scene_msg,
                              std::vector<StartState>& start_states, std::vector<PathConstraints>& path_constraints,
                              std::vector<PathConstraints>& goal_constraints,
                              std::vector<TrajectoryConstraints>& traj_constraints,
                              std::vector<BenchmarkRequest>& queries) override
  {
    // Load planning scene
    if (!psm_)
      psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (!psm_->newPlanningSceneMessage(scene_msg))
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to load planning scene");
      return false;
    }

    // Load robot model
    if (!psm_->getRobotModel())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to load robot model");
      return false;
    }

    // Select planning group to use for predefined poses
    std::string predefined_poses_group = opts.getPredefinedPosesGroup();
    if (predefined_poses_group.empty())
    {
      ROS_WARN_NAMED(LOGNAME, "Parameter predefined_poses_group is not set, using default planning group instead");
      predefined_poses_group = opts.getGroupName();
    }
    const auto& joint_model_group = psm_->getRobotModel()->getJointModelGroup(predefined_poses_group);
    if (!joint_model_group)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Robot model has no joint model group named '" << predefined_poses_group << "'");
      return false;
    }

    // Iterate over all predefined poses and use each as start and goal states
    moveit::core::RobotState robot_state(psm_->getRobotModel());
    start_states.clear();
    goal_constraints.clear();
    for (const auto& pose_id : opts.getPredefinedPoses())
    {
      if (!robot_state.setToDefaultValues(joint_model_group, pose_id))
      {
        ROS_WARN_STREAM_NAMED(LOGNAME, "Failed to set robot state to named target '" << pose_id << "'");
        continue;
      }
      // Create start state
      start_states.emplace_back();
      start_states.back().name = pose_id;
      moveit::core::robotStateToRobotStateMsg(robot_state, start_states.back().state);

      // Create goal constraints
      goal_constraints.emplace_back();
      goal_constraints.back().name = pose_id;
      goal_constraints.back().constraints.push_back(
          kinematic_constraints::constructGoalConstraints(robot_state, joint_model_group));
    }
    if (start_states.empty() || goal_constraints.empty())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to init start and goal states from predefined_poses");
      return false;
    }

    // We don't use path/trajectory constraints or custom queries
    path_constraints.clear();
    traj_constraints.clear();
    queries.clear();
    return true;
  }

private:
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};
}  // namespace moveit_ros_benchmarks

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_run_benchmark");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Read benchmark options from param server
  moveit_ros_benchmarks::BenchmarkOptions opts(ros::this_node::getName());
  // Setup benchmark server
  moveit_ros_benchmarks::CombinePredefinedPosesBenchmark server;

  std::vector<std::string> planning_pipelines;
  opts.getPlanningPipelineNames(planning_pipelines);
  server.initialize(planning_pipelines);

  // Running benchmarks
  if (!server.runBenchmarks(opts))
    ROS_ERROR("Failed to run all benchmarks");
}
