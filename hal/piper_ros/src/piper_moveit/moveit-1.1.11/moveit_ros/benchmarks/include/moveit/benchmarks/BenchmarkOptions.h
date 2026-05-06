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

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <moveit_msgs/WorkspaceParameters.h>

namespace moveit_ros_benchmarks
{
class BenchmarkOptions
{
public:
  /** \brief Constructor */
  BenchmarkOptions();
  /** \brief Constructor accepting a custom namespace for parameter lookup */
  BenchmarkOptions(const std::string& ros_namespace);
  /** \brief Destructor */
  virtual ~BenchmarkOptions();

  /** \brief Set the ROS namespace the node handle should use for parameter lookup */
  void setNamespace(const std::string& ros_namespace);

  /** \brief Get the name of the warehouse database host server */
  const std::string& getHostName() const;
  /** \brief Get the port of the warehouse database host server */
  int getPort() const;
  /** \brief Get the reference name of the planning scene stored inside the warehouse database */
  const std::string& getSceneName() const;

  /** \brief Get the specified number of benchmark query runs */
  int getNumRuns() const;
  /** \brief Get the maximum timeout per planning attempt */
  double getTimeout() const;
  /** \brief Get the reference name of the benchmark */
  const std::string& getBenchmarkName() const;
  /** \brief Get the name of the planning group to run the benchmark with */
  const std::string& getGroupName() const;
  /** \brief Get the target directory for the generated benchmark result data */
  const std::string& getOutputDirectory() const;
  /** \brief Get the regex expression for matching the names of all queries to run */
  const std::string& getQueryRegex() const;
  /** \brief Get the regex expression for matching the names of all start states to plan from */
  const std::string& getStartStateRegex() const;
  /** \brief Get the regex expression for matching the names of all goal constraints to plan to */
  const std::string& getGoalConstraintRegex() const;
  /** \brief Get the regex expression for matching the names of all path constraints to plan with */
  const std::string& getPathConstraintRegex() const;
  /** \brief Get the regex expression for matching the names of all trajectory constraints to plan with */
  const std::string& getTrajectoryConstraintRegex() const;
  /** \brief Get the names of all predefined poses to consider for planning */
  const std::vector<std::string>& getPredefinedPoses() const;
  /** \brief Get the name of the planning group for which the predefined poses are defined */
  const std::string& getPredefinedPosesGroup() const;
  /** \brief Get the constant position/orientation offset to be used for shifting all goal constraints */
  void getGoalOffsets(std::vector<double>& offsets) const;
  /** \brief Get all planning pipeline names mapped to their parameter configuration */
  const std::map<std::string, std::vector<std::string>>& getPlanningPipelineConfigurations() const;
  /** \brief Get all planning pipeline names */
  void getPlanningPipelineNames(std::vector<std::string>& planning_pipeline_names) const;

  /* \brief Get the frame id of the planning workspace */
  const std::string& getWorkspaceFrameID() const;
  /* \brief Get the parameter set of the planning workspace */
  const moveit_msgs::WorkspaceParameters& getWorkspaceParameters() const;

protected:
  void readBenchmarkOptions(const std::string& ros_namespace);

  void readWarehouseOptions(ros::NodeHandle& nh);
  void readBenchmarkParameters(ros::NodeHandle& nh);
  void readPlannerConfigs(ros::NodeHandle& nh);

  void readWorkspaceParameters(ros::NodeHandle& nh);
  void readGoalOffset(ros::NodeHandle& nh);

  /// warehouse parameters
  std::string hostname_;
  int port_;
  std::string scene_name_;

  /// benchmark parameters
  int runs_;
  double timeout_;
  std::string benchmark_name_;
  std::string group_name_;
  std::string output_directory_;
  std::string query_regex_;
  std::string start_state_regex_;
  std::string goal_constraint_regex_;
  std::string path_constraint_regex_;
  std::string trajectory_constraint_regex_;
  std::vector<std::string> predefined_poses_;
  std::string predefined_poses_group_;
  double goal_offsets[6];

  /// planner configurations
  std::map<std::string, std::vector<std::string>> planning_pipelines_;

  moveit_msgs::WorkspaceParameters workspace_;
};
}  // namespace moveit_ros_benchmarks
