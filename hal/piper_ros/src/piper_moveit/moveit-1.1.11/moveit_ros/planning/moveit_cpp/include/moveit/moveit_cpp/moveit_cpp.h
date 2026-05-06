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
   Desc: A high level interface that does not require the use of ROS Actions, Services, and Messages to access the
   core MoveIt functionality
*/

#pragma once

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace moveit_cpp
{
MOVEIT_CLASS_FORWARD(MoveItCpp);  // Defines MoveItCppPtr, ConstPtr, WeakPtr... etc

class MoveItCpp
{
public:
  /// Specification of options to use when constructing the MoveItCpp class
  struct PlanningSceneMonitorOptions
  {
    void load(const ros::NodeHandle& nh)
    {
      std::string ns = "planning_scene_monitor_options/";
      nh.param<std::string>(ns + "name", name, "planning_scene_monitor");
      nh.param<std::string>(ns + "robot_description", robot_description, "robot_description");
      nh.param(ns + "joint_state_topic", joint_state_topic,
               planning_scene_monitor::PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC);
      nh.param(ns + "attached_collision_object_topic", attached_collision_object_topic,
               planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC);
      nh.param(ns + "monitored_planning_scene_topic", monitored_planning_scene_topic,
               planning_scene_monitor::PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC);
      nh.param(ns + "publish_planning_scene_topic", publish_planning_scene_topic,
               planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC);
      nh.param<double>(ns + "wait_for_initial_state_timeout", wait_for_initial_state_timeout, 0.0);
    }
    std::string name;
    std::string robot_description;
    std::string joint_state_topic;
    std::string attached_collision_object_topic;
    std::string monitored_planning_scene_topic;
    std::string publish_planning_scene_topic;
    double wait_for_initial_state_timeout;
  };

  /// struct contains the the variables used for loading the planning pipeline
  struct PlanningPipelineOptions
  {
    void load(const ros::NodeHandle& nh)
    {
      std::string ns = "planning_pipelines/";
      nh.getParam(ns + "pipeline_names", pipeline_names);
      nh.getParam(ns + "namespace", parent_namespace);
    }
    std::vector<std::string> pipeline_names;
    std::string parent_namespace;
  };

  /// Parameter container for initializing MoveItCpp
  struct Options
  {
    Options(const ros::NodeHandle& nh)
    {
      planning_scene_monitor_options.load(nh);
      planning_pipeline_options.load(nh);
    }

    PlanningSceneMonitorOptions planning_scene_monitor_options;
    PlanningPipelineOptions planning_pipeline_options;
  };

  /** \brief Constructor */
  MoveItCpp(const ros::NodeHandle& nh, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = {});
  MoveItCpp(const Options& options, const ros::NodeHandle& nh, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = {});

  /**
   * @brief This class owns unique resources (e.g. action clients, threads) and its not very
   * meaningful to copy. Pass by references, move it, or simply create multiple instances where
   * required.
   */
  MoveItCpp(const MoveItCpp&) = delete;
  MoveItCpp& operator=(const MoveItCpp&) = delete;

  MoveItCpp(MoveItCpp&& other) = default;
  MoveItCpp& operator=(MoveItCpp&& other) = default;

  /** \brief Destructor */
  ~MoveItCpp();

  /** \brief Get the RobotModel object. */
  moveit::core::RobotModelConstPtr getRobotModel() const;

  /** \brief Get the ROS node handle of this instance operates on */
  const ros::NodeHandle& getNodeHandle() const;

  /** \brief Get the current state queried from the current state monitor
      \param wait_seconds the time in seconds for the state monitor to wait for a robot state. */
  bool getCurrentState(moveit::core::RobotStatePtr& current_state, double wait_seconds);

  /** \brief Get the current state queried from the current state monitor
      \param wait_seconds the time in seconds for the state monitor to wait for a robot state. */
  moveit::core::RobotStatePtr getCurrentState(double wait_seconds = 0.0);

  /** \brief Get all loaded planning pipeline instances mapped to their reference names */
  const std::map<std::string, planning_pipeline::PlanningPipelinePtr>& getPlanningPipelines() const;

  /** \brief Get the stored instance of the planning scene monitor */
  const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor() const;
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitorNonConst();

  const std::shared_ptr<tf2_ros::Buffer>& getTFBuffer() const;

  /** \brief Get the stored instance of the trajectory execution manager */
  const trajectory_execution_manager::TrajectoryExecutionManagerPtr& getTrajectoryExecutionManager() const;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr getTrajectoryExecutionManagerNonConst();

  /** \brief Execute a trajectory on the planning group specified by group_name using the trajectory execution manager.
   * If blocking is set to false, the execution is run in background and the function returns immediately. */
  bool execute(const std::string& group_name, const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
               bool blocking = true);

  /** \brief Utility to terminate a given planning pipeline */
  bool terminatePlanningPipeline(std::string const& pipeline_name);

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private:
  //  Core properties and instances
  ros::NodeHandle node_handle_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Planning
  std::map<std::string, planning_pipeline::PlanningPipelinePtr> planning_pipelines_;
  std::map<std::string, std::set<std::string>> groups_algorithms_map_;

  // Execution
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;

  /** \brief Reset all member variables */
  void clearContents();

  /** \brief Initialize and setup the planning scene monitor */
  bool loadPlanningSceneMonitor(const PlanningSceneMonitorOptions& options);

  /** \brief Initialize and setup the planning pipelines */
  bool loadPlanningPipelines(const PlanningPipelineOptions& options);
};
}  // namespace moveit_cpp

namespace moveit
{
namespace planning_interface
{
using MoveItCpp [[deprecated("use moveit_cpp")]] = moveit_cpp::MoveItCpp;
[[deprecated("use moveit_cpp")]] MOVEIT_DECLARE_PTR(MoveItCpp, moveit_cpp::MoveItCpp);
}  // namespace planning_interface
}  // namespace moveit
