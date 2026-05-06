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
#include <moveit/moveit_cpp/moveit_cpp.h>

namespace moveit_cpp
{
constexpr char LOGNAME[] = "moveit_cpp";
constexpr char PLANNING_PLUGIN_PARAM[] = "planning_plugin";

MoveItCpp::MoveItCpp(const ros::NodeHandle& nh, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : MoveItCpp(Options(nh), nh, tf_buffer)
{
}

MoveItCpp::MoveItCpp(const Options& options, const ros::NodeHandle& nh,
                     const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : tf_buffer_(tf_buffer), node_handle_(nh)
{
  if (!tf_buffer_)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Configure planning scene monitor
  if (!loadPlanningSceneMonitor(options.planning_scene_monitor_options))
  {
    const std::string error = "Unable to configure planning scene monitor";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  robot_model_ = planning_scene_monitor_->getRobotModel();
  if (!robot_model_)
  {
    const std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                              "parameter server.";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  bool load_planning_pipelines = true;
  if (load_planning_pipelines && !loadPlanningPipelines(options.planning_pipeline_options))
  {
    const std::string error = "Failed to load planning pipelines from parameter server";
    ROS_FATAL_STREAM_NAMED(LOGNAME, error);
    throw std::runtime_error(error);
  }

  // TODO(henningkayser): configure trajectory execution manager
  trajectory_execution_manager_ = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(
      robot_model_, planning_scene_monitor_->getStateMonitor());

  ROS_DEBUG_NAMED(LOGNAME, "MoveItCpp running");
}

MoveItCpp::~MoveItCpp()
{
  ROS_INFO_NAMED(LOGNAME, "Deleting MoveItCpp");
  clearContents();
}

bool MoveItCpp::loadPlanningSceneMonitor(const PlanningSceneMonitorOptions& options)
{
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(options.robot_description,
                                                                                           tf_buffer_, options.name);
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Configuring Planning Scene Monitor");
  if (planning_scene_monitor_->getPlanningScene())
  {
    // Start state and scene monitors
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s' for joint states", options.joint_state_topic.c_str());
    // Subscribe to JointState sensor messages
    planning_scene_monitor_->startStateMonitor(options.joint_state_topic, options.attached_collision_object_topic);
    // Publish planning scene updates to remote monitors like RViz
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          options.monitored_planning_scene_topic);
    // Monitor and apply planning scene updates from remote publishers like the PlanningSceneInterface
    planning_scene_monitor_->startSceneMonitor(options.publish_planning_scene_topic);
    // Monitor requests for changes in the collision environment
    planning_scene_monitor_->startWorldGeometryMonitor();
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning scene not configured");
    return false;
  }

  // Wait for complete state to be recieved
  if (options.wait_for_initial_state_timeout > 0.0)
  {
    return planning_scene_monitor_->getStateMonitor()->waitForCurrentState(ros::Time::now(),
                                                                           options.wait_for_initial_state_timeout);
  }

  return true;
}

bool MoveItCpp::loadPlanningPipelines(const PlanningPipelineOptions& options)
{
  ros::NodeHandle node_handle(options.parent_namespace.empty() ? "~" : options.parent_namespace);
  for (const auto& planning_pipeline_name : options.pipeline_names)
  {
    if (planning_pipelines_.count(planning_pipeline_name) > 0)
    {
      ROS_WARN_NAMED(LOGNAME, "Skipping duplicate entry for planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }
    ROS_INFO_NAMED(LOGNAME, "Loading planning pipeline '%s'", planning_pipeline_name.c_str());
    ros::NodeHandle child_nh(node_handle, planning_pipeline_name);
    planning_pipeline::PlanningPipelinePtr pipeline;
    pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, child_nh, PLANNING_PLUGIN_PARAM);

    if (!pipeline->getPlannerManager())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to initialize planning pipeline '%s'.", planning_pipeline_name.c_str());
      continue;
    }

    planning_pipelines_[planning_pipeline_name] = pipeline;
  }

  if (planning_pipelines_.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to load any planning pipelines.");
    return false;
  }

  return true;
}

moveit::core::RobotModelConstPtr MoveItCpp::getRobotModel() const
{
  return robot_model_;
}

const ros::NodeHandle& MoveItCpp::getNodeHandle() const
{
  return node_handle_;
}

bool MoveItCpp::getCurrentState(moveit::core::RobotStatePtr& current_state, double wait_seconds)
{
  if (wait_seconds > 0.0 &&
      !planning_scene_monitor_->getStateMonitor()->waitForCurrentState(ros::Time::now(), wait_seconds))
  {
    ROS_ERROR_NAMED(LOGNAME, "Did not receive robot state");
    return false;
  }
  {  // Lock planning scene
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    current_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
  }  // Unlock planning scene
  return true;
}

moveit::core::RobotStatePtr MoveItCpp::getCurrentState(double wait)
{
  moveit::core::RobotStatePtr current_state;
  getCurrentState(current_state, wait);
  return current_state;
}

const std::map<std::string, planning_pipeline::PlanningPipelinePtr>& MoveItCpp::getPlanningPipelines() const
{
  return planning_pipelines_;
}

const planning_scene_monitor::PlanningSceneMonitorPtr& MoveItCpp::getPlanningSceneMonitor() const
{
  return planning_scene_monitor_;
}

planning_scene_monitor::PlanningSceneMonitorPtr MoveItCpp::getPlanningSceneMonitorNonConst()
{
  return planning_scene_monitor_;
}

const trajectory_execution_manager::TrajectoryExecutionManagerPtr& MoveItCpp::getTrajectoryExecutionManager() const
{
  return trajectory_execution_manager_;
}

trajectory_execution_manager::TrajectoryExecutionManagerPtr MoveItCpp::getTrajectoryExecutionManagerNonConst()
{
  return trajectory_execution_manager_;
}

bool MoveItCpp::execute(const std::string& group_name, const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                        bool blocking)
{
  if (!robot_trajectory)
  {
    ROS_ERROR_NAMED(LOGNAME, "Robot trajectory is undefined");
    return false;
  }

  // Check if there are controllers that can handle the execution
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(group_name))
  {
    ROS_ERROR_NAMED(LOGNAME, "Execution failed! No active controllers configured for group '%s'", group_name.c_str());
    return false;
  }

  // Execute trajectory
  moveit_msgs::RobotTrajectory robot_trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg);
  // TODO: cambel
  // blocking is the only valid option right now. Add non-bloking use case
  if (blocking)
  {
    trajectory_execution_manager_->push(robot_trajectory_msg);
    trajectory_execution_manager_->execute();
    return trajectory_execution_manager_->waitForExecution();
  }
  return true;
}

bool MoveItCpp::terminatePlanningPipeline(std::string const& pipeline_name)
{
  try
  {
    auto const& planning_pipeline = planning_pipelines_.at(pipeline_name);
    if (planning_pipeline->isActive())
    {
      planning_pipeline->terminate();
    }
    return true;
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot terminate pipeline '%s' because no pipeline with that name exists",
                    pipeline_name.c_str());
    return false;
  }
}

const std::shared_ptr<tf2_ros::Buffer>& MoveItCpp::getTFBuffer() const
{
  return tf_buffer_;
}

void MoveItCpp::clearContents()
{
  tf_listener_.reset();
  tf_buffer_.reset();
  planning_scene_monitor_.reset();
  robot_model_.reset();
  planning_pipelines_.clear();
}
}  // namespace moveit_cpp
