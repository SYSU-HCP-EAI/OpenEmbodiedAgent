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

#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/collision_detection/collision_tools.h>
#include <boost/algorithm/string/join.hpp>

#include <dynamic_reconfigure/server.h>
#include <moveit_ros_planning/SenseForPlanDynamicReconfigureConfig.h>

namespace plan_execution
{
using namespace moveit_ros_planning;

class PlanWithSensing::DynamicReconfigureImpl
{
public:
  DynamicReconfigureImpl(PlanWithSensing* owner)
    : owner_(owner), dynamic_reconfigure_server_(ros::NodeHandle("~/sense_for_plan"))
  {
    dynamic_reconfigure_server_.setCallback(
        [this](const auto& config, uint32_t level) { dynamicReconfigureCallback(config, level); });
  }

private:
  void dynamicReconfigureCallback(const SenseForPlanDynamicReconfigureConfig& config, uint32_t /*level*/)
  {
    owner_->setMaxSafePathCost(config.max_safe_path_cost);
    owner_->setMaxCostSources(config.max_cost_sources);
    owner_->setMaxLookAttempts(config.max_look_attempts);
    owner_->setDiscardOverlappingCostSources(config.discard_overlapping_cost_sources);
  }

  PlanWithSensing* owner_;
  dynamic_reconfigure::Server<SenseForPlanDynamicReconfigureConfig> dynamic_reconfigure_server_;
};
}  // namespace plan_execution

plan_execution::PlanWithSensing::PlanWithSensing(
    const trajectory_execution_manager::TrajectoryExecutionManagerPtr& trajectory_execution)
  : node_handle_("~"), trajectory_execution_manager_(trajectory_execution)
{
  default_max_look_attempts_ = 3;
  default_max_safe_path_cost_ = 0.5;

  discard_overlapping_cost_sources_ = 0.8;
  max_cost_sources_ = 100;

  // by default we do not display path cost sources
  display_cost_sources_ = false;

  // load the sensor manager plugin, if needed
  if (node_handle_.hasParam("moveit_sensor_manager"))
  {
    try
    {
      sensor_manager_loader_ = std::make_unique<pluginlib::ClassLoader<moveit_sensor_manager::MoveItSensorManager>>(
          "moveit_core", "moveit_sensor_manager::MoveItSensorManager");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while creating sensor manager plugin loader: " << ex.what());
    }
    if (sensor_manager_loader_)
    {
      std::string manager;
      if (node_handle_.getParam("moveit_sensor_manager", manager))
        try
        {
          sensor_manager_ = sensor_manager_loader_->createUniqueInstance(manager);
        }
        catch (pluginlib::PluginlibException& ex)
        {
          ROS_ERROR_STREAM("Exception while loading sensor manager '" << manager << "': " << ex.what());
        }
    }
    if (sensor_manager_)
    {
      std::vector<std::string> sensors;
      sensor_manager_->getSensorsList(sensors);
      ROS_INFO_STREAM("PlanWithSensing is aware of the following sensors: " << boost::algorithm::join(sensors, ", "));
    }
  }

  // start the dynamic-reconfigure server
  reconfigure_impl_ = new DynamicReconfigureImpl(this);
}

plan_execution::PlanWithSensing::~PlanWithSensing()
{
  delete reconfigure_impl_;
}

void plan_execution::PlanWithSensing::displayCostSources(bool flag)
{
  if (flag && !display_cost_sources_)
    // publisher for cost sources
    cost_sources_publisher_ =
        node_handle_.advertise<visualization_msgs::MarkerArray>("display_cost_sources", 100, true);
  else if (!flag && display_cost_sources_)
    cost_sources_publisher_.shutdown();
  display_cost_sources_ = flag;
}

bool plan_execution::PlanWithSensing::computePlan(ExecutableMotionPlan& plan,
                                                  const ExecutableMotionPlanComputationFn& motion_planner,
                                                  unsigned int max_look_attempts, double max_safe_path_cost)
{
  if (max_safe_path_cost <= std::numeric_limits<double>::epsilon())
    max_safe_path_cost = default_max_safe_path_cost_;

  if (max_look_attempts == 0)
    max_look_attempts = default_max_look_attempts_;

  double previous_cost = 0.0;
  unsigned int look_attempts = 0;

  // this flag is set to true when all conditions for looking around are met, and the command is sent.
  // the intention is for the planning loop not to terminate when having just looked around
  bool just_looked_around = false;

  // this flag indicates whether the last lookAt() operation failed. If this operation fails once, we assume that
  // maybe some information was gained anyway (the sensor moved part of the way) and we try to plan one more time.
  // If we have two sensor pointing failures in a row, we fail
  bool look_around_failed = false;

  // there can be a maximum number of looking attempts as well that lead to replanning, if the cost
  // of the path is above a maximum threshold.
  do
  {
    bool solved = motion_planner(plan);
    if (!solved || plan.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
      return solved;

    // determine the sources of cost for this path
    std::set<collision_detection::CostSource> cost_sources;
    {
      planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);  // it is ok if
                                                                                           // planning_scene_monitor_ is
                                                                                           // null; there just will be
                                                                                           // no locking done
      for (std::size_t i = 0; i < plan.plan_components_.size(); ++i)
      {
        std::set<collision_detection::CostSource> cost_sources_i;
        plan.planning_scene_->getCostSources(*plan.plan_components_[i].trajectory_, max_cost_sources_,
                                             plan.plan_components_[i].trajectory_->getGroupName(), cost_sources_i,
                                             discard_overlapping_cost_sources_);
        cost_sources.insert(cost_sources_i.begin(), cost_sources_i.end());
        if (cost_sources.size() > max_cost_sources_)
        {
          std::set<collision_detection::CostSource> other;
          other.swap(cost_sources);
          std::size_t j = 0;
          for (std::set<collision_detection::CostSource>::iterator it = other.begin(); j < max_cost_sources_; ++it, ++j)
            cost_sources.insert(*it);
        }
      }
    }

    // display the costs if needed
    if (display_cost_sources_)
    {
      visualization_msgs::MarkerArray arr;
      collision_detection::getCostMarkers(arr, plan.planning_scene_->getPlanningFrame(), cost_sources);
      cost_sources_publisher_.publish(arr);
    }

    double cost = collision_detection::getTotalCost(cost_sources);
    ROS_DEBUG("The total cost of the trajectory is %lf.", cost);
    if (previous_cost > 0.0)
      ROS_DEBUG("The change in the trajectory cost is %lf after the perception step.", cost - previous_cost);
    if (cost > max_safe_path_cost && look_attempts < max_look_attempts)
    {
      ++look_attempts;
      ROS_INFO("The cost of the trajectory is %lf, which is above the maximum safe cost of %lf. Attempt %u (of at most "
               "%u) at looking around.",
               cost, max_safe_path_cost, look_attempts, max_look_attempts);

      bool looked_at_result = lookAt(cost_sources, plan.planning_scene_->getPlanningFrame());
      if (looked_at_result)
        ROS_INFO("Sensor was successfully actuated. Attempting to recompute a motion plan.");
      else
      {
        if (look_around_failed)
          ROS_WARN("Looking around seems to keep failing. Giving up.");
        else
          ROS_WARN("Looking around seems to have failed. Attempting to recompute a motion plan anyway.");
      }
      if (looked_at_result || !look_around_failed)
      {
        previous_cost = cost;
        just_looked_around = true;
      }
      look_around_failed = !looked_at_result;
      // if we are unable to look, let this loop continue into the next if statement
      if (just_looked_around)
        continue;
    }

    if (cost > max_safe_path_cost)
    {
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA;
      return true;
    }
    else
      return true;
  } while (true);

  return false;
}

bool plan_execution::PlanWithSensing::lookAt(const std::set<collision_detection::CostSource>& cost_sources,
                                             const std::string& frame_id)
{
  if (!sensor_manager_)
  {
    ROS_WARN("It seems looking around would be useful, but no MoveIt Sensor Manager is loaded. Did you set "
             "~moveit_sensor_manager ?");
    return false;
  }

  if (before_look_callback_)
    before_look_callback_();

  std::vector<std::string> names;
  sensor_manager_->getSensorsList(names);
  geometry_msgs::PointStamped point;
  for (const std::string& name : names)
    if (collision_detection::getSensorPositioning(point.point, cost_sources))
    {
      point.header.stamp = ros::Time::now();
      point.header.frame_id = frame_id;
      ROS_DEBUG_STREAM("Pointing sensor " << name << " to:\n" << point);
      moveit_msgs::RobotTrajectory sensor_trajectory;
      if (sensor_manager_->pointSensorTo(name, point, sensor_trajectory))
      {
        if (!trajectory_processing::isTrajectoryEmpty(sensor_trajectory))
          return trajectory_execution_manager_->push(sensor_trajectory) &&
                 trajectory_execution_manager_->executeAndWait();
        else
          return true;
      }
    }
  return false;
}
