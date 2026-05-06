/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Raghavender Sahdev.
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
 *   * Neither the name of Raghavender Sahdev nor the names of its
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

/* Author: Raghavender Sahdev */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>

#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>

#include <moveit/robot_state/conversions.h>

#include <vector>
#include <eigen3/Eigen/Core>

namespace chomp
{
class OptimizerAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  OptimizerAdapter() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& nh) override
  {
    if (!nh.getParam("planning_time_limit", params_.planning_time_limit_))
    {
      params_.planning_time_limit_ = 10.0;
      ROS_INFO_STREAM("Param planning_time_limit was not set. Using default value: " << params_.planning_time_limit_);
    }
    if (!nh.getParam("max_iterations", params_.max_iterations_))
    {
      params_.max_iterations_ = 200;
      ROS_INFO_STREAM("Param max_iterations was not set. Using default value: " << params_.max_iterations_);
    }
    if (!nh.getParam("max_iterations_after_collision_free", params_.max_iterations_after_collision_free_))
    {
      params_.max_iterations_after_collision_free_ = 5;
      ROS_INFO_STREAM("Param max_iterations_after_collision_free was not set. Using default value: "
                      << params_.max_iterations_after_collision_free_);
    }
    if (!nh.getParam("smoothness_cost_weight", params_.smoothness_cost_weight_))
    {
      params_.smoothness_cost_weight_ = 0.1;
      ROS_INFO_STREAM(
          "Param smoothness_cost_weight was not set. Using default value: " << params_.smoothness_cost_weight_);
    }
    if (!nh.getParam("obstacle_cost_weight", params_.obstacle_cost_weight_))
    {
      params_.obstacle_cost_weight_ = 1.0;
      ROS_INFO_STREAM("Param obstacle_cost_weight was not set. Using default value: " << params_.obstacle_cost_weight_);
    }
    if (!nh.getParam("learning_rate", params_.learning_rate_))
    {
      params_.learning_rate_ = 0.01;
      ROS_INFO_STREAM("Param learning_rate was not set. Using default value: " << params_.learning_rate_);
    }
    if (!nh.getParam("smoothness_cost_velocity", params_.smoothness_cost_velocity_))
    {
      params_.smoothness_cost_velocity_ = 0.0;
      ROS_INFO_STREAM(
          "Param smoothness_cost_velocity was not set. Using default value: " << params_.smoothness_cost_velocity_);
    }
    if (!nh.getParam("smoothness_cost_acceleration", params_.smoothness_cost_acceleration_))
    {
      params_.smoothness_cost_acceleration_ = 1.0;
      ROS_INFO_STREAM("Param smoothness_cost_acceleration was not set. Using default value: "
                      << params_.smoothness_cost_acceleration_);
    }
    if (!nh.getParam("smoothness_cost_jerk", params_.smoothness_cost_jerk_))
    {
      params_.smoothness_cost_jerk_ = 0.0;
      ROS_INFO_STREAM("Param smoothness_cost_jerk_ was not set. Using default value: " << params_.smoothness_cost_jerk_);
    }
    if (!nh.getParam("ridge_factor", params_.ridge_factor_))
    {
      params_.ridge_factor_ = 0.0;
      ROS_INFO_STREAM("Param ridge_factor_ was not set. Using default value: " << params_.ridge_factor_);
    }
    if (!nh.getParam("use_pseudo_inverse", params_.use_pseudo_inverse_))
    {
      params_.use_pseudo_inverse_ = 0.0;
      ROS_INFO_STREAM("Param use_pseudo_inverse_ was not set. Using default value: " << params_.use_pseudo_inverse_);
    }
    if (!nh.getParam("pseudo_inverse_ridge_factor", params_.pseudo_inverse_ridge_factor_))
    {
      params_.pseudo_inverse_ridge_factor_ = 1e-4;
      ROS_INFO_STREAM("Param pseudo_inverse_ridge_factor was not set. Using default value: "
                      << params_.pseudo_inverse_ridge_factor_);
    }
    if (!nh.getParam("joint_update_limit", params_.joint_update_limit_))
    {
      params_.joint_update_limit_ = 0.1;
      ROS_INFO_STREAM("Param joint_update_limit was not set. Using default value: " << params_.joint_update_limit_);
    }
    // TODO: remove this warning after 06/2022
    if (!nh.hasParam("min_clearance") && nh.hasParam("min_clearence"))
      ROS_WARN("The param 'min_clearence' has been renamed to 'min_clearance', please update your config!");
    if (!nh.getParam("min_clearance", params_.min_clearance_))
    {
      params_.min_clearance_ = 0.2;
      ROS_INFO_STREAM("Param min_clearance was not set. Using default value: " << params_.min_clearance_);
    }
    if (!nh.getParam("collision_threshold", params_.collision_threshold_))
    {
      params_.collision_threshold_ = 0.07;
      ROS_INFO_STREAM("Param collision_threshold_ was not set. Using default value: " << params_.collision_threshold_);
    }
    if (!nh.getParam("use_stochastic_descent", params_.use_stochastic_descent_))
    {
      params_.use_stochastic_descent_ = true;
      ROS_INFO_STREAM(
          "Param use_stochastic_descent was not set. Using default value: " << params_.use_stochastic_descent_);
    }
    // default
    params_.trajectory_initialization_method_ = std::string("fillTrajectory");
    std::string trajectory_initialization_method;
    if (!nh.getParam("trajectory_initialization_method", trajectory_initialization_method))
    {
      ROS_INFO_STREAM("Param trajectory_initialization_method was not set. Using value: "
                      << params_.trajectory_initialization_method_);
    }
    else if (!params_.setTrajectoryInitializationMethod(trajectory_initialization_method))
    {
      ROS_ERROR_STREAM("Param trajectory_initialization_method set to invalid value '"
                       << trajectory_initialization_method << "'. Using '" << params_.trajectory_initialization_method_
                       << "' instead.");
    }
  }

  std::string getDescription() const override
  {
    return "CHOMP Optimizer";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    // following call to planner() calls the OMPL planner and stores the trajectory inside the MotionPlanResponse res
    // variable which is then used by CHOMP for optimization of the computed trajectory
    if (!planner(ps, req, res))
      return false;

    // create a hybrid collision detector to set the collision checker as hybrid
    collision_detection::CollisionDetectorAllocatorPtr hybrid_cd(
        collision_detection::CollisionDetectorAllocatorHybrid::create());

    // create a writable planning scene
    planning_scene::PlanningScenePtr planning_scene = ps->diff();
    ROS_DEBUG_STREAM("Configuring Planning Scene for CHOMP ...");
    planning_scene->setActiveCollisionDetector(hybrid_cd, true);

    chomp::ChompPlanner chomp_planner;
    planning_interface::MotionPlanDetailedResponse res_detailed;
    res_detailed.trajectory_.push_back(res.trajectory_);

    bool planning_success = chomp_planner.solve(planning_scene, req, params_, res_detailed);

    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ += res_detailed.processing_time_[0];
    }
    res.error_code_ = res_detailed.error_code_;

    return planning_success;
  }

private:
  chomp::ChompParameters params_;
};
}  // namespace chomp

CLASS_LOADER_REGISTER_CLASS(chomp::OptimizerAdapter, planning_request_adapter::PlanningRequestAdapter);
