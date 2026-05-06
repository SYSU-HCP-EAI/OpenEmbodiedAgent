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

#include <moveit/macros/class_forward.h>
#include <moveit/pick_place/manipulation_pipeline.h>
#include <moveit/pick_place/pick_place_params.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <boost/noncopyable.hpp>
#include <memory>

namespace pick_place
{
MOVEIT_CLASS_FORWARD(PickPlace);  // Defines PickPlacePtr, ConstPtr, WeakPtr... etc

class PickPlacePlanBase
{
public:
  PickPlacePlanBase(const PickPlaceConstPtr& pick_place, const std::string& name);
  ~PickPlacePlanBase();

  const std::vector<ManipulationPlanPtr>& getSuccessfulManipulationPlans() const
  {
    return pipeline_.getSuccessfulManipulationPlans();
  }
  const std::vector<ManipulationPlanPtr>& getFailedManipulationPlans() const
  {
    return pipeline_.getFailedManipulationPlans();
  }

  const moveit_msgs::MoveItErrorCodes& getErrorCode() const
  {
    return error_code_;
  }

  double getLastPlanTime() const
  {
    return last_plan_time_;
  }

protected:
  void initialize();
  void waitForPipeline(const ros::WallTime& endtime);
  void foundSolution();
  void emptyQueue();

  PickPlaceConstPtr pick_place_;
  ManipulationPipeline pipeline_;

  double last_plan_time_;
  bool done_;
  bool pushed_all_poses_;
  boost::condition_variable done_condition_;
  boost::mutex done_mutex_;
  moveit_msgs::MoveItErrorCodes error_code_;
};

MOVEIT_CLASS_FORWARD(PickPlan);  // Defines PickPlanPtr, ConstPtr, WeakPtr... etc

class PickPlan : public PickPlacePlanBase
{
public:
  PickPlan(const PickPlaceConstPtr& pick_place);
  bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::PickupGoal& goal);
};

MOVEIT_CLASS_FORWARD(PlacePlan);  // Defines PlacePlanPtr, ConstPtr, WeakPtr... etc

class PlacePlan : public PickPlacePlanBase
{
public:
  PlacePlan(const PickPlaceConstPtr& pick_place);
  bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::PlaceGoal& goal);
};

class PickPlace : private boost::noncopyable, public std::enable_shared_from_this<PickPlace>
{
public:
  static const std::string DISPLAY_PATH_TOPIC;
  static const std::string DISPLAY_GRASP_TOPIC;

  // the amount of time (maximum) to wait for achieving a grasp posture
  static const double DEFAULT_GRASP_POSTURE_COMPLETION_DURATION;  // seconds

  PickPlace(const planning_pipeline::PlanningPipelinePtr& planning_pipeline);

  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintsSamplerManager() const
  {
    return constraint_sampler_manager_loader_->getConstraintSamplerManager();
  }

  const planning_pipeline::PlanningPipelinePtr& getPlanningPipeline() const
  {
    return planning_pipeline_;
  }

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return planning_pipeline_->getRobotModel();
  }

  /** \brief Plan the sequence of motions that perform a pickup action */
  PickPlanPtr planPick(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const moveit_msgs::PickupGoal& goal) const;

  /** \brief Plan the sequence of motions that perform a placement action */
  PlacePlanPtr planPlace(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const moveit_msgs::PlaceGoal& goal) const;

  void displayComputedMotionPlans(bool flag);
  void displayProcessedGrasps(bool flag);

  void visualizePlan(const ManipulationPlanPtr& plan) const;

  void visualizeGrasp(const ManipulationPlanPtr& plan) const;

  void visualizeGrasps(const std::vector<ManipulationPlanPtr>& plans) const;

private:
  ros::NodeHandle nh_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  bool display_computed_motion_plans_;
  bool display_grasps_;
  ros::Publisher display_path_publisher_;
  ros::Publisher grasps_publisher_;

  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
};
}  // namespace pick_place
