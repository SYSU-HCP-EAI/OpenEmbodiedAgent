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

#include <moveit/pick_place/pick_place.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

namespace pick_place
{
const std::string PickPlace::DISPLAY_PATH_TOPIC = planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC;
const std::string PickPlace::DISPLAY_GRASP_TOPIC = "display_grasp_markers";
const double PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION = 7.0;  // seconds

// functionality specific to pick-only is in pick.cpp;
// functionality specific to place-only is in place.cpp;

PickPlacePlanBase::PickPlacePlanBase(const PickPlaceConstPtr& pick_place, const std::string& name)
  : pick_place_(pick_place), pipeline_(name, 4), last_plan_time_(0.0), done_(false)
{
  pipeline_.setSolutionCallback([this] { foundSolution(); });
  pipeline_.setEmptyQueueCallback([this] { emptyQueue(); });
}

PickPlacePlanBase::~PickPlacePlanBase() = default;

void PickPlacePlanBase::foundSolution()
{
  boost::mutex::scoped_lock slock(done_mutex_);
  done_ = true;
  done_condition_.notify_all();
}

void PickPlacePlanBase::emptyQueue()
{
  boost::mutex::scoped_lock slock(done_mutex_);
  if (pushed_all_poses_)
  {
    done_ = true;
    done_condition_.notify_all();
  }
}

void PickPlacePlanBase::initialize()
{
  done_ = false;
  pushed_all_poses_ = false;
}

void PickPlacePlanBase::waitForPipeline(const ros::WallTime& endtime)
{
  // wait till we're done
  boost::unique_lock<boost::mutex> lock(done_mutex_);
  pushed_all_poses_ = true;
  while (!done_ && endtime > ros::WallTime::now())
    done_condition_.timed_wait(lock, (endtime - ros::WallTime::now()).toBoost());
}

PickPlace::PickPlace(const planning_pipeline::PlanningPipelinePtr& planning_pipeline)
  : nh_("~"), planning_pipeline_(planning_pipeline), display_computed_motion_plans_(false), display_grasps_(false)
{
  constraint_sampler_manager_loader_ =
      std::make_shared<constraint_sampler_manager_loader::ConstraintSamplerManagerLoader>();
}

void PickPlace::displayProcessedGrasps(bool flag)
{
  if (display_grasps_ && !flag)
    grasps_publisher_.shutdown();
  else if (!display_grasps_ && flag)
    grasps_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(DISPLAY_GRASP_TOPIC, 10, true);
  display_grasps_ = flag;
}

void PickPlace::displayComputedMotionPlans(bool flag)
{
  if (display_computed_motion_plans_ && !flag)
    display_path_publisher_.shutdown();
  else if (!display_computed_motion_plans_ && flag)
    display_path_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PATH_TOPIC, 10, true);
  display_computed_motion_plans_ = flag;
}

void PickPlace::visualizePlan(const ManipulationPlanPtr& plan) const
{
  moveit_msgs::DisplayTrajectory dtraj;
  dtraj.model_id = getRobotModel()->getName();
  bool first = true;
  for (const plan_execution::ExecutableTrajectory& traj : plan->trajectories_)
  {
    if (!traj.trajectory_ || traj.trajectory_->empty())
      continue;
    if (first)
    {
      moveit::core::robotStateToRobotStateMsg(traj.trajectory_->getFirstWayPoint(), dtraj.trajectory_start);
      first = false;
    }
    dtraj.trajectory.resize(dtraj.trajectory.size() + 1);
    traj.trajectory_->getRobotTrajectoryMsg(dtraj.trajectory.back());
  }
  display_path_publisher_.publish(dtraj);
}

void PickPlace::visualizeGrasp(const ManipulationPlanPtr& plan) const
{
  visualizeGrasps(std::vector<ManipulationPlanPtr>(1, plan));
}

namespace
{
std::vector<std_msgs::ColorRGBA> setupDefaultGraspColors()
{
  std::vector<std_msgs::ColorRGBA> result;
  result.resize(6);
  result[0].r = 0.5f;
  result[0].g = 0.5f;
  result[0].b = 0.5f;
  result[0].a = 1.0f;
  result[1].r = 1.0f;
  result[1].g = 0.0f;
  result[1].b = 0.0f;
  result[1].a = 1.0f;
  result[2].r = 1.0f;
  result[2].g = 0.5f;
  result[2].b = 0.0f;
  result[2].a = 1.0f;
  result[3].r = 0.0f;
  result[3].g = 1.0f;
  result[3].b = 1.0f;
  result[3].a = 1.0f;
  result[4].r = 0.0f;
  result[4].g = 1.0f;
  result[4].b = 0.0f;
  result[4].a = 1.0f;
  result[5].r = 1.0f;
  result[5].g = 0.0f;
  result[5].b = 1.0f;
  result[5].a = 0.75f;
  return result;
}
}  // namespace

void PickPlace::visualizeGrasps(const std::vector<ManipulationPlanPtr>& plans) const
{
  if (plans.empty())
    return;

  moveit::core::RobotState state(getRobotModel());
  state.setToDefaultValues();

  static std::vector<std_msgs::ColorRGBA> colors(setupDefaultGraspColors());
  visualization_msgs::MarkerArray ma;
  for (const ManipulationPlanPtr& plan : plans)
  {
    const moveit::core::JointModelGroup* jmg = plan->shared_data_->end_effector_group_;
    if (jmg)
    {
      unsigned int type = std::min(plan->processing_stage_, colors.size() - 1);
      state.updateStateWithLinkAt(plan->shared_data_->ik_link_, plan->transformed_goal_pose_);
      state.getRobotMarkers(ma, jmg->getLinkModelNames(), colors[type],
                            "moveit_grasps:stage_" + boost::lexical_cast<std::string>(plan->processing_stage_),
                            ros::Duration(60));
    }
  }

  grasps_publisher_.publish(ma);
}
}  // namespace pick_place
