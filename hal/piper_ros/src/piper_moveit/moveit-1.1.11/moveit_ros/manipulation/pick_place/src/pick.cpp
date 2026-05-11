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
#include <moveit/pick_place/reachable_valid_pose_filter.h>
#include <moveit/pick_place/approach_and_translate_stage.h>
#include <moveit/pick_place/plan_stage.h>
#include <moveit/utils/message_checks.h>
#include <ros/console.h>

namespace pick_place
{
PickPlan::PickPlan(const PickPlaceConstPtr& pick_place) : PickPlacePlanBase(pick_place, "pick")
{
}

namespace
{
struct OrderGraspQuality
{
  OrderGraspQuality(const std::vector<moveit_msgs::Grasp>& grasps) : grasps_(grasps)
  {
  }

  bool operator()(const std::size_t a, const std::size_t b) const
  {
    return grasps_[a].grasp_quality > grasps_[b].grasp_quality;
  }

  const std::vector<moveit_msgs::Grasp>& grasps_;
};
}  // namespace

bool PickPlan::plan(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::PickupGoal& goal)
{
  double timeout = goal.allowed_planning_time;
  ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);

  std::string planning_group = goal.group_name;
  std::string end_effector = goal.end_effector;
  if (end_effector.empty() && !planning_group.empty())
  {
    const moveit::core::JointModelGroup* jmg = planning_scene->getRobotModel()->getJointModelGroup(planning_group);
    if (!jmg)
    {
      error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return false;
    }
    const std::vector<std::string>& eefs = jmg->getAttachedEndEffectorNames();
    if (!eefs.empty())
    {
      end_effector = eefs.front();
      if (eefs.size() > 1)
        ROS_WARN_STREAM_NAMED("manipulation", "Choice of end-effector for group '"
                                                  << planning_group << "' is ambiguous. Assuming '" << end_effector
                                                  << "'");
    }
  }
  else if (!end_effector.empty() && planning_group.empty())
  {
    const moveit::core::JointModelGroup* jmg = planning_scene->getRobotModel()->getEndEffector(end_effector);
    if (!jmg)
    {
      error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return false;
    }
    planning_group = jmg->getEndEffectorParentGroup().first;
    if (planning_group.empty())
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "No parent group to plan in was identified based on end-effector '"
                                                 << end_effector << "'. Please define a parent group in the SRDF.");
      error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return false;
    }
    else
      ROS_INFO_STREAM_NAMED("manipulation", "Assuming the planning group for end effector '" << end_effector << "' is '"
                                                                                             << planning_group << "'");
  }
  const moveit::core::JointModelGroup* eef =
      end_effector.empty() ? nullptr : planning_scene->getRobotModel()->getEndEffector(end_effector);
  if (!eef)
  {
    ROS_ERROR_NAMED("manipulation", "No end-effector specified for pick action");
    error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }
  const std::string& ik_link = eef->getEndEffectorParentGroup().second;

  ros::WallTime start_time = ros::WallTime::now();

  // construct common data for possible manipulation plans
  ManipulationPlanSharedDataPtr plan_data(new ManipulationPlanSharedData());
  ManipulationPlanSharedDataConstPtr const_plan_data = plan_data;
  plan_data->planning_group_ = planning_scene->getRobotModel()->getJointModelGroup(planning_group);
  plan_data->end_effector_group_ = eef;
  plan_data->ik_link_ = planning_scene->getRobotModel()->getLinkModel(ik_link);
  plan_data->timeout_ = endtime;
  plan_data->path_constraints_ = goal.path_constraints;
  plan_data->planner_id_ = goal.planner_id;
  plan_data->minimize_object_distance_ = goal.minimize_object_distance;
  plan_data->max_goal_sampling_attempts_ = 2;
  moveit_msgs::AttachedCollisionObject& attach_object_msg = plan_data->diff_attached_object_;

  // construct the attached object message that will change the world to what it would become after a pick
  attach_object_msg.link_name = ik_link;
  attach_object_msg.object.id = goal.target_name;
  attach_object_msg.object.operation = moveit_msgs::CollisionObject::ADD;
  attach_object_msg.touch_links =
      goal.attached_object_touch_links.empty() ? eef->getLinkModelNames() : goal.attached_object_touch_links;
  collision_detection::AllowedCollisionMatrixPtr approach_grasp_acm(
      new collision_detection::AllowedCollisionMatrix(planning_scene->getAllowedCollisionMatrix()));

  // we are allowed to touch the target object
  approach_grasp_acm->setEntry(goal.target_name, attach_object_msg.touch_links, true);
  // we are allowed to touch certain other objects with the gripper
  approach_grasp_acm->setEntry(eef->getLinkModelNames(), goal.allowed_touch_objects, true);
  if (!goal.support_surface_name.empty())
  {
    // we are allowed to have contact between the target object and the support surface before the grasp
    approach_grasp_acm->setEntry(goal.support_surface_name, goal.target_name, true);

    // optionally, it may be allowed to touch the support surface with the gripper
    if (goal.allow_gripper_support_collision)
      approach_grasp_acm->setEntry(goal.support_surface_name, eef->getLinkModelNames(), true);
  }

  // configure the manipulation pipeline
  pipeline_.reset();
  ManipulationStagePtr stage1(
      new ReachableAndValidPoseFilter(planning_scene, approach_grasp_acm, pick_place_->getConstraintsSamplerManager()));
  ManipulationStagePtr stage2(new ApproachAndTranslateStage(planning_scene, approach_grasp_acm));
  ManipulationStagePtr stage3(new PlanStage(planning_scene, pick_place_->getPlanningPipeline()));
  pipeline_.addStage(stage1).addStage(stage2).addStage(stage3);

  initialize();
  pipeline_.start();

  // order the grasps by quality
  std::vector<std::size_t> grasp_order(goal.possible_grasps.size());
  for (std::size_t i = 0; i < goal.possible_grasps.size(); ++i)
    grasp_order[i] = i;
  OrderGraspQuality oq(goal.possible_grasps);
  // using stable_sort to preserve order of grasps with equal quality
  std::stable_sort(grasp_order.begin(), grasp_order.end(), oq);

  // feed the available grasps to the stages we set up
  for (std::size_t i = 0; i < goal.possible_grasps.size(); ++i)
  {
    ManipulationPlanPtr p(new ManipulationPlan(const_plan_data));
    const moveit_msgs::Grasp& g = goal.possible_grasps[grasp_order[i]];
    p->approach_ = g.pre_grasp_approach;
    p->retreat_ = g.post_grasp_retreat;
    p->goal_pose_ = g.grasp_pose;
    p->id_ = grasp_order[i];
    // if no frame of reference was specified, assume the transform to be in the reference frame of the object
    if (p->goal_pose_.header.frame_id.empty())
      p->goal_pose_.header.frame_id = goal.target_name;
    p->approach_posture_ = g.pre_grasp_posture;
    p->retreat_posture_ = g.grasp_posture;
    pipeline_.push(p);
  }

  // wait till we're done
  waitForPipeline(endtime);
  pipeline_.stop();

  last_plan_time_ = (ros::WallTime::now() - start_time).toSec();

  if (!getSuccessfulManipulationPlans().empty())
    error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
  {
    if (last_plan_time_ > timeout)
      error_code_.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    else
    {
      error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      if (!goal.possible_grasps.empty())
      {
        ROS_WARN_NAMED("manipulation", "All supplied grasps failed. Retrying last grasp in verbose mode.");
        // everything failed. we now start the pipeline again in verbose mode for one grasp
        initialize();
        pipeline_.setVerbose(true);
        pipeline_.start();
        pipeline_.reprocessLastFailure();
        waitForPipeline(ros::WallTime::now() + ros::WallDuration(1.0));
        pipeline_.stop();
        pipeline_.setVerbose(false);
      }
    }
  }
  ROS_INFO_NAMED("manipulation", "Pickup planning completed after %lf seconds", last_plan_time_);

  return error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

PickPlanPtr PickPlace::planPick(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                const moveit_msgs::PickupGoal& goal) const
{
  PickPlanPtr p(new PickPlan(shared_from_this()));

  if (moveit::core::isEmpty(goal.planning_options.planning_scene_diff))
    p->plan(planning_scene, goal);
  else
    p->plan(planning_scene->diff(goal.planning_options.planning_scene_diff), goal);

  if (display_computed_motion_plans_)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = p->getSuccessfulManipulationPlans();
    if (!success.empty())
      visualizePlan(success.back());
  }

  if (display_grasps_)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = p->getSuccessfulManipulationPlans();
    visualizeGrasps(success);
    const std::vector<pick_place::ManipulationPlanPtr>& failed = p->getFailedManipulationPlans();
    visualizeGrasps(failed);
  }

  return p;
}
}  // namespace pick_place
