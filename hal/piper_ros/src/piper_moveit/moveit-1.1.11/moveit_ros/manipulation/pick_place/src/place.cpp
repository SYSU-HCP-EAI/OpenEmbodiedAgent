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
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/console.h>

namespace pick_place
{
PlacePlan::PlacePlan(const PickPlaceConstPtr& pick_place) : PickPlacePlanBase(pick_place, "place")
{
}

namespace
{
struct OrderPlaceLocationQuality
{
  OrderPlaceLocationQuality(const std::vector<moveit_msgs::PlaceLocation>& places) : places_(places)
  {
  }

  bool operator()(const std::size_t a, const std::size_t b) const
  {
    return places_[a].quality > places_[b].quality;
  }

  const std::vector<moveit_msgs::PlaceLocation>& places_;
};

bool transformToEndEffectorGoal(const geometry_msgs::PoseStamped& goal_pose,
                                const moveit::core::AttachedBody* attached_body, geometry_msgs::PoseStamped& place_pose)
{
  const EigenSTL::vector_Isometry3d& fixed_transforms = attached_body->getShapePosesInLinkFrame();
  if (fixed_transforms.empty())
    return false;

  Eigen::Isometry3d end_effector_transform;
  tf2::fromMsg(goal_pose.pose, end_effector_transform);
  end_effector_transform = end_effector_transform * fixed_transforms[0].inverse();
  place_pose.header = goal_pose.header;
  place_pose.pose = tf2::toMsg(end_effector_transform);
  return true;
}
}  // namespace

bool PlacePlan::plan(const planning_scene::PlanningSceneConstPtr& planning_scene, const moveit_msgs::PlaceGoal& goal)
{
  double timeout = goal.allowed_planning_time;
  ros::WallTime endtime = ros::WallTime::now() + ros::WallDuration(timeout);
  std::string attached_object_name = goal.attached_object_name;
  const moveit::core::JointModelGroup* jmg = nullptr;
  const moveit::core::JointModelGroup* eef = nullptr;

  // if the group specified is actually an end-effector, we use it as such
  if (planning_scene->getRobotModel()->hasEndEffector(goal.group_name))
  {
    eef = planning_scene->getRobotModel()->getEndEffector(goal.group_name);
    if (eef)
    {  // if we correctly found the eef, then we try to find out what the planning group is
      const std::string& eef_parent = eef->getEndEffectorParentGroup().first;
      if (eef_parent.empty())
      {
        ROS_ERROR_STREAM_NAMED("manipulation", "No parent group to plan in was identified based on end-effector '"
                                                   << goal.group_name
                                                   << "'. Please define a parent group in the SRDF.");
        error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return false;
      }
      else
        jmg = planning_scene->getRobotModel()->getJointModelGroup(eef_parent);
    }
  }
  else
  {
    // if a group name was specified, try to use it
    jmg = goal.group_name.empty() ? nullptr : planning_scene->getRobotModel()->getJointModelGroup(goal.group_name);
    if (jmg)
    {
      // we also try to find the corresponding eef
      const std::vector<std::string>& eef_names = jmg->getAttachedEndEffectorNames();
      if (eef_names.empty())
      {
        ROS_ERROR_STREAM_NAMED("manipulation",
                               "There are no end-effectors specified for group '" << goal.group_name << "'");
        error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return false;
      }
      else
        // check to see if there is an end effector that has attached objects associaded, so we can complete the place
        for (const std::string& eef_name : eef_names)
        {
          std::vector<const moveit::core::AttachedBody*> attached_bodies;
          const moveit::core::JointModelGroup* eg = planning_scene->getRobotModel()->getEndEffector(eef_name);
          if (eg)
          {
            // see if there are objects attached to links in the eef
            planning_scene->getCurrentState().getAttachedBodies(attached_bodies, eg);

            // is is often possible that the objects are attached to the same link that the eef itself is attached,
            // so we check for attached bodies there as well
            const moveit::core::LinkModel* attached_link_model =
                planning_scene->getRobotModel()->getLinkModel(eg->getEndEffectorParentGroup().second);
            if (attached_link_model)
            {
              std::vector<const moveit::core::AttachedBody*> attached_bodies2;
              planning_scene->getCurrentState().getAttachedBodies(attached_bodies2, attached_link_model);
              attached_bodies.insert(attached_bodies.end(), attached_bodies2.begin(), attached_bodies2.end());
            }
          }

          // if this end effector has attached objects, we go on
          if (!attached_bodies.empty())
          {
            // if the user specified the name of the attached object to place, we check that indeed
            // the group contains this attachd body
            if (!attached_object_name.empty())
            {
              bool found = false;
              for (const moveit::core::AttachedBody* attached_body : attached_bodies)
                if (attached_body->getName() == attached_object_name)
                {
                  found = true;
                  break;
                }
              // if the attached body this group has is not the same as the one specified,
              // we cannot use this eef
              if (!found)
                continue;
            }

            // if we previoulsy have set the eef it means we have more options we could use, so things are ambiguous
            if (eef)
            {
              ROS_ERROR_STREAM_NAMED("manipulation", "There are multiple end-effectors for group '"
                                                         << goal.group_name
                                                         << "' that are currently holding objects. It is ambiguous "
                                                            "which end-effector to use. Please specify it explicitly.");
              error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
              return false;
            }
            // set the end effector (this was initialized to NULL above)
            eef = planning_scene->getRobotModel()->getEndEffector(eef_name);
          }
        }
    }
  }

  // if we know the attached object, but not the eef, we can try to identify that
  if (!attached_object_name.empty() && !eef)
  {
    const moveit::core::AttachedBody* attached_body =
        planning_scene->getCurrentState().getAttachedBody(attached_object_name);
    if (attached_body)
    {
      // get the robot model link this attached body is associated to
      const moveit::core::LinkModel* link = attached_body->getAttachedLink();
      // check to see if there is a unique end effector containing the link
      const std::vector<const moveit::core::JointModelGroup*>& eefs =
          planning_scene->getRobotModel()->getEndEffectors();
      for (const moveit::core::JointModelGroup* end_effector : eefs)
        if (end_effector->hasLinkModel(link->getName()))
        {
          if (eef)
          {
            ROS_ERROR_STREAM_NAMED("manipulation", "There are multiple end-effectors that include the link '"
                                                       << link->getName() << "' which is where the body '"
                                                       << attached_object_name
                                                       << "' is attached. It is unclear which end-effector to use.");
            error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return false;
          }
          eef = end_effector;
        }
    }
    // if the group is also unknown, but we just found out the eef
    if (!jmg && eef)
    {
      const std::string& eef_parent = eef->getEndEffectorParentGroup().first;
      if (eef_parent.empty())
      {
        ROS_ERROR_STREAM_NAMED("manipulation", "No parent group to plan in was identified based on end-effector '"
                                                   << goal.group_name
                                                   << "'. Please define a parent group in the SRDF.");
        error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return false;
      }
      else
        jmg = planning_scene->getRobotModel()->getJointModelGroup(eef_parent);
    }
  }

  if (!jmg || !eef)
  {
    error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return false;
  }

  // try to infer attached body name if possible
  int loop_count = 0;
  while (attached_object_name.empty() && loop_count < 2)
  {
    // in the first try, look for objects attached to the eef, if the eef is known;
    // otherwise, look for attached bodies in the planning group itself
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    planning_scene->getCurrentState().getAttachedBodies(attached_bodies, loop_count == 0 ? eef : jmg);

    loop_count++;
    if (attached_bodies.size() > 1)
    {
      ROS_ERROR_NAMED("manipulation",
                      "Multiple attached bodies for group '%s' but no explicit attached object to place was specified",
                      goal.group_name.c_str());
      error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
      return false;
    }
    else
      attached_object_name = attached_bodies[0]->getName();
  }

  const moveit::core::AttachedBody* attached_body =
      planning_scene->getCurrentState().getAttachedBody(attached_object_name);
  if (!attached_body)
  {
    ROS_ERROR_NAMED("manipulation", "There is no object to detach for place action");
    error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return false;
  }

  ros::WallTime start_time = ros::WallTime::now();

  // construct common data for possible manipulation plans
  ManipulationPlanSharedDataPtr plan_data(new ManipulationPlanSharedData());
  ManipulationPlanSharedDataConstPtr const_plan_data = plan_data;
  plan_data->planning_group_ = jmg;
  plan_data->end_effector_group_ = eef;
  plan_data->ik_link_ = planning_scene->getRobotModel()->getLinkModel(eef->getEndEffectorParentGroup().second);

  plan_data->timeout_ = endtime;
  plan_data->path_constraints_ = goal.path_constraints;
  plan_data->planner_id_ = goal.planner_id;
  plan_data->minimize_object_distance_ = false;
  plan_data->max_goal_sampling_attempts_ = 2;
  moveit_msgs::AttachedCollisionObject& detach_object_msg = plan_data->diff_attached_object_;

  // construct the attached object message that will change the world to what it would become after a placement
  detach_object_msg.link_name = attached_body->getAttachedLinkName();
  detach_object_msg.object.id = attached_object_name;
  detach_object_msg.object.operation = moveit_msgs::CollisionObject::REMOVE;

  collision_detection::AllowedCollisionMatrixPtr approach_place_acm(
      new collision_detection::AllowedCollisionMatrix(planning_scene->getAllowedCollisionMatrix()));

  // we are allowed to touch certain other objects with the gripper
  approach_place_acm->setEntry(eef->getLinkModelNames(), goal.allowed_touch_objects, true);

  // we are allowed to touch the target object slightly while retreating the end effector
  std::vector<std::string> touch_links(attached_body->getTouchLinks().begin(), attached_body->getTouchLinks().end());
  approach_place_acm->setEntry(attached_object_name, touch_links, true);

  if (!goal.support_surface_name.empty())
  {
    // we are allowed to have contact between the target object and the support surface before the place
    approach_place_acm->setEntry(goal.support_surface_name, attached_object_name, true);

    // optionally, it may be allowed to touch the support surface with the gripper
    if (goal.allow_gripper_support_collision)
      approach_place_acm->setEntry(goal.support_surface_name, eef->getLinkModelNames(), true);
  }

  // configure the manipulation pipeline
  pipeline_.reset();

  ManipulationStagePtr stage1(
      new ReachableAndValidPoseFilter(planning_scene, approach_place_acm, pick_place_->getConstraintsSamplerManager()));
  ManipulationStagePtr stage2(new ApproachAndTranslateStage(planning_scene, approach_place_acm));
  ManipulationStagePtr stage3(new PlanStage(planning_scene, pick_place_->getPlanningPipeline()));
  pipeline_.addStage(stage1).addStage(stage2).addStage(stage3);

  initialize();

  pipeline_.start();

  // order the place locations by quality
  std::vector<std::size_t> place_locations_order(goal.place_locations.size());
  for (std::size_t i = 0; i < goal.place_locations.size(); ++i)
    place_locations_order[i] = i;
  OrderPlaceLocationQuality oq(goal.place_locations);
  // using stable_sort to preserve order of place locations with equal quality
  std::stable_sort(place_locations_order.begin(), place_locations_order.end(), oq);

  // add possible place locations
  for (std::size_t i = 0; i < goal.place_locations.size(); ++i)
  {
    ManipulationPlanPtr p(new ManipulationPlan(const_plan_data));
    const moveit_msgs::PlaceLocation& pl = goal.place_locations[place_locations_order[i]];

    if (goal.place_eef)
      p->goal_pose_ = pl.place_pose;
    else
        // The goals are specified for the attached body
        // but we want to transform them into goals for the end-effector instead
        if (!transformToEndEffectorGoal(pl.place_pose, attached_body, p->goal_pose_))
    {
      p->goal_pose_ = pl.place_pose;
      ROS_ERROR_NAMED("manipulation", "Unable to transform the desired pose of the object to the pose of the "
                                      "end-effector");
    }

    p->approach_ = pl.pre_place_approach;
    p->retreat_ = pl.post_place_retreat;
    p->retreat_posture_ = pl.post_place_posture;
    p->id_ = place_locations_order[i];
    if (p->retreat_posture_.joint_names.empty())
      p->retreat_posture_ = attached_body->getDetachPosture();
    pipeline_.push(p);
  }
  ROS_INFO_NAMED("manipulation", "Added %d place locations", (int)goal.place_locations.size());

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
      if (!goal.place_locations.empty())
      {
        ROS_WARN_NAMED("manipulation", "All supplied place locations failed. Retrying last location in verbose mode.");
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
  ROS_INFO_NAMED("manipulation", "Place planning completed after %lf seconds", last_plan_time_);

  return error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

PlacePlanPtr PickPlace::planPlace(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const moveit_msgs::PlaceGoal& goal) const
{
  PlacePlanPtr p(new PlacePlan(shared_from_this()));
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
