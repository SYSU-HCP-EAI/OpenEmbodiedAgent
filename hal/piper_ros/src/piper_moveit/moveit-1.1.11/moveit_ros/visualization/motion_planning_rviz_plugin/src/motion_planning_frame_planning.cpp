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

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <std_srvs/Empty.h>
#include <moveit_msgs/RobotState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
void MotionPlanningFrame::planButtonClicked()
{
  publishSceneIfNeeded();
  planning_display_->addBackgroundJob([this] { computePlanButtonClicked(); }, "compute plan");
}

void MotionPlanningFrame::executeButtonClicked()
{
  ui_->execute_button->setEnabled(false);
  // execution is done in a separate thread, to not block other background jobs by blocking for synchronous execution
  planning_display_->spawnBackgroundJob([this] { computeExecuteButtonClicked(); });
}

void MotionPlanningFrame::planAndExecuteButtonClicked()
{
  publishSceneIfNeeded();
  ui_->plan_and_execute_button->setEnabled(false);
  ui_->execute_button->setEnabled(false);
  // execution is done in a separate thread, to not block other background jobs by blocking for synchronous execution
  planning_display_->spawnBackgroundJob([this] { computePlanAndExecuteButtonClicked(); });
}

void MotionPlanningFrame::stopButtonClicked()
{
  ui_->stop_button->setEnabled(false);  // avoid clicking again
  planning_display_->addBackgroundJob([this] { computeStopButtonClicked(); }, "stop");
}

void MotionPlanningFrame::allowReplanningToggled(bool checked)
{
  if (move_group_)
    move_group_->allowReplanning(checked);
}

void MotionPlanningFrame::allowLookingToggled(bool checked)
{
  if (move_group_)
    move_group_->allowLooking(checked);
}

void MotionPlanningFrame::pathConstraintsIndexChanged(int index)
{
  if (move_group_)
  {
    if (index > 0)
    {
      std::string c = ui_->path_constraints_combo_box->itemText(index).toStdString();
      if (!move_group_->setPathConstraints(c))
        ROS_WARN_STREAM("Unable to set the path constraints: " << c);
    }
    else
      move_group_->clearPathConstraints();
  }
}

void MotionPlanningFrame::onClearOctomapClicked()
{
  std_srvs::Empty srv;
  clear_octomap_service_client_.call(srv);
  ui_->clear_octomap_button->setEnabled(false);
}

bool MotionPlanningFrame::computeCartesianPlan()
{
  ros::WallTime start = ros::WallTime::now();
  // get goal pose
  moveit::core::RobotState goal = *planning_display_->getQueryGoalState();
  std::vector<geometry_msgs::Pose> waypoints;
  const std::string& link_name = move_group_->getEndEffectorLink();
  const moveit::core::LinkModel* link = move_group_->getRobotModel()->getLinkModel(link_name);
  if (!link)
  {
    ROS_ERROR_STREAM("Failed to determine unique end-effector link: " << link_name);
    return false;
  }
  waypoints.push_back(tf2::toMsg(goal.getGlobalLinkTransform(link)));

  // setup default params
  double cart_step_size = 0.01;
  double cart_jump_thresh = 0.0;
  bool avoid_collisions = true;

  // compute trajectory
  moveit_msgs::RobotTrajectory trajectory;
  double fraction =
      move_group_->computeCartesianPath(waypoints, cart_step_size, cart_jump_thresh, trajectory, avoid_collisions);

  if (fraction >= 1.0)
  {
    ROS_INFO("Achieved %f %% of Cartesian path", fraction * 100.);

    // Compute time parameterization to also provide velocities
    // https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success =
        iptp.computeTimeStamps(rt, ui_->velocity_scaling_factor->value(), ui_->acceleration_scaling_factor->value());
    ROS_INFO("Computing time stamps %s", success ? "SUCCEDED" : "FAILED");

    // Store trajectory in current_plan_
    current_plan_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
    rt.getRobotTrajectoryMsg(current_plan_->trajectory_);
    current_plan_->planning_time_ = (ros::WallTime::now() - start).toSec();
    return success;
  }
  return false;
}

bool MotionPlanningFrame::computeJointSpacePlan()
{
  current_plan_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
  return move_group_->plan(*current_plan_) == moveit::core::MoveItErrorCode::SUCCESS;
}

void MotionPlanningFrame::computePlanButtonClicked()
{
  if (!move_group_)
    return;

  // Clear status
  ui_->result_label->setText("Planning...");

  configureForPlanning();
  planning_display_->rememberPreviousStartState();
  bool success = (ui_->use_cartesian_path->isEnabled() && ui_->use_cartesian_path->checkState()) ?
                     computeCartesianPlan() :
                     computeJointSpacePlan();

  if (success)
  {
    moveit::core::RobotState start_state(planning_display_->getRobotModel());
    moveit::core::RobotState current_state(planning_display_->getPlanningSceneRO()->getCurrentState());
    moveit::core::robotStateMsgToRobotState(current_plan_->start_state_, start_state, true);
    if (moveit::core::haveSameAttachedObjects(start_state, current_state))
      ui_->execute_button->setEnabled(true);
    ui_->result_label->setText(QString("Time: ").append(QString::number(current_plan_->planning_time_, 'f', 3)));
  }
  else
  {
    current_plan_.reset();
    ui_->result_label->setText("Failed");
  }
  Q_EMIT planningFinished();
}

void MotionPlanningFrame::computeExecuteButtonClicked()
{
  // ensures the MoveGroupInterface is not destroyed while executing
  moveit::planning_interface::MoveGroupInterfacePtr mgi(move_group_);
  if (mgi && current_plan_)
  {
    ui_->stop_button->setEnabled(true);  // enable stopping
    bool success = mgi->execute(*current_plan_) == moveit::core::MoveItErrorCode::SUCCESS;
    onFinishedExecution(success);
  }
}

void MotionPlanningFrame::computePlanAndExecuteButtonClicked()
{
  // ensures the MoveGroupInterface is not destroyed while executing
  moveit::planning_interface::MoveGroupInterfacePtr mgi(move_group_);
  if (!mgi)
    return;
  configureForPlanning();
  planning_display_->rememberPreviousStartState();
  // move_group::move() on the server side, will always start from the current state
  // to suppress a warning, we pass an empty state (which encodes "start from current state")
  mgi->setStartStateToCurrentState();
  ui_->stop_button->setEnabled(true);
  if (ui_->use_cartesian_path->isEnabled() && ui_->use_cartesian_path->checkState())
  {
    if (computeCartesianPlan())
      computeExecuteButtonClicked();
  }
  else
  {
    bool success = mgi->move() == moveit::core::MoveItErrorCode::SUCCESS;
    onFinishedExecution(success);
  }
  ui_->plan_and_execute_button->setEnabled(true);
}

void MotionPlanningFrame::computeStopButtonClicked()
{
  if (move_group_)
    move_group_->stop();
}

void MotionPlanningFrame::onFinishedExecution(bool success)
{
  // visualize result of execution
  if (success)
    ui_->result_label->setText("Executed");
  else
    ui_->result_label->setText(!ui_->stop_button->isEnabled() ? "Stopped" : "Failed");
  // disable stop button
  ui_->stop_button->setEnabled(false);

  // update query start state to current if neccessary
  if (ui_->start_state_combo_box->currentText() == "<current>")
    startStateTextChanged(ui_->start_state_combo_box->currentText());

  // auto-update goal to stored previous state (but only on success)
  // on failure, the user must update the goal to the previous state himself
  if (ui_->goal_state_combo_box->currentText() == "<previous>")
    goalStateTextChanged(ui_->goal_state_combo_box->currentText());
}

void MotionPlanningFrame::onNewPlanningSceneState()
{
  moveit::core::RobotState current(planning_display_->getPlanningSceneRO()->getCurrentState());
  if (ui_->start_state_combo_box->currentText() == "<current>")
  {
    planning_display_->setQueryStartState(current);
    planning_display_->rememberPreviousStartState();
  }
  if (ui_->goal_state_combo_box->currentText() == "<current>")
    planning_display_->setQueryGoalState(current);
}

void MotionPlanningFrame::startStateTextChanged(const QString& start_state)
{
  // use background job: fetching the current state might take up to a second
  planning_display_->addBackgroundJob([this, state = start_state.toStdString()] { startStateTextChangedExec(state); },
                                      "update start state");
}

void MotionPlanningFrame::startStateTextChangedExec(const std::string& start_state)
{
  moveit::core::RobotState start = *planning_display_->getQueryStartState();
  updateQueryStateHelper(start, start_state);
  planning_display_->setQueryStartState(start);
}

void MotionPlanningFrame::goalStateTextChanged(const QString& goal_state)
{
  // use background job: fetching the current state might take up to a second
  planning_display_->addBackgroundJob([this, state = goal_state.toStdString()] { goalStateTextChangedExec(state); },
                                      "update goal state");
}

void MotionPlanningFrame::goalStateTextChangedExec(const std::string& goal_state)
{
  moveit::core::RobotState goal = *planning_display_->getQueryGoalState();
  updateQueryStateHelper(goal, goal_state);
  planning_display_->setQueryGoalState(goal);
}

void MotionPlanningFrame::planningGroupTextChanged(const QString& planning_group)
{
  planning_display_->changePlanningGroup(planning_group.toStdString());
}

void MotionPlanningFrame::updateQueryStateHelper(moveit::core::RobotState& state, const std::string& v)
{
  if (v == "<random>")
  {
    configureWorkspace();
    if (const moveit::core::JointModelGroup* jmg =
            state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
      state.setToRandomPositions(jmg);
    return;
  }

  if (v == "<random valid>")
  {
    configureWorkspace();

    if (const moveit::core::JointModelGroup* jmg =
            state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
    {
      // Loop until a collision free state is found
      static const int MAX_ATTEMPTS = 100;
      int attempt_count = 0;  // prevent loop for going forever
      while (attempt_count < MAX_ATTEMPTS)
      {
        // Generate random state
        state.setToRandomPositions(jmg);

        state.update();  // prevent dirty transforms

        // Test for collision
        if (planning_display_->getPlanningSceneRO()->isStateValid(state, "", false))
          break;

        attempt_count++;
      }
      // Explain if no valid rand state found
      if (attempt_count >= MAX_ATTEMPTS)
        ROS_WARN("Unable to find a random collision free configuration after %d attempts", MAX_ATTEMPTS);
    }
    else
    {
      ROS_WARN_STREAM("Unable to get joint model group " << planning_display_->getCurrentPlanningGroup());
    }
    return;
  }

  if (v == "<current>")
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
      state = ps->getCurrentState();
    return;
  }

  if (v == "<same as goal>")
  {
    state = *planning_display_->getQueryGoalState();
    return;
  }

  if (v == "<same as start>")
  {
    state = *planning_display_->getQueryStartState();
    return;
  }

  if (v == "<previous>")
  {
    state = planning_display_->getPreviousState();
    return;
  }

  // maybe it is a named state
  if (const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(planning_display_->getCurrentPlanningGroup()))
    state.setToDefaultValues(jmg, v);
}

void MotionPlanningFrame::populatePlannersList(const std::vector<moveit_msgs::PlannerInterfaceDescription>& desc)
{
  ui_->planning_pipeline_combo_box->clear();

  planner_descriptions_ = desc;
  size_t default_planner_index = 0;
  for (auto& d : planner_descriptions_)
  {
    QString item_text(d.pipeline_id.c_str());
    // Check for default planning pipeline
    if (d.pipeline_id == default_planning_pipeline_)
    {
      if (item_text.isEmpty())
        item_text = QString::fromStdString(d.name);
      default_planner_index = ui_->planning_pipeline_combo_box->count();
    }
    ui_->planning_pipeline_combo_box->addItem(item_text);
  }
  QFont font;
  font.setBold(true);
  ui_->planning_pipeline_combo_box->setItemData(default_planner_index, font, Qt::FontRole);

  // Select default pipeline - triggers populatePlannerDescription() via callback
  if (!planner_descriptions_.empty())
    ui_->planning_pipeline_combo_box->setCurrentIndex(default_planner_index);
}

void MotionPlanningFrame::populatePlannerDescription(const moveit_msgs::PlannerInterfaceDescription& desc)
{
  std::string group = planning_display_->getCurrentPlanningGroup();
  ui_->planning_algorithm_combo_box->clear();

  // set the label for the planning library
  ui_->library_label->setText(QString::fromStdString(desc.name));
  ui_->library_label->setStyleSheet("QLabel { color : green; font: bold }");

  bool found_group = false;
  // the name of a planner is either "GROUP[planner_id]" or "planner_id"
  if (!group.empty())
  {
    for (const std::string& planner_id : desc.planner_ids)
      if (planner_id == group)
        found_group = true;
      else if (planner_id.substr(0, group.length()) == group)
      {
        if (planner_id.size() > group.length() && planner_id[group.length()] == '[')
        {
          std::string id = planner_id.substr(group.length());
          if (id.size() > 2)
          {
            id.resize(id.length() - 1);
            ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(id.substr(1)));
          }
        }
      }
  }
  if (ui_->planning_algorithm_combo_box->count() == 0 && !found_group)
    for (const std::string& planner_id : desc.planner_ids)
      ui_->planning_algorithm_combo_box->addItem(QString::fromStdString(planner_id));

  ui_->planning_algorithm_combo_box->insertItem(0, "<unspecified>");

  // retrieve default planner config from parameter server
  const std::string& default_planner_config = move_group_->getDefaultPlannerId(found_group ? group : std::string());
  int default_index = ui_->planning_algorithm_combo_box->findText(QString::fromStdString(default_planner_config));
  if (default_index < 0)
    default_index = 0;  // 0 is <unspecified> fallback
  ui_->planning_algorithm_combo_box->setCurrentIndex(default_index);

  QFont font;
  font.setBold(true);
  ui_->planning_algorithm_combo_box->setItemData(default_index, font, Qt::FontRole);
}

void MotionPlanningFrame::populateConstraintsList()
{
  if (move_group_)
    planning_display_->addMainLoopJob([this]() { populateConstraintsList(move_group_->getKnownConstraints()); });
}

void MotionPlanningFrame::populateConstraintsList(const std::vector<std::string>& constr)
{
  ui_->path_constraints_combo_box->clear();
  ui_->path_constraints_combo_box->addItem("None");
  for (const std::string& constraint : constr)
    ui_->path_constraints_combo_box->addItem(QString::fromStdString(constraint));
}

void MotionPlanningFrame::constructPlanningRequest(moveit_msgs::MotionPlanRequest& mreq)
{
  mreq.group_name = planning_display_->getCurrentPlanningGroup();
  mreq.num_planning_attempts = ui_->planning_attempts->value();
  mreq.allowed_planning_time = ui_->planning_time->value();
  mreq.max_velocity_scaling_factor = ui_->velocity_scaling_factor->value();
  mreq.max_acceleration_scaling_factor = ui_->acceleration_scaling_factor->value();
  moveit::core::robotStateToRobotStateMsg(*planning_display_->getQueryStartState(), mreq.start_state);
  mreq.workspace_parameters.min_corner.x = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.min_corner.y = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.min_corner.z = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  mreq.workspace_parameters.max_corner.x = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  mreq.workspace_parameters.max_corner.y = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  mreq.workspace_parameters.max_corner.z = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;
  moveit::core::RobotStateConstPtr s = planning_display_->getQueryGoalState();
  const moveit::core::JointModelGroup* jmg = s->getJointModelGroup(mreq.group_name);
  if (jmg)
  {
    mreq.goal_constraints.resize(1);
    mreq.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(*s, jmg);
  }
}

void MotionPlanningFrame::configureWorkspace()
{
  moveit::core::VariableBounds bx, by, bz;
  bx.position_bounded_ = by.position_bounded_ = bz.position_bounded_ = true;

  moveit::core::JointModel::Bounds b(3);
  bx.min_position_ = ui_->wcenter_x->value() - ui_->wsize_x->value() / 2.0;
  bx.max_position_ = ui_->wcenter_x->value() + ui_->wsize_x->value() / 2.0;
  by.min_position_ = ui_->wcenter_y->value() - ui_->wsize_y->value() / 2.0;
  by.max_position_ = ui_->wcenter_y->value() + ui_->wsize_y->value() / 2.0;
  bz.min_position_ = ui_->wcenter_z->value() - ui_->wsize_z->value() / 2.0;
  bz.max_position_ = ui_->wcenter_z->value() + ui_->wsize_z->value() / 2.0;

  if (move_group_)
    move_group_->setWorkspace(bx.min_position_, by.min_position_, bz.min_position_, bx.max_position_, by.max_position_,
                              bz.max_position_);
  planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_display_->getPlanningSceneMonitor();
  // get non-const access to the robot_model and update planar & floating joints as indicated by the workspace settings
  if (psm && psm->getRobotModelLoader() && psm->getRobotModelLoader()->getModel())
  {
    const moveit::core::RobotModelPtr& robot_model = psm->getRobotModelLoader()->getModel();
    const std::vector<moveit::core::JointModel*>& jm = robot_model->getJointModels();
    for (moveit::core::JointModel* joint : jm)
      if (joint->getType() == moveit::core::JointModel::PLANAR)
      {
        joint->setVariableBounds(joint->getName() + "/" + joint->getLocalVariableNames()[0], bx);
        joint->setVariableBounds(joint->getName() + "/" + joint->getLocalVariableNames()[1], by);
      }
      else if (joint->getType() == moveit::core::JointModel::FLOATING)
      {
        joint->setVariableBounds(joint->getName() + "/" + joint->getLocalVariableNames()[0], bx);
        joint->setVariableBounds(joint->getName() + "/" + joint->getLocalVariableNames()[1], by);
        joint->setVariableBounds(joint->getName() + "/" + joint->getLocalVariableNames()[2], bz);
      }
  }
}

void MotionPlanningFrame::configureForPlanning()
{
  move_group_->setStartState(*planning_display_->getQueryStartState());
  move_group_->setJointValueTarget(*planning_display_->getQueryGoalState());
  move_group_->setPlanningTime(ui_->planning_time->value());
  move_group_->setNumPlanningAttempts(ui_->planning_attempts->value());
  move_group_->setMaxVelocityScalingFactor(ui_->velocity_scaling_factor->value());
  move_group_->setMaxAccelerationScalingFactor(ui_->acceleration_scaling_factor->value());
  configureWorkspace();
  if (static_cast<bool>(planning_display_))
    planning_display_->dropVisualizedTrajectory();
}

void MotionPlanningFrame::remotePlanCallback(const std_msgs::EmptyConstPtr& /*msg*/)
{
  planButtonClicked();
}

void MotionPlanningFrame::remoteExecuteCallback(const std_msgs::EmptyConstPtr& /*msg*/)
{
  executeButtonClicked();
}

void MotionPlanningFrame::remoteStopCallback(const std_msgs::EmptyConstPtr& /*msg*/)
{
  stopButtonClicked();
}

void MotionPlanningFrame::remoteUpdateStartStateCallback(const std_msgs::EmptyConstPtr& /*msg*/)
{
  if (move_group_ && planning_display_)
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      moveit::core::RobotState state = ps->getCurrentState();
      planning_display_->setQueryStartState(state);
    }
  }
}

void MotionPlanningFrame::remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& /*msg*/)
{
  if (move_group_ && planning_display_)
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      moveit::core::RobotState state = ps->getCurrentState();
      planning_display_->setQueryGoalState(state);
    }
  }
}

void MotionPlanningFrame::remoteUpdateCustomStartStateCallback(const moveit_msgs::RobotStateConstPtr& msg)
{
  moveit_msgs::RobotState msg_no_attached(*msg);
  msg_no_attached.attached_collision_objects.clear();
  msg_no_attached.is_diff = true;
  if (move_group_ && planning_display_)
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      moveit::core::RobotStatePtr state(new moveit::core::RobotState(ps->getCurrentState()));
      moveit::core::robotStateMsgToRobotState(ps->getTransforms(), msg_no_attached, *state);
      planning_display_->setQueryStartState(*state);
    }
  }
}

void MotionPlanningFrame::remoteUpdateCustomGoalStateCallback(const moveit_msgs::RobotStateConstPtr& msg)
{
  moveit_msgs::RobotState msg_no_attached(*msg);
  msg_no_attached.attached_collision_objects.clear();
  msg_no_attached.is_diff = true;
  if (move_group_ && planning_display_)
  {
    planning_display_->waitForCurrentRobotState();
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      moveit::core::RobotStatePtr state(new moveit::core::RobotState(ps->getCurrentState()));
      moveit::core::robotStateMsgToRobotState(ps->getTransforms(), msg_no_attached, *state);
      planning_display_->setQueryGoalState(*state);
    }
  }
}
}  // namespace moveit_rviz_plugin
