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

/* Author: Sachin Chitta */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <object_recognition_msgs/ObjectRecognitionGoal.h>

#include <tf2_eigen/tf2_eigen.h>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
/////////////// Object Detection ///////////////////////
void MotionPlanningFrame::detectObjectsButtonClicked()
{
  if (!semantic_world_)
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    if (ps)
    {
      semantic_world_ = std::make_shared<moveit::semantic_world::SemanticWorld>(ps);
    }
    if (semantic_world_)
    {
      semantic_world_->addTableCallback([this] { updateTables(); });
    }
  }
  planning_display_->addBackgroundJob([this] { triggerObjectDetection(); }, "detect objects");
}

void MotionPlanningFrame::processDetectedObjects()
{
  pick_object_name_.clear();

  std::vector<std::string> object_ids;
  double min_x = ui_->roi_center_x->value() - ui_->roi_size_x->value() / 2.0;
  double min_y = ui_->roi_center_y->value() - ui_->roi_size_y->value() / 2.0;
  double min_z = ui_->roi_center_z->value() - ui_->roi_size_z->value() / 2.0;

  double max_x = ui_->roi_center_x->value() + ui_->roi_size_x->value() / 2.0;
  double max_y = ui_->roi_center_y->value() + ui_->roi_size_y->value() / 2.0;
  double max_z = ui_->roi_center_z->value() + ui_->roi_size_z->value() / 2.0;

  ros::Time start_time = ros::Time::now();
  while (object_ids.empty() && ros::Time::now() - start_time <= ros::Duration(3.0))
  {
    // collect all objects in region of interest
    {
      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
      const collision_detection::WorldConstPtr& world = ps->getWorld();
      object_ids.clear();
      for (const auto& object : *world)
      {
        const auto& position = object.second->pose_.translation();
        if (position.x() >= min_x && position.x() <= max_x && position.y() >= min_y && position.y() <= max_y &&
            position.z() >= min_z && position.z() <= max_z)
        {
          object_ids.push_back(object.first);
        }
      }
      if (!object_ids.empty())
        break;
    }
    ros::Duration(0.5).sleep();
  }

  ROS_DEBUG("Found %d objects", (int)object_ids.size());
  updateDetectedObjectsList(object_ids);
}

void MotionPlanningFrame::selectedDetectedObjectChanged()
{
  QList<QListWidgetItem*> sel = ui_->detected_objects_list->selectedItems();
  if (sel.empty())
  {
    ROS_INFO("No objects to select");
    return;
  }
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  std_msgs::ColorRGBA pick_object_color;
  pick_object_color.r = 1.0;
  pick_object_color.g = 0.0;
  pick_object_color.b = 0.0;
  pick_object_color.a = 1.0;

  if (ps)
  {
    if (!selected_object_name_.empty())
      ps->removeObjectColor(selected_object_name_);
    selected_object_name_ = sel[0]->text().toStdString();
    ps->setObjectColor(selected_object_name_, pick_object_color);
  }
}

void MotionPlanningFrame::detectedObjectChanged(QListWidgetItem* /*item*/)
{
}

void MotionPlanningFrame::triggerObjectDetection()
{
  if (!object_recognition_client_)
  {
    object_recognition_client_ =
        std::make_unique<actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>>(
            OBJECT_RECOGNITION_ACTION, false);
    try
    {
      waitForAction(object_recognition_client_, ros::Duration(3.0), OBJECT_RECOGNITION_ACTION);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Object recognition action: %s", ex.what());
      return;
    }
  }
  object_recognition_msgs::ObjectRecognitionGoal goal;
  object_recognition_client_->sendGoal(goal);
  if (!object_recognition_client_->waitForResult())
  {
    ROS_INFO_STREAM("Object recognition client returned early");
  }
  if (object_recognition_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_WARN_STREAM("Fail: " << object_recognition_client_->getState().toString() << ": "
                             << object_recognition_client_->getState().getText());
  }
}

void MotionPlanningFrame::listenDetectedObjects(const object_recognition_msgs::RecognizedObjectArrayPtr& /*msg*/)
{
  ros::Duration(1.0).sleep();
  planning_display_->addMainLoopJob([this] { processDetectedObjects(); });
}

void MotionPlanningFrame::updateDetectedObjectsList(const std::vector<std::string>& object_ids)
{
  ui_->detected_objects_list->setUpdatesEnabled(false);
  bool old_state = ui_->detected_objects_list->blockSignals(true);
  ui_->detected_objects_list->clear();
  {
    for (std::size_t i = 0; i < object_ids.size(); ++i)
    {
      QListWidgetItem* item =
          new QListWidgetItem(QString::fromStdString(object_ids[i]), ui_->detected_objects_list, (int)i);
      item->setToolTip(item->text());
      Qt::ItemFlags flags = item->flags();
      flags &= ~(Qt::ItemIsUserCheckable);
      item->setFlags(flags);
      ui_->detected_objects_list->addItem(item);
    }
  }
  ui_->detected_objects_list->blockSignals(old_state);
  ui_->detected_objects_list->setUpdatesEnabled(true);
  if (!object_ids.empty())
    ui_->pick_button->setEnabled(true);
}

/////////////////////// Support Surfaces ///////////////////////
void MotionPlanningFrame::updateTables()
{
  ROS_DEBUG("Update table callback");
  planning_display_->addBackgroundJob([this] { publishTables(); }, "publish tables");
}

void MotionPlanningFrame::publishTables()
{
  semantic_world_->addTablesToCollisionWorld();
  planning_display_->addMainLoopJob([this] { updateSupportSurfacesList(); });
}

void MotionPlanningFrame::selectedSupportSurfaceChanged()
{
  QList<QListWidgetItem*> sel = ui_->support_surfaces_list->selectedItems();
  if (sel.empty())
  {
    ROS_INFO("No tables to select");
    return;
  }
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  std_msgs::ColorRGBA selected_support_surface_color;
  selected_support_surface_color.r = 0.0;
  selected_support_surface_color.g = 0.0;
  selected_support_surface_color.b = 1.0;
  selected_support_surface_color.a = 1.0;

  if (ps)
  {
    if (!selected_support_surface_name_.empty())
      ps->removeObjectColor(selected_support_surface_name_);
    selected_support_surface_name_ = sel[0]->text().toStdString();
    ps->setObjectColor(selected_support_surface_name_, selected_support_surface_color);
  }
}

void MotionPlanningFrame::updateSupportSurfacesList()
{
  double min_x = ui_->roi_center_x->value() - ui_->roi_size_x->value() / 2.0;
  double min_y = ui_->roi_center_y->value() - ui_->roi_size_y->value() / 2.0;
  double min_z = ui_->roi_center_z->value() - ui_->roi_size_z->value() / 2.0;

  double max_x = ui_->roi_center_x->value() + ui_->roi_size_x->value() / 2.0;
  double max_y = ui_->roi_center_y->value() + ui_->roi_size_y->value() / 2.0;
  double max_z = ui_->roi_center_z->value() + ui_->roi_size_z->value() / 2.0;
  std::vector<std::string> support_ids = semantic_world_->getTableNamesInROI(min_x, min_y, min_z, max_x, max_y, max_z);
  ROS_INFO("%d Tables in collision world", (int)support_ids.size());

  ui_->support_surfaces_list->setUpdatesEnabled(false);
  bool old_state = ui_->support_surfaces_list->blockSignals(true);
  ui_->support_surfaces_list->clear();
  {
    for (std::size_t i = 0; i < support_ids.size(); ++i)
    {
      QListWidgetItem* item =
          new QListWidgetItem(QString::fromStdString(support_ids[i]), ui_->support_surfaces_list, (int)i);
      item->setToolTip(item->text());
      Qt::ItemFlags flags = item->flags();
      flags &= ~(Qt::ItemIsUserCheckable);
      item->setFlags(flags);
      ui_->support_surfaces_list->addItem(item);
    }
  }
  ui_->support_surfaces_list->blockSignals(old_state);
  ui_->support_surfaces_list->setUpdatesEnabled(true);
}

/////////////////////////////// Pick & Place /////////////////////////////////
void MotionPlanningFrame::pickObjectButtonClicked()
{
  QList<QListWidgetItem*> sel = ui_->detected_objects_list->selectedItems();
  QList<QListWidgetItem*> sel_table = ui_->support_surfaces_list->selectedItems();

  std::string group_name = planning_display_->getCurrentPlanningGroup();
  if (sel.empty())
  {
    ROS_INFO("No objects to pick");
    return;
  }
  pick_object_name_[group_name] = sel[0]->text().toStdString();

  if (!sel_table.empty())
    support_surface_name_ = sel_table[0]->text().toStdString();
  else
  {
    if (semantic_world_)
    {
      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
      if (ps->getWorld()->hasObject(pick_object_name_[group_name]))
      {
        geometry_msgs::Pose object_pose(tf2::toMsg(ps->getWorld()->getTransform(pick_object_name_[group_name])));
        ROS_DEBUG_STREAM("Finding current table for object: " << pick_object_name_[group_name]);
        support_surface_name_ = semantic_world_->findObjectTable(object_pose);
      }
      else
        support_surface_name_.clear();
    }
    else
      support_surface_name_.clear();
  }
  ROS_INFO("Trying to pick up object %s from support surface %s", pick_object_name_[group_name].c_str(),
           support_surface_name_.c_str());
  planning_display_->addBackgroundJob([this] { pickObject(); }, "pick");
}

void MotionPlanningFrame::placeObjectButtonClicked()
{
  QList<QListWidgetItem*> sel_table = ui_->support_surfaces_list->selectedItems();
  std::string group_name = planning_display_->getCurrentPlanningGroup();

  if (!sel_table.empty())
    support_surface_name_ = sel_table[0]->text().toStdString();
  else
  {
    support_surface_name_.clear();
    ROS_ERROR("Need to specify table to place object on");
    return;
  }

  ui_->pick_button->setEnabled(false);
  ui_->place_button->setEnabled(false);

  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  if (!ps)
  {
    ROS_ERROR("No planning scene");
    return;
  }
  const moveit::core::JointModelGroup* jmg = ps->getCurrentState().getJointModelGroup(group_name);
  if (jmg)
    ps->getCurrentState().getAttachedBodies(attached_bodies, jmg);

  if (attached_bodies.empty())
  {
    ROS_ERROR("No bodies to place");
    return;
  }

  geometry_msgs::Quaternion upright_orientation;
  upright_orientation.w = 1.0;

  // Else place the first one
  place_poses_.clear();
  place_poses_ = semantic_world_->generatePlacePoses(support_surface_name_, attached_bodies[0]->getShapes()[0],
                                                     upright_orientation, 0.1);
  planning_display_->visualizePlaceLocations(place_poses_);
  place_object_name_ = attached_bodies[0]->getName();
  planning_display_->addBackgroundJob([this] { placeObject(); }, "place");
}

void MotionPlanningFrame::pickObject()
{
  std::string group_name = planning_display_->getCurrentPlanningGroup();
  ui_->pick_button->setEnabled(false);
  if (pick_object_name_.find(group_name) == pick_object_name_.end())
  {
    ROS_ERROR("No pick object set for this group");
    return;
  }
  if (!support_surface_name_.empty())
  {
    move_group_->setSupportSurfaceName(support_surface_name_);
  }
  if (move_group_->pick(pick_object_name_[group_name]))
  {
    ui_->place_button->setEnabled(true);
  }
}

void MotionPlanningFrame::placeObject()
{
  move_group_->place(place_object_name_, place_poses_);
  return;
}
}  // namespace moveit_rviz_plugin
