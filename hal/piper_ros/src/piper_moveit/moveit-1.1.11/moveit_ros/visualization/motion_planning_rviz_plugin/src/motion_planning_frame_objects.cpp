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

/* Author: Ioan Sucan, Mario Prats */

#include <moveit/warehouse/planning_scene_storage.h>

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <interactive_markers/tools.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/window_manager_interface.h>

#include <tf2_eigen/tf2_eigen.h>
#include <geometric_shapes/shape_operations.h>

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace
{
QString subframe_poses_to_qstring(const moveit::core::FixedTransformsMap& subframes)
{
  QString status_text = "\nIt has the subframes '";
  for (const auto& subframe : subframes)
  {
    status_text += QString::fromStdString(subframe.first) + "', '";
  }
  status_text.chop(3);
  status_text += ".";
  return status_text;
}
}  // namespace

namespace moveit_rviz_plugin
{
void MotionPlanningFrame::shapesComboBoxChanged(const QString& /*text*/)
{
  switch (ui_->shapes_combo_box->currentData().toInt())  // fetch shape ID from current combobox item
  {
    case shapes::BOX:
      ui_->shape_size_x_spin_box->setEnabled(true);
      ui_->shape_size_y_spin_box->setEnabled(true);
      ui_->shape_size_z_spin_box->setEnabled(true);
      break;
    case shapes::SPHERE:
      ui_->shape_size_x_spin_box->setEnabled(true);
      ui_->shape_size_y_spin_box->setEnabled(false);
      ui_->shape_size_z_spin_box->setEnabled(false);
      break;
    case shapes::CYLINDER:
    case shapes::CONE:
      ui_->shape_size_x_spin_box->setEnabled(true);
      ui_->shape_size_y_spin_box->setEnabled(false);
      ui_->shape_size_z_spin_box->setEnabled(true);
      break;
    case shapes::MESH:
      ui_->shape_size_x_spin_box->setEnabled(false);
      ui_->shape_size_y_spin_box->setEnabled(false);
      ui_->shape_size_z_spin_box->setEnabled(false);
      break;
    default:
      break;
  }
}

void MotionPlanningFrame::setLocalSceneEdited(bool dirty)
{
  ui_->publish_current_scene_button->setEnabled(dirty);
}

bool MotionPlanningFrame::isLocalSceneDirty() const
{
  return ui_->publish_current_scene_button->isEnabled();
}

void MotionPlanningFrame::publishScene()
{
  const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  if (ps)
  {
    moveit_msgs::PlanningScene msg;
    ps->getPlanningSceneMsg(msg);
    planning_scene_publisher_.publish(msg);
    setLocalSceneEdited(false);
  }
}

void MotionPlanningFrame::publishSceneIfNeeded()
{
  if (isLocalSceneDirty() &&
      QMessageBox::question(this, "Update PlanningScene",
                            "You have local changes to your planning scene.\n"
                            "Publish them to the move_group node?",
                            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) == QMessageBox::Yes)
    publishScene();
}

void MotionPlanningFrame::clearScene()
{
  if (auto ps = planning_display_->getPlanningSceneRW())
  {
    ps->getWorldNonConst()->clearObjects();
    ps->getCurrentStateNonConst().clearAttachedBodies();
    setLocalSceneEdited(true);
    planning_display_->updateQueryStates(ps->getCurrentState());
    populateCollisionObjectsList(&ps);  // update list + internal vars
    planning_display_->queueRenderSceneGeometry();
  }
}

void MotionPlanningFrame::sceneScaleChanged(int value)
{
  const double scaling_factor = (double)value / 100.0;  // The GUI slider gives percent values
  if (scaled_object_)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    if (ps)
    {
      if (ps->getWorld()->hasObject(scaled_object_->id_))
      {
        ps->getWorldNonConst()->removeObject(scaled_object_->id_);
        for (std::size_t i = 0; i < scaled_object_->shapes_.size(); ++i)
        {
          shapes::Shape* s = scaled_object_->shapes_[i]->clone();
          s->scale(scaling_factor);

          Eigen::Isometry3d scaled_shape_pose = scaled_object_->shape_poses_[i];
          scaled_shape_pose.translation() *= scaling_factor;

          ps->getWorldNonConst()->addToObject(scaled_object_->id_, scaled_object_->pose_, shapes::ShapeConstPtr(s),
                                              scaled_shape_pose);
        }
        moveit::core::FixedTransformsMap scaled_subframes = scaled_object_->subframe_poses_;
        for (auto& subframe_pair : scaled_subframes)
          subframe_pair.second.translation() *= scaling_factor;

        ps->getWorldNonConst()->setSubframesOfObject(scaled_object_->id_, scaled_subframes);
        setLocalSceneEdited();
        if (scene_marker_)
          scene_marker_->processMessage(createObjectMarkerMsg(ps->getWorld()->getObject(scaled_object_->id_)));
        planning_display_->queueRenderSceneGeometry();
      }
      else
        scaled_object_.reset();
    }
    else
      scaled_object_.reset();
  }
}

void MotionPlanningFrame::sceneScaleStartChange()
{
  auto current = ui_->collision_objects_list->currentItem();
  if (!current)
    return;
  if (planning_display_->getPlanningSceneMonitor() && current->checkState() == Qt::Unchecked)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    if (ps)
    {
      scaled_object_ = ps->getWorld()->getObject(current->text().toStdString());
      scaled_object_subframes_ = scaled_object_->subframe_poses_;
      scaled_object_shape_poses_ = scaled_object_->shape_poses_;
    }
  }
}

void MotionPlanningFrame::sceneScaleEndChange()
{
  scaled_object_.reset();
  ui_->scene_scale->setSliderPosition(100);
}

void MotionPlanningFrame::removeSceneObjects()
{
  QList<QListWidgetItem*> selection = ui_->collision_objects_list->selectedItems();
  if (selection.empty())
    return;

  if (auto ps = planning_display_->getPlanningSceneRW())
  {
    bool removed_attached = false;
    for (QListWidgetItem* item : selection)
      if (item->checkState() == Qt::Unchecked)
        ps->getWorldNonConst()->removeObject(item->text().toStdString());
      else
      {
        ps->getCurrentStateNonConst().clearAttachedBody(item->text().toStdString());
        removed_attached = true;
      }

    if (removed_attached)
      planning_display_->updateQueryStates(ps->getCurrentState());

    populateCollisionObjectsList(&ps);
  }
  scene_marker_.reset();
  setLocalSceneEdited();
  planning_display_->queueRenderSceneGeometry();
}

static QString decideStatusText(const collision_detection::CollisionEnv::ObjectConstPtr& obj)
{
  QString status_text = "'" + QString::fromStdString(obj->id_) + "' is a collision object with ";
  if (obj->shapes_.empty())
    status_text += "no geometry";
  else
  {
    std::vector<QString> shape_names;
    for (const shapes::ShapeConstPtr& shape : obj->shapes_)
      shape_names.push_back(QString::fromStdString(shapes::shapeStringName(shape.get())));
    if (shape_names.size() == 1)
      status_text += "one " + shape_names[0];
    else
    {
      status_text += QString::fromStdString(boost::lexical_cast<std::string>(shape_names.size())) + " shapes:";
      for (const QString& shape_name : shape_names)
        status_text += " " + shape_name;
    }
    status_text += ".";
  }
  if (!obj->subframe_poses_.empty())
  {
    status_text += subframe_poses_to_qstring(obj->subframe_poses_);
  }
  return status_text;
}

static QString decideStatusText(const moveit::core::AttachedBody* attached_body)
{
  QString status_text = "'" + QString::fromStdString(attached_body->getName()) + "' is attached to '" +
                        QString::fromStdString(attached_body->getAttachedLinkName()) + "'.";
  if (!attached_body->getSubframes().empty())
  {
    status_text += subframe_poses_to_qstring(attached_body->getSubframes());
  }
  return status_text;
}

void MotionPlanningFrame::currentCollisionObjectChanged()
{
  auto setValue = [](QDoubleSpinBox* w, float value) {  // NOLINT(readability-identifier-naming)
    QSignalBlocker block(w);
    w->setValue(value);
  };

  auto current = ui_->collision_objects_list->currentItem();
  if (!current)
  {
    setValue(ui_->object_x, 0.0);
    setValue(ui_->object_y, 0.0);
    setValue(ui_->object_z, 0.0);
    setValue(ui_->object_rx, 0.0);
    setValue(ui_->object_ry, 0.0);
    setValue(ui_->object_rz, 0.0);
    ui_->object_status->setText("");
    scene_marker_.reset();
    ui_->pose_scale_group_box->setEnabled(false);
  }
  else if (planning_display_->getPlanningSceneMonitor())
  {
    // if this is a CollisionWorld element
    if (current->checkState() == Qt::Unchecked)
    {
      ui_->pose_scale_group_box->setEnabled(true);
      bool update_scene_marker = false;
      Eigen::Isometry3d obj_pose;
      {
        const auto& ps = planning_display_->getPlanningSceneRO();
        if (const auto& obj = ps->getWorld()->getObject(current->text().toStdString()))
        {
          ui_->object_status->setText(decideStatusText(obj));
          obj_pose = obj->pose_;  // valid isometry by contract
          Eigen::Vector3d xyz = obj_pose.linear().eulerAngles(0, 1, 2);
          update_scene_marker = true;  // do the marker update outside locked scope to avoid deadlock
          setValue(ui_->object_x, obj_pose.translation()[0]);
          setValue(ui_->object_y, obj_pose.translation()[1]);
          setValue(ui_->object_z, obj_pose.translation()[2]);
          setValue(ui_->object_rx, xyz[0]);
          setValue(ui_->object_ry, xyz[1]);
          setValue(ui_->object_rz, xyz[2]);
        }
        else
          ui_->object_status->setText("ERROR: '" + current->text() + "' should be a collision object but it is not");
      }
      if (update_scene_marker && ui_->tabWidget->tabText(ui_->tabWidget->currentIndex()).toStdString() == TAB_OBJECTS)
      {
        createSceneInteractiveMarker();
      }
    }
    else
    {
      ui_->pose_scale_group_box->setEnabled(false);
      // if it is an attached object
      scene_marker_.reset();
      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
      const moveit::core::AttachedBody* attached_body =
          ps->getCurrentState().getAttachedBody(current->text().toStdString());
      if (attached_body)
        ui_->object_status->setText(decideStatusText(attached_body));
      else
        ui_->object_status->setText("ERROR: '" + current->text() + "' should be an attached object but it is not");
    }
  }
}

void MotionPlanningFrame::objectPoseValueChanged(double /* value */)
{
  updateCollisionObjectPose(true);
}

void MotionPlanningFrame::updateCollisionObjectPose(bool update_marker_position)
{
  auto current = ui_->collision_objects_list->currentItem();
  if (!current)
    return;
  if (auto ps = planning_display_->getPlanningSceneRW())
  {
    collision_detection::CollisionEnv::ObjectConstPtr obj = ps->getWorld()->getObject(current->text().toStdString());
    if (obj)
    {
      Eigen::Isometry3d p;
      p.translation()[0] = ui_->object_x->value();
      p.translation()[1] = ui_->object_y->value();
      p.translation()[2] = ui_->object_z->value();

      p = Eigen::Translation3d(p.translation()) *
          (Eigen::AngleAxisd(ui_->object_rx->value(), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(ui_->object_ry->value(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(ui_->object_rz->value(), Eigen::Vector3d::UnitZ()));

      ps->getWorldNonConst()->setObjectPose(obj->id_, p);
      planning_display_->queueRenderSceneGeometry();
      setLocalSceneEdited();

      // Update the interactive marker pose to match the manually introduced one
      if (update_marker_position && scene_marker_)
      {
        // p is isometry by construction
        Eigen::Quaterniond eq(p.linear());
        scene_marker_->setPose(Ogre::Vector3(ui_->object_x->value(), ui_->object_y->value(), ui_->object_z->value()),
                               Ogre::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()), "");
      }
    }
  }
}

void MotionPlanningFrame::collisionObjectChanged(QListWidgetItem* item)
{
  if (item->type() < (int)known_collision_objects_.size() && planning_display_->getPlanningSceneMonitor())
  {
    // if we have a name change
    if (known_collision_objects_[item->type()].first != item->text().toStdString())
      renameCollisionObject(item);
    else
    {
      bool checked = item->checkState() == Qt::Checked;
      if (known_collision_objects_[item->type()].second != checked)
        attachDetachCollisionObject(item);
    }
  }
}

/* Receives feedback from the interactive marker and updates the shape pose in the world accordingly */
void MotionPlanningFrame::imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback)
{
  bool old_state = ui_->object_x->blockSignals(true);
  ui_->object_x->setValue(feedback.pose.position.x);
  ui_->object_x->blockSignals(old_state);

  old_state = ui_->object_y->blockSignals(true);
  ui_->object_y->setValue(feedback.pose.position.y);
  ui_->object_y->blockSignals(old_state);

  old_state = ui_->object_z->blockSignals(true);
  ui_->object_z->setValue(feedback.pose.position.z);
  ui_->object_z->blockSignals(old_state);

  Eigen::Quaterniond q;
  tf2::fromMsg(feedback.pose.orientation, q);
  Eigen::Vector3d xyz = q.matrix().eulerAngles(0, 1, 2);

  old_state = ui_->object_rx->blockSignals(true);
  ui_->object_rx->setValue(xyz[0]);
  ui_->object_rx->blockSignals(old_state);

  old_state = ui_->object_ry->blockSignals(true);
  ui_->object_ry->setValue(xyz[1]);
  ui_->object_ry->blockSignals(old_state);

  old_state = ui_->object_rz->blockSignals(true);
  ui_->object_rz->setValue(xyz[2]);
  ui_->object_rz->blockSignals(old_state);

  updateCollisionObjectPose(false);
}

void MotionPlanningFrame::copySelectedCollisionObjects()
{
  QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
  if (sel.empty())
    return;

  auto ps = planning_display_->getPlanningSceneRW();
  if (!ps)
    return;

  for (const QListWidgetItem* item : sel)
  {
    std::string name = item->text().toStdString();
    collision_detection::CollisionEnv::ObjectConstPtr obj = ps->getWorld()->getObject(name);
    if (!obj)
      continue;

    // find a name for the copy
    name.insert(0, "Copy of ");
    if (ps->getWorld()->hasObject(name))
    {
      name += " ";
      unsigned int n = 1;
      while (ps->getWorld()->hasObject(name + boost::lexical_cast<std::string>(n)))
        n++;
      name += boost::lexical_cast<std::string>(n);
    }
    ps->getWorldNonConst()->addToObject(name, obj->shapes_, obj->shape_poses_);
    ROS_DEBUG("Copied collision object to '%s'", name.c_str());
  }
  setLocalSceneEdited();
  populateCollisionObjectsList(&ps);
}

void MotionPlanningFrame::computeSaveSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    moveit_msgs::PlanningScene msg;
    planning_display_->getPlanningSceneRO()->getPlanningSceneMsg(msg);
    try
    {
      planning_scene_storage_->removePlanningScene(msg.name);
      planning_scene_storage_->addPlanningScene(msg);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    planning_display_->addMainLoopJob([this] { populatePlanningSceneTreeView(); });
  }
}

void MotionPlanningFrame::computeSaveQueryButtonClicked(const std::string& scene, const std::string& query_name)
{
  moveit_msgs::MotionPlanRequest mreq;
  constructPlanningRequest(mreq);
  if (planning_scene_storage_)
  {
    try
    {
      if (!query_name.empty())
        planning_scene_storage_->removePlanningQuery(scene, query_name);
      planning_scene_storage_->addPlanningQuery(mreq, scene, query_name);
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    planning_display_->addMainLoopJob([this] { populatePlanningSceneTreeView(); });
  }
}

void MotionPlanningFrame::computeDeleteSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        try
        {
          planning_scene_storage_->removePlanningScene(scene);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      else
      {
        // if we selected a query name, then we overwrite that query
        std::string scene = s->parent()->text(0).toStdString();
        try
        {
          planning_scene_storage_->removePlanningScene(scene);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
      }
      planning_display_->addMainLoopJob([this] { populatePlanningSceneTreeView(); });
    }
  }
}

void MotionPlanningFrame::computeDeleteQueryButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_QUERY)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        try
        {
          planning_scene_storage_->removePlanningQuery(scene, query_name);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }
        planning_display_->addMainLoopJob([this, s] { computeDeleteQueryButtonClickedHelper(s); });
      }
    }
  }
}

void MotionPlanningFrame::computeDeleteQueryButtonClickedHelper(QTreeWidgetItem* s)
{
  ui_->planning_scene_tree->setUpdatesEnabled(false);
  s->parent()->removeChild(s);
  ui_->planning_scene_tree->setUpdatesEnabled(true);
}

void MotionPlanningFrame::checkPlanningSceneTreeEnabledButtons()
{
  QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
  if (sel.empty())
  {
    ui_->load_scene_button->setEnabled(false);
    ui_->load_query_button->setEnabled(false);
    ui_->save_query_button->setEnabled(false);
    ui_->delete_scene_button->setEnabled(false);
  }
  else
  {
    ui_->save_query_button->setEnabled(true);

    QTreeWidgetItem* s = sel.front();

    // if the item is a PlanningScene
    if (s->type() == ITEM_TYPE_SCENE)
    {
      ui_->load_scene_button->setEnabled(true);
      ui_->load_query_button->setEnabled(false);
      ui_->delete_scene_button->setEnabled(true);
      ui_->delete_query_button->setEnabled(false);
      ui_->save_query_button->setEnabled(true);
    }
    else
    {
      // if the item is a query
      ui_->load_scene_button->setEnabled(false);
      ui_->load_query_button->setEnabled(true);
      ui_->delete_scene_button->setEnabled(false);
      ui_->delete_query_button->setEnabled(true);
    }
  }
}

void MotionPlanningFrame::computeLoadSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        ROS_DEBUG("Attempting to load scene '%s'", scene.c_str());
        moveit_warehouse::PlanningSceneWithMetadata scene_m;
        bool got_ps = false;
        try
        {
          got_ps = planning_scene_storage_->getPlanningScene(scene_m, scene);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        if (got_ps)
        {
          ROS_INFO("Loaded scene '%s'", scene.c_str());
          if (planning_display_->getPlanningSceneMonitor())
          {
            if (scene_m->robot_model_name != planning_display_->getRobotModel()->getName())
            {
              ROS_INFO("Scene '%s' was saved for robot '%s' but we are using robot '%s'. Using scene geometry only",
                       scene.c_str(), scene_m->robot_model_name.c_str(),
                       planning_display_->getRobotModel()->getName().c_str());
              planning_scene_world_publisher_.publish(scene_m->world);
              // publish the parts that are not in the world
              moveit_msgs::PlanningScene diff;
              diff.is_diff = true;
              diff.name = scene_m->name;
              planning_scene_publisher_.publish(diff);
            }
            else
              planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
          }
          else
            planning_scene_publisher_.publish(static_cast<const moveit_msgs::PlanningScene&>(*scene_m));
        }
        else
          ROS_WARN("Failed to load scene '%s'. Has the message format changed since the scene was saved?",
                   scene.c_str());
      }
    }
  }
}

void MotionPlanningFrame::computeLoadQueryButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();
      if (s->type() == ITEM_TYPE_QUERY)
      {
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();
        moveit_warehouse::MotionPlanRequestWithMetadata mp;
        bool got_q = false;
        try
        {
          got_q = planning_scene_storage_->getPlanningQuery(mp, scene, query_name);
        }
        catch (std::exception& ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        if (got_q)
        {
          moveit::core::RobotStatePtr start_state(
              new moveit::core::RobotState(*planning_display_->getQueryStartState()));
          moveit::core::robotStateMsgToRobotState(planning_display_->getPlanningSceneRO()->getTransforms(),
                                                  mp->start_state, *start_state);
          planning_display_->setQueryStartState(*start_state);

          moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*planning_display_->getQueryGoalState()));
          for (const moveit_msgs::Constraints& goal_constraint : mp->goal_constraints)
            if (!goal_constraint.joint_constraints.empty())
            {
              std::map<std::string, double> vals;
              for (const moveit_msgs::JointConstraint& joint_constraint : goal_constraint.joint_constraints)
                vals[joint_constraint.joint_name] = joint_constraint.position;
              goal_state->setVariablePositions(vals);
              break;
            }
          planning_display_->setQueryGoalState(*goal_state);
        }
        else
          ROS_ERROR("Failed to load planning query '%s'. Has the message format changed since the query was saved?",
                    query_name.c_str());
      }
    }
  }
}

visualization_msgs::InteractiveMarker
MotionPlanningFrame::createObjectMarkerMsg(const collision_detection::CollisionEnv::ObjectConstPtr& obj)
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double scale = 0.2;

  if (!obj->shapes_.empty())
    shapes::computeShapeBoundingSphere(obj->shapes_[0].get(), center, scale);
  geometry_msgs::PoseStamped shape_pose = tf2::toMsg(
      tf2::Stamped<Eigen::Isometry3d>(obj->pose_, ros::Time(), planning_display_->getRobotModel()->getModelFrame()));
  scale = (scale + center.cwiseAbs().maxCoeff()) * 2.0 * 1.2;  // add padding of 20% size

  // create an interactive marker msg for the given shape
  visualization_msgs::InteractiveMarker imarker =
      robot_interaction::make6DOFMarker("marker_scene_object", shape_pose, scale);
  imarker.description = obj->id_;
  interactive_markers::autoComplete(imarker);
  return imarker;
}

void MotionPlanningFrame::createSceneInteractiveMarker()
{
  auto current = ui_->collision_objects_list->currentItem();
  if (!current)
    return;

  const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  if (!ps)
    return;

  if (const auto& obj = ps->getWorld()->getObject(current->text().toStdString()))
  {
    scene_marker_ = std::make_shared<rviz::InteractiveMarker>(planning_display_->getSceneNode(), context_);
    scene_marker_->processMessage(createObjectMarkerMsg(obj));
    scene_marker_->setShowAxes(false);

    // Connect signals
    connect(scene_marker_.get(), SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)), this,
            SLOT(imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback&)));
  }
  else
  {
    scene_marker_.reset();
  }
}

void MotionPlanningFrame::renameCollisionObject(QListWidgetItem* item)
{
  long unsigned int version = known_collision_objects_version_;
  if (item->text().isEmpty())
  {
    QMessageBox::warning(this, "Invalid object name", "Cannot set an empty object name.");
    if (version == known_collision_objects_version_)
      item->setText(QString::fromStdString(known_collision_objects_[item->type()].first));
    return;
  }

  std::string item_text = item->text().toStdString();
  bool already_exists = planning_display_->getPlanningSceneRO()->getWorld()->hasObject(item_text);
  if (!already_exists)
    already_exists = planning_display_->getPlanningSceneRO()->getCurrentState().hasAttachedBody(item_text);
  if (already_exists)
  {
    QMessageBox::warning(this, "Duplicate object name",
                         QString("The name '")
                             .append(item->text())
                             .append("' already exists. Not renaming object ")
                             .append((known_collision_objects_[item->type()].first.c_str())));
    if (version == known_collision_objects_version_)
      item->setText(QString::fromStdString(known_collision_objects_[item->type()].first));
    return;
  }

  if (item->checkState() == Qt::Unchecked)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    collision_detection::CollisionEnv::ObjectConstPtr obj =
        ps->getWorld()->getObject(known_collision_objects_[item->type()].first);
    if (obj)
    {
      known_collision_objects_[item->type()].first = item_text;
      ps->getWorldNonConst()->removeObject(obj->id_);
      ps->getWorldNonConst()->addToObject(known_collision_objects_[item->type()].first, obj->pose_, obj->shapes_,
                                          obj->shape_poses_);
      ps->getWorldNonConst()->setSubframesOfObject(obj->id_, obj->subframe_poses_);
      if (scene_marker_)
      {
        scene_marker_.reset();
        planning_display_->addMainLoopJob([this] { createSceneInteractiveMarker(); });
      }
    }
  }
  else
  {
    // rename attached body
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    moveit::core::RobotState& cs = ps->getCurrentStateNonConst();
    const moveit::core::AttachedBody* ab = cs.getAttachedBody(known_collision_objects_[item->type()].first);
    if (ab)
    {
      known_collision_objects_[item->type()].first = item_text;
      auto new_ab = std::make_unique<moveit::core::AttachedBody>(
          ab->getAttachedLink(), known_collision_objects_[item->type()].first, ab->getPose(), ab->getShapes(),
          ab->getShapePoses(), ab->getTouchLinks(), ab->getDetachPosture(), ab->getSubframes());
      cs.clearAttachedBody(ab->getName());
      cs.attachBody(std::move(new_ab));
    }
  }
  setLocalSceneEdited();
}

void MotionPlanningFrame::attachDetachCollisionObject(QListWidgetItem* item)
{
  long unsigned int version = known_collision_objects_version_;
  bool checked = item->checkState() == Qt::Checked;
  std::pair<std::string, bool> data = known_collision_objects_[item->type()];
  moveit_msgs::AttachedCollisionObject aco;

  if (checked)  // we need to attach a known collision object
  {
    QStringList links;
    const std::vector<std::string>& links_std = planning_display_->getRobotModel()->getLinkModelNames();
    for (const std::string& link : links_std)
      links.append(QString::fromStdString(link));
    bool ok = false;
    QString response =
        QInputDialog::getItem(this, tr("Select Link Name"), tr("Choose the link to attach to:"), links, 0, false, &ok);
    if (!ok)
    {
      if (version == known_collision_objects_version_)
        item->setCheckState(Qt::Unchecked);
      return;
    }
    aco.link_name = response.toStdString();
    aco.object.id = data.first;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
  }
  else  // we need to detach an attached object
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
    const moveit::core::AttachedBody* attached_body = ps->getCurrentState().getAttachedBody(data.first);
    if (attached_body)
    {
      aco.link_name = attached_body->getAttachedLinkName();
      aco.object.id = attached_body->getName();
      aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
    }
  }

  moveit::core::RobotState rs(planning_display_->getRobotModel());
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    // we loop through the list in case updates were received since the start of the function
    for (std::pair<std::string, bool>& known_collision_object : known_collision_objects_)
      if (known_collision_object.first == data.first)
      {
        known_collision_object.second = checked;
        break;
      }
    ps->processAttachedCollisionObjectMsg(aco);
    rs = ps->getCurrentState();
  }

  currentCollisionObjectChanged();
  setLocalSceneEdited();
  planning_display_->updateQueryStates(rs);
  planning_display_->queueRenderSceneGeometry();
}

QListWidgetItem* MotionPlanningFrame::addCollisionObjectToList(const std::string& name, int row, bool attached)
{
  QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(name), ui_->collision_objects_list, row);
  item->setFlags(item->flags() | Qt::ItemIsEditable);
  item->setToolTip(item->text());
  item->setCheckState(attached ? Qt::Checked : Qt::Unchecked);
  known_collision_objects_.push_back(std::make_pair(name, attached));
  return item;
}

void MotionPlanningFrame::populateCollisionObjectsList(planning_scene_monitor::LockedPlanningSceneRO* pps)
{
  ui_->collision_objects_list->setUpdatesEnabled(false);
  bool octomap_in_scene = false;

  {
    QSignalBlocker block(ui_->collision_objects_list);
    QList<QListWidgetItem*> sel = ui_->collision_objects_list->selectedItems();
    std::set<std::string> to_select;
    for (QListWidgetItem* item : sel)
      to_select.insert(item->text().toStdString());
    ui_->collision_objects_list->clear();
    known_collision_objects_.clear();
    known_collision_objects_version_++;

    auto ps = pps ? *pps : planning_display_->getPlanningSceneRO();
    if (ps)
    {
      for (const std::string& name : ps->getWorld()->getObjectIds())
      {
        if (name == planning_scene::PlanningScene::OCTOMAP_NS)
        {
          octomap_in_scene = true;
          continue;
        }
        QListWidgetItem* item = addCollisionObjectToList(name, ui_->collision_objects_list->count(), false);
        if (to_select.find(name) != to_select.end())
          item->setSelected(true);
      }

      std::vector<const moveit::core::AttachedBody*> attached_bodies;
      ps->getCurrentState().getAttachedBodies(attached_bodies);
      for (const auto& body : attached_bodies)
      {
        QListWidgetItem* item = addCollisionObjectToList(body->getName(), ui_->collision_objects_list->count(), true);
        if (to_select.find(body->getName()) != to_select.end())
          item->setSelected(true);
      }
    }
  }

  ui_->clear_octomap_button->setEnabled(octomap_in_scene);
  ui_->collision_objects_list->setUpdatesEnabled(true);
  planning_display_->addMainLoopJob([this] { currentCollisionObjectChanged(); });
}

void MotionPlanningFrame::exportGeometryAsTextButtonClicked()
{
  QString path =
      QFileDialog::getSaveFileName(this, tr("Export Scene Geometry"), tr(""), tr("Scene Geometry (*.scene)"));
  if (!path.isEmpty())
    planning_display_->addBackgroundJob([this, path = path.toStdString()] { computeExportGeometryAsText(path); },
                                        "export as text");
}

void MotionPlanningFrame::computeExportGeometryAsText(const std::string& path)
{
  planning_scene_monitor::LockedPlanningSceneRO ps = planning_display_->getPlanningSceneRO();
  if (ps)
  {
    std::string p = (path.length() < 7 || path.substr(path.length() - 6) != ".scene") ? path + ".scene" : path;
    std::ofstream fout(p.c_str());
    if (fout.good())
    {
      ps->saveGeometryToStream(fout);
      fout.close();
      ROS_INFO("Saved current scene geometry to '%s'", p.c_str());
    }
    else
      ROS_WARN("Unable to save current scene geometry to '%s'", p.c_str());
  }
}

void MotionPlanningFrame::computeImportGeometryFromText(const std::string& path)
{
  bool success = false;
  if (auto ps = planning_display_->getPlanningSceneRW())
  {
    std::ifstream fin(path.c_str());
    if (ps->loadGeometryFromStream(fin))
    {
      ROS_INFO("Loaded scene geometry from '%s'", path.c_str());
      populateCollisionObjectsList(&ps);
      planning_display_->queueRenderSceneGeometry();
      setLocalSceneEdited();
      success = true;
    }
  }
  if (!success)
    planning_display_->addMainLoopJob([] {
      QMessageBox::warning(nullptr, "Loading scene geometry",
                           "Failed to load scene geometry.\n"
                           "See console output for more details.");
    });
}

void MotionPlanningFrame::importGeometryFromTextButtonClicked()
{
  QString path =
      QFileDialog::getOpenFileName(this, tr("Import Scene Geometry"), tr(""), tr("Scene Geometry (*.scene)"));
  if (!path.isEmpty())
    planning_display_->addBackgroundJob([this, path = path.toStdString()] { computeImportGeometryFromText(path); },
                                        "import from text");
}
}  // namespace moveit_rviz_plugin
