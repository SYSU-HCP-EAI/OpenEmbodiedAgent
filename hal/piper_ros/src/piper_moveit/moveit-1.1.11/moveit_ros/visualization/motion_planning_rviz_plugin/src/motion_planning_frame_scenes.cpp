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

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <interactive_markers/tools.h>

#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

#include <boost/math/constants/constants.hpp>

#include <memory>

namespace moveit_rviz_plugin
{
void MotionPlanningFrame::saveSceneButtonClicked()
{
  if (planning_scene_storage_)
  {
    const std::string& name = planning_display_->getPlanningSceneRO()->getName();
    if (name.empty() || planning_scene_storage_->hasPlanningScene(name))
    {
      std::unique_ptr<QMessageBox> q;
      if (name.empty())
        q = std::make_unique<QMessageBox>(
            QMessageBox::Question, "Change Planning Scene Name",
            QString("The name for the planning scene should not be empty. Would you like to rename "
                    "the planning scene?'"),
            QMessageBox::Cancel, this);
      else
        q = std::make_unique<QMessageBox>(QMessageBox::Question, "Confirm Planning Scene Overwrite",
                                          QString("A planning scene named '")
                                              .append(name.c_str())
                                              .append("' already exists. Do you wish to "
                                                      "overwrite that scene?"),
                                          QMessageBox::Yes | QMessageBox::No, this);
      std::unique_ptr<QPushButton> rename(q->addButton("&Rename", QMessageBox::AcceptRole));
      if (q->exec() != QMessageBox::Yes)
      {
        if (q->clickedButton() == rename.get())
        {
          bool ok = false;
          QString new_name = QInputDialog::getText(this, "Rename Planning Scene",
                                                   "New name for the planning scene:", QLineEdit::Normal,
                                                   QString::fromStdString(name), &ok);
          if (ok)
          {
            planning_display_->getPlanningSceneRW()->setName(new_name.toStdString());
            rviz::Property* prop = planning_display_->subProp("Scene Geometry")->subProp("Scene Name");
            if (prop)
            {
              bool old = prop->blockSignals(true);
              prop->setValue(new_name);
              prop->blockSignals(old);
            }
            saveSceneButtonClicked();
          }
          return;
        }
        return;
      }
    }

    planning_display_->addBackgroundJob([this] { computeSaveSceneButtonClicked(); }, "save scene");
  }
}

void MotionPlanningFrame::planningSceneItemClicked()
{
  checkPlanningSceneTreeEnabledButtons();
}

void MotionPlanningFrame::saveQueryButtonClicked()
{
  if (planning_scene_storage_)
  {
    QList<QTreeWidgetItem*> sel = ui_->planning_scene_tree->selectedItems();
    if (!sel.empty())
    {
      QTreeWidgetItem* s = sel.front();

      // if we have selected a PlanningScene, add the query as a new one, under that planning scene
      if (s->type() == ITEM_TYPE_SCENE)
      {
        std::string scene = s->text(0).toStdString();
        planning_display_->addBackgroundJob([this, scene] { computeSaveQueryButtonClicked(scene, ""); }, "save query");
      }
      else
      {
        // if we selected a query name, then we overwrite that query
        std::string scene = s->parent()->text(0).toStdString();
        std::string query_name = s->text(0).toStdString();

        while (query_name.empty() || planning_scene_storage_->hasPlanningQuery(scene, query_name))
        {
          std::unique_ptr<QMessageBox> q;
          if (query_name.empty())
            q = std::make_unique<QMessageBox>(
                QMessageBox::Question, "Change Planning Query Name",
                QString("The name for the planning query should not be empty. Would you like to "
                        "rename the planning query?'"),
                QMessageBox::Cancel, this);
          else
            q = std::make_unique<QMessageBox>(QMessageBox::Question, "Confirm Planning Query Overwrite",
                                              QString("A planning query named '")
                                                  .append(query_name.c_str())
                                                  .append("' already exists. Do you wish "
                                                          "to overwrite that query?"),
                                              QMessageBox::Yes | QMessageBox::No, this);
          std::unique_ptr<QPushButton> rename(q->addButton("&Rename", QMessageBox::AcceptRole));
          if (q->exec() == QMessageBox::Yes)
            break;
          else
          {
            if (q->clickedButton() == rename.get())
            {
              bool ok = false;
              QString new_name = QInputDialog::getText(this, "Rename Planning Query",
                                                       "New name for the planning query:", QLineEdit::Normal,
                                                       QString::fromStdString(query_name), &ok);
              if (ok)
                query_name = new_name.toStdString();
              else
                return;
            }
            else
              return;
          }
        }
        planning_display_->addBackgroundJob(
            [this, scene, query_name] { computeSaveQueryButtonClicked(scene, query_name); }, "save query");
      }
    }
  }
}

void MotionPlanningFrame::deleteSceneButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeDeleteSceneButtonClicked(); }, "delete scene");
}

void MotionPlanningFrame::deleteQueryButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeDeleteQueryButtonClicked(); }, "delete query");
}

void MotionPlanningFrame::loadSceneButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeLoadSceneButtonClicked(); }, "load scene");
}

void MotionPlanningFrame::loadQueryButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeLoadQueryButtonClicked(); }, "load query");
}

void MotionPlanningFrame::warehouseItemNameChanged(QTreeWidgetItem* item, int column)
{
  if (item->text(column) == item->toolTip(column) || item->toolTip(column).length() == 0)
    return;
  moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage = planning_scene_storage_;
  if (!planning_scene_storage)
    return;

  if (item->type() == ITEM_TYPE_SCENE)
  {
    std::string new_name = item->text(column).toStdString();

    if (planning_scene_storage->hasPlanningScene(new_name))
    {
      planning_display_->addMainLoopJob([this] { populatePlanningSceneTreeView(); });
      QMessageBox::warning(this, "Scene not renamed",
                           QString("The scene name '").append(item->text(column)).append("' already exists"));
      return;
    }
    else
    {
      std::string old_name = item->toolTip(column).toStdString();
      planning_scene_storage->renamePlanningScene(old_name, new_name);
      item->setToolTip(column, item->text(column));
    }
  }
  else
  {
    std::string scene = item->parent()->text(0).toStdString();
    std::string new_name = item->text(column).toStdString();
    if (planning_scene_storage->hasPlanningQuery(scene, new_name))
    {
      planning_display_->addMainLoopJob([this] { populatePlanningSceneTreeView(); });
      QMessageBox::warning(this, "Query not renamed",
                           QString("The query name '")
                               .append(item->text(column))
                               .append("' already exists for scene ")
                               .append(item->parent()->text(0)));
      return;
    }
    else
    {
      std::string old_name = item->toolTip(column).toStdString();
      planning_scene_storage->renamePlanningQuery(scene, old_name, new_name);
      item->setToolTip(column, item->text(column));
    }
  }
}

void MotionPlanningFrame::populatePlanningSceneTreeView()
{
  moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage = planning_scene_storage_;
  if (!planning_scene_storage)
    return;

  ui_->planning_scene_tree->setUpdatesEnabled(false);

  // remember which items were expanded
  std::set<std::string> expanded;
  for (int i = 0; i < ui_->planning_scene_tree->topLevelItemCount(); ++i)
  {
    QTreeWidgetItem* it = ui_->planning_scene_tree->topLevelItem(i);
    if (it->isExpanded())
      expanded.insert(it->text(0).toStdString());
  }

  ui_->planning_scene_tree->clear();
  std::vector<std::string> names;
  planning_scene_storage->getPlanningSceneNames(names);

  for (const std::string& name : names)
  {
    std::vector<std::string> query_names;
    planning_scene_storage->getPlanningQueriesNames(query_names, name);
    QTreeWidgetItem* item =
        new QTreeWidgetItem(ui_->planning_scene_tree, QStringList(QString::fromStdString(name)), ITEM_TYPE_SCENE);
    item->setFlags(item->flags() | Qt::ItemIsEditable);
    item->setToolTip(0, item->text(0));  // we use the tool tip as a backup of the old name when renaming
    for (const std::string& query_name : query_names)
    {
      QTreeWidgetItem* subitem =
          new QTreeWidgetItem(item, QStringList(QString::fromStdString(query_name)), ITEM_TYPE_QUERY);
      subitem->setFlags(subitem->flags() | Qt::ItemIsEditable);
      subitem->setToolTip(0, subitem->text(0));
      item->addChild(subitem);
    }

    ui_->planning_scene_tree->insertTopLevelItem(ui_->planning_scene_tree->topLevelItemCount(), item);
    if (expanded.find(name) != expanded.end())
      ui_->planning_scene_tree->expandItem(item);
  }
  ui_->planning_scene_tree->sortItems(0, Qt::AscendingOrder);
  ui_->planning_scene_tree->setUpdatesEnabled(true);
  checkPlanningSceneTreeEnabledButtons();
}
}  // namespace moveit_rviz_plugin
