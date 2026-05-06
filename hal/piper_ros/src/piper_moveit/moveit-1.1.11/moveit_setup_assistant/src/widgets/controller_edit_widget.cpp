/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Mohamad Ayman.
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
 *   * The name of Mohamad Ayman may be used to endorse or promote products derived
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

/* Author: Mohamad Ayman */

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include "controller_edit_widget.h"

namespace moveit_setup_assistant
{
// ******************************************************************************************
//  ControllerEditWidget constructor, create controller edit screen GUI
// ******************************************************************************************
ControllerEditWidget::ControllerEditWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : QWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  QGroupBox* controller_options_group = new QGroupBox("Controller Options");

  // Label ------------------------------------------------
  title_ = new QLabel(this);  // specify the title from the parent widget
  QFont group_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  title_->setFont(group_title_font);
  layout->addWidget(title_);

  QFormLayout* form_layout = new QFormLayout();
  form_layout->setContentsMargins(0, 15, 0, 15);

  // Controller Name
  controller_name_field_ = new QLineEdit(this);
  controller_name_field_->setMaximumWidth(400);
  form_layout->addRow("Controller Name:", controller_name_field_);

  // Controller Type
  controller_type_field_ = new QComboBox(this);
  controller_type_field_->setEditable(false);
  controller_type_field_->setMaximumWidth(400);
  form_layout->addRow("Controller Type:", controller_type_field_);

  controller_options_group->setLayout(form_layout);

  layout->addWidget(controller_options_group);

  layout->setAlignment(Qt::AlignTop);

  // New Controller Options  ---------------------------------------------------------
  new_buttons_widget_ = new QWidget();
  QVBoxLayout* new_buttons_layout = new QVBoxLayout();

  QLabel* save_and_add = new QLabel("Next, Add Components To Controller:", this);
  QFont save_and_add_font(QFont().defaultFamily(), 12, QFont::Bold);
  save_and_add->setFont(save_and_add_font);
  new_buttons_layout->addWidget(save_and_add);

  QLabel* add_subtitle = new QLabel("Recommended: ", this);
  QFont add_subtitle_font(QFont().defaultFamily(), 10, QFont::Bold);
  add_subtitle->setFont(add_subtitle_font);
  new_buttons_layout->addWidget(add_subtitle);

  // Save and add groups
  QPushButton* btn_save_groups_joints = new QPushButton("Add Planning Group Joints", this);
  btn_save_groups_joints->setMaximumWidth(200);
  connect(btn_save_groups_joints, SIGNAL(clicked()), this, SIGNAL(saveJointsGroups()));
  new_buttons_layout->addWidget(btn_save_groups_joints);

  QLabel* add_subtitle2 = new QLabel("Advanced Options:", this);
  add_subtitle2->setFont(add_subtitle_font);
  new_buttons_layout->addWidget(add_subtitle2);

  // Save and add joints
  QPushButton* btn_save_joints = new QPushButton("Add Individual Joints", this);
  btn_save_joints->setMaximumWidth(200);
  connect(btn_save_joints, SIGNAL(clicked()), this, SIGNAL(saveJoints()));
  new_buttons_layout->addWidget(btn_save_joints);

  // Create widget and add to main layout
  new_buttons_widget_->setLayout(new_buttons_layout);
  layout->addWidget(new_buttons_widget_);

  // Vertical Spacer
  layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding));

  // Bottom Controls ---------------------------------------------------------
  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Delete
  btn_delete_ = new QPushButton("&Delete Controller", this);
  btn_delete_->setMaximumWidth(200);
  connect(btn_delete_, SIGNAL(clicked()), this, SIGNAL(deleteController()));
  controls_layout->addWidget(btn_delete_);
  controls_layout->setAlignment(btn_delete_, Qt::AlignRight);

  // Horizontal Spacer
  controls_layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));

  // Save
  btn_save_ = new QPushButton("&Save", this);
  btn_save_->setMaximumWidth(200);
  connect(btn_save_, SIGNAL(clicked()), this, SIGNAL(save()));
  controls_layout->addWidget(btn_save_);
  controls_layout->setAlignment(btn_save_, Qt::AlignRight);

  // Cancel
  QPushButton* btn_cancel = new QPushButton("&Cancel", this);
  btn_cancel->setMaximumWidth(200);
  connect(btn_cancel, SIGNAL(clicked()), this, SIGNAL(cancelEditing()));
  controls_layout->addWidget(btn_cancel);
  controls_layout->setAlignment(btn_cancel, Qt::AlignRight);

  // Add layout
  layout->addLayout(controls_layout);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Set the fields with previous values
// ******************************************************************************************
void ControllerEditWidget::setSelected(const std::string& controller_name)
{
  controller_name_field_->setText(QString(controller_name.c_str()));
  moveit_setup_assistant::ControllerConfig* searched_controller = config_data_->findControllerByName(controller_name);
  if (searched_controller != nullptr)
  {
    const std::string controller_type = searched_controller->type_;
    int type_index = controller_type_field_->findText(controller_type.c_str());

    // Set the controller type combo box
    if (type_index == -1)
    {
      QMessageBox::warning(this, "Missing Controller Type", QString("Setting controller type to the default value"));
      return;
    }
    else
    {
      controller_type_field_->setCurrentIndex(type_index);
    }
  }
  else
  {
    controller_type_field_->setCurrentIndex(0);
  }
}

// ******************************************************************************************
// Populate the combo dropdown box with controllers types
// ******************************************************************************************
void ControllerEditWidget::loadControllersTypesComboBox()
{
  // Only load this combo box once
  if (has_loaded_)
    return;
  has_loaded_ = true;

  const std::vector<std::string> default_types = { "effort_controllers/JointTrajectoryController",
                                                   "velocity_controllers/JointTrajectoryController",
                                                   "position_controllers/JointTrajectoryController",
                                                   "FollowJointTrajectory", "GripperCommand" };

  // Remove all old items
  controller_type_field_->clear();

  // Loop through all controller default_types and add to combo box
  for (const std::string& type : default_types)
    controller_type_field_->addItem(type.c_str());
}

void ControllerEditWidget::hideDelete()
{
  btn_delete_->hide();
}

void ControllerEditWidget::hideSave()
{
  btn_save_->hide();
}

void ControllerEditWidget::hideNewButtonsWidget()
{
  new_buttons_widget_->hide();
}

void ControllerEditWidget::showDelete()
{
  btn_delete_->show();
}

void ControllerEditWidget::showSave()
{
  btn_save_->show();
}

void ControllerEditWidget::showNewButtonsWidget()
{
  new_buttons_widget_->show();
}

void ControllerEditWidget::setTitle(const QString& title)
{
  title_->setText(title);
}

std::string ControllerEditWidget::getControllerName()
{
  return controller_name_field_->text().trimmed().toStdString();
}

std::string ControllerEditWidget::getControllerType()
{
  return controller_type_field_->currentText().toStdString();
}

}  // namespace moveit_setup_assistant
