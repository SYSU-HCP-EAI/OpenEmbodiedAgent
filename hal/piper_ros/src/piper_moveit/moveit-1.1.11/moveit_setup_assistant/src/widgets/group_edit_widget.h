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

/* Author: Dave Coleman */

#pragma once

#include <QWidget>
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;

#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

namespace moveit_setup_assistant
{
class GroupEditWidget : public QWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  GroupEditWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);

  /// Set the previous data
  void setSelected(const std::string& group_name);

  /// Populate the combo dropdown box with kinematic planners
  void loadKinematicPlannersComboBox();

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLabel* title_;  // specify the title from the parent widget
  QLineEdit* group_name_field_;
  QComboBox* kinematics_solver_field_;
  QLineEdit* kinematics_resolution_field_;
  QLineEdit* kinematics_timeout_field_;
  QLineEdit* kinematics_parameters_file_field_;
  QComboBox* default_planner_field_;
  QPushButton* btn_delete_;      // this button is hidden for new groups
  QPushButton* btn_save_;        // this button is hidden for new groups
  QWidget* new_buttons_widget_;  // for showing/hiding the new group buttons

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Shows a file dialog to select an additional parameter file for kinematics
  void selectKinematicsFile();

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signals
  // ******************************************************************************************

  /// Button event for new groups, progressing to adding joints
  void saveJoints();

  /// Button event for new groups, progressing to adding links
  void saveLinks();

  /// Button event for new groups, progressing to adding a chain
  void saveChain();

  /// Button event for new groups, progressing to adding subgroups
  void saveSubgroups();

  /// Button event for just saving, when in edit mode
  void save();

  /// Event sent when user presses cancel button
  void cancelEditing();

  /// Event sent when delete is being requested for group
  void deleteGroup();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************
};
}  // namespace moveit_setup_assistant
