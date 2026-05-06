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

// Qt
class QComboBox;
class QLineEdit;
class QPushButton;
class QStackedWidget;
class QTableWidget;

// SA
#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
class EndEffectorsWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  EndEffectorsWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);

  /// Received when this widget is chosen from the navigation menu
  void focusGiven() override;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QTableWidget* data_table_;
  QPushButton* btn_edit_;
  QPushButton* btn_delete_;
  QPushButton* btn_save_;
  QPushButton* btn_cancel_;
  QStackedWidget* stacked_widget_;
  QLineEdit* effector_name_field_;
  QComboBox* parent_name_field_;
  QComboBox* parent_group_name_field_;
  QComboBox* group_name_field_;
  QWidget* effector_list_widget_;
  QWidget* effector_edit_widget_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Show edit screen
  void showNewScreen();

  /// Edit whatever element is selected
  void editSelected();

  /// Edit the double clicked element
  void editDoubleClicked(int row, int column);

  /// Preview whatever element is selected
  void previewClicked(int row, int column);

  /// Preview the planning group that is selected
  void previewClickedString(const QString& name);

  /// Delete currently editing ite
  void deleteSelected();

  /// Save editing changes
  void doneEditing();

  /// Cancel changes
  void cancelEditing();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  /// Orignal name of effector currently being edited. This is used to find the element in the vector
  std::string current_edit_effector_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /**
   * Find the associated data by name
   *
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::EndEffector* findEffectorByName(const std::string& name);

  /**
   * Create the main list view of effectors for robot
   *
   * @return the widget
   */
  QWidget* createContentsWidget();

  /**
   * Create the screen for editing effectors
   *
   * @return the widget
   */
  QWidget* createEditWidget();

  /**
   * Load the robot effectors into the table
   *
   */
  void loadDataTable();

  /**
   * Populate the combo dropdown box with avail group names
   *
   */
  void loadGroupsComboBox();

  /**
   * Populate the combo dropdown box with avail parent links
   *
   */
  void loadParentComboBox();

  /**
   * Edit the effector with the input name
   *
   * @param name name of effector
   */
  void edit(const std::string& name);
};

}  // namespace moveit_setup_assistant
