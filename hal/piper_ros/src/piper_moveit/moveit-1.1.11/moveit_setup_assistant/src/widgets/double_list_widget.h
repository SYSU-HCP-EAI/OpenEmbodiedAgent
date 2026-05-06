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
class QLabel;
class QTableWidget;
class QTableWidgetItem;
class QItemSelection;

#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

namespace moveit_setup_assistant
{
class DoubleListWidget : public QWidget
{
  Q_OBJECT

  // ******************************************************************************************
  // Reusable double list widget for selecting and deselecting a subset from a set
  // ******************************************************************************************
public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  DoubleListWidget(QWidget* parent, const MoveItConfigDataPtr& config_data, const QString& long_name,
                   const QString& short_name, bool add_ok_cancel = true);

  /// Loads the availble data list
  void setAvailable(const std::vector<std::string>& items);

  /// Set the right box
  void setSelected(const std::vector<std::string>& items);

  void clearContents();

  /// Convenience function for reusing set table code
  void setTable(const std::vector<std::string>& items, QTableWidget* table);

  /// Set the names of the two columns in the widget
  void setColumnNames(const QString& col1, const QString& col2);

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QTableWidget* data_table_;
  QTableWidget* selected_data_table_;
  QLabel* title_;  // specify the title from the parent widget
  QLabel* column1_label_;
  QLabel* column2_label_;

  /// Name of datatype
  QString long_name_;
  QString short_name_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Move selected data right
  void selectDataButtonClicked();

  /// Move selected data left
  void deselectDataButtonClicked();

  /// Event when data table is clicked
  void previewSelectedLeft(const QItemSelection& selected, const QItemSelection& deselected);
  void previewSelectedRight(const QItemSelection& selected, const QItemSelection& deselected);

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signals
  // ******************************************************************************************
  /// Event sent when this widget is done making data changes and parent widget can save
  void doneEditing();

  /// Event sent when user presses cancel button
  void cancelEditing();

  /// Signal to highlight parts of robot
  void previewSelected(std::vector<std::string> /*_t1*/);

  /// When the set of selected items has changed
  void selectionUpdated();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// Event when data table is clicked
  void previewSelected(const QList<QTableWidgetItem*>& selected);
};

}  // namespace moveit_setup_assistant
