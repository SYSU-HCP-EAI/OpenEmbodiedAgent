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

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QTableWidget>
#include <QVBoxLayout>
#include "double_list_widget.h"

namespace moveit_setup_assistant
{
// ******************************************************************************************
//
// ******************************************************************************************
DoubleListWidget::DoubleListWidget(QWidget* parent, const MoveItConfigDataPtr& config_data, const QString& long_name,
                                   const QString& short_name, bool add_ok_cancel)
  : QWidget(parent), long_name_(long_name), short_name_(short_name), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Label ------------------------------------------------
  title_ = new QLabel("", this);  // specify the title from the parent widget
  QFont group_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  title_->setFont(group_title_font);
  layout->addWidget(title_);

  // Double selection lists -------------------------------
  QHBoxLayout* hlayout = new QHBoxLayout();

  // Left column -------------------------------------------
  QVBoxLayout* column1 = new QVBoxLayout();

  // Label
  column1_label_ = new QLabel(QString("Available ").append(short_name_).append('s'), this);
  column1->addWidget(column1_label_);

  // Table
  data_table_ = new QTableWidget(this);
  data_table_->setColumnCount(1);
  data_table_->setSortingEnabled(true);
  column1->addWidget(data_table_);
  connect(data_table_->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this,
          SLOT(previewSelectedLeft(QItemSelection, QItemSelection)));

  // Table headers
  QStringList data_header_list;
  data_header_list.append(QString(" Names").prepend(short_name_));
  data_table_->setHorizontalHeaderLabels(data_header_list);
  data_table_->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);

  // Add layouts
  hlayout->addLayout(column1);

  // Center column ------------------------------------------
  QVBoxLayout* column2 = new QVBoxLayout();
  column2->setSizeConstraint(QLayout::SetFixedSize);  // constraint it

  // Right Arrow Button
  QPushButton* btn_right = new QPushButton(">", this);
  btn_right->setMaximumSize(25, 80);
  connect(btn_right, SIGNAL(clicked()), this, SLOT(selectDataButtonClicked()));
  column2->addWidget(btn_right);

  // Left Arrow Button
  QPushButton* btn_left = new QPushButton("<", this);
  btn_left->setMaximumSize(25, 80);
  connect(btn_left, SIGNAL(clicked()), this, SLOT(deselectDataButtonClicked()));
  column2->addWidget(btn_left);

  // Add layouts
  hlayout->addLayout(column2);

  // Right column -------------------------------------------
  QVBoxLayout* column3 = new QVBoxLayout();

  // Label
  column2_label_ = new QLabel(QString("Selected ").append(short_name_).append("s"), this);
  column3->addWidget(column2_label_);

  // Table
  selected_data_table_ = new QTableWidget(this);
  selected_data_table_->setColumnCount(1);
  selected_data_table_->setSortingEnabled(true);
  column3->addWidget(selected_data_table_);
  connect(selected_data_table_->selectionModel(), SIGNAL(selectionChanged(QItemSelection, QItemSelection)), this,
          SLOT(previewSelectedRight(QItemSelection, QItemSelection)));

  // Table Headers (use same)
  selected_data_table_->setHorizontalHeaderLabels(data_header_list);

  // Add layouts
  hlayout->addLayout(column3);

  // End Double Selection List ---------------------------------
  layout->addLayout(hlayout);

  if (add_ok_cancel)
  {
    // Button controls -------------------------------------------
    QHBoxLayout* controls_layout = new QHBoxLayout();
    controls_layout->setContentsMargins(0, 25, 0, 15);

    // Spacer
    controls_layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));

    // Save
    QPushButton* btn_save = new QPushButton("&Save", this);
    // btn_save->setMaximumWidth( 200 );
    connect(btn_save, SIGNAL(clicked()), this, SIGNAL(doneEditing()));
    controls_layout->addWidget(btn_save);
    controls_layout->setAlignment(btn_save, Qt::AlignRight);

    // Cancel
    QPushButton* btn_cancel = new QPushButton("&Cancel", this);
    // btn_cancel->setMaximumWidth( 200 );
    connect(btn_cancel, SIGNAL(clicked()), this, SIGNAL(cancelEditing()));
    controls_layout->addWidget(btn_cancel);
    controls_layout->setAlignment(btn_cancel, Qt::AlignRight);

    // Add layout
    layout->addLayout(controls_layout);
  }

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Set the left box
// ******************************************************************************************
void DoubleListWidget::setAvailable(const std::vector<std::string>& items)
{
  setTable(items, data_table_);

  // Resize both tables
  data_table_->resizeColumnToContents(0);
  selected_data_table_->setColumnWidth(0, data_table_->columnWidth(0));
}

// ******************************************************************************************
// Set the right box
// ******************************************************************************************
void DoubleListWidget::setSelected(const std::vector<std::string>& items)
{
  setTable(items, selected_data_table_);
}

void DoubleListWidget::clearContents()
{
  selected_data_table_->clearContents();
  data_table_->clearContents();
}

void DoubleListWidget::setColumnNames(const QString& col1, const QString& col2)
{
  column1_label_->setText(col1);
  column2_label_->setText(col2);
}

// ******************************************************************************************
// Convenience function for reusing set table code
// ******************************************************************************************
void DoubleListWidget::setTable(const std::vector<std::string>& items, QTableWidget* table)
{
  // Disable Table
  table->setUpdatesEnabled(false);  // prevent table from updating until we are completely done
  table->setDisabled(true);         // make sure we disable it so that the cellChanged event is not called
  table->clearContents();

  // Set size of datatable
  table->setRowCount(items.size());

  // Loop through every item
  int row = 0;
  for (const std::string& item : items)
  {
    // This is a hack to prevent a dummy joint from being added. Not really the best place to place this but
    // here is computationally smart
    if (item == "ASSUMED_FIXED_ROOT_JOINT")
      continue;

    // Create row elements
    QTableWidgetItem* data_name = new QTableWidgetItem(item.c_str());
    data_name->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);

    // Add to table
    table->setItem(row, 0, data_name);

    // Increment counter
    ++row;
  }

  table->setRowCount(row);

  // Reenable
  table->setUpdatesEnabled(true);  // prevent table from updating until we are completely done
  table->setDisabled(false);       // make sure we disable it so that the cellChanged event is not called
}

// ******************************************************************************************
// Move selected data right
// ******************************************************************************************
void DoubleListWidget::selectDataButtonClicked()
{
  // Get list of all selected items
  QList<QTableWidgetItem*> selected = data_table_->selectedItems();

  // Loop through all selected items
  for (QTableWidgetItem* item : selected)
  {
    std::string name = item->text().toStdString();
    bool already_exists = false;
    int row_to_add = 0;

    // Check if this selected joint is already in the selected joint table
    for (int r = 0; r < selected_data_table_->rowCount(); r++)
    {
      QTableWidgetItem* item = selected_data_table_->item(r, 0);

      if (item->text().toStdString() == name)
      {
        already_exists = true;
        break;
      }
      row_to_add = r + 1;
    }

    // This joint needs to be added to the selected joint table
    if (!already_exists)
    {
      selected_data_table_->setRowCount(selected_data_table_->rowCount() + 1);
      QTableWidgetItem* new_item = new QTableWidgetItem(name.c_str());
      new_item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      selected_data_table_->setItem(row_to_add, 0, new_item);
    }
  }

  Q_EMIT(selectionUpdated());
}

// ******************************************************************************************
// Move selected data left
// ******************************************************************************************
void DoubleListWidget::deselectDataButtonClicked()
{
  // Get list of joints to be removed from selected list
  QList<QTableWidgetItem*> deselected = selected_data_table_->selectedItems();

  // loop through deselect list and remove
  for (QTableWidgetItem* item : deselected)
  {
    selected_data_table_->removeRow(item->row());
  }

  Q_EMIT(selectionUpdated());
}

// ******************************************************************************************
// Highlight links of robot for left list
// ******************************************************************************************
void DoubleListWidget::previewSelectedLeft(const QItemSelection& /*selected*/, const QItemSelection& /*deselected*/)
{
  const QList<QTableWidgetItem*> selected_items = data_table_->selectedItems();
  previewSelected(selected_items);
}

// ******************************************************************************************
// Highlight links of robot for right list
// ******************************************************************************************
void DoubleListWidget::previewSelectedRight(const QItemSelection& /*selected*/, const QItemSelection& /*deselected*/)
{
  const QList<QTableWidgetItem*> selected_items = selected_data_table_->selectedItems();
  previewSelected(selected_items);
}

// ******************************************************************************************
// Highlight links of robot
// ******************************************************************************************
void DoubleListWidget::previewSelected(const QList<QTableWidgetItem*>& selected)
{
  // Check that an element was selected
  if (selected.empty())
    return;

  std::vector<std::string> selected_vector;

  // Convert QList to std vector
  selected_vector.reserve(selected.size());
  for (QTableWidgetItem* item : selected)
    selected_vector.emplace_back(item->text().toStdString());

  // Send to shared function
  Q_EMIT(previewSelected(selected_vector));
}

}  // namespace moveit_setup_assistant
