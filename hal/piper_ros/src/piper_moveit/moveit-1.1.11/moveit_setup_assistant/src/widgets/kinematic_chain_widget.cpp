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

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QGridLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QLabel>
#include <QTreeWidget>
#include "kinematic_chain_widget.h"

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
KinematicChainWidget::KinematicChainWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : QWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Label ------------------------------------------------
  title_ = new QLabel("", this);  // specify the title from the parent widget
  QFont group_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  title_->setFont(group_title_font);
  layout->addWidget(title_);

  // Create link tree ------------------------------------------------------
  link_tree_ = new QTreeWidget(this);
  link_tree_->setHeaderLabel("Robot Links");
  connect(link_tree_, SIGNAL(itemSelectionChanged()), this, SLOT(itemSelected()));
  layout->addWidget(link_tree_);

  // Create Grid Layout for form --------------------------------------------
  QGridLayout* form_grid = new QGridLayout();
  form_grid->setContentsMargins(20, 20, 20, 20);  // left top right bottom

  // Row 1: Base Link
  QLabel* base_link_label = new QLabel("Base Link", this);
  form_grid->addWidget(base_link_label, 0, 0, Qt::AlignRight);

  base_link_field_ = new QLineEdit(this);
  base_link_field_->setMinimumWidth(300);
  form_grid->addWidget(base_link_field_, 0, 1, Qt::AlignLeft);

  QPushButton* btn_base_link = new QPushButton("Choose Selected", this);
  connect(btn_base_link, SIGNAL(clicked()), this, SLOT(baseLinkTreeClick()));
  form_grid->addWidget(btn_base_link, 0, 2, Qt::AlignLeft);

  // Row 2: Tip Link
  QLabel* tip_link_label = new QLabel("Tip Link", this);
  form_grid->addWidget(tip_link_label, 1, 0, Qt::AlignRight);

  tip_link_field_ = new QLineEdit(this);
  tip_link_field_->setMinimumWidth(300);
  form_grid->addWidget(tip_link_field_, 1, 1, Qt::AlignLeft);

  QPushButton* btn_tip_link = new QPushButton("Choose Selected", this);
  connect(btn_tip_link, SIGNAL(clicked()), this, SLOT(tipLinkTreeClick()));
  form_grid->addWidget(btn_tip_link, 1, 2, Qt::AlignLeft);

  // Add form grid layout
  layout->addLayout(form_grid);

  // Bottom Controls ---------------------------------------------------------
  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Expand/Contract controls
  QLabel* expand_controls = new QLabel(this);
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  connect(expand_controls, SIGNAL(linkActivated(const QString)), this, SLOT(alterTree(const QString)));
  controls_layout->addWidget(expand_controls);

  // Spacer
  controls_layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));

  // Save
  QPushButton* btn_save = new QPushButton("&Save", this);
  btn_save->setMaximumWidth(200);
  connect(btn_save, SIGNAL(clicked()), this, SIGNAL(doneEditing()));
  controls_layout->addWidget(btn_save);
  controls_layout->setAlignment(btn_save, Qt::AlignRight);

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

  // Remember that we have no loaded the chains yet
  kinematic_chain_loaded_ = false;
}

// ******************************************************************************************
// Load robot links into tree
// ******************************************************************************************
void KinematicChainWidget::setAvailable()
{
  // Only load the kinematic chain once
  if (kinematic_chain_loaded_)
    return;

  // Retrieve pointer to the shared kinematic model
  const moveit::core::RobotModelConstPtr& model = config_data_->getRobotModel();

  // Get the root joint
  const moveit::core::JointModel* root_joint = model->getRootJoint();

  addLinktoTreeRecursive(root_joint->getChildLinkModel(), nullptr);

  // Remember that we have loaded the chain
  kinematic_chain_loaded_ = true;
}

// ******************************************************************************************
//
// ******************************************************************************************
void KinematicChainWidget::addLinktoTreeRecursive(const moveit::core::LinkModel* link,
                                                  const moveit::core::LinkModel* parent)
{
  // Create new tree item
  QTreeWidgetItem* new_item = new QTreeWidgetItem(link_tree_);

  // Add item to tree
  if (parent == nullptr)
  {
    new_item->setText(0, link->getName().c_str());
    link_tree_->addTopLevelItem(new_item);
  }
  else
  {
    for (int i = 0; i < link_tree_->topLevelItemCount(); i++)
    {
      if (addLinkChildRecursive(link_tree_->topLevelItem(i), link, parent->getName()))
      {
        break;
      }
    }
  }
  for (std::size_t i = 0; i < link->getChildJointModels().size(); i++)
  {
    addLinktoTreeRecursive(link->getChildJointModels()[i]->getChildLinkModel(), link);
  }
}

// ******************************************************************************************
//
// ******************************************************************************************
bool KinematicChainWidget::addLinkChildRecursive(QTreeWidgetItem* parent, const moveit::core::LinkModel* link,
                                                 const std::string& parent_name)
{
  if (parent->text(0).toStdString() == parent_name)
  {
    QTreeWidgetItem* new_item = new QTreeWidgetItem(parent);
    new_item->setText(0, link->getName().c_str());

    parent->addChild(new_item);
    return true;
  }
  else
  {
    for (int i = 0; i < parent->childCount(); i++)
    {
      if (addLinkChildRecursive(parent->child(i), link, parent_name))
      {
        return true;
      }
    }
  }

  return false;
}

// ******************************************************************************************
// Set the link field with previous value
// ******************************************************************************************
void KinematicChainWidget::setSelected(const std::string& base_link, const std::string& tip_link)
{
  base_link_field_->setText(QString(base_link.c_str()));
  tip_link_field_->setText(QString(tip_link.c_str()));
}

// ******************************************************************************************
// Choose the base link
// ******************************************************************************************
void KinematicChainWidget::baseLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if (item != nullptr)
  {
    base_link_field_->setText(item->text(0));
  }
}

// ******************************************************************************************
// Choose the tip link
// ******************************************************************************************
void KinematicChainWidget::tipLinkTreeClick()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if (item != nullptr)
  {
    tip_link_field_->setText(item->text(0));
  }
}

// ******************************************************************************************
// Expand/Collapse Tree
// ******************************************************************************************
void KinematicChainWidget::alterTree(const QString& link)
{
  if (link.contains("expand"))
    link_tree_->expandAll();
  else
    link_tree_->collapseAll();
}

// ******************************************************************************************
// Highlight the currently selected link
// ******************************************************************************************
void KinematicChainWidget::itemSelected()
{
  QTreeWidgetItem* item = link_tree_->currentItem();
  if (item != nullptr)
  {
    Q_EMIT unhighlightAll();

    std::string name = item->text(0).toStdString();

    // Don't try to highlight empty links!
    if (name.empty())
      return;

    // Check that item is not empty
    Q_EMIT highlightLink(item->text(0).toStdString(), QColor(255, 0, 0));
  }
}
}  // namespace moveit_setup_assistant
