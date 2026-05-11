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

// SA
#include "controllers_widget.h"
#include "double_list_widget.h"
#include "controller_edit_widget.h"
#include "header_widget.h"
#include <moveit_msgs/JointLimits.h>
// Qt
#include <QApplication>
#include <QDoubleValidator>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QString>
#include <QTableWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <regex>
#include <moveit/robot_state/conversions.h>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
ControllersWidget::ControllersWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  layout->setAlignment(Qt::AlignTop);

  // Title
  this->setWindowTitle("Controller Configuration");  // title of window

  // Top Header Area ------------------------------------------------
  moveit_setup_assistant::HeaderWidget* header = new moveit_setup_assistant::HeaderWidget(
      "Setup Controllers",
      "Configure controllers to be used by MoveIt's controller manager(s) to operate the robot's physical hardware",
      this);
  layout->addWidget(header);

  // Tree Box ----------------------------------------------------------------------
  controllers_tree_widget_ = createContentsWidget();

  // Joints edit widget
  joints_widget_ = new moveit_setup_assistant::DoubleListWidget(this, config_data_, "Joint Collection", "Joint");
  connect(joints_widget_, SIGNAL(cancelEditing()), this, SLOT(cancelEditing()));
  connect(joints_widget_, SIGNAL(doneEditing()), this, SLOT(saveJointsScreen()));
  connect(joints_widget_, SIGNAL(previewSelected(std::vector<std::string>)), this,
          SLOT(previewSelectedJoints(std::vector<std::string>)));

  // Joints Groups Widget
  joint_groups_widget_ =
      new moveit_setup_assistant::DoubleListWidget(this, config_data_, "Group Joints Collection", "Group");
  connect(joint_groups_widget_, SIGNAL(cancelEditing()), this, SLOT(cancelEditing()));
  connect(joint_groups_widget_, SIGNAL(doneEditing()), this, SLOT(saveJointsGroupsScreen()));
  connect(joint_groups_widget_, SIGNAL(previewSelected(std::vector<std::string>)), this,
          SLOT(previewSelectedGroup(std::vector<std::string>)));

  // Controller Edit Widget
  controller_edit_widget_ = new ControllerEditWidget(this, config_data_);
  connect(controller_edit_widget_, SIGNAL(cancelEditing()), this, SLOT(cancelEditing()));
  connect(controller_edit_widget_, SIGNAL(deleteController()), this, SLOT(deleteController()));
  connect(controller_edit_widget_, SIGNAL(save()), this, SLOT(saveControllerScreenEdit()));
  connect(controller_edit_widget_, SIGNAL(saveJoints()), this, SLOT(saveControllerScreenJoints()));
  connect(controller_edit_widget_, SIGNAL(saveJointsGroups()), this, SLOT(saveControllerScreenGroups()));

  // Combine into stack
  stacked_widget_ = new QStackedWidget(this);
  stacked_widget_->addWidget(controllers_tree_widget_);  // screen index 0
  stacked_widget_->addWidget(joints_widget_);            // screen index 1
  stacked_widget_->addWidget(controller_edit_widget_);   // screen index 2
  stacked_widget_->addWidget(joint_groups_widget_);      // screen index 3
  layout->addWidget(stacked_widget_);

  this->setLayout(layout);
}

// ******************************************************************************************
// Create the main tree view widget
// ******************************************************************************************
QWidget* ControllersWidget::createContentsWidget()
{
  // Main widget
  QWidget* content_widget = new QWidget(this);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  QHBoxLayout* upper_controls_layout = new QHBoxLayout();

  // Add default controller
  QPushButton* btn_add_default =
      new QPushButton("Auto Add &FollowJointsTrajectory \n Controllers For Each Planning Group", this);
  btn_add_default->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_add_default->setMaximumWidth(600);
  connect(btn_add_default, SIGNAL(clicked()), this, SLOT(addDefaultControllers()));
  upper_controls_layout->addWidget(btn_add_default);
  upper_controls_layout->setAlignment(btn_add_default, Qt::AlignLeft);

  // Add Controls to layout
  layout->addLayout(upper_controls_layout);

  // Tree Box ----------------------------------------------------------------------
  controllers_tree_ = new QTreeWidget(this);
  controllers_tree_->setColumnCount(2);
  QStringList labels;
  labels << "Controller"
         << "Controller Type";
  controllers_tree_->setHeaderLabels(labels);
  controllers_tree_->setColumnWidth(0, 250);
  connect(controllers_tree_, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this, SLOT(editSelected()));
  connect(controllers_tree_, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this,
          SLOT(previewSelected(QTreeWidgetItem*, int)));
  connect(controllers_tree_, SIGNAL(itemSelectionChanged()), this, SLOT(itemSelectionChanged()));
  layout->addWidget(controllers_tree_);

  // Bottom Controls -------------------------------------------------------------

  controls_layout_ = new QHBoxLayout();

  // Expand/Contract controls
  QLabel* expand_controls = new QLabel(this);
  expand_controls->setText("<a href='expand'>Expand All</a> <a href='contract'>Collapse All</a>");
  connect(expand_controls, SIGNAL(linkActivated(const QString)), this, SLOT(alterTree(const QString)));
  controls_layout_->addWidget(expand_controls);

  // Spacer
  controls_layout_->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));

  // Delete
  btn_delete_ = new QPushButton("&Delete Controller", this);
  btn_delete_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_delete_->setMaximumWidth(200);
  connect(btn_delete_, SIGNAL(clicked()), this, SLOT(deleteController()));
  controls_layout_->addWidget(btn_delete_);
  controls_layout_->setAlignment(btn_delete_, Qt::AlignRight);

  // Add Controller Button
  btn_add_ = new QPushButton("&Add Controller", this);
  btn_add_->setMaximumWidth(300);
  connect(btn_add_, SIGNAL(clicked()), this, SLOT(addController()));
  controls_layout_->addWidget(btn_add_);
  controls_layout_->setAlignment(btn_add_, Qt::AlignRight);

  // Edit
  btn_edit_ = new QPushButton("&Edit Selected", this);
  btn_edit_->setMaximumWidth(300);
  connect(btn_edit_, SIGNAL(clicked()), this, SLOT(editSelected()));
  controls_layout_->addWidget(btn_edit_);
  controls_layout_->setAlignment(btn_edit_, Qt::AlignRight);

  // Add Controls to layout
  layout->addLayout(controls_layout_);

  // Set layout
  content_widget->setLayout(layout);

  return content_widget;
}

// ******************************************************************************************
// Displays data in the controller_configs_ data structure into a QtTableWidget
// ******************************************************************************************
void ControllersWidget::loadControllersTree()
{
  // Disable Tree
  controllers_tree_->setUpdatesEnabled(false);  // prevent table from updating until we are completely done
  controllers_tree_->setDisabled(true);         // make sure we disable it so that the cellChanged event is not called
  controllers_tree_->clear();                   // reset the tree

  // Display all controllers by looping through them
  for (ControllerConfig& controller : config_data_->getControllers())
  {
    loadToControllersTree(controller);
  }

  // Reenable Tree
  controllers_tree_->setUpdatesEnabled(true);  // prevent table from updating until we are completely done
  controllers_tree_->setDisabled(false);       // make sure we disable it so that the cellChanged event is not called
  current_edit_controller_.clear();            // no controller is being edited
  alterTree("expand");
}

// ******************************************************************************************
//  Displays data in the controller_config_ data structure into a QtTableWidget
// ******************************************************************************************
void ControllersWidget::loadToControllersTree(const moveit_setup_assistant::ControllerConfig& controller_it)
{
  // Fonts for tree
  const QFont top_level_font(QFont().defaultFamily(), 11, QFont::Bold);
  const QFont type_font(QFont().defaultFamily(), 11, QFont::Normal, QFont::StyleItalic);

  QTreeWidgetItem* controller;

  controller = new QTreeWidgetItem();

  // First column
  controller->setText(0, controller_it.name_.c_str());
  controller->setFont(0, top_level_font);
  controller->setData(0, Qt::UserRole, QVariant::fromValue(0));

  // Second column
  controller->setText(1, controller_it.type_.c_str());
  controller->setFont(1, type_font);
  controller->setData(1, Qt::UserRole, QVariant::fromValue(4));
  controllers_tree_->addTopLevelItem(controller);

  if (!controller_it.joints_.empty())
  {
    // Joints --------------------------------------------------------------
    QTreeWidgetItem* joints = new QTreeWidgetItem(controller);
    joints->setText(0, "Joints");
    joints->setFont(0, type_font);
    joints->setData(0, Qt::UserRole, QVariant::fromValue(1));
    controller->addChild(joints);

    // Loop through all available joints
    for (const std::string& joint : controller_it.joints_)
    {
      QTreeWidgetItem* joint_item = new QTreeWidgetItem(joints);
      joint_item->setData(0, Qt::UserRole, QVariant::fromValue(2));

      // Add to tree
      joint_item->setText(0, joint.c_str());
      joints->addChild(joint_item);
    }
  }
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void ControllersWidget::focusGiven()
{
  // load controllers tree
  btn_edit_->setEnabled(false);
  btn_delete_->setEnabled(false);
  loadControllersTree();
}

// ******************************************************************************************
// Load the popup screen with correct data for joints
// ******************************************************************************************
void ControllersWidget::loadJointsScreen(moveit_setup_assistant::ControllerConfig* this_controller)
{
  // Retrieve pointer to the shared kinematic model
  const moveit::core::RobotModelConstPtr& model = config_data_->getRobotModel();

  // Get the names of the all joints
  const std::vector<std::string>& joints = model->getJointModelNames();

  if (joints.empty())
  {
    QMessageBox::critical(this, "Error Loading", "No joints found for robot model");
    return;
  }

  // Set the available joints (left box)
  joints_widget_->setAvailable(joints);

  // Set the selected joints (right box)
  joints_widget_->setSelected(this_controller->joints_);

  // Set the title
  joints_widget_->title_->setText(
      QString("Edit '").append(QString::fromUtf8(this_controller->name_.c_str())).append("' Joint Collection"));

  // Remember what is currently being edited so we can later save changes
  current_edit_controller_ = this_controller->name_;
}

// ******************************************************************************************
// Load the popup screen with correct data for gcroups
// ******************************************************************************************
void ControllersWidget::loadGroupsScreen(moveit_setup_assistant::ControllerConfig* this_controller)
{
  // Load all groups into the subgroup screen
  std::vector<std::string> groups;

  // Display all groups by looping through them
  for (const srdf::Model::Group& group : config_data_->srdf_->srdf_model_->getGroups())
  {
    // Add to available groups list
    groups.push_back(group.name_);
  }

  // Set the available groups (left box)
  joint_groups_widget_->setAvailable(groups);

  // Set the selected groups (right box)
  joint_groups_widget_->setSelected(this_controller->joints_);

  // Set the title
  joint_groups_widget_->title_->setText(
      QString("Edit '").append(QString::fromUtf8(this_controller->name_.c_str())).append("' Joints groups collection"));

  // Remember what is currently being edited so we can later save changes
  current_edit_controller_ = this_controller->name_;
}

// ******************************************************************************************
// Delete a controller
// ******************************************************************************************
void ControllersWidget::deleteController()
{
  std::string controller_name = current_edit_controller_;

  if (controller_name.empty())
  {
    QTreeWidgetItem* item = controllers_tree_->currentItem();
    // Check that something was actually selected
    if (item == nullptr)
      return;

    // Get the user custom properties of the currently selected row
    int type = item->data(0, Qt::UserRole).value<int>();
    if (type == 0)
      controller_name = item->text(0).toUtf8().constData();
  }

  // Confirm user wants to delete controller
  if (QMessageBox::question(
          this, "Confirm Controller Deletion",
          QString("Are you sure you want to delete the controller '").append(controller_name.c_str()).append(" ?"),
          QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
  {
    return;
  }
  // Delete actual controller
  if (config_data_->deleteController(controller_name))
  {
    ROS_INFO_STREAM_NAMED("Setup Assistant", "Controller " << controller_name << " deleted succefully");
  }
  else
  {
    ROS_WARN_STREAM_NAMED("Setup Assistant", "Couldn't delete Controller " << controller_name);
  }

  current_edit_controller_.clear();

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadControllersTree();
}

// ******************************************************************************************
// Create a new controller
// ******************************************************************************************
void ControllersWidget::addController()
{
  // New Controller
  adding_new_controller_ = true;

  // Load the data
  loadControllerScreen(nullptr);  // NULL indicates this is a new controller, not an existing one
  changeScreen(2);                // 1 is index of controller edit
}

// ******************************************************************************************
// Add a Follow Joint Trajectory action Controller for each Planning Group
// ******************************************************************************************
void ControllersWidget::addDefaultControllers()
{
  if (!config_data_->addDefaultControllers())
    QMessageBox::warning(this, "Error adding contollers", "No Planning Groups configured!");
  loadControllersTree();
}

// ******************************************************************************************
// Load the popup screen with correct data for controllers
// ******************************************************************************************
void ControllersWidget::loadControllerScreen(moveit_setup_assistant::ControllerConfig* this_controller)
{
  // Load the avail controllers. This function only runs once
  controller_edit_widget_->loadControllersTypesComboBox();

  if (this_controller == nullptr)  // this is a new screen
  {
    current_edit_controller_.clear();  // provide a blank controller name
    controller_edit_widget_->setTitle("Create New Controller");
    controller_edit_widget_->hideDelete();
    controller_edit_widget_->showNewButtonsWidget();  // helps user choose next step
    controller_edit_widget_->showSave();              // this is only for edit mode
  }
  else  // load the controller name into the widget
  {
    current_edit_controller_ = this_controller->name_;
    controller_edit_widget_->setTitle(QString("Edit Controller '").append(current_edit_controller_.c_str()).append("'"));
    controller_edit_widget_->showDelete();
    controller_edit_widget_->hideNewButtonsWidget();  // not necessary for existing controllers
    controller_edit_widget_->showSave();              // this is only for edit mode
  }

  // Set the data in the edit box
  controller_edit_widget_->setSelected(current_edit_controller_);
}

// ******************************************************************************************
// Call when edit screen is canceled
// ******************************************************************************************
void ControllersWidget::cancelEditing()
{
  if (!current_edit_controller_.empty() && adding_new_controller_)
  {
    moveit_setup_assistant::ControllerConfig* editing = config_data_->findControllerByName(current_edit_controller_);
    if (editing && editing->joints_.empty())
    {
      config_data_->deleteController(current_edit_controller_);
      current_edit_controller_.clear();

      // Load the data to the tree
      loadControllersTree();
    }
  }
  else
  {
    current_edit_controller_.clear();
  }

  // Switch to main screen
  showMainScreen();
}

// ******************************************************************************************
// Called from Double List widget to highlight joints
// ******************************************************************************************
void ControllersWidget::previewSelectedJoints(const std::vector<std::string>& joints)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for (const std::string& joint : joints)
  {
    const moveit::core::JointModel* joint_model = config_data_->getRobotModel()->getJointModel(joint);

    // Check that a joint model was found
    if (!joint_model)
    {
      continue;
    }

    // Get the name of the link
    const std::string link = joint_model->getChildLinkModel()->getName();

    if (link.empty())
    {
      continue;
    }

    // Highlight link
    Q_EMIT highlightLink(link, QColor(255, 0, 0));
  }
}

// ******************************************************************************************
// Called from Double List widget to highlight a group
// ******************************************************************************************
void ControllersWidget::previewSelectedGroup(const std::vector<std::string>& groups)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for (const std::string& group : groups)
  {
    // Highlight group
    Q_EMIT highlightGroup(group);
  }
}

// ******************************************************************************************
// Called when an item is seleceted from the controllers tree
// ******************************************************************************************
void ControllersWidget::previewSelected(QTreeWidgetItem* selected_item, int /*column*/)
{
  // Get the user custom properties of the currently selected row
  int type = selected_item->data(0, Qt::UserRole).value<int>();
  btn_edit_->setEnabled(true);

  // Enable delete button if a controller was selected
  if (type == 0)
  {
    btn_delete_->setEnabled(true);
  }
  else
  {
    btn_delete_->setEnabled(false);
  }
}

// ******************************************************************************************
// Call when a new controller is created and ready to progress to next screen
// ******************************************************************************************
void ControllersWidget::saveControllerScreenJoints()
{
  // Save the controller
  if (!saveControllerScreen())
    return;

  // Find the controller we are editing based on the controller name string
  moveit_setup_assistant::ControllerConfig* editing_controller =
      config_data_->findControllerByName(current_edit_controller_);

  loadJointsScreen(editing_controller);

  // Switch to screen
  changeScreen(1);  // 1 is index of joints
}

// ******************************************************************************************
// Call when a new controller is created and ready to progress to next screen
// ******************************************************************************************
void ControllersWidget::saveControllerScreenGroups()
{
  // Save the controller
  if (!saveControllerScreen())
    return;

  // Find the controller we are editing based on the controller name string
  moveit_setup_assistant::ControllerConfig* editing_controller =
      config_data_->findControllerByName(current_edit_controller_);

  loadGroupsScreen(editing_controller);

  // Switch to screen
  changeScreen(3);  // 3 is index of groups
}

// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void ControllersWidget::saveJointsScreen()
{
  // Find the controller we are editing based on the controller name string
  moveit_setup_assistant::ControllerConfig* searched_controller =
      config_data_->findControllerByName(current_edit_controller_);

  // Clear the old data
  searched_controller->joints_.clear();

  // Copy the data
  for (int i = 0; i < joints_widget_->selected_data_table_->rowCount(); ++i)
  {
    searched_controller->joints_.push_back(joints_widget_->selected_data_table_->item(i, 0)->text().toStdString());
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadControllersTree();
}

// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void ControllersWidget::saveJointsGroupsScreen()
{
  // Find the controller we are editing based on the controller name string
  moveit_setup_assistant::ControllerConfig* searched_controller =
      config_data_->findControllerByName(current_edit_controller_);

  // Clear the old data
  searched_controller->joints_.clear();

  // Copy the data
  for (int i = 0; i < joint_groups_widget_->selected_data_table_->rowCount(); ++i)
  {
    // Get list of associated joints
    const moveit::core::JointModelGroup* joint_model_group = config_data_->getRobotModel()->getJointModelGroup(
        joint_groups_widget_->selected_data_table_->item(i, 0)->text().toStdString());
    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();

    // Iterate through the joints
    for (const moveit::core::JointModel* joint : joint_models)
    {
      if (joint->isPassive() || joint->getMimic() != nullptr || joint->getType() == moveit::core::JointModel::FIXED)
        continue;
      searched_controller->joints_.push_back(joint->getName());
    }
  }

  // Switch to main screen
  showMainScreen();

  // Reload main screen table
  loadControllersTree();
}

// ******************************************************************************************
// Call when joints edit sceen is done and needs to be saved
// ******************************************************************************************
void ControllersWidget::saveControllerScreenEdit()
{
  // Save the controller
  if (!saveControllerScreen())
    return;

  // Switch to main screen
  showMainScreen();
}

// ******************************************************************************************
// Call when controller edit sceen is done and needs to be saved
// ******************************************************************************************
bool ControllersWidget::saveControllerScreen()
{
  // Get a reference to the supplied strings
  const std::string& controller_name = controller_edit_widget_->getControllerName();
  const std::string& controller_type = controller_edit_widget_->getControllerType();

  // Used for editing existing controllers
  moveit_setup_assistant::ControllerConfig* searched_controller = nullptr;

  std::smatch invalid_name_match;
  std::regex invalid_reg_ex("[^a-z|^1-9|^_]");

  // Check that a valid controller name has been given
  if (controller_name.empty() || std::regex_search(controller_name, invalid_name_match, invalid_reg_ex))
  {
    QMessageBox::warning(this, "Error Saving", "Invalid controller name");
    return false;
  }

  // Check if this is an existing controller
  if (!current_edit_controller_.empty())
  {
    // Find the controller we are editing based on the goup name string
    searched_controller = config_data_->findControllerByName(current_edit_controller_);
  }

  // Check that the controller name is unique
  for (const auto& controller : config_data_->getControllers())
  {
    if (controller.name_.compare(controller_name) == 0)  // the names are the same
    {
      // is this our existing controller? check if controller pointers are same
      if (&controller != searched_controller)
      {
        QMessageBox::warning(this, "Error Saving", "A controller already exists with that name!");
        return false;
      }
    }
  }

  adding_new_controller_ = false;

  // Save the new controller name or create the new controller
  if (searched_controller == nullptr)  // create new
  {
    moveit_setup_assistant::ControllerConfig new_controller;
    new_controller.name_ = controller_name;
    new_controller.type_ = controller_type;
    config_data_->addController(new_controller);

    adding_new_controller_ = true;  // remember this controller is new
  }
  else
  {
    // Remember old controller name and type
    const std::string old_controller_name = searched_controller->name_;

    // Change controller name
    searched_controller->name_ = controller_name;
    searched_controller->type_ = controller_type;
  }

  // Reload main screen table
  loadControllersTree();

  // Update the current edit controller so that we can proceed to the next screen, if user desires
  current_edit_controller_ = controller_name;

  return true;
}

// ******************************************************************************************
// Edit whenever element is selected in the controllers tree view
// ******************************************************************************************
void ControllersWidget::editSelected()
{
  QTreeWidgetItem* item = controllers_tree_->currentItem();
  QTreeWidgetItem* controller_item;

  // Check that something was actually selected
  if (item == nullptr)
    return;

  adding_new_controller_ = false;

  // Get the user custom properties of the currently selected row
  int type = item->data(0, Qt::UserRole).value<int>();

  if (type == 2)
  {
    // The controller this joint belong to
    controller_item = item->parent()->parent();
    current_edit_controller_ = controller_item->text(0).toUtf8().constData();
    moveit_setup_assistant::ControllerConfig* this_controller =
        config_data_->findControllerByName(current_edit_controller_);

    // Load the data
    loadJointsScreen(this_controller);

    // Switch to screen
    changeScreen(1);  // 1 is index of joints
  }
  else if (type == 1)
  {
    controller_item = item->parent();
    current_edit_controller_ = controller_item->text(0).toUtf8().constData();
    moveit_setup_assistant::ControllerConfig* this_controller =
        config_data_->findControllerByName(current_edit_controller_);

    // Load the data
    loadJointsScreen(this_controller);

    // Switch to screen
    changeScreen(1);  // 1 is index of joints
  }
  else if (type == 0)
  {
    // Load the data
    current_edit_controller_ = item->text(0).toUtf8().constData();
    moveit_setup_assistant::ControllerConfig* this_controller =
        config_data_->findControllerByName(current_edit_controller_);
    loadControllerScreen(this_controller);

    // Switch to screen
    changeScreen(2);  // 1 is index of controller edit
  }
  else
  {
    QMessageBox::critical(this, "Error Loading", "An internal error has occured while loading.");
  }
}

// ******************************************************************************************
// Edit a controller
// ******************************************************************************************
void ControllersWidget::editController()
{
  QTreeWidgetItem* item = controllers_tree_->currentItem();

  // Check that something was actually selected
  if (item == nullptr)
    return;

  adding_new_controller_ = false;

  loadControllerScreen(config_data_->findControllerByName(current_edit_controller_));

  // Switch to screen
  changeScreen(2);  // 1 is index of controller edit
}

// ******************************************************************************************
// Switch to current controllers view
// ******************************************************************************************
void ControllersWidget::showMainScreen()
{
  stacked_widget_->setCurrentIndex(0);

  // Announce that this widget is not in modal mode
  Q_EMIT isModal(false);
}

// ******************************************************************************************
// Switch which screen is being shown
// ******************************************************************************************
void ControllersWidget::changeScreen(int index)
{
  stacked_widget_->setCurrentIndex(index);

  // Announce this widget's mode
  Q_EMIT isModal(index != 0);
}

// ******************************************************************************************
// Expand/Collapse Tree
// ******************************************************************************************
void ControllersWidget::alterTree(const QString& link)
{
  if (link.contains("expand"))
    controllers_tree_->expandAll();
  else
    controllers_tree_->collapseAll();
}

void ControllersWidget::itemSelectionChanged()
{
  QList<QTreeWidgetItem*> selected_items = controllers_tree_->selectedItems();
  if (selected_items.empty())
  {
    btn_edit_->setEnabled(false);
    btn_delete_->setEnabled(false);
  }
}

}  // namespace moveit_setup_assistant
