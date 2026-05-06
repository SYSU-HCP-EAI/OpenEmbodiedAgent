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

// SA
#include "setup_screen_widget.h"  // a base class for screens in the setup assistant
#include "setup_assistant_widget.h"
#include "header_widget.h"

// Qt
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QStackedWidget>
#include <QString>
#include <pluginlib/class_loader.hpp>  // for loading all avail kinematic planners
// Rviz
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <moveit/robot_state_rviz_plugin/robot_state_display.h>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
SetupAssistantWidget::SetupAssistantWidget(QWidget* parent, const boost::program_options::variables_map& args)
  : QWidget(parent)
{
  rviz_manager_ = nullptr;
  rviz_render_panel_ = nullptr;

  // Create object to hold all MoveIt configuration data
  config_data_ = std::make_shared<MoveItConfigData>();

  // Set debug mode flag if necessary
  if (args.count("debug"))
    config_data_->debug_ = true;

  // Setting the window icon
  std::string moveit_ros_visualization_package_path = ros::package::getPath("moveit_ros_visualization");
  moveit_ros_visualization_package_path += "/icons/classes/MotionPlanning.png";
  this->setWindowIcon(QIcon(moveit_ros_visualization_package_path.c_str()));

  // Basic widget container -----------------------------------------
  QHBoxLayout* layout = new QHBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Create main content stack for various screens
  main_content_ = new QStackedWidget();
  current_index_ = 0;

  // Screens --------------------------------------------------------

  // Start Screen
  start_screen_widget_ = new StartScreenWidget(this, config_data_);
  start_screen_widget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  connect(start_screen_widget_, SIGNAL(readyToProgress()), this, SLOT(progressPastStartScreen()));
  connect(start_screen_widget_, SIGNAL(loadRviz()), this, SLOT(loadRviz()));
  main_content_->addWidget(start_screen_widget_);

  // Pass command arg values to start screen and show appropriate part of screen
  if (args.count("urdf_path"))
  {
    start_screen_widget_->urdf_file_->setPath(args["urdf_path"].as<std::string>());
    start_screen_widget_->select_mode_->btn_new_->click();
  }
  if (args.count("config_pkg"))
  {
    start_screen_widget_->stack_path_->setPath(args["config_pkg"].as<std::string>());
    start_screen_widget_->select_mode_->btn_exist_->click();
  }
  else
  {
    start_screen_widget_->stack_path_->setPath(QString(getenv("PWD")));
  }

  // Add Navigation Buttons (but do not load widgets yet except start screen)
  nav_name_list_ << "Start";
  nav_name_list_ << "Self-Collisions";
  nav_name_list_ << "Virtual Joints";
  nav_name_list_ << "Planning Groups";
  nav_name_list_ << "Robot Poses";
  nav_name_list_ << "End Effectors";
  nav_name_list_ << "Passive Joints";
  nav_name_list_ << "Controllers";
  nav_name_list_ << "Simulation";
  nav_name_list_ << "3D Perception";
  nav_name_list_ << "Author Information";
  nav_name_list_ << "Configuration Files";

  // Navigation Left Pane --------------------------------------------------
  navs_view_ = new NavigationWidget(this);
  navs_view_->setNavs(nav_name_list_);
  navs_view_->setDisabled(true);
  navs_view_->setSelected(0);  // start screen

  // Rviz View Right Pane ---------------------------------------------------
  rviz_container_ = new QWidget(this);
  rviz_container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  rviz_container_->hide();  // do not show until after the start screen

  // Split screen -----------------------------------------------------
  splitter_ = new QSplitter(Qt::Horizontal, this);
  splitter_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter_->addWidget(navs_view_);
  splitter_->addWidget(main_content_);
  splitter_->addWidget(rviz_container_);
  splitter_->setHandleWidth(6);
  // splitter_->setCollapsible( 0, false ); // don't let navigation collapse
  layout->addWidget(splitter_);

  // Add event for switching between screens -------------------------
  connect(navs_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(navigationClicked(const QModelIndex&)));

  // Final Layout Setup ---------------------------------------------
  this->setLayout(layout);

  // Title
  this->setWindowTitle("MoveIt Setup Assistant");  // title of window

  // Show screen before message
  QApplication::processEvents();
}

// ******************************************************************************************
// Decontructor
// ******************************************************************************************
SetupAssistantWidget::~SetupAssistantWidget()
{
  if (rviz_manager_ != nullptr)
    rviz_manager_->removeAllDisplays();
  if (rviz_render_panel_ != nullptr)
    delete rviz_render_panel_;
  if (rviz_manager_ != nullptr)
    delete rviz_manager_;
}

void SetupAssistantWidget::virtualJointReferenceFrameChanged()
{
  if (rviz_manager_ && robot_state_display_)
  {
    rviz_manager_->setFixedFrame(QString::fromStdString(config_data_->getRobotModel()->getModelFrame()));
    robot_state_display_->reset();
    robot_state_display_->setVisible(true);
  }
}

// ******************************************************************************************
// Change screens of Setup Assistant
// ******************************************************************************************
void SetupAssistantWidget::navigationClicked(const QModelIndex& index)
{
  // Convert QModelIndex to int
  moveToScreen(index.row());
}

// ******************************************************************************************
// Change screens
// ******************************************************************************************
void SetupAssistantWidget::moveToScreen(const int index)
{
  boost::mutex::scoped_lock slock(change_screen_lock_);

  if (current_index_ != index)
  {
    // Send the focus lost command to the screen widget
    SetupScreenWidget* ssw = qobject_cast<SetupScreenWidget*>(main_content_->widget(current_index_));
    if (!ssw->focusLost())
    {
      navs_view_->setSelected(current_index_);
      return;  // switching not accepted
    }

    current_index_ = index;

    // Unhighlight anything on robot
    unhighlightAll();

    // Change screens
    main_content_->setCurrentIndex(index);

    // Send the focus given command to the screen widget
    ssw = qobject_cast<SetupScreenWidget*>(main_content_->widget(index));
    ssw->focusGiven();

    // Change navigation selected option
    navs_view_->setSelected(index);  // Select first item in list
  }
}

// ******************************************************************************************
// Loads other windows, enables navigation
// ******************************************************************************************
void SetupAssistantWidget::progressPastStartScreen()
{
  // Load all widgets ------------------------------------------------

  // Self-Collisions
  default_collisions_widget_ = new DefaultCollisionsWidget(this, config_data_);
  main_content_->addWidget(default_collisions_widget_);
  connect(default_collisions_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(default_collisions_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(default_collisions_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Virtual Joints
  virtual_joints_widget_ = new VirtualJointsWidget(this, config_data_);
  main_content_->addWidget(virtual_joints_widget_);
  connect(virtual_joints_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(virtual_joints_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(virtual_joints_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(virtual_joints_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));
  connect(virtual_joints_widget_, SIGNAL(referenceFrameChanged()), this, SLOT(virtualJointReferenceFrameChanged()));

  // Planning Groups
  planning_groups_widget = new PlanningGroupsWidget(this, config_data_);
  main_content_->addWidget(planning_groups_widget);
  connect(planning_groups_widget, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(planning_groups_widget, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(planning_groups_widget, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(planning_groups_widget, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Robot Poses
  robot_poses_widget_ = new RobotPosesWidget(this, config_data_);
  main_content_->addWidget(robot_poses_widget_);
  connect(robot_poses_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(robot_poses_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(robot_poses_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(robot_poses_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // End Effectors
  end_effectors_widget_ = new EndEffectorsWidget(this, config_data_);
  main_content_->addWidget(end_effectors_widget_);
  connect(end_effectors_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(end_effectors_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(end_effectors_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(end_effectors_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Virtual Joints
  passive_joints_widget_ = new PassiveJointsWidget(this, config_data_);
  main_content_->addWidget(passive_joints_widget_);
  connect(passive_joints_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(passive_joints_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(passive_joints_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(passive_joints_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Controllers
  controllers_widget_ = new ControllersWidget(this, config_data_);
  main_content_->addWidget(controllers_widget_);
  connect(controllers_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(controllers_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(controllers_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(controllers_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Simulation Screen
  simulation_widget_ = new SimulationWidget(this, config_data_);
  main_content_->addWidget(simulation_widget_);
  connect(simulation_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(simulation_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(simulation_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(simulation_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Perception
  perception_widget_ = new PerceptionWidget(this, config_data_);
  main_content_->addWidget(perception_widget_);
  connect(perception_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(perception_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(perception_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(perception_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Author Information
  author_information_widget_ = new AuthorInformationWidget(this, config_data_);
  main_content_->addWidget(author_information_widget_);
  connect(author_information_widget_, SIGNAL(isModal(bool)), this, SLOT(setModalMode(bool)));
  connect(author_information_widget_, SIGNAL(highlightLink(const std::string&, const QColor&)), this,
          SLOT(highlightLink(const std::string&, const QColor&)));
  connect(author_information_widget_, SIGNAL(highlightGroup(const std::string&)), this,
          SLOT(highlightGroup(const std::string&)));
  connect(author_information_widget_, SIGNAL(unhighlightAll()), this, SLOT(unhighlightAll()));

  // Configuration Files
  configuration_files_widget_ = new ConfigurationFilesWidget(this, config_data_);
  main_content_->addWidget(configuration_files_widget_);

  // Enable all nav buttons -------------------------------------------
  for (int i = 0; i < nav_name_list_.count(); ++i)
  {
    navs_view_->setEnabled(i, true);
  }

  // Enable navigation
  navs_view_->setDisabled(false);

  // Replace logo with Rviz screen
  rviz_container_->show();

  // Move to next screen in debug mode
  if (config_data_->debug_)
  {
    moveToScreen(3);
  }
}

// ******************************************************************************************
// Ping ROS on internval
// ******************************************************************************************
void SetupAssistantWidget::updateTimer()
{
  ros::spinOnce();  // keep ROS node alive
}

// ******************************************************************************************
// Load Rviz once we have a robot description ready
// ******************************************************************************************
void SetupAssistantWidget::loadRviz()
{
  // Create rviz frame
  rviz_render_panel_ = new rviz::RenderPanel();
  rviz_render_panel_->setMinimumWidth(200);
  rviz_render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  rviz_manager_ = new rviz::VisualizationManager(rviz_render_panel_);
  rviz_render_panel_->initialize(rviz_manager_->getSceneManager(), rviz_manager_);
  rviz_manager_->initialize();
  rviz_manager_->startUpdate();

  // Set the fixed and target frame
  rviz_manager_->setFixedFrame(QString::fromStdString(config_data_->getRobotModel()->getModelFrame()));

  // Create the MoveIt Rviz Plugin and attach to display
  robot_state_display_ = new moveit_rviz_plugin::RobotStateDisplay();
  robot_state_display_->setName("Robot State");

  rviz_manager_->addDisplay(robot_state_display_, true);

  // Set the topic on which the moveit_msgs::PlanningScene messages are received
  robot_state_display_->subProp("Robot State Topic")->setValue(QString::fromStdString(MOVEIT_ROBOT_STATE));

  // Set robot description
  robot_state_display_->subProp("Robot Description")->setValue(QString::fromStdString(ROBOT_DESCRIPTION));
  robot_state_display_->setVisible(true);

  // Zoom into robot
  rviz::ViewController* view = rviz_manager_->getViewManager()->getCurrent();
  view->subProp("Distance")->setValue(4.0f);

  // Add Rviz to Planning Groups Widget
  QVBoxLayout* rviz_layout = new QVBoxLayout();
  rviz_layout->addWidget(rviz_render_panel_);
  rviz_container_->setLayout(rviz_layout);

  // visual / collision buttons
  auto btn_layout = new QHBoxLayout();
  rviz_layout->addLayout(btn_layout);

  QCheckBox* btn;
  btn_layout->addWidget(btn = new QCheckBox("visual"), 0);
  btn->setChecked(true);
  connect(btn, &QCheckBox::toggled,
          [this](bool checked) { robot_state_display_->subProp("Visual Enabled")->setValue(checked); });

  btn_layout->addWidget(btn = new QCheckBox("collision"), 1);
  btn->setChecked(false);
  connect(btn, &QCheckBox::toggled,
          [this](bool checked) { robot_state_display_->subProp("Collision Enabled")->setValue(checked); });

  rviz_container_->show();
}

// ******************************************************************************************
// Highlight a robot link
// ******************************************************************************************
void SetupAssistantWidget::highlightLink(const std::string& link_name, const QColor& color)
{
  const moveit::core::LinkModel* lm = config_data_->getRobotModel()->getLinkModel(link_name);
  if (!lm->getShapes().empty())  // skip links with no geometry
    robot_state_display_->setLinkColor(link_name, color);
}

// ******************************************************************************************
// Highlight a robot group
// ******************************************************************************************
void SetupAssistantWidget::highlightGroup(const std::string& group_name)
{
  // Highlight the selected planning group by looping through the links
  if (!config_data_->getRobotModel()->hasJointModelGroup(group_name))
    return;

  const moveit::core::JointModelGroup* joint_model_group =
      config_data_->getRobotModel()->getJointModelGroup(group_name);
  if (joint_model_group)
  {
    // Iterate through the links
    for (const moveit::core::LinkModel* lm : joint_model_group->getLinkModels())
      highlightLink(lm->getName(), QColor(255, 0, 0));
  }
}

// ******************************************************************************************
// Unhighlight all robot links
// ******************************************************************************************
void SetupAssistantWidget::unhighlightAll()
{
  // Get the names of the all links robot
  const std::vector<std::string>& links = config_data_->getRobotModel()->getLinkModelNamesWithCollisionGeometry();

  // Quit if no links found
  if (links.empty())
  {
    return;
  }

  // check if rviz is ready
  if (!rviz_manager_ || !robot_state_display_)
  {
    return;
  }

  // Iterate through the links
  for (const std::string& link : links)
  {
    if (link.empty())
      continue;

    robot_state_display_->unsetLinkColor(link);
  }
}

// ******************************************************************************************
// Qt close event function for reminding user to save
// ******************************************************************************************
void SetupAssistantWidget::closeEvent(QCloseEvent* event)
{
  // Only prompt to close if not in debug mode
  if (!config_data_->debug_)
  {
    if (QMessageBox::question(this, "Exit Setup Assistant",
                              QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                              QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
    {
      event->ignore();
      return;
    }
  }

  // Shutdown app
  event->accept();
}

// ******************************************************************************************
// Qt Error Handling - TODO
// ******************************************************************************************
bool SetupAssistantWidget::notify(QObject* /*receiver*/, QEvent* /*event*/)
{
  QMessageBox::critical(this, "Error", "An error occurred and was caught by Qt notify event handler.", QMessageBox::Ok);

  return false;
}

// ******************************************************************************************
// Change the widget modal state based on subwidgets state
// ******************************************************************************************
void SetupAssistantWidget::setModalMode(bool isModal)
{
  navs_view_->setDisabled(isModal);

  for (int i = 0; i < nav_name_list_.count(); ++i)
  {
    navs_view_->setEnabled(i, !isModal);
  }
}

}  // namespace moveit_setup_assistant
