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

// Qt
#include <QApplication>
#include <QFileDialog>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

// ROS
#include <ros/package.h>  // for getting file path for loadng images
// SA
#include "header_widget.h"  // title and instructions
#include "start_screen_widget.h"
// C
#include <fstream>  // for reading in urdf
#include <streambuf>
// Boost
#include <boost/filesystem/path.hpp>        // for reading folders/files
#include <boost/filesystem/operations.hpp>  // for reading folders/files
// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>

// chdir
#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

namespace moveit_setup_assistant
{
// Boost file system
namespace fs = boost::filesystem;

// ******************************************************************************************
// Start screen user interface for MoveIt Configuration Assistant
// ******************************************************************************************
StartScreenWidget::StartScreenWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout* hlayout = new QHBoxLayout();
  // Left side of screen
  QVBoxLayout* left_layout = new QVBoxLayout();
  // Right side of screen
  QVBoxLayout* right_layout = new QVBoxLayout();

  // Right Image Area ----------------------------------------------
  right_image_ = new QImage();
  right_image_label_ = new QLabel(this);
  std::string image_path = "./resources/MoveIt_Setup_Assistant2.png";
  if (chdir(config_data_->setup_assistant_path_.c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO moveit_setup_assistant");
  }

  if (right_image_->load(image_path.c_str()))
  {
    right_image_label_->setPixmap(QPixmap::fromImage(*right_image_));
    right_image_label_->setMinimumHeight(384);  // size of right_image_label_
  }
  else
  {
    ROS_ERROR_STREAM("FAILED TO LOAD " << image_path);
  }

  right_layout->addWidget(right_image_label_);
  right_layout->setAlignment(right_image_label_, Qt::AlignRight | Qt::AlignTop);

  // Top Label Area ---------------------------------------------------
  HeaderWidget* header = new HeaderWidget("MoveIt Setup Assistant",
                                          "These tools will assist you in creating a Semantic Robot "
                                          "Description Format (SRDF) file, various yaml configuration and many "
                                          "roslaunch files for utilizing all aspects of MoveIt functionality.",
                                          this);
  layout->addWidget(header);

  // Select Mode Area -------------------------------------------------
  select_mode_ = new SelectModeWidget(this);
  connect(select_mode_->btn_new_, SIGNAL(clicked()), this, SLOT(showNewOptions()));
  connect(select_mode_->btn_exist_, SIGNAL(clicked()), this, SLOT(showExistingOptions()));
  left_layout->addWidget(select_mode_);

  // Path Box Area ----------------------------------------------------

  // Stack Path Dialog
  stack_path_ =
      new LoadPathArgsWidget("Load MoveIt Configuration Package",
                             "Specify the package name or path of an existing MoveIt configuration package to be "
                             "edited for your robot. Example package name: <i>panda_moveit_config</i>",
                             "optional xacro arguments:", this, true);  // directory
  // user needs to select option before this is shown
  stack_path_->hide();
  stack_path_->setArgs("");
  connect(stack_path_, SIGNAL(pathChanged(QString)), this, SLOT(onPackagePathChanged(QString)));
  left_layout->addWidget(stack_path_);

  // URDF File Dialog
  urdf_file_ = new LoadPathArgsWidget("Load a URDF or COLLADA Robot Model",
                                      "Specify the location of an existing Universal Robot Description Format or "
                                      "COLLADA file for your robot",
                                      "optional xacro arguments:", this, false, true);  // no directory, load only
  // user needs to select option before this is shown
  urdf_file_->hide();
  urdf_file_->setArgs("");
  connect(urdf_file_, SIGNAL(pathChanged(QString)), this, SLOT(onUrdfPathChanged(QString)));
  left_layout->addWidget(urdf_file_);

  // Load settings box ---------------------------------------------
  QHBoxLayout* load_files_layout = new QHBoxLayout();

  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide();
  load_files_layout->addWidget(progress_bar_);

  btn_load_ = new QPushButton("&Load Files", this);
  btn_load_->setMinimumWidth(180);
  btn_load_->setMinimumHeight(40);
  btn_load_->hide();
  load_files_layout->addWidget(btn_load_);
  load_files_layout->setAlignment(btn_load_, Qt::AlignRight);
  connect(btn_load_, SIGNAL(clicked()), this, SLOT(loadFilesClick()));

  // Next step instructions
  next_label_ = new QLabel(this);
  QFont next_label_font(QFont().defaultFamily(), 11, QFont::Bold);
  next_label_->setFont(next_label_font);
  next_label_->setText("Success! Use the left navigation pane to continue.");
  next_label_->hide();  // only show once the files have been loaded.

  // Final Layout Setup ---------------------------------------------
  // Alignment
  layout->setAlignment(Qt::AlignTop);
  hlayout->setAlignment(Qt::AlignTop);
  left_layout->setAlignment(Qt::AlignTop);
  right_layout->setAlignment(Qt::AlignTop);

  // Stretch
  left_layout->setSpacing(10);

  // Attach Layouts
  hlayout->addLayout(left_layout);
  hlayout->addLayout(right_layout);
  layout->addLayout(hlayout);

  // Verticle Spacer
  layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding));

  // Attach bottom layout
  layout->addWidget(next_label_);
  layout->setAlignment(next_label_, Qt::AlignRight);
  layout->addLayout(load_files_layout);

  this->setLayout(layout);

  // Debug mode: auto load the configuration file by clicking button after a timeout
  if (config_data_->debug_)
  {
    // select_mode_->btn_exist_->click();

    QTimer* update_timer = new QTimer(this);
    update_timer->setSingleShot(true);  // only run once
    connect(update_timer, SIGNAL(timeout()), btn_load_, SLOT(click()));
    update_timer->start(100);
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
StartScreenWidget::~StartScreenWidget()
{
  delete right_image_;  // does not have a parent passed to it
}

// ******************************************************************************************
// Show options for creating a new configuration package
// ******************************************************************************************
void StartScreenWidget::showNewOptions()
{
  // Do GUI stuff
  select_mode_->btn_exist_->setChecked(false);
  select_mode_->btn_new_->setChecked(true);
  select_mode_->widget_instructions_->hide();
  urdf_file_->show();
  stack_path_->hide();
  btn_load_->show();

  // Remember choice
  create_new_package_ = true;
}

// ******************************************************************************************
// Show options for editing an existing configuration package
// ******************************************************************************************
void StartScreenWidget::showExistingOptions()
{
  // Do GUI stuff
  select_mode_->btn_exist_->setChecked(true);
  select_mode_->btn_new_->setChecked(false);
  select_mode_->widget_instructions_->hide();
  urdf_file_->hide();
  stack_path_->show();
  btn_load_->show();

  // Remember choice
  create_new_package_ = false;
}

// ******************************************************************************************
// Load files to parameter server - CLICK
// ******************************************************************************************
void StartScreenWidget::loadFilesClick()
{
  // Disable start screen GUI components from being changed
  urdf_file_->setDisabled(true);
  // srdf_file_->setDisabled(true);
  stack_path_->setDisabled(true);
  select_mode_->setDisabled(true);
  btn_load_->setDisabled(true);
  progress_bar_->show();

  bool result;

  // Decide if this is a new config package, or loading an old one
  if (create_new_package_)
  {
    result = loadNewFiles();
    // Load 3d_sensors config file
    load3DSensorsFile();
  }
  else
  {
    result = loadExistingFiles();
  }

  // Check if there was a failure loading files
  if (!result)
  {
    // Renable components
    urdf_file_->setDisabled(false);
    // srdf_file_->setDisabled(false);
    stack_path_->setDisabled(false);
    select_mode_->setDisabled(false);
    btn_load_->setDisabled(false);
    progress_bar_->hide();
  }
  else
  {
    // Hide the logo image so that other screens can resize the rviz thing properly
    right_image_label_->hide();
  }
}

void StartScreenWidget::onPackagePathChanged(const QString& /*path*/)
{
  if (!loadPackageSettings(false))
    return;
  // set xacro args from loaded settings
  stack_path_->setArgs(QString::fromStdString(config_data_->xacro_args_));
}

void StartScreenWidget::onUrdfPathChanged(const QString& path)
{
  urdf_file_->setArgsEnabled(rdf_loader::RDFLoader::isXacroFile(path.toStdString()));
}

bool StartScreenWidget::loadPackageSettings(bool show_warnings)
{
  // Get the package path
  std::string package_path_input = stack_path_->getPath();
  // Check that input is provided
  if (package_path_input.empty())
  {
    if (show_warnings)
      QMessageBox::warning(this, "Error Loading Files", "Please specify a configuration package path to load.");
    return false;
  }

  // check that the folder exists
  if (!config_data_->setPackagePath(package_path_input))
  {
    if (show_warnings)
      QMessageBox::critical(this, "Error Loading Files", "The specified path is not a directory or is not accessable");
    return false;
  }

  std::string setup_assistant_path;

  // Check if the old package is a setup assistant package. If it is not, quit
  if (!config_data_->getSetupAssistantYAMLPath(setup_assistant_path))
  {
    if (show_warnings)
      QMessageBox::warning(this, "Incorrect Directory/Package",
                           QString(
                               "The chosen package location exists but was not created using MoveIt Setup Assistant. "
                               "If this is a mistake, provide the missing file: ")
                               .append(setup_assistant_path.c_str()));
    return false;
  }

  // Get setup assistant data
  if (!config_data_->inputSetupAssistantYAML(setup_assistant_path))
  {
    if (show_warnings)
      QMessageBox::warning(this, "Setup Assistant File Error",
                           QString("Unable to correctly parse the setup assistant configuration file: ")
                               .append(setup_assistant_path.c_str()));
    return false;
  }
  return true;
}

// ******************************************************************************************
// Load exisiting package files
// ******************************************************************************************
bool StartScreenWidget::loadExistingFiles()
{
  // Progress Indicator
  progress_bar_->setValue(10);
  QApplication::processEvents();

  if (!loadPackageSettings(true))
    return false;

  // Progress Indicator
  progress_bar_->setValue(30);
  QApplication::processEvents();

  // Get the URDF path using the loaded .setup_assistant data and check it
  if (!createFullURDFPath())
    return false;  // error occured

  // use xacro args from GUI
  config_data_->xacro_args_ = stack_path_->getArgs().toStdString();

  // Load the URDF
  if (!loadURDFFile(config_data_->urdf_path_, config_data_->xacro_args_))
    return false;  // error occured

  // Get the SRDF path using the loaded .setup_assistant data and check it
  if (!createFullSRDFPath(config_data_->config_pkg_path_))
    return false;  // error occured

  // Progress Indicator
  progress_bar_->setValue(50);
  QApplication::processEvents();

  // Load the SRDF
  if (!loadSRDFFile(config_data_->srdf_path_, config_data_->xacro_args_))
    return false;  // error occured

  // Progress Indicator
  progress_bar_->setValue(60);
  QApplication::processEvents();

  // Load the allowed collision matrix
  config_data_->loadAllowedCollisionMatrix(*config_data_->srdf_);

  // Load kinematics yaml file if available --------------------------------------------------
  fs::path kinematics_yaml_path = config_data_->config_pkg_path_;
  kinematics_yaml_path /= "config/kinematics.yaml";

  if (!config_data_->inputKinematicsYAML(kinematics_yaml_path.make_preferred().string()))
  {
    QMessageBox::warning(this, "No Kinematic YAML File",
                         QString("Failed to parse kinematics yaml file. This file is not critical but any previous "
                                 "kinematic solver settings have been lost. To re-populate this file edit each "
                                 "existing planning group and choose a solver, then save each change. \n\nFile error "
                                 "at location ")
                             .append(kinematics_yaml_path.make_preferred().string().c_str()));
  }
  else
  {
    fs::path planning_context_launch_path = config_data_->config_pkg_path_;
    planning_context_launch_path /= "launch/planning_context.launch";
    config_data_->inputPlanningContextLaunch(planning_context_launch_path.make_preferred().string());
  }

  // Load 3d_sensors config file
  load3DSensorsFile();

  // Load ros_controllers.yaml file if available-----------------------------------------------
  fs::path ros_controllers_yaml_path = config_data_->config_pkg_path_;
  ros_controllers_yaml_path /= "config/ros_controllers.yaml";
  config_data_->inputROSControllersYAML(ros_controllers_yaml_path.make_preferred().string());

  fs::path ompl_yaml_path = config_data_->config_pkg_path_;
  ompl_yaml_path /= "config/ompl_planning.yaml";
  config_data_->inputOMPLYAML(ompl_yaml_path.make_preferred().string());

  // DONE LOADING --------------------------------------------------------------------------

  // Call a function that enables navigation
  Q_EMIT readyToProgress();

  // Progress Indicator
  progress_bar_->setValue(70);
  QApplication::processEvents();

  // Load Rviz
  Q_EMIT loadRviz();

  // Progress Indicator
  progress_bar_->setValue(100);
  QApplication::processEvents();

  next_label_->show();  // only show once the files have been loaded

  ROS_INFO("Loading Setup Assistant Complete");
  return true;  // success!
}

// ******************************************************************************************
// Load chosen files for creating new package
// ******************************************************************************************
bool StartScreenWidget::loadNewFiles()
{
  // Get URDF file path
  config_data_->urdf_path_ = urdf_file_->getPath();

  // Check that box is filled out
  if (config_data_->urdf_path_.empty())
  {
    QMessageBox::warning(this, "Error Loading Files", "No robot model file specified");
    return false;
  }

  // Check that this file exits
  if (!fs::is_regular_file(config_data_->urdf_path_))
  {
    QMessageBox::warning(this, "Error Loading Files",
                         QString("Unable to locate the URDF file: ").append(config_data_->urdf_path_.c_str()));
    return false;
  }

  // Attempt to get the ROS package from the path
  if (!extractPackageNameFromPath())
  {
    return false;  // An error occurred
  }

  // Progress Indicator
  progress_bar_->setValue(20);
  QApplication::processEvents();

  // use xacro args from GUI
  config_data_->xacro_args_ = urdf_file_->getArgs().toStdString();

  // Load the URDF to the parameter server and check that it is correct format
  if (!loadURDFFile(config_data_->urdf_path_, config_data_->xacro_args_))
    return false;  // error occurred

  // Progress Indicator
  progress_bar_->setValue(50);
  QApplication::processEvents();

  // Create blank SRDF file
  const std::string blank_srdf = "<?xml version='1.0'?><robot name='" + config_data_->urdf_model_->getName() +
                                 "'></"
                                 "robot>";

  // Load a blank SRDF file to the parameter server
  if (!setSRDFFile(blank_srdf))
  {
    QMessageBox::warning(this, "Error Loading Files", "Failure loading blank SRDF file.");
    return false;
  }

  // Progress Indicator
  progress_bar_->setValue(60);
  QApplication::processEvents();

  // DONE LOADING --------------------------------------------------------------------------

  // Call a function that enables navigation
  Q_EMIT readyToProgress();

  // Progress Indicator
  progress_bar_->setValue(70);
  QApplication::processEvents();

  // Load Rviz
  Q_EMIT loadRviz();

  // Progress Indicator
  progress_bar_->setValue(100);
  QApplication::processEvents();

  next_label_->show();  // only show once the files have been loaded

  ROS_INFO("Loading Setup Assistant Complete");
  return true;  // success!
}

// ******************************************************************************************
// Load URDF File to Parameter Server
// ******************************************************************************************
bool StartScreenWidget::loadURDFFile(const std::string& urdf_file_path, const std::string& xacro_args)
{
  if (!rdf_loader::RDFLoader::loadXmlFileToString(config_data_->urdf_string_, urdf_file_path, { xacro_args }))
  {
    QMessageBox::warning(this, "Error Loading Files",
                         QString("URDF/COLLADA file not found: ").append(urdf_file_path.c_str()));
    return false;
  }

  if (config_data_->urdf_string_.empty() && rdf_loader::RDFLoader::isXacroFile(urdf_file_path))
  {
    QMessageBox::warning(this, "Error Loading Files", "Running xacro failed.\nPlease check console for errors.");
    return false;
  }

  // Verify that file is in correct format / not an XACRO by loading into robot model
  if (!config_data_->urdf_model_->initString(config_data_->urdf_string_))
  {
    QMessageBox::warning(this, "Error Loading Files", "URDF/COLLADA file is not a valid robot model.");
    return false;
  }
  config_data_->urdf_from_xacro_ = rdf_loader::RDFLoader::isXacroFile(urdf_file_path);

  ROS_INFO_STREAM("Loaded " << config_data_->urdf_model_->getName() << " robot model.");

  // Load the robot model to the parameter server
  ros::NodeHandle nh;
  int steps = 0;
  while (!nh.ok() && steps < 4)
  {
    ROS_WARN("Waiting for node handle");
    ros::Duration(1).sleep();
    steps++;
    ros::spinOnce();
  }

  ROS_INFO("Setting Param Server with Robot Description");
  // ROS_WARN("Ignore the following error message 'Failed to contact master'. This is a known issue.");
  nh.setParam("/robot_description", config_data_->urdf_string_);

  return true;
}

// ******************************************************************************************
// Load SRDF File to Parameter Server
// ******************************************************************************************
bool StartScreenWidget::loadSRDFFile(const std::string& srdf_file_path, const std::string& xacro_args)
{
  std::string srdf_string;
  if (!rdf_loader::RDFLoader::loadXmlFileToString(srdf_string, srdf_file_path, { xacro_args }))
  {
    QMessageBox::warning(this, "Error Loading Files", QString("SRDF file not found: ").append(srdf_file_path.c_str()));
    return false;
  }

  // Put on param server
  return setSRDFFile(srdf_string);
}

// ******************************************************************************************
// Put SRDF File on Parameter Server
// ******************************************************************************************
bool StartScreenWidget::setSRDFFile(const std::string& srdf_string)
{
  // Verify that file is in correct format / not an XACRO by loading into robot model
  if (!config_data_->srdf_->initString(*config_data_->urdf_model_, srdf_string))
  {
    QMessageBox::warning(this, "Error Loading Files", "SRDF file not a valid semantic robot description model.");
    return false;
  }
  ROS_INFO_STREAM("Robot semantic model successfully loaded.");

  // Load to param server
  ros::NodeHandle nh;
  int steps = 0;
  while (!nh.ok() && steps < 4)
  {
    ROS_WARN("Waiting for node handle");
    ros::Duration(1).sleep();
    steps++;
    ros::spinOnce();
  }

  ROS_INFO("Setting Param Server with Robot Semantic Description");
  nh.setParam("/robot_description_semantic", srdf_string);

  return true;
}

// ******************************************************************************************
// Extract the package/stack name and relative path to urdf from an absolute path name
// Input:  cofig_data_->urdf_path_
// Output: cofig_data_->urdf_pkg_name_
//         cofig_data_->urdf_pkg_relative_path_
// ******************************************************************************************
bool StartScreenWidget::extractPackageNameFromPath()
{
  std::string relative_path;  // holds the path after the sub_path
  std::string package_name;   // result

  bool package_found = config_data_->extractPackageNameFromPath(config_data_->urdf_path_, package_name, relative_path);

  // Assign data to moveit_config_data
  if (!package_found)
  {
    // No package name found, we must be outside ROS
    config_data_->urdf_pkg_name_ = "";
    config_data_->urdf_pkg_relative_path_ = config_data_->urdf_path_;  // just the absolute path
  }
  else
  {
    // Check that ROS can find the package
    const std::string robot_desc_pkg_path = ros::package::getPath(package_name);

    if (robot_desc_pkg_path.empty())
    {
      QMessageBox::warning(this, "Package Not Found In ROS Workspace",
                           QString("ROS was unable to find the package name '")
                               .append(package_name.c_str())
                               .append("' within the ROS workspace. This may cause issues later."));
    }

    // Success
    config_data_->urdf_pkg_name_ = package_name;
    config_data_->urdf_pkg_relative_path_ = relative_path;
  }

  ROS_DEBUG_STREAM("URDF Package Name: " << config_data_->urdf_pkg_name_);
  ROS_DEBUG_STREAM("URDF Package Path: " << config_data_->urdf_pkg_relative_path_);

  return true;  // success
}

// ******************************************************************************************
// Make the full URDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool StartScreenWidget::createFullURDFPath()
{
  if (!config_data_->createFullURDFPath())
  {
    if (config_data_->urdf_path_.empty())  // no path could be resolved
    {
      QMessageBox::warning(this, "Error Loading Files",
                           QString("ROS was unable to find the package name '")
                               .append(config_data_->urdf_pkg_name_.c_str())
                               .append("'. Verify this package is inside your ROS "
                                       "workspace and is a proper ROS package."));
    }
    else
    {
      QMessageBox::warning(this, "Error Loading Files",
                           QString("Unable to locate the URDF file in package. Expected File: \n")
                               .append(config_data_->urdf_path_.c_str()));
    }
    return false;
  }

  // Check if a package name was provided
  if (config_data_->urdf_pkg_name_.empty())
  {
    ROS_WARN("The URDF path is absolute to the filesystem and not relative to a ROS package/stack");
  }

  return true;  // success
}

// ******************************************************************************************
// Make the full SRDF path using the loaded .setup_assistant data
// ******************************************************************************************
bool StartScreenWidget::createFullSRDFPath(const std::string& package_path)
{
  if (!config_data_->createFullSRDFPath(package_path))
  {
    QMessageBox::warning(this, "Error Loading Files",
                         QString("Unable to locate the SRDF file: ").append(config_data_->srdf_path_.c_str()));
    return false;
  }

  return true;  // success
}

// ******************************************************************************************
// Loads sensors_3d yaml file
// ******************************************************************************************
bool StartScreenWidget::load3DSensorsFile()
{
  // Loads parameters values from sensors_3d yaml file if available
  fs::path sensors_3d_yaml_path = config_data_->config_pkg_path_;
  sensors_3d_yaml_path /= "config/sensors_3d.yaml";

  if (fs::is_regular_file(sensors_3d_yaml_path))
    config_data_->input3DSensorsYAML(sensors_3d_yaml_path.make_preferred().string());
  return true;
}

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
SelectModeWidget::SelectModeWidget(QWidget* parent) : QFrame(parent)
{
  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout* hlayout = new QHBoxLayout();

  // Widget Title
  QLabel* widget_title = new QLabel(this);
  widget_title->setText("Create new or edit existing?");
  QFont widget_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  widget_title->setFont(widget_title_font);
  layout->addWidget(widget_title);
  layout->setAlignment(widget_title, Qt::AlignTop);

  // Widget Instructions
  widget_instructions_ = new QLabel(this);
  widget_instructions_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  widget_instructions_->setWordWrap(true);
  widget_instructions_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  widget_instructions_->setText(
      "All settings for MoveIt are stored in the MoveIt configuration package. Here you have "
      "the option to create a new configuration package or load an existing one. Note: "
      "changes to a MoveIt configuration package outside this Setup Assistant are likely to be "
      "overwritten by this tool.");

  layout->addWidget(widget_instructions_);
  layout->setAlignment(widget_instructions_, Qt::AlignTop);

  // New Button
  btn_new_ = new QPushButton(this);
  btn_new_->setText("Create &New MoveIt\nConfiguration Package");
  hlayout->addWidget(btn_new_);

  // Exist Button
  btn_exist_ = new QPushButton(this);
  btn_exist_->setText("&Edit Existing MoveIt\nConfiguration Package");
  btn_exist_->setCheckable(true);
  hlayout->addWidget(btn_exist_);

  // Add horizontal layer to verticle layer
  layout->addLayout(hlayout);
  setLayout(layout);
  btn_new_->setCheckable(true);
}

}  // namespace moveit_setup_assistant
