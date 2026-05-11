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
#include <QAction>
#include <QApplication>
#include <QLabel>
#include <QList>
#include <QListWidget>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QPushButton>
#include <QRegExp>
#include <QSplitter>
#include <QVBoxLayout>

#include "configuration_files_widget.h"
#include "header_widget.h"

// Boost
#include <boost/algorithm/string.hpp>       // for string find and replace in templates
#include <boost/filesystem/path.hpp>        // for creating folders/files
#include <boost/filesystem/operations.hpp>  // is_regular_file, is_directory, etc.
// Read write files
#include <iostream>  // For writing yaml and launch files
#include <fstream>

namespace moveit_setup_assistant
{
// Boost file system
namespace fs = boost::filesystem;

static const std::string SETUP_ASSISTANT_FILE = ".setup_assistant";
static const std::string CONFIG_PATH = "config";

// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
ConfigurationFilesWidget::ConfigurationFilesWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data), has_generated_pkg_(false)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  HeaderWidget* header =
      new HeaderWidget("Generate Configuration Files",
                       "Create or update the configuration files package needed to run your robot with MoveIt. Uncheck "
                       "files to disable them from being generated - this is useful if you have made custom changes to "
                       "them. Files in orange have been automatically detected as changed.",
                       this);
  layout->addWidget(header);

  // Path Widget ----------------------------------------------------

  // Stack Path Dialog
  stack_path_ = new LoadPathWidget("Configuration Package Save Path",
                                   "Specify the desired directory for the MoveIt configuration package to be "
                                   "generated. Overwriting an existing configuration package directory is acceptable. "
                                   "Example: <i>/u/robot/ros/panda_moveit_config</i>",
                                   this, true);  // is directory
  layout->addWidget(stack_path_);

  // Pass the package path from start screen to configuration files screen
  stack_path_->setPath(config_data_->config_pkg_path_);

  // Generated Files List -------------------------------------------
  QLabel* generated_list = new QLabel("Check files you want to be generated:", this);
  layout->addWidget(generated_list);

  QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // List Box
  action_list_ = new QListWidget(this);
  action_list_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  action_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  connect(action_list_, SIGNAL(currentRowChanged(int)), this, SLOT(changeActionDesc(int)));
  // Allow checking / unchecking of multiple items
  action_list_->setContextMenuPolicy(Qt::ActionsContextMenu);
  QAction* action = new QAction("Check all selected files", this);
  connect(action, &QAction::triggered, [this]() { setCheckSelected(true); });
  action_list_->addAction(action);
  action = new QAction("Uncheck all selected files", this);
  connect(action, &QAction::triggered, [this]() { setCheckSelected(false); });
  action_list_->addAction(action);

  // Description
  action_label_ = new QLabel(this);
  action_label_->setFrameShape(QFrame::StyledPanel);
  action_label_->setFrameShadow(QFrame::Raised);
  action_label_->setLineWidth(1);
  action_label_->setMidLineWidth(0);
  action_label_->setWordWrap(true);
  action_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  action_label_->setMinimumWidth(100);
  action_label_->setAlignment(Qt::AlignTop);
  action_label_->setOpenExternalLinks(true);  // open with web browser

  // Add to splitter
  splitter->addWidget(action_list_);
  splitter->addWidget(action_label_);

  // Add Layout
  layout->addWidget(splitter);

  // Progress bar and generate buttons ---------------------------------------------------
  QHBoxLayout* hlayout1 = new QHBoxLayout();

  // Progress Bar
  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  hlayout1->addWidget(progress_bar_);
  // hlayout1->setContentsMargins( 20, 30, 20, 30 );

  // Generate Package Button
  btn_save_ = new QPushButton("&Generate Package", this);
  // btn_save_->setMinimumWidth(180);
  btn_save_->setMinimumHeight(40);
  connect(btn_save_, SIGNAL(clicked()), this, SLOT(savePackage()));
  hlayout1->addWidget(btn_save_);

  // Add Layout
  layout->addLayout(hlayout1);

  // Bottom row --------------------------------------------------

  QHBoxLayout* hlayout3 = new QHBoxLayout();

  // Success label
  success_label_ = new QLabel(this);
  QFont success_label_font(QFont().defaultFamily(), 12, QFont::Bold);
  success_label_->setFont(success_label_font);
  success_label_->hide();  // only show once the files have been generated
  success_label_->setText("Configuration package generated successfully!");
  hlayout3->addWidget(success_label_);
  hlayout3->setAlignment(success_label_, Qt::AlignRight);

  // Exit button
  QPushButton* btn_exit = new QPushButton("E&xit Setup Assistant", this);
  btn_exit->setMinimumWidth(180);
  connect(btn_exit, SIGNAL(clicked()), this, SLOT(exitSetupAssistant()));
  hlayout3->addWidget(btn_exit);
  hlayout3->setAlignment(btn_exit, Qt::AlignRight);

  layout->addLayout(hlayout3);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

void ConfigurationFilesWidget::setCheckSelected(bool checked)
{
  for (const QModelIndex& row : action_list_->selectionModel()->selectedRows())
    action_list_->model()->setData(row, checked ? Qt::Checked : Qt::Unchecked, Qt::CheckStateRole);
}

// ******************************************************************************************
// Populate the 'Files to be generated' list
// ******************************************************************************************
bool ConfigurationFilesWidget::loadGenFiles()
{
  GenerateFile file;          // re-used
  std::string template_path;  // re-used
  const std::string robot_name = config_data_->srdf_->robot_name_;

  gen_files_.clear();  // reset vector

  // Get template package location ----------------------------------------------------------------------
  fs::path template_package_path = config_data_->setup_assistant_path_;
  template_package_path /= "templates";
  template_package_path /= "moveit_config_pkg_template";
  config_data_->template_package_path_ = template_package_path.make_preferred().string();

  if (!fs::is_directory(config_data_->template_package_path_))
  {
    QMessageBox::critical(
        this, "Error Generating",
        QString("Unable to find package template directory: ").append(config_data_->template_package_path_.c_str()));
    return false;
  }

  // -------------------------------------------------------------------------------------------------------------------
  // ROS PACKAGE FILES AND FOLDERS ----------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------

  // package.xml --------------------------------------------------------------------------------------
  // Note: we call the file package.xml.template so that it isn't automatically indexed by rosprofile
  // in the scenario where we want to disabled the setup_assistant by renaming its root package.xml
  file.file_name_ = "package.xml";
  file.rel_path_ = file.file_name_;
  template_path = config_data_->appendPaths(config_data_->template_package_path_, "package.xml.template");
  file.description_ = "Defines a ROS package";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = MoveItConfigData::AUTHOR_INFO;
  gen_files_.push_back(file);

  // CMakeLists.txt --------------------------------------------------------------------------------------
  file.file_name_ = "CMakeLists.txt";
  file.rel_path_ = file.file_name_;
  template_path = config_data_->appendPaths(config_data_->template_package_path_, file.file_name_);
  file.description_ = "CMake build system configuration file";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // -------------------------------------------------------------------------------------------------------------------
  // CONIG FILES -------------------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------

  // config/ --------------------------------------------------------------------------------------
  file.file_name_ = "config/";
  file.rel_path_ = file.file_name_;
  file.description_ = "Folder containing all MoveIt configuration files for your robot. This folder is required and "
                      "cannot be disabled.";
  file.gen_func_ = [this](const std::string& output_path) { return createFolder(output_path); };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // robot.srdf ----------------------------------------------------------------------------------------------
  file.file_name_ = config_data_->srdf_pkg_relative_path_.empty() ? config_data_->urdf_model_->getName() + ".srdf" :
                                                                    config_data_->srdf_pkg_relative_path_;
  file.rel_path_ = config_data_->srdf_pkg_relative_path_.empty() ?
                       config_data_->appendPaths(CONFIG_PATH, file.file_name_) :
                       config_data_->srdf_pkg_relative_path_;
  file.description_ = "SRDF (<a href='http://www.ros.org/wiki/srdf'>Semantic Robot Description Format</a>) is a "
                      "representation of semantic information about robots. This format is intended to represent "
                      "information about the robot that is not in the URDF file, but it is useful for a variety of "
                      "applications. The intention is to include information that has a semantic aspect to it.";
  file.gen_func_ = [&writer = *config_data_->srdf_](const std::string& output_path) {
    return writer.writeSRDF(output_path);
  };
  file.write_on_changes = MoveItConfigData::SRDF;
  gen_files_.push_back(file);
  // special step required so the generated .setup_assistant yaml has this value
  config_data_->srdf_pkg_relative_path_ = file.rel_path_;

  // gazebo_<ROBOT>.urdf ---------------------------------------------------------------------------------------
  if (!config_data_->gazebo_urdf_string_.empty())
  {
    file.file_name_ = "gazebo_" + config_data_->urdf_model_->getName() + ".urdf";
    file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
    file.description_ =
        "This <a href='https://wiki.ros.org/urdf'>URDF</a> file comprises your original robot description "
        "augmented with tags required for use with Gazebo, i.e. defining inertia and transmission properties. "
        "Checkout the <a href='http://gazebosim.org/tutorials/?tut=ros_urdf'>URDF Gazebo documentation</a> "
        "for more infos.";
    file.gen_func_ = [this](const std::string& output_path) { return config_data_->outputGazeboURDFFile(output_path); };
    file.write_on_changes = MoveItConfigData::SIMULATION;
    gen_files_.push_back(file);
  }

  // ompl_planning.yaml --------------------------------------------------------------------------------------
  file.file_name_ = "ompl_planning.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Configures the OMPL (<a href='http://ompl.kavrakilab.org/'>Open Motion Planning Library</a>) "
                      "planning plugin. For every planning group defined in the SRDF, a number of planning "
                      "configurations are specified (under planner_configs). Additionally, default settings for the "
                      "state space to plan in for a particular group can be specified, such as the collision checking "
                      "resolution. Each planning configuration specified for a group must be defined under the "
                      "planner_configs tag. While defining a planner configuration, the only mandatory parameter is "
                      "'type', which is the name of the motion planner to be used. Any other planner-specific "
                      "parameters can be defined but are optional.";
  file.gen_func_ = [this](const std::string& output_path) { return config_data_->outputOMPLPlanningYAML(output_path); };
  file.write_on_changes = MoveItConfigData::GROUPS;
  gen_files_.push_back(file);

  // chomp_planning.yaml  --------------------------------------------------------------------------------------
  file.file_name_ = "chomp_planning.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  template_path = config_data_->appendPaths(config_data_->template_package_path_, file.rel_path_);
  file.description_ = "Specifies which chomp planning plugin parameters to be used for the CHOMP planner";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  gen_files_.push_back(file);

  // stomp_planning.yaml  --------------------------------------------------------------------------------------
  file.file_name_ = "stomp_planning.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Specifies which stomp planning plugin parameters to be used for the STOMP planner";
  file.gen_func_ = [this](const std::string& output_path) { return config_data_->outputSTOMPPlanningYAML(output_path); };
  file.write_on_changes = MoveItConfigData::GROUPS;  // need to double check if this is actually correct!
  gen_files_.push_back(file);

  // kinematics.yaml  --------------------------------------------------------------------------------------
  file.file_name_ = "kinematics.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Specifies which kinematic solver plugin to use for each planning group in the SRDF, as well as "
                      "the kinematic solver search resolution.";
  file.gen_func_ = [this](const std::string& output_path) { return config_data_->outputKinematicsYAML(output_path); };
  file.write_on_changes = MoveItConfigData::GROUPS | MoveItConfigData::GROUP_KINEMATICS;
  gen_files_.push_back(file);

  // joint_limits.yaml --------------------------------------------------------------------------------------
  file.file_name_ = "joint_limits.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Contains additional information about joints that appear in your planning groups that is not "
                      "contained in the URDF, as well as allowing you to set maximum and minimum limits for velocity "
                      "and acceleration than those contained in your URDF. This information is used by our trajectory "
                      "filtering system to assign reasonable velocities and timing for the trajectory before it is "
                      "passed to the robots controllers.";
  file.gen_func_ = [this](const std::string& output_path) { return config_data_->outputJointLimitsYAML(output_path); };
  file.write_on_changes = 0;  // Can they be changed?
  gen_files_.push_back(file);

  // cartesian_limits.yaml --------------------------------------------------------------------------------------
  file.file_name_ = "cartesian_limits.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  template_path = config_data_->appendPaths(config_data_->template_package_path_, file.rel_path_);
  file.description_ = "Cartesian velocity for planning in the workspace."
                      "The velocity is used by pilz industrial motion planner as maximum velocity for cartesian "
                      "planning requests scaled by the velocity scaling factor of an individual planning request.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;  // Can they be changed?
  gen_files_.push_back(file);

  // fake_controllers.yaml --------------------------------------------------------------------------------------
  file.file_name_ = "fake_controllers.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Creates dummy configurations for controllers that correspond to defined groups. This is mostly "
                      "useful for testing.";
  file.gen_func_ = [this](const std::string& output_path) {
    return config_data_->outputFakeControllersYAML(output_path);
  };
  file.write_on_changes = MoveItConfigData::GROUPS;
  gen_files_.push_back(file);

  // simple_moveit_controllers.yaml -------------------------------------------------------------------------------
  file.file_name_ = "simple_moveit_controllers.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Creates controller configuration for SimpleMoveItControllerManager";
  file.gen_func_ = [this](const std::string& output_path) {
    return config_data_->outputSimpleControllersYAML(output_path);
  };
  file.write_on_changes = MoveItConfigData::GROUPS;
  gen_files_.push_back(file);

  // gazebo_controllers.yaml ------------------------------------------------------------------
  file.file_name_ = "gazebo_controllers.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  template_path = config_data_->appendPaths(config_data_->template_package_path_, file.rel_path_);
  file.description_ = "Configuration of Gazebo controllers";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  gen_files_.push_back(file);

  // ros_controllers.yaml --------------------------------------------------------------------------------------
  file.file_name_ = "ros_controllers.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Creates controller configurations for ros_control.";
  file.gen_func_ = [this](const std::string& output_path) {
    return config_data_->outputROSControllersYAML(output_path);
  };
  file.write_on_changes = MoveItConfigData::GROUPS;
  gen_files_.push_back(file);

  // sensors_3d.yaml --------------------------------------------------------------------------------------
  file.file_name_ = "sensors_3d.yaml";
  file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
  file.description_ = "Creates configurations 3d sensors.";
  file.gen_func_ = [this](const std::string& output_path) {
    return config_data_->output3DSensorPluginYAML(output_path);
  };
  file.write_on_changes = MoveItConfigData::SENSORS_CONFIG;
  gen_files_.push_back(file);

  // -------------------------------------------------------------------------------------------------------------------
  // LAUNCH FILES ------------------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------
  std::string launch_path = "launch";
  const std::string template_launch_path = config_data_->appendPaths(config_data_->template_package_path_, launch_path);

  // launch/ --------------------------------------------------------------------------------------
  file.file_name_ = "launch/";
  file.rel_path_ = file.file_name_;
  file.description_ = "Folder containing all MoveIt launch files for your robot. "
                      "This folder is required and cannot be disabled.";
  file.gen_func_ = [this](const std::string& output_path) { return createFolder(output_path); };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // move_group.launch --------------------------------------------------------------------------------------
  file.file_name_ = "move_group.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Launches the move_group node that provides the MoveGroup action and other parameters <a "
                      "href='http://moveit.ros.org/doxygen/"
                      "classmoveit_1_1planning__interface_1_1MoveGroup.html#details'>MoveGroup action</a>";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // planning_context.launch --------------------------------------------------------------------------------------
  file.file_name_ = "planning_context.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Loads settings for the ROS parameter server, required for running MoveIt. This includes the "
                      "SRDF, joints_limits.yaml file, ompl_planning.yaml file, optionally the URDF, etc";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // moveit_rviz.launch --------------------------------------------------------------------------------------
  file.file_name_ = "moveit_rviz.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Visualize in Rviz the robot's planning groups running with interactive markers that allow goal "
                      "states to be set.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // ompl_planning_pipeline.launch
  // --------------------------------------------------------------------------------------
  file.file_name_ = "ompl_planning_pipeline.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Intended to be included in other launch files that require the OMPL planning plugin. Defines "
                      "the proper plugin name on the parameter server and a default selection of planning request "
                      "adapters.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // pilz_industrial_motion_planner_planning_pipeline.launch
  // --------------------------------------------------------------------------------------
  file.file_name_ = "pilz_industrial_motion_planner_planning_pipeline.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Intended to be included in other launch files that require the Pilz industrial motion planner "
                      "planning plugin. Defines the proper plugin name on the parameter server and a default selection "
                      "of planning request adapters.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // chomp_planning_pipeline.launch
  // --------------------------------------------------------------------------------------
  file.file_name_ = "chomp_planning_pipeline.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Intended to be included in other launch files that require the CHOMP planning plugin. Defines "
                      "the proper plugin name on the parameter server and a default selection of planning request "
                      "adapters.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // stomp_planning_pipeline.launch
  // --------------------------------------------------------------------------------------
  file.file_name_ = "stomp_planning_pipeline.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Intended to be included in other launch files that require the STOMP planning plugin. Defines "
                      "the proper plugin name on the parameter server and a default selection of planning request "
                      "adapters.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // ompl-chomp_planning_pipeline.launch.xml
  // --------------------------------------------------------------------------------------
  file.file_name_ = "ompl-chomp_planning_pipeline.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ =
      "Intended to be included in other launch files that require the OMPL-CHOMP planning plugins. Defines "
      "the proper plugin name on the parameter server and a default selection of planning request "
      "adapters.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // planning_pipeline.launch --------------------------------------------------------------------------------------
  file.file_name_ = "planning_pipeline.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Helper launch file that can choose between different planning pipelines to be loaded.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // warehouse_settings.launch --------------------------------------------------------------------------------------
  file.file_name_ = "warehouse_settings.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Helper launch file that specifies default settings for MongoDB.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // warehouse.launch --------------------------------------------------------------------------------------
  file.file_name_ = "warehouse.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Launch file for starting MongoDB.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // default_warehouse_db.launch --------------------------------------------------------------------------------------
  file.file_name_ = "default_warehouse_db.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Launch file for starting the warehouse with a default MongoDB.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // run_benchmark_ompl.launch --------------------------------------------------------------------------------------
  file.file_name_ = "run_benchmark_ompl.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Launch file for benchmarking OMPL planners";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // sensor_manager.launch --------------------------------------------------------------------------------------
  file.file_name_ = "sensor_manager.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Helper launch file that can choose between different sensor managers to be loaded.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // robot_moveit_sensor_manager.launch ------------------------------------------------------------------
  file.file_name_ = robot_name + "_moveit_sensor_manager.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, "moveit_sensor_manager.launch.xml");
  file.description_ = "Placeholder for settings specific to the MoveIt sensor manager implemented for you robot.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // trajectory_execution.launch ------------------------------------------------------------------
  file.file_name_ = "trajectory_execution.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Loads settings for the ROS parameter server required for executing trajectories using the "
                      "trajectory_execution_manager::TrajectoryExecutionManager.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  file.file_name_ = "fake_moveit_controller_manager.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Loads the fake controller plugin.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  file.file_name_ = "simple_moveit_controller_manager.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Loads the default controller plugin.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  file.file_name_ = "ros_control_moveit_controller_manager.launch.xml";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Loads the ros_control controller plugin.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // demo.launch ------------------------------------------------------------------
  file.file_name_ = "demo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Run a demo of MoveIt.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // demo_gazebo.launch ------------------------------------------------------------------
  file.file_name_ = "demo_gazebo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Run a demo of MoveIt with Gazebo and Rviz";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // gazebo.launch ------------------------------------------------------------------
  file.file_name_ = "gazebo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, "gazebo.launch");
  file.description_ = "Gazebo launch file which also launches ros_controllers and sends robot urdf to param server, "
                      "then using gazebo_ros pkg the robot is spawned to Gazebo";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // joystick_control.launch ------------------------------------------------------------------
  file.file_name_ = "joystick_control.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Control the Rviz Motion Planning Plugin with a joystick";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // setup_assistant.launch ------------------------------------------------------------------
  file.file_name_ = "setup_assistant.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(
      template_launch_path, "edit_configuration_package.launch");  // named this so that this launch file is not mixed
                                                                   // up with the SA's real launch file
  file.description_ = "Launch file for easily re-starting the MoveIt Setup Assistant to edit this robot's generated "
                      "configuration package.";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // ros_controllers.launch ------------------------------------------------------------------
  file.file_name_ = "ros_controllers.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, "ros_controllers.launch");
  file.description_ = "ros_controllers launch file";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = MoveItConfigData::GROUPS;
  gen_files_.push_back(file);

  // moveit.rviz ------------------------------------------------------------------
  file.file_name_ = "moveit.rviz";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, "moveit.rviz");
  file.description_ = "Configuration file for Rviz with the Motion Planning Plugin already setup. Used by passing "
                      "roslaunch moveit_rviz.launch config:=true";
  file.gen_func_ = [this, template_path](const std::string& output_path) {
    return copyTemplate(template_path, output_path);
  };
  file.write_on_changes = 0;
  gen_files_.push_back(file);

  // -------------------------------------------------------------------------------------------------------------------
  // OTHER FILES -------------------------------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------------

  // .setup_assistant ------------------------------------------------------------------
  file.file_name_ = SETUP_ASSISTANT_FILE;
  file.rel_path_ = file.file_name_;
  file.description_ = "MoveIt Setup Assistant's hidden settings file. You should not need to edit this file.";
  file.gen_func_ = [this](const std::string& output_path) {
    return config_data_->outputSetupAssistantFile(output_path);
  };
  file.write_on_changes = -1;  // write on any changes
  gen_files_.push_back(file);

  return true;
}

// ******************************************************************************************
// Verify with user if certain screens have not been completed
// ******************************************************************************************
bool ConfigurationFilesWidget::checkDependencies()
{
  QStringList dependencies;
  bool required_actions = false;

  // Check that at least 1 planning group exists
  if (config_data_->srdf_->groups_.empty())
  {
    dependencies << "No robot model planning groups have been created";
  }

  // Check that at least 1 link pair is disabled from collision checking
  if (config_data_->srdf_->disabled_collision_pairs_.empty())
  {
    dependencies << "No self-collisions have been disabled";
  }

  // Check that there is at least 1 end effector added
  if (config_data_->srdf_->end_effectors_.empty())
  {
    dependencies << "No end effectors have been added";
  }

  // Check that there is at least 1 virtual joint added
  if (config_data_->srdf_->virtual_joints_.empty())
  {
    dependencies << "No virtual joints have been added";
  }

  // Check that there is a author name
  if (config_data_->author_name_.find_first_not_of(' ') == std::string::npos)
  {
    // There is no name or it consists of whitespaces only
    dependencies << "<b>No author name added</b>";
    required_actions = true;
  }

  // Check that email information is filled
  QRegExp mail_regex("\\b[A-Z0-9._%+-]+@[A-Z0-9.-]+\\.[A-Z]{2,4}\\b");
  mail_regex.setCaseSensitivity(Qt::CaseInsensitive);
  mail_regex.setPatternSyntax(QRegExp::RegExp);
  QString test_email = QString::fromStdString(config_data_->author_email_);
  if (!mail_regex.exactMatch(test_email))
  {
    dependencies << "<b>No valid email address added</b>";
    required_actions = true;
  }

  // Display all accumumlated errors:
  if (!dependencies.empty())
  {
    // Create a dependency message
    QString dep_message;
    if (!required_actions)
    {
      dep_message = "Some setup steps have not been completed. None of the steps are required, but here is a reminder "
                    "of what was not filled in, just in case something was forgotten:<br /><ul>";
    }
    else
    {
      dep_message = "Some setup steps have not been completed. Please fix the required steps (printed in bold), "
                    "otherwise the setup cannot be completed:<br /><ul>";
    }

    for (int i = 0; i < dependencies.size(); ++i)
    {
      dep_message.append("<li>").append(dependencies.at(i)).append("</li>");
    }

    if (!required_actions)
    {
      dep_message.append("</ul><br/>Press Ok to continue generating files.");
      if (QMessageBox::question(this, "Incomplete MoveIt Setup Assistant Steps", dep_message,
                                QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
      {
        return false;  // abort
      }
    }
    else
    {
      QMessageBox::warning(this, "Incomplete MoveIt Setup Assistant Steps", dep_message);
      return false;
    }
  }

  return true;
}

// ******************************************************************************************
// A function for showing progress and user feedback about what happened
// ******************************************************************************************
void ConfigurationFilesWidget::updateProgress()
{
  action_num_++;

  // Calc percentage
  progress_bar_->setValue(double(action_num_) / gen_files_.size() * 100);

  // allow the progress bar to be shown
  QApplication::processEvents();
}

// ******************************************************************************************
// Display the selected action in the desc box
// ******************************************************************************************
void ConfigurationFilesWidget::changeActionDesc(int id)
{
  // Only allow event if list is not empty
  if (id >= 0)
  {
    // Show the selected text
    action_label_->setText(action_desc_.at(id));
  }
}

// ******************************************************************************************
// Disable or enable item in gen_files_ array
// ******************************************************************************************
void ConfigurationFilesWidget::changeCheckedState(QListWidgetItem* item)
{
  std::size_t index = item->data(Qt::UserRole).toUInt();
  bool generate = (item->checkState() == Qt::Checked);

  if (!generate && (gen_files_[index].write_on_changes & config_data_->changes))
  {
    QMessageBox::warning(this, "Package Generation",
                         "You should generate this file to ensure your changes will take "
                         "effect.");
  }

  // Enable/disable file
  gen_files_[index].generate_ = generate;
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void ConfigurationFilesWidget::focusGiven()
{
  loadGenFiles();

  // Which files have been modified outside the Setup Assistant?
  bool files_already_modified = checkGenFiles();

  // disable reaction to checkbox changes
  disconnect(action_list_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(changeCheckedState(QListWidgetItem*)));

  // Show files in GUI
  bool have_conflicting_changes = showGenFiles();

  // react to manual changes only (not programmatic ones)
  connect(action_list_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(changeCheckedState(QListWidgetItem*)));

  // Allow list box to populate
  QApplication::processEvents();

  if (files_already_modified)
  {
    // Some were found to be modified
    QString msg("Some files have been modified outside of the Setup Assistant (according to timestamp). "
                "The Setup Assistant will not overwrite these changes by default because often changing configuration "
                "files manually is necessary, "
                "but we recommend you check the list and enable the checkbox next to files you would like to "
                "overwrite. ");
    if (have_conflicting_changes)
      msg += "<br/><font color='red'>Attention:</font> Some files (<font color='red'>marked red</font>) are changed "
             "both, externally and in Setup Assistant.";
    QMessageBox::information(this, "Files Modified", msg);
  }
}

// ******************************************************************************************
// Check the list of files to be generated for modification
// Returns true if files were detected as modified
// ******************************************************************************************
bool ConfigurationFilesWidget::checkGenFiles()
{
  // Check if we are 'editing' a prev generated config pkg
  if (config_data_->config_pkg_path_.empty())
    return false;  // this is a new package

  // Check if we have the previous modification timestamp to compare agains
  if (config_data_->config_pkg_generated_timestamp_ == 0)
    return false;  // this package has not been generated with a timestamp, backwards compatible.

  static const std::time_t TIME_MOD_TOLERANCE = 10;

  // Check all old file's modification time
  bool found_modified = false;
  for (GenerateFile& gen_file : gen_files_)
  {
    GenerateFile* file = &gen_file;

    fs::path file_path = config_data_->appendPaths(config_data_->config_pkg_path_, file->rel_path_);

    // Don't disable folders from being generated
    if (fs::is_directory(file_path))
      continue;

    if (fs::is_regular_file(file_path))
    {
      std::time_t mod_time = fs::last_write_time(file_path);

      // ROS_DEBUG_STREAM("File " << file->file_name_ << " file modified " << mod_time << " pkg modified " <<
      // config_data_->config_pkg_generated_timestamp_);

      if (mod_time > config_data_->config_pkg_generated_timestamp_ + TIME_MOD_TOLERANCE ||
          mod_time < config_data_->config_pkg_generated_timestamp_ - TIME_MOD_TOLERANCE)
      {
        ROS_INFO_STREAM("Manual editing detected: not over-writing by default file " << file->file_name_);

        if (file->write_on_changes & config_data_->changes)
          ROS_WARN_STREAM("Editing in Setup Assistant conflicts with external editing of file " << file->file_name_);

        file->generate_ = false;  // do not overwrite by default
        file->modified_ = true;
        found_modified = true;
      }
      else
      {
        file->modified_ = false;
      }
    }
  }

  // Warn user if files have been modified outside Setup Assistant
  return found_modified;
}

// ******************************************************************************************
// Show the list of files to be generated
// ******************************************************************************************
bool ConfigurationFilesWidget::showGenFiles()
{
  bool have_conflicting_changes = false;
  action_list_->clear();

  // Display this list in the GUI
  for (std::size_t i = 0; i < gen_files_.size(); ++i)
  {
    GenerateFile* file = &gen_files_[i];

    // Create a formatted row
    QListWidgetItem* item = new QListWidgetItem(QString(file->rel_path_.c_str()), action_list_, 0);

    fs::path file_path = config_data_->appendPaths(config_data_->config_pkg_path_, file->rel_path_);

    // Checkbox
    item->setCheckState(file->generate_ ? Qt::Checked : Qt::Unchecked);
    // externally modified?
    if (file->modified_)
    {
      if (file->write_on_changes & config_data_->changes)
      {
        have_conflicting_changes = true;
        item->setForeground(QBrush(QColor(255, 0, 0)));
      }
      else
        item->setForeground(QBrush(QColor(255, 135, 0)));
    }

    // Don't allow folders to be disabled
    if (fs::is_directory(file_path))
    {
      item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    }

    // Link the gen_files_ index to this item
    item->setData(Qt::UserRole, QVariant(static_cast<qulonglong>(i)));

    // Add actions to list
    action_list_->addItem(item);
    action_desc_.append(QString(file->description_.c_str()));
  }

  // Select the first item in the list so that a description is visible
  action_list_->setCurrentRow(0);
  return have_conflicting_changes;
}

// ******************************************************************************************
// Save configuration click event
// ******************************************************************************************
void ConfigurationFilesWidget::savePackage()
{
  // Feedback
  success_label_->hide();

  // Reset the progress bar counter and GUI stuff
  action_num_ = 0;
  progress_bar_->setValue(0);

  if (!generatePackage())
  {
    ROS_ERROR_STREAM("Failed to generate entire configuration package");
    return;
  }

  // Alert user it completed successfully --------------------------------------------------
  progress_bar_->setValue(100);
  success_label_->show();
  has_generated_pkg_ = true;
}

// ******************************************************************************************
// Save package using default path
// ******************************************************************************************
bool ConfigurationFilesWidget::generatePackage()
{
  // Get path name
  std::string new_package_path = stack_path_->getPath();

  // Check that a valid stack package name has been given
  if (new_package_path.empty())
  {
    QMessageBox::warning(this, "Error Generating",
                         "No package path provided. Please choose a directory location to "
                         "generate the MoveIt configuration files.");
    return false;
  }

  // Check setup assist deps
  if (!checkDependencies())
    return false;  // canceled

  // Check that all groups have components
  if (!noGroupsEmpty())
    return false;  // not ready

  // Trim whitespace from user input
  boost::trim(new_package_path);

  // Get the package name ---------------------------------------------------------------------------------
  new_package_name_ = getPackageName(new_package_path);

  const std::string setup_assistant_file = config_data_->appendPaths(new_package_path, SETUP_ASSISTANT_FILE);

  // Make sure old package is correct package type and verify over write
  if (fs::is_directory(new_package_path) && !fs::is_empty(new_package_path))
  {
    // Check if the old package is a setup assistant package. If it is not, quit
    if (!fs::is_regular_file(setup_assistant_file))
    {
      QMessageBox::warning(
          this, "Incorrect Folder/Package",
          QString("The chosen package location already exists but was not previously created using this MoveIt Setup "
                  "Assistant. "
                  "If this is a mistake, add the missing file: ")
              .append(setup_assistant_file.c_str()));
      return false;
    }

    // Confirm overwrite
    if (QMessageBox::question(this, "Confirm Package Update",
                              QString("Are you sure you want to overwrite this existing package with updated "
                                      "configurations?<br /><i>")
                                  .append(new_package_path.c_str())
                                  .append("</i>"),
                              QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
    {
      return false;  // abort
    }
  }
  else  // this is a new package (but maybe the folder already exists)
  {
    // Create new directory ------------------------------------------------------------------
    try
    {
      fs::create_directory(new_package_path) && !fs::is_directory(new_package_path);
    }
    catch (...)
    {
      QMessageBox::critical(this, "Error Generating Files",
                            QString("Unable to create directory ").append(new_package_path.c_str()));
      return false;
    }
  }

  loadTemplateStrings();

  // Begin to create files and folders ----------------------------------------------------------------------
  std::string absolute_path;

  for (GenerateFile& gen_file : gen_files_)
  {
    GenerateFile* file = &gen_file;

    // Check if we should skip this file
    if (!file->generate_)
    {
      continue;
    }

    // Create the absolute path
    absolute_path = config_data_->appendPaths(new_package_path, file->rel_path_);
    ROS_DEBUG_STREAM("Creating file " << absolute_path);

    // Run the generate function
    if (!file->gen_func_(absolute_path))
    {
      // Error occured
      QMessageBox::critical(this, "Error Generating File",
                            QString("Failed to generate folder or file: '")
                                .append(file->rel_path_.c_str())
                                .append("' at location:\n")
                                .append(absolute_path.c_str()));
      return false;
    }
    updateProgress();  // Increment and update GUI
  }

  return true;
}

// ******************************************************************************************
// Quit the program because we are done
// ******************************************************************************************
void ConfigurationFilesWidget::exitSetupAssistant()
{
  if (has_generated_pkg_ || QMessageBox::question(this, "Exit Setup Assistant",
                                                  QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                                                  QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Ok)
  {
    QApplication::quit();
  }
}

// ******************************************************************************************
// Get the last folder name in a directory path
// ******************************************************************************************
const std::string ConfigurationFilesWidget::getPackageName(std::string package_path)
{
  // Remove end slash if there is one
  if (!package_path.compare(package_path.size() - 1, 1, "/"))
  {
    package_path = package_path.substr(0, package_path.size() - 1);
  }

  // Get the last directory name
  std::string package_name;
  fs::path fs_package_path = package_path;

  package_name = fs_package_path.filename().string();

  // check for empty
  if (package_name.empty())
    package_name = "unknown";

  return package_name;
}

// ******************************************************************************************
// Check that no group is empty (without links/joints/etc)
// ******************************************************************************************
bool ConfigurationFilesWidget::noGroupsEmpty()
{
  // Loop through all groups
  for (const auto& group : config_data_->srdf_->groups_)
  {
    // Whenever 1 of the 4 component types are found, stop checking this group
    if (!group.joints_.empty())
      continue;
    if (!group.links_.empty())
      continue;
    if (!group.chains_.empty())
      continue;
    if (!group.subgroups_.empty())
      continue;

    // This group has no contents, bad
    QMessageBox::warning(
        this, "Empty Group",
        QString("The planning group '")
            .append(group.name_.c_str())
            .append("' is empty and has no subcomponents associated with it (joints/links/chains/subgroups). You must "
                    "edit or remove this planning group before this configuration package can be saved."));
    return false;
  }

  return true;  // good
}

// ******************************************************************************************
// Load the strings that will be replaced in all templates
// ******************************************************************************************
void ConfigurationFilesWidget::loadTemplateStrings()
{
  // Clear strings (in case export is run multiple times)
  template_strings_.clear();

  // Pair 1
  addTemplateString("[GENERATED_PACKAGE_NAME]", new_package_name_);

  // Pair 2
  std::string urdf_location = config_data_->urdf_pkg_name_.empty() ? config_data_->urdf_path_ :
                                                                     "$(find " + config_data_->urdf_pkg_name_ + ")/" +
                                                                         config_data_->urdf_pkg_relative_path_;
  addTemplateString("[URDF_LOCATION]", urdf_location);

  // Pair 3
  if (config_data_->urdf_from_xacro_)
    addTemplateString("[URDF_LOAD_ATTRIBUTE]",
                      "command=\"xacro " + config_data_->xacro_args_ + " '" + urdf_location + "'\"");
  else
    addTemplateString("[URDF_LOAD_ATTRIBUTE]", "textfile=\"" + urdf_location + "\"");

  // Pair 4
  if (config_data_->changes & MoveItConfigData::SIMULATION && !config_data_->gazebo_urdf_string_.empty())
  {
    std::string file_name = "gazebo_" + config_data_->urdf_model_->getName() + ".urdf";
    std::string rel_path = config_data_->appendPaths(CONFIG_PATH, file_name);
    addTemplateString("[GAZEBO_URDF_LOAD_ATTRIBUTE]", "textfile=\"$(find " + new_package_name_ + ")/" + rel_path + "\"");
  }
  else  // reuse [URDF_LOAD_ATTRIBUTE] template
    addTemplateString("[GAZEBO_URDF_LOAD_ATTRIBUTE]", template_strings_.back().second);

  // Pair 5
  addTemplateString("[ROBOT_NAME]", config_data_->srdf_->robot_name_);

  // Pair 6
  addTemplateString("[ROBOT_ROOT_LINK]", config_data_->getRobotModel()->getRootLinkName());

  // Pair 7
  addTemplateString("[PLANNING_FRAME]", config_data_->getRobotModel()->getModelFrame());

  // Pair 8
  std::stringstream vjb;
  for (std::size_t i = 0; i < config_data_->srdf_->virtual_joints_.size(); ++i)
  {
    const srdf::Model::VirtualJoint& vj = config_data_->srdf_->virtual_joints_[i];
    vjb << "  <node pkg=\"tf2_ros\" type=\"static_transform_publisher\" name=\"virtual_joint_broadcaster_" << i
        << "\" args=\"0 0 0 0 0 0 " << vj.parent_frame_ << " " << vj.child_link_ << "\" />" << std::endl;
  }
  addTemplateString("[VIRTUAL_JOINT_BROADCASTER]", vjb.str());

  // Pair 9 - Add dependencies to package.xml if the robot.urdf file is relative to a ROS package
  if (config_data_->urdf_pkg_name_.empty())
  {
    addTemplateString("[OTHER_DEPENDENCIES", "");  // not relative to a ROS package
  }
  else
  {
    std::stringstream deps;
    deps << "  <run_depend>" << config_data_->urdf_pkg_name_ << "</run_depend>\n";
    addTemplateString("[OTHER_DEPENDENCIES]", deps.str());  // not relative to a ROS package
  }

  // Pair 10 - List of ROS Controllers to load in ros_controllers.launch file
  if (config_data_->getControllers().empty())
  {
    addTemplateString("[ROS_CONTROLLERS]", "");
  }
  else
  {
    std::stringstream controllers;
    for (ControllerConfig& controller : config_data_->getControllers())
    {
      // Check if the controller belongs to controller_list namespace
      if (controller.type_ != "FollowJointTrajectory")
        controllers << controller.name_ << " ";
    }
    addTemplateString("[ROS_CONTROLLERS]", controllers.str());
  }

  // Pair 11 - Add parameter files for the kinematics solvers that should be loaded
  // in addition to kinematics.yaml by planning_context.launch
  std::string kinematics_parameters_files_block;
  for (const auto& groups : config_data_->group_meta_data_)
  {
    if (groups.second.kinematics_parameters_file_.empty())
      continue;

    // add a linebreak if we have more than one entry
    if (!kinematics_parameters_files_block.empty())
      kinematics_parameters_files_block += "\n";

    std::string line = "    <rosparam command=\"load\" ns=\"" + groups.first + "\" file=\"" +
                       groups.second.kinematics_parameters_file_ + "\"/>";
    kinematics_parameters_files_block += line;
  }
  addTemplateString("[KINEMATICS_PARAMETERS_FILE_NAMES_BLOCK]", kinematics_parameters_files_block);

  addTemplateString("[AUTHOR_NAME]", config_data_->author_name_);
  addTemplateString("[AUTHOR_EMAIL]", config_data_->author_email_);

  {
    std::stringstream joints;
    for (const auto& pair : config_data_->getInitialJoints())
      joints << " -J " << pair.first << " " << pair.second;
    addTemplateString("[GAZEBO_INITIAL_JOINTS]", joints.str());
  }
}

// ******************************************************************************************
// Insert a string pair into the template_strings_ datastructure
// ******************************************************************************************
bool ConfigurationFilesWidget::addTemplateString(const std::string& key, const std::string& value)
{
  template_strings_.push_back(std::pair<std::string, std::string>(key, value));

  return true;
}

// ******************************************************************************************
// Copy a template from location <template_path> to location <output_path> and replace package name
// ******************************************************************************************
bool ConfigurationFilesWidget::copyTemplate(const std::string& template_path, const std::string& output_path)
{
  // Error check file
  if (!fs::is_regular_file(template_path))
  {
    ROS_ERROR_STREAM("Unable to find template file " << template_path);
    return false;
  }

  // Load file
  std::ifstream template_stream(template_path.c_str());
  if (!template_stream.good())  // File not found
  {
    ROS_ERROR_STREAM("Unable to load file " << template_path);
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  std::string template_string;
  template_stream.seekg(0, std::ios::end);
  template_string.reserve(template_stream.tellg());
  template_stream.seekg(0, std::ios::beg);
  template_string.assign((std::istreambuf_iterator<char>(template_stream)), std::istreambuf_iterator<char>());
  template_stream.close();

  // Replace keywords in string ------------------------------------------------------------
  for (std::pair<std::string, std::string>& it : template_strings_)
  {
    boost::replace_all(template_string, it.first, it.second);
  }

  // Save string to new location -----------------------------------------------------------
  std::ofstream output_stream(output_path.c_str(), std::ios_base::trunc);
  if (!output_stream.good())
  {
    ROS_ERROR_STREAM("Unable to open file for writing " << output_path);
    return false;
  }

  output_stream << template_string.c_str();
  output_stream.close();

  return true;  // file created successfully
}

// ******************************************************************************************
// Create a folder
// ******************************************************************************************
bool ConfigurationFilesWidget::createFolder(const std::string& output_path)
{
  if (!fs::is_directory(output_path))
  {
    if (!fs::create_directory(output_path))
    {
      QMessageBox::critical(this, "Error Generating Files",
                            QString("Unable to create directory ").append(output_path.c_str()));
      return false;
    }
  }
  return true;
}

}  // namespace moveit_setup_assistant
