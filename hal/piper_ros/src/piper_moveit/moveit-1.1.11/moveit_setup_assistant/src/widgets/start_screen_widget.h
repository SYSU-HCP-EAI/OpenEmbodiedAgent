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

class QLabel;
class QProgressBar;
class QPushButton;

#ifndef Q_MOC_RUN
#include <urdf/model.h>                                       // for testing a valid urdf is loaded
#include <srdfdom/model.h>                                    // for testing a valid srdf is loaded
#include <moveit/setup_assistant/tools/moveit_config_data.h>  // common datastructure class
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
// Class Prototypes
class SelectModeWidget;
class LoadPathArgsWidget;

/**
 * \brief Start screen user interface for MoveIt Configuration Assistant
 */
class StartScreenWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * \brief Start screen user interface for MoveIt Configuration Assistant
   */
  StartScreenWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);

  ~StartScreenWidget() override;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  SelectModeWidget* select_mode_;
  LoadPathArgsWidget* stack_path_;
  LoadPathArgsWidget* urdf_file_;
  QPushButton* btn_load_;
  QLabel* next_label_;
  QProgressBar* progress_bar_;
  QImage* right_image_;
  QLabel* right_image_label_;

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// User has chosen to show new options
  void showNewOptions();

  /// User has chosen to show edit options
  void showExistingOptions();

  /// Button event for loading user chosen files
  void loadFilesClick();

  /// load package settings
  void onPackagePathChanged(const QString& path);

  /// enable xacro arguments
  void onUrdfPathChanged(const QString& path);

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signal Functions
  // ******************************************************************************************

  /// Event that is fired when the start screen has all its requirements completed and user can move on
  void readyToProgress();

  /// Inform the parent widget to load rviz. This is done so that progress bar can be more accurate
  void loadRviz();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Create new config files, or load existing one?
  bool create_new_package_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// load package settings from .setup_assistant file
  bool loadPackageSettings(bool show_warnings);

  /// Load chosen files for creating new package
  bool loadNewFiles();

  /// Load exisiting package files
  bool loadExistingFiles();

  /// Load URDF File to Parameter Server
  bool loadURDFFile(const std::string& urdf_file_path, const std::string& xacro_args);

  /// Load SRDF File
  bool loadSRDFFile(const std::string& srdf_file_path, const std::string& xacro_args);

  /// Put SRDF File on Parameter Server
  bool setSRDFFile(const std::string& srdf_string);

  //// Extract the package/stack name and relative path to urdf from an absolute path name
  bool extractPackageNameFromPath();

  /// Make the full URDF path using the loaded .setup_assistant data
  bool createFullURDFPath();

  /// Make the full SRDF path using the loaded .setup_assistant data
  bool createFullSRDFPath(const std::string& package_path);

  /// Loads sensors_3d yaml file
  bool load3DSensorsFile();
};

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

class SelectModeWidget : public QFrame
{
  Q_OBJECT

private:
private Q_SLOTS:

public:
  SelectModeWidget(QWidget* parent);

  // Load file button
  QPushButton* btn_new_;
  QPushButton* btn_exist_;
  QLabel* widget_instructions_;
};
}  // namespace moveit_setup_assistant
