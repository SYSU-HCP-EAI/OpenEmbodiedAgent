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

#include <QList>
class QLabel;
class QListWidget;
class QListWidgetItem;
class QProgressBar;
class QPushButton;

#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
class LoadPathWidget;

// Struct for storing all the file operations
struct GenerateFile
{
  GenerateFile() : write_on_changes(0), generate_(true), modified_(false)
  {
  }
  std::string file_name_;
  std::string rel_path_;
  std::string description_;
  unsigned long write_on_changes;  // bitfield indicating required rewrite
  bool generate_;                  // "generate" checkbox ticked?
  bool modified_;                  // file externally modified?
  std::function<bool(std::string)> gen_func_;
};

// Typedef for storing template string replacement pairs
typedef std::vector<std::pair<std::string, std::string> > StringPairVector;

// Class
class ConfigurationFilesWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  ConfigurationFilesWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);

  /// Received when this widget is chosen from the navigation menu
  void focusGiven() override;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QPushButton* btn_save_;
  LoadPathWidget* stack_path_;
  QProgressBar* progress_bar_;
  QListWidget* action_list_;
  QLabel* action_label_;
  QLabel* success_label_;
  QList<QString> action_desc_;  // Holds the descriptions explaining all performed actions

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Save package click event
  void savePackage();

  /// Generate the package
  bool generatePackage();

  /// Quit the program because we are done
  void exitSetupAssistant();

  /// Display the selected action in the desc box
  void changeActionDesc(int id);

  /// Disable or enable item in gen_files_ array
  void changeCheckedState(QListWidgetItem* item);

  /// Set checked state of all selected items
  void setCheckSelected(bool checked);

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;

  /// Name of the new package that is being (or going) to be generated, based on user specified save path
  std::string new_package_name_;

  /// Track progress
  unsigned int action_num_;

  /// Has the package been generated yet this program execution? Used for popping up exit warning
  bool has_generated_pkg_;

  /// Vector of all files to be generated
  std::vector<GenerateFile> gen_files_;

  /// Vector of all strings to search for in templates, and their replacements
  StringPairVector template_strings_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// Populate the 'Files to be generated' list
  bool loadGenFiles();

  /// Check the list of files to be generated for modification
  /// Returns true if files were detected as modified
  bool checkGenFiles();

  /// Show the list of files to be generated
  bool showGenFiles();

  /// Verify with user if certain screens have not been completed
  bool checkDependencies();

  /// A function for showing progress and user feedback about what happened
  void updateProgress();

  /// Get the last folder name in a directory path
  const std::string getPackageName(std::string package_path);

  /// Check that no group is empty (without links/joints/etc)
  bool noGroupsEmpty();

  /**
   * \brief Load the strings that will be replaced in all templates
   * \return void
   */
  void loadTemplateStrings();

  /**
   * \brief Insert a string pair into the template_strings_ datastructure
   * \param key string to search in template
   * \param value string to replace with
   * \return void
   */
  bool addTemplateString(const std::string& key, const std::string& value);

  /**
   * Copy a template from location <template_path> to location <output_path> and replace package name
   *
   * @param template_path path to template file
   * @param output_path desired path to copy to
   * @param new_package_name name of the new package being created, to replace key word in template
   *
   * @return bool if the template was copied correctly
   */
  bool copyTemplate(const std::string& template_path, const std::string& output_path);

  /**
   * \brief Create a folder
   * \param output_path name of folder relative to package
   * \return bool if success
   */
  bool createFolder(const std::string& output_path);
};

}  // namespace moveit_setup_assistant
