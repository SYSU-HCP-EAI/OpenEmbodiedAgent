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
 *   * The name of Mohamad Ayman may not be used to endorse or promote products derived
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

#pragma once

// Qt
class QLabel;
class QTextEdit;
class QPushButton;

// SA
#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
// ******************************************************************************************
// ******************************************************************************************
// Class for showing changes needed to help user bring his robot into gazebo simulation
// ******************************************************************************************
// ******************************************************************************************
class SimulationWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  SimulationWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);
  void focusGiven() override;
  bool focusLost() override;

private:
  /// Generate Gazebo-compatible URDF, starting from original URDF
  std::string generateGazeboCompatibleURDF() const;

private Q_SLOTS:
  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Mark document as changed
  void setDirty(bool dirty = true);
  /// Overwrite original URDF with content of document
  void overwriteURDF();
  /// Open original URDF with system editor
  void openURDF();
  /// Copy the content of the URDF document to the clipboard
  void copyURDF();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QTextEdit* simulation_text_;
  QLabel* no_changes_label_;
  QPushButton* btn_overwrite_;
  QPushButton* btn_open_;
  QLabel* copy_urdf_;

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;
};

}  // namespace moveit_setup_assistant
