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
#include <QFrame>
class QLabel;
class QLineEdit;

namespace moveit_setup_assistant
{
// ******************************************************************************************
// ******************************************************************************************
// Class for showing the title and instructions
// ******************************************************************************************
// ******************************************************************************************
class HeaderWidget : public QWidget
{
  Q_OBJECT

public:
  /// Contructor
  HeaderWidget(const std::string& title, const std::string& instructions, QWidget* parent);
};

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting files
// ******************************************************************************************
// ******************************************************************************************
class LoadPathWidget : public QFrame
{
  Q_OBJECT

private:
  // Only allow user to select folders
  bool dir_only_;
  // Only allow user to load files (not save)
  bool load_only_;
  // Stores the path qstring
  QLineEdit* path_box_;

Q_SIGNALS:
  void pathChanged(const QString& path);
  void pathEditingFinished();

private Q_SLOTS:
  /// Load the file dialog
  void btnFileDialog();

public:
  /// Constructor
  LoadPathWidget(const QString& title, const QString& instructions, QWidget* parent, const bool dir_only = false,
                 const bool load_only = false);

  /// Returns the file path in QString format
  QString getQPath() const;

  /// Returns the file path in std::string format
  std::string getPath() const;

  /// Set the path with QString
  void setPath(const QString& path);

  /// Set the path with std string
  void setPath(const std::string& path);
};

/// Extend LoadPathWidget with additional line edit for arguments
class LoadPathArgsWidget : public LoadPathWidget
{
  Q_OBJECT

private:
  QLineEdit* args_;
  QLabel* args_instructions_;

public:
  /// Constructor
  LoadPathArgsWidget(const QString& title, const QString& instructions, const QString& arg_instructions,
                     QWidget* parent, const bool dir_only = false, const bool load_only = false);

  QString getArgs() const;
  void setArgs(const QString& args);
  void setArgsEnabled(bool enabled = true);
};
}  // namespace moveit_setup_assistant
