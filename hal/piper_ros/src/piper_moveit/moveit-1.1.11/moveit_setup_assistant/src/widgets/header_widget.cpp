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

#include "header_widget.h"
#include <QFileDialog>
#include <QFont>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// ******************************************************************************************
// Class for Header of Screen
// ******************************************************************************************
// ******************************************************************************************

HeaderWidget::HeaderWidget(const std::string& title, const std::string& instructions, QWidget* parent) : QWidget(parent)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Page Title
  QLabel* page_title = new QLabel(this);
  page_title->setText(title.c_str());
  QFont page_title_font(QFont().defaultFamily(), 18, QFont::Bold);
  page_title->setFont(page_title_font);
  page_title->setWordWrap(true);
  page_title->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  layout->addWidget(page_title);

  // Page Instructions
  QLabel* page_instructions = new QLabel(this);
  page_instructions->setText(instructions.c_str());
  page_instructions->setWordWrap(true);
  page_instructions->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  layout->addWidget(page_instructions);

  // Margin on bottom
  layout->setContentsMargins(0, 0, 0, 0);  // last 15

  this->setLayout(layout);

  // For some reason, this style sheet setting affects the placement layout!?
  this->setStyleSheet(QString("background-color:%1;").arg(QWidget::palette().color(QWidget::backgroundRole()).name()));
}

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting files
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
LoadPathWidget::LoadPathWidget(const QString& title, const QString& instructions, QWidget* parent, const bool dir_only,
                               const bool load_only)
  : QFrame(parent), dir_only_(dir_only), load_only_(load_only)
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
  widget_title->setText(title);
  QFont widget_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  widget_title->setFont(widget_title_font);
  layout->addWidget(widget_title);
  layout->setAlignment(widget_title, Qt::AlignTop);

  // Widget Instructions
  QLabel* widget_instructions = new QLabel(this);
  widget_instructions->setText(instructions);
  widget_instructions->setWordWrap(true);
  widget_instructions->setTextFormat(Qt::RichText);
  layout->addWidget(widget_instructions);
  layout->setAlignment(widget_instructions, Qt::AlignTop);

  // Line Edit for path
  path_box_ = new QLineEdit(this);
  connect(path_box_, SIGNAL(textChanged(QString)), this, SIGNAL(pathChanged(QString)));
  connect(path_box_, SIGNAL(editingFinished()), this, SIGNAL(pathEditingFinished()));
  hlayout->addWidget(path_box_);

  // Button
  QPushButton* browse_button = new QPushButton(this);
  browse_button->setText("Browse");
  connect(browse_button, SIGNAL(clicked()), this, SLOT(btnFileDialog()));
  hlayout->addWidget(browse_button);

  // Add horizontal layer to verticle layer
  layout->addLayout(hlayout);

  setLayout(layout);
}

// ******************************************************************************************
// Load the file dialog
// ******************************************************************************************
void LoadPathWidget::btnFileDialog()
{
  QString path;
  if (dir_only_)  // only allow user to select a directory
  {
    path = QFileDialog::getExistingDirectory(this, "Open Package Directory", path_box_->text(),
                                             QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  }
  else  // only allow user to select file
  {
    QString start_path;

    start_path = path_box_->text();

    if (load_only_)
    {
      path = QFileDialog::getOpenFileName(this, "Open File", start_path, "");
    }
    else
    {
      path = QFileDialog::getSaveFileName(this, "Create/Load File", start_path, "");
    }
  }

  // check they did not press cancel
  if (!path.isNull())
    path_box_->setText(path);
}

// ******************************************************************************************
// Get the QString path
// ******************************************************************************************
QString LoadPathWidget::getQPath() const
{
  return path_box_->text();
}

// ******************************************************************************************
// Get Std String path
// ******************************************************************************************
std::string LoadPathWidget::getPath() const
{
  return getQPath().toStdString();
}

// ******************************************************************************************
// Set the file/dir path
// ******************************************************************************************
void LoadPathWidget::setPath(const QString& path)
{
  path_box_->setText(path);
}

// ******************************************************************************************
// Set the file/dir path with std string
// ******************************************************************************************
void LoadPathWidget::setPath(const std::string& path)
{
  path_box_->setText(QString(path.c_str()));
}

LoadPathArgsWidget::LoadPathArgsWidget(const QString& title, const QString& instructions,
                                       const QString& arg_instructions, QWidget* parent, const bool dir_only,
                                       const bool load_only)
  : LoadPathWidget(title, instructions, parent, dir_only, load_only)
{
  // Line Edit for xacro args
  args_instructions_ = new QLabel(arg_instructions, this);
  args_ = new QLineEdit(this);

  layout()->addWidget(args_instructions_);
  layout()->addWidget(args_);
}

QString LoadPathArgsWidget::getArgs() const
{
  return args_->text();
}

void LoadPathArgsWidget::setArgs(const QString& args)
{
  args_->setText(args);
}

void LoadPathArgsWidget::setArgsEnabled(bool enabled)
{
  args_->setEnabled(enabled);
}
}  // namespace moveit_setup_assistant
