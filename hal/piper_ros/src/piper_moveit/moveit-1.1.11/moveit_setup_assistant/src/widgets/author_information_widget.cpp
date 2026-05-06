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

/* Author: Dave Coleman, Michael 'v4hn' Goerner */

// Qt
#include <QVBoxLayout>
#include <QPushButton>
#include <QMessageBox>
#include <QApplication>
#include <QLabel>
#include <QLineEdit>
#include "author_information_widget.h"
#include "header_widget.h"

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
AuthorInformationWidget::AuthorInformationWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget("Specify Author Information",
                                          "Input contact information of the author and initial maintainer of the "
                                          "generated package. catkin requires valid details in the package's "
                                          "package.xml",
                                          this);
  layout->addWidget(header);

  QLabel* name_title = new QLabel(this);
  name_title->setText("Name of the maintainer of this MoveIt configuration:");
  layout->addWidget(name_title);

  name_edit_ = new QLineEdit(this);
  connect(name_edit_, SIGNAL(editingFinished()), this, SLOT(editedName()));
  layout->addWidget(name_edit_);

  QLabel* email_title = new QLabel(this);
  email_title->setText("Email of the maintainer of this MoveIt configuration:");
  layout->addWidget(email_title);

  email_edit_ = new QLineEdit(this);
  connect(email_edit_, SIGNAL(editingFinished()), this, SLOT(editedEmail()));
  layout->addWidget(email_edit_);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void AuthorInformationWidget::focusGiven()
{
  // Allow list box to populate
  this->name_edit_->setText(QString::fromStdString(config_data_->author_name_));
  this->email_edit_->setText(QString::fromStdString(config_data_->author_email_));
}

void AuthorInformationWidget::editedName()
{
  config_data_->author_name_ = this->name_edit_->text().toStdString();
  config_data_->changes |= MoveItConfigData::AUTHOR_INFO;
}

void AuthorInformationWidget::editedEmail()
{
  config_data_->author_email_ = this->email_edit_->text().toStdString();
  config_data_->changes |= MoveItConfigData::AUTHOR_INFO;
}

}  // namespace moveit_setup_assistant
