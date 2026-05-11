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

#pragma once

class QLineEdit;

#ifndef Q_MOC_RUN
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
class AuthorInformationWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  AuthorInformationWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);

  /// Received when this widget is chosen from the navigation menu
  void focusGiven() override;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLineEdit* name_edit_;

  QLineEdit* email_edit_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void editedName();
  void editedEmail();

private:
  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;
};

}  // namespace moveit_setup_assistant
