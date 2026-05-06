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

// ******************************************************************************************
// Provides the title and instructions
// ******************************************************************************************
class SetupScreenWidget : public QWidget
{
  Q_OBJECT

public:
  SetupScreenWidget(QWidget* parent) : QWidget(parent)
  {
  }

  /// function called when widget is activated, allows to update/initialize GUI
  virtual void focusGiven();

  /// function called when widget lost focus, allows to accept/reject changes and to reject switching (returning false)
  virtual bool focusLost();

  // ******************************************************************************************
  // Emitted Signal Functions
  // ******************************************************************************************

Q_SIGNALS:

  /// Event for when the current screen is in modal view. Essential disabled the left navigation
  void isModal(bool isModal);

  /// Event for telling rviz to highlight a link of the robot
  void highlightLink(const std::string& name, const QColor& /*_t2*/);

  /// Event for telling rviz to highlight a group of the robot
  void highlightGroup(const std::string& name);

  /// Event for telling rviz to unhighlight all links of the robot
  void unhighlightAll();
};
