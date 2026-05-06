/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Yannick Jonetzko
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

/* Author: Yannick Jonetzko */

#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

#include <QSlider>
#include <QLabel>
#include <QPushButton>

namespace moveit_rviz_plugin
{
class TrajectoryPanel : public rviz::Panel
{
  Q_OBJECT

public:
  TrajectoryPanel(QWidget* parent = nullptr);

  ~TrajectoryPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();
  void update(int way_point_count);

  // Switches between pause and play mode
  void pauseButton(bool check);

  void setSliderPosition(int position);

  int getSliderPosition() const
  {
    return slider_->sliderPosition();
  }

  bool isPaused() const
  {
    return paused_;
  }

private Q_SLOTS:
  void sliderValueChanged(int value);
  void buttonClicked();

protected:
  QSlider* slider_;
  QLabel* maximum_label_;
  QLabel* minimum_label_;
  QPushButton* button_;

  bool paused_;
  int last_way_point_;
};

}  // namespace moveit_rviz_plugin
