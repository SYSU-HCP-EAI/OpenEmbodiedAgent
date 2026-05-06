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

#include <moveit/rviz_plugin_render_tools/trajectory_panel.h>
#include <QHBoxLayout>

namespace moveit_rviz_plugin
{
TrajectoryPanel::TrajectoryPanel(QWidget* parent) : Panel(parent)
{
}

TrajectoryPanel::~TrajectoryPanel() = default;

void TrajectoryPanel::onInitialize()
{
  slider_ = new QSlider(Qt::Horizontal);
  slider_->setTickInterval(1);
  slider_->setMinimum(0);
  slider_->setMaximum(0);
  slider_->setTickPosition(QSlider::TicksBelow);
  slider_->setPageStep(1);
  slider_->setEnabled(false);
  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));

  maximum_label_ = new QLabel(QString::number(slider_->maximum()));
  minimum_label_ = new QLabel(QString::number(slider_->minimum()));
  minimum_label_->setFixedWidth(20);

  button_ = new QPushButton();
  button_->setText("Pause");
  button_->setEnabled(false);
  connect(button_, SIGNAL(clicked()), this, SLOT(buttonClicked()));

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(new QLabel("Waypoint:"));
  layout->addWidget(minimum_label_);
  layout->addWidget(slider_);
  layout->addWidget(maximum_label_);
  layout->addWidget(button_);
  setLayout(layout);

  paused_ = false;
  parentWidget()->setVisible(false);
}

void TrajectoryPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void TrajectoryPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void TrajectoryPanel::update(int way_point_count)
{
  last_way_point_ = std::max(0, way_point_count - 1);

  slider_->setEnabled(way_point_count != 0);
  button_->setEnabled(way_point_count != 0);

  slider_->setSliderPosition(0);
  slider_->setMaximum(last_way_point_);
  maximum_label_->setText(QString::number(last_way_point_));
  pauseButton(false);
}

void TrajectoryPanel::pauseButton(bool pause)
{
  if (pause)
  {
    button_->setText("Play");
    paused_ = true;
  }
  else
  {
    button_->setText("Pause");
    paused_ = false;
    if (slider_->sliderPosition() == last_way_point_)
      slider_->setSliderPosition(0);
  }
}

void TrajectoryPanel::setSliderPosition(int position)
{
  slider_->setSliderPosition(position);
}

void TrajectoryPanel::sliderValueChanged(int value)
{
  minimum_label_->setText(QString::number(value));
}

void TrajectoryPanel::buttonClicked()
{
  if (paused_)
    pauseButton(false);
  else
    pauseButton(true);
}

}  // namespace moveit_rviz_plugin
