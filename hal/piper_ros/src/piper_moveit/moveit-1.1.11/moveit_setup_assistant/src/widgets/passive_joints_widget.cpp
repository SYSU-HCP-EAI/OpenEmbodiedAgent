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

// SA
#include "passive_joints_widget.h"
#include "header_widget.h"
#include "double_list_widget.h"

// Qt
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QTableWidget>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
PassiveJointsWidget::PassiveJointsWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget("Define Passive Joints",
                                          "Specify the set of passive joints (not actuated). Joint "
                                          "state is not expected to be published for these joints.",
                                          this);
  layout->addWidget(header);

  // Joints edit widget
  joints_widget_ = new DoubleListWidget(this, config_data_, "Joint Collection", "Joint", false);
  connect(joints_widget_, SIGNAL(selectionUpdated()), this, SLOT(selectionUpdated()));
  connect(joints_widget_, SIGNAL(previewSelected(std::vector<std::string>)), this,
          SLOT(previewSelectedJoints(std::vector<std::string>)));

  // Set the title
  joints_widget_->title_->setText("");

  joints_widget_->setColumnNames("Active Joints", "Passive Joints");

  layout->addWidget(joints_widget_);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

// ******************************************************************************************
//
// ******************************************************************************************
void PassiveJointsWidget::focusGiven()
{
  joints_widget_->clearContents();

  // Retrieve pointer to the shared kinematic model
  const moveit::core::RobotModelConstPtr& model = config_data_->getRobotModel();

  // Get the names of the all joints
  const std::vector<std::string>& joints = model->getJointModelNames();

  if (joints.empty())
  {
    QMessageBox::critical(this, "Error Loading", "No joints found for robot model");
    return;
  }
  std::vector<std::string> active_joints;
  for (const std::string& joint : joints)
    if (model->getJointModel(joint)->getVariableCount() > 0)
      active_joints.push_back(joint);

  // Set the available joints (left box)
  joints_widget_->setAvailable(active_joints);

  std::vector<std::string> passive_joints;
  for (srdf::Model::PassiveJoint& passive_joint : config_data_->srdf_->passive_joints_)
    passive_joints.push_back(passive_joint.name_);
  joints_widget_->setSelected(passive_joints);
}

// ******************************************************************************************
//
// ******************************************************************************************
void PassiveJointsWidget::selectionUpdated()
{
  config_data_->srdf_->passive_joints_.clear();
  for (int i = 0; i < joints_widget_->selected_data_table_->rowCount(); ++i)
  {
    srdf::Model::PassiveJoint pj;
    pj.name_ = joints_widget_->selected_data_table_->item(i, 0)->text().toStdString();
    config_data_->srdf_->passive_joints_.push_back(pj);
  }
  config_data_->changes |= MoveItConfigData::PASSIVE_JOINTS;
}

// ******************************************************************************************
// Called from Double List widget to highlight joints
// ******************************************************************************************
void PassiveJointsWidget::previewSelectedJoints(const std::vector<std::string>& joints)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  for (const std::string& joint : joints)
  {
    const moveit::core::JointModel* joint_model = config_data_->getRobotModel()->getJointModel(joint);

    // Check that a joint model was found
    if (!joint_model)
    {
      continue;
    }

    // Get the name of the link
    const std::string link = joint_model->getChildLinkModel()->getName();

    if (link.empty())
    {
      continue;
    }

    // Highlight link
    Q_EMIT highlightLink(link, QColor(255, 0, 0));
  }
}

}  // namespace moveit_setup_assistant
