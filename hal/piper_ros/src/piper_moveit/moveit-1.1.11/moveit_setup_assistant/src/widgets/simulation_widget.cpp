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

// SA
#include "simulation_widget.h"
#include "header_widget.h"
#include "../tools/xml_syntax_highlighter.h"
#include <moveit/setup_assistant/tools/xml_manipulation.h>

// Qt
#include <QColor>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QProcess>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <regex>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
SimulationWidget::SimulationWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), config_data_(config_data)
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------

  HeaderWidget* header = new HeaderWidget(
      "Gazebo Simulation",
      QString("For use in the Gazebo physics simulation, the URDF needs to define inertial properties "
              "for all links as well as control interfaces for all joints. "
              "The required changes to your URDF are <b>highlighted below in "
              "<font color=\"darkgreen\">green</font></b>.<br>"
              "You can accept these suggestions and overwrite your existing URDF, or manually "
              "adapt your URDF opening your preferred editor. "
              "By default, a new file comprising those changes will be written to <tt>config/gazebo_%1.urdf</tt>")
          .arg(config_data_->urdf_model_->getName().c_str())
          .toStdString(),
      this);
  layout->addWidget(header);
  layout->addSpacerItem(new QSpacerItem(1, 8, QSizePolicy::Fixed, QSizePolicy::Fixed));

  // Top Buttons --------------------------------------------------
  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Used to overwrite the original URDF
  btn_overwrite_ = new QPushButton("Over&write original URDF", this);
  btn_overwrite_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  connect(btn_overwrite_, SIGNAL(clicked()), this, SLOT(overwriteURDF()));
  controls_layout->addWidget(btn_overwrite_);

  btn_open_ = new QPushButton("&Open original URDF", this);
  btn_open_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_open_->setToolTip("Open original URDF file in editor");
  connect(btn_open_, SIGNAL(clicked()), this, SLOT(openURDF()));
  controls_layout->addWidget(btn_open_);

  // Align buttons to the left
  controls_layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Fixed));

  // Add layout
  layout->addLayout(controls_layout);

  // When there are no changes to be made
  no_changes_label_ = new QLabel(this);
  no_changes_label_->setText("URDF is ready for Gazebo. No changes required.");
  no_changes_label_->setFont(QFont(QFont().defaultFamily(), 18));
  no_changes_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  no_changes_label_->setAlignment(Qt::AlignTop);
  layout->addWidget(no_changes_label_);

  // URDF text
  simulation_text_ = new QTextEdit(this);
  simulation_text_->setLineWrapMode(QTextEdit::NoWrap);
  connect(simulation_text_, &QTextEdit::textChanged, this, [this]() { setDirty(); });
  layout->addWidget(simulation_text_);
  // Configure highlighter
  auto highlighter = new XmlSyntaxHighlighter(simulation_text_->document());
  QTextCharFormat format;
  format.setForeground(Qt::darkGreen);
  highlighter->addTag("inertial", format);
  highlighter->addTag("transmission", format);
  highlighter->addTag("gazebo", format);

  // Copy URDF link, hidden initially
  copy_urdf_ = new QLabel(this);
  copy_urdf_->setText("<a href='contract'>Copy to Clipboard</a>");
  connect(copy_urdf_, &QLabel::linkActivated, this, &SimulationWidget::copyURDF);
  layout->addWidget(copy_urdf_);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

void SimulationWidget::setDirty(bool dirty)
{
  if (dirty)
    config_data_->changes |= MoveItConfigData::SIMULATION;
  else
    config_data_->changes &= ~MoveItConfigData::SIMULATION;
  btn_overwrite_->setEnabled(dirty && !config_data_->urdf_from_xacro_);
}

void SimulationWidget::focusGiven()
{
  if (!simulation_text_->document()->isEmpty())
    return;  // don't change existing content

  simulation_text_->setVisible(true);
  std::string text = generateGazeboCompatibleURDF();
  config_data_->gazebo_urdf_string_ = text;

  simulation_text_->document()->setPlainText(QString::fromStdString(text));

  // Add generated Gazebo URDF to config file if not empty
  bool have_changes = !text.empty();

  // GUI elements are visible only if there are URDF changes to display/edit
  simulation_text_->setVisible(have_changes);
  btn_overwrite_->setVisible(have_changes);
  btn_open_->setVisible(have_changes);
  copy_urdf_->setVisible(have_changes);
  no_changes_label_->setVisible(!have_changes);

  // Explain why overwrite button is disabled
  QString tooltip;
  if (config_data_->urdf_from_xacro_)
    tooltip = tr("Cannot overwrite original, <i>xacro-based</i> URDF");
  else
    tooltip = tr("Overwrite URDF in original location:<br><tt>%1</tt>").arg(config_data_->urdf_path_.c_str());
  btn_overwrite_->setToolTip(tooltip);

  setDirty(have_changes);
}

bool SimulationWidget::focusLost()
{
  if (!(config_data_->changes & MoveItConfigData::SIMULATION))
    return true;  // saving is disabled anyway

  // validate XML
  TiXmlDocument doc;
  auto urdf = simulation_text_->document()->toPlainText().toStdString();
  doc.Parse(urdf.c_str(), nullptr, TIXML_ENCODING_UTF8);
  if (!urdf.empty() && doc.Error())
  {
    QTextCursor cursor = simulation_text_->textCursor();
    cursor.movePosition(QTextCursor::Start);
    cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, doc.ErrorRow());
    cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, doc.ErrorCol());
    simulation_text_->setTextCursor(cursor);
    QMessageBox::warning(this, tr("Gazebo URDF"), tr("Error parsing XML:\n").append(doc.ErrorDesc()));
    simulation_text_->setFocus(Qt::OtherFocusReason);
    return false;  // reject switching
  }
  else
    config_data_->gazebo_urdf_string_ = std::move(urdf);
  return true;
}

// ******************************************************************************************
// Called when save URDF button is clicked
// ******************************************************************************************
void SimulationWidget::overwriteURDF()
{
  if (!focusLost())  // validate XML
    return;

  if (!config_data_->outputGazeboURDFFile(config_data_->urdf_path_))
    QMessageBox::warning(this, "Gazebo URDF", tr("Failed to save to ").append(config_data_->urdf_path_.c_str()));
  else
  {
    // Remove Gazebo URDF file from list of to-be-written config files
    setDirty(false);
    config_data_->gazebo_urdf_string_.clear();
  }
}

void SimulationWidget::openURDF()
{
  QString editor = qgetenv("EDITOR");
  if (editor.isEmpty())
    editor = "xdg-open";
  auto command = QString("%1 %2").arg(editor, config_data_->urdf_path_.c_str());
  if (!QProcess::startDetached(command))
    QMessageBox::warning(this, "URDF Editor", tr("Failed to open editor: <pre>%1</pre>").arg(editor));
}

// ******************************************************************************************
// Called the copy to clipboard button is clicked
// ******************************************************************************************
void SimulationWidget::copyURDF()
{
  simulation_text_->selectAll();
  simulation_text_->copy();
}

// Generate a Gazebo-compatible robot URDF
std::string SimulationWidget::generateGazeboCompatibleURDF() const
{
  TiXmlDocument doc;
  doc.Parse(config_data_->urdf_string_.c_str(), nullptr, TIXML_ENCODING_UTF8);
  auto root = doc.RootElement();

  // Normalize original urdf_string_
  TiXmlPrinter orig_urdf;
  doc.Accept(&orig_urdf);

  // Map existing SimpleTransmission elements to their joint name
  std::map<std::string, TiXmlElement*> transmission_elements;
  for (TiXmlElement* element = root->FirstChildElement("transmission"); element != nullptr;
       element = element->NextSiblingElement(element->Value()))
  {
    auto type_tag = element->FirstChildElement("type");
    auto joint_tag = element->FirstChildElement("joint");
    if (!type_tag || !type_tag->GetText() || !joint_tag || !joint_tag->Attribute("name"))
      continue;  // ignore invalid tags
    if (std::string(type_tag->GetText()) == "transmission_interface/SimpleTransmission")
      transmission_elements[element->FirstChildElement("joint")->Attribute("name")] = element;
  }

  // Loop through Link and Joint elements and add Gazebo tags if not present
  for (TiXmlElement* element = root->FirstChildElement(); element != nullptr; element = element->NextSiblingElement())
  {
    const std::string tag_name(element->Value());
    if (tag_name == "link" && element->FirstChildElement("collision"))
    {
      TiXmlElement* inertial = uniqueInsert(*element, "inertial");
      uniqueInsert(*inertial, "mass", { { "value", "0.1" } });
      uniqueInsert(*inertial, "origin", { { "xyz", "0 0 0" }, { "rpy", "0 0 0" } });
      uniqueInsert(*inertial, "inertia",
                   { { "ixx", "0.03" },
                     { "iyy", "0.03" },
                     { "izz", "0.03" },
                     { "ixy", "0.0" },
                     { "ixz", "0.0" },
                     { "iyz", "0.0" } });
    }
    else if (tag_name == "joint")
    {
      const char* joint_type = element->Attribute("type");
      const char* joint_name = element->Attribute("name");
      if (!joint_type || !joint_name || strcmp(joint_type, "fixed") == 0)
        continue;  // skip invalid or fixed joints

      // find existing or create new transmission element for this joint
      TiXmlElement* transmission;
      auto it = transmission_elements.find(joint_name);
      if (it != transmission_elements.end())
        transmission = it->second;
      else
      {
        transmission = root->InsertEndChild(TiXmlElement("transmission"))->ToElement();
        transmission->SetAttribute("name", std::string("trans_") + joint_name);
      }

      uniqueInsert(*transmission, "type", {}, "transmission_interface/SimpleTransmission");

      std::string hw_interface = config_data_->getJointHardwareInterface(joint_name);
      auto* joint = uniqueInsert(*transmission, "joint", { { "name", joint_name } });
      uniqueInsert(*joint, "hardwareInterface", {}, hw_interface.c_str());

      auto actuator_name = joint_name + std::string("_motor");
      auto* actuator = uniqueInsert(*transmission, "actuator", { { "name", actuator_name.c_str() } });
      uniqueInsert(*actuator, "hardwareInterface", {}, hw_interface.c_str());
      uniqueInsert(*actuator, "mechanicalReduction", {}, "1");
    }
  }

  // Add gazebo_ros_control plugin which reads the transmission tags
  TiXmlElement* gazebo = uniqueInsert(*root, "gazebo");
  TiXmlElement* plugin = uniqueInsert(*gazebo, "plugin", { { "name", "gazebo_ros_control", true } });
  uniqueInsert(*plugin, "robotNamespace", {}, "/");

  // generate new URDF
  TiXmlPrinter new_urdf;
  doc.Accept(&new_urdf);
  // and return it when there are changes
  return orig_urdf.Str() == new_urdf.Str() ? std::string() : new_urdf.Str();
}

}  // namespace moveit_setup_assistant
