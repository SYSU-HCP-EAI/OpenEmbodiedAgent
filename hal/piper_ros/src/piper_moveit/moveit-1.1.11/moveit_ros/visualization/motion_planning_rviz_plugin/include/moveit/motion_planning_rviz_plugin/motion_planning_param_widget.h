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

/* Author: Robert Haschke */

#pragma once

#include <moveit/macros/class_forward.h>
#include <rviz/properties/property_tree_widget.h>

namespace moveit
{
namespace planning_interface
{
MOVEIT_CLASS_FORWARD(MoveGroupInterface);  // Defines MoveGroupInterfacePtr, ConstPtr, WeakPtr... etc
}
}  // namespace moveit

namespace moveit_rviz_plugin
{
class MotionPlanningParamWidget : public rviz::PropertyTreeWidget
{
  Q_OBJECT
public:
  MotionPlanningParamWidget(const MotionPlanningParamWidget&) = delete;
  MotionPlanningParamWidget(QWidget* parent = nullptr);
  ~MotionPlanningParamWidget() override;

  void setMoveGroup(const moveit::planning_interface::MoveGroupInterfacePtr& mg);
  void setGroupName(const std::string& group_name);

public Q_SLOTS:
  void setPlannerId(const std::string& planner_id);

private Q_SLOTS:
  void changedValue();

private:
  rviz::Property* createPropertyTree();

private:
  rviz::PropertyTreeModel* property_tree_model_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  std::string group_name_;
  std::string planner_id_;
};
}  // namespace moveit_rviz_plugin
