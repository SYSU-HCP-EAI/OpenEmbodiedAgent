/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, CITEC, Bielefeld University
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
 *   * Neither the name of CITEC / Bielefeld University nor the names of
 *     its contributors may be used to endorse or promote products derived
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
#include <moveit/robot_state/robot_state.h>
#include <Eigen/SVD>
#include <QAbstractItemModel>
#include <QWidget>
#include <QStyledItemDelegate>
#include <vector>
#include <memory>

class QSlider;

namespace Ui
{
class MotionPlanningFrameJointsUI;
}
namespace robot_interaction
{
MOVEIT_CLASS_FORWARD(InteractionHandler);
}
namespace moveit_rviz_plugin
{
/** TableModel to display joint values of a referenced RobotState.
 *
 * Unfortunately we cannot store the RobotStatePtr (and thus ensure existence of the state during
 * the lifetime of this class instance), because RobotInteraction (which is the initial use case)
 * allocates internally a new RobotState if any other copy is held somewhere else.
 * Hence, we also store an (unsafe) raw pointer. Lifetime of this raw pointer needs to be ensured.
 */
class JMGItemModel : public QAbstractTableModel
{
  Q_OBJECT
  moveit::core::RobotState robot_state_;
  const moveit::core::JointModelGroup* jmg_;

public:
  JMGItemModel(const moveit::core::RobotState& robot_state, const std::string& group_name, QObject* parent = nullptr);

  // QAbstractItemModel interface
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  Qt::ItemFlags flags(const QModelIndex& index) const override;
  QVariant data(const QModelIndex& index, int role) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
  bool setData(const QModelIndex& index, const QVariant& value, int role) override;

  /// call this on any external change of the RobotState
  void updateRobotState(const moveit::core::RobotState& state);

  moveit::core::RobotState& getRobotState()
  {
    return robot_state_;
  }
  const moveit::core::RobotState& getRobotState() const
  {
    return robot_state_;
  }
  const moveit::core::JointModelGroup* getJointModelGroup() const
  {
    return jmg_;
  }

private:
  /// retrieve the JointModel corresponding to the variable referenced by index
  const moveit::core::JointModel* getJointModel(const QModelIndex& index) const;
  /// retrieve the variable bounds referenced by variable index
  const moveit::core::VariableBounds* getVariableBounds(const moveit::core::JointModel* jm,
                                                        const QModelIndex& index) const;
};

class JointsWidgetEventFilter : public QObject
{
  Q_OBJECT
  QModelIndex active_;               // joint index being operated on
  float jmin_, jmax_, pmin_, pmax_;  // joint and pixel min/max values
  float delta_ = 0.0f;               // speed of joint value changes from keyboard interaction

public:
  JointsWidgetEventFilter(QAbstractItemView* view);

protected:
  bool eventFilter(QObject* target, QEvent* event) override;
};

class MotionPlanningDisplay;
class MotionPlanningFrameJointsWidget : public QWidget
{
  Q_OBJECT

public:
  MotionPlanningFrameJointsWidget(const MotionPlanningFrameJointsWidget&) = delete;
  MotionPlanningFrameJointsWidget(MotionPlanningDisplay* display, QWidget* parent = nullptr);
  ~MotionPlanningFrameJointsWidget() override;

  void clearRobotModel();
  void changePlanningGroup(const std::string& group_name,
                           const robot_interaction::InteractionHandlerPtr& start_state_handler,
                           const robot_interaction::InteractionHandlerPtr& goal_state_handler);

public Q_SLOTS:
  void queryStartStateChanged();
  void queryGoalStateChanged();
  void jogNullspace(double value);

protected:
  void setActiveModel(JMGItemModel* model);
  void triggerUpdate(JMGItemModel* model);
  void updateNullspaceSliders();
  QSlider* createNSSlider(int i);

private:
  Ui::MotionPlanningFrameJointsUI* ui_;
  MotionPlanningDisplay* planning_display_;
  robot_interaction::InteractionHandlerPtr start_state_handler_;
  robot_interaction::InteractionHandlerPtr goal_state_handler_;
  std::unique_ptr<JMGItemModel> start_state_model_;
  std::unique_ptr<JMGItemModel> goal_state_model_;
  // break circular loop of stateChanged() -> dataChanged() |-> PlanningDisplay::setQuery*State()
  bool ignore_state_changes_ = false;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::MatrixXd nullspace_;
  std::vector<QSlider*> ns_sliders_;
};

/// Delegate to show the joint value as with a progress bar indicator between min and max.
class ProgressBarDelegate : public QStyledItemDelegate
{
  Q_OBJECT

public:
  enum CustomRole
  {
    JointTypeRole = Qt::UserRole,  // NOLINT(readability-identifier-naming)
    VariableBoundsRole             // NOLINT(readability-identifier-naming)
  };

  ProgressBarDelegate(QWidget* parent = nullptr) : QStyledItemDelegate(parent)
  {
  }

  void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};

/// Slider that jumps back to zero
class JogSlider : public QSlider
{
  Q_OBJECT
  int timer_id_;
  int timer_interval_;  // ms
  double maximum_;

public:
  JogSlider(QWidget* parent = nullptr);

  int timerInterval() const
  {
    return timer_interval_;
  }
  void setTimerInterval(int ms);
  void setResolution(unsigned int resolution);
  void setMaximum(double value);
  double value() const
  {
    return QSlider::value() * maximum_ / QSlider::maximum();
  }

protected:
  void timerEvent(QTimerEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

private:
  using QSlider::setMaximum;
  using QSlider::setMinimum;
  using QSlider::setRange;

Q_SIGNALS:
  void triggered(double value);
};
}  // namespace moveit_rviz_plugin
