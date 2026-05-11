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

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include "ui_motion_planning_rviz_plugin_frame_joints.h"
#include <QPainter>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QMouseEvent>

namespace moveit_rviz_plugin
{
JMGItemModel::JMGItemModel(const moveit::core::RobotState& robot_state, const std::string& group_name, QObject* parent)
  : QAbstractTableModel(parent), robot_state_(robot_state), jmg_(nullptr)
{
  if (robot_state_.getRobotModel()->hasJointModelGroup(group_name))
    jmg_ = robot_state_.getRobotModel()->getJointModelGroup(group_name);
}

int JMGItemModel::rowCount(const QModelIndex& /*parent*/) const
{
  if (!jmg_)
    return robot_state_.getVariableCount();
  else
    return jmg_->getVariableCount();
}

int JMGItemModel::columnCount(const QModelIndex& /*parent*/) const
{
  return 2;
}

Qt::ItemFlags JMGItemModel::flags(const QModelIndex& index) const
{
  if (!index.isValid())
    return Qt::ItemFlags();

  Qt::ItemFlags f = QAbstractTableModel::flags(index);

  const moveit::core::JointModel* jm = getJointModel(index);
  bool is_editable = !jm->isPassive() && !jm->getMimic();
  f.setFlag(Qt::ItemIsEnabled, is_editable);
  if (index.column() == 1)
    f.setFlag(Qt::ItemIsEditable, is_editable);
  return f;
}

QVariant JMGItemModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid())
    return QVariant();

  int idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  switch (index.column())
  {
    case 0:  // joint name column
      switch (role)
      {
        case Qt::DisplayRole:
          return QString::fromStdString(robot_state_.getVariableNames()[idx]);
        case Qt::TextAlignmentRole:
          return Qt::AlignLeft;
      }
      break;
    case 1:  // joint value column
    {
      double value = robot_state_.getVariablePosition(idx);
      const moveit::core::JointModel* jm = robot_state_.getRobotModel()->getJointOfVariable(idx);
      switch (role)
      {
        case Qt::DisplayRole:
          return value;
        case Qt::EditRole:
          if (jm)
            return jm->getType() == moveit::core::JointModel::REVOLUTE ? value * 180 / M_PI : value;
          break;
        case ProgressBarDelegate::JointTypeRole:
          if (jm)
            return jm->getType();
          break;
        case ProgressBarDelegate::VariableBoundsRole:
          if (const moveit::core::VariableBounds* bounds = getVariableBounds(jm, index))
            return QPointF(bounds->min_position_, bounds->max_position_);
          break;
        case Qt::TextAlignmentRole:
          return Qt::AlignLeft;
      }
    }
  }
  return QVariant();
}

QVariant JMGItemModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    return section == 0 ? "Joint Name" : "Value";
  return QAbstractTableModel::headerData(section, orientation, role);
}

bool JMGItemModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if (index.column() != 1 || role != Qt::EditRole)
    return false;

  int var_idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  const moveit::core::JointModel* jm = robot_state_.getRobotModel()->getJointOfVariable(var_idx);
  if (!value.canConvert<double>())
    return false;

  bool ok;
  double v = value.toDouble(&ok);
  if (!ok)
    return false;

  // for revolute joints, we convert degrees to radians
  if (jm && jm->getType() == moveit::core::JointModel::REVOLUTE)
    v *= M_PI / 180;

  robot_state_.setVariablePosition(var_idx, v);
  jm->enforcePositionBounds(robot_state_.getVariablePositions() + jm->getFirstVariableIndex());
  dataChanged(index, index);
  return true;
}

const moveit::core::JointModel* JMGItemModel::getJointModel(const QModelIndex& index) const
{
  if (!index.isValid())
    return nullptr;
  int var_idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  return robot_state_.getRobotModel()->getJointOfVariable(var_idx);
}

const moveit::core::VariableBounds* JMGItemModel::getVariableBounds(const moveit::core::JointModel* jm,
                                                                    const QModelIndex& index) const
{
  if (!jm)
    return nullptr;
  int var_idx = jmg_ ? jmg_->getVariableIndexList()[index.row()] : index.row();
  const moveit::core::VariableBounds* bounds = &jm->getVariableBounds()[var_idx - jm->getFirstVariableIndex()];
  return bounds->position_bounded_ ? bounds : nullptr;
}

// copy positions from new_state and notify about these changes
void JMGItemModel::updateRobotState(const moveit::core::RobotState& state)
{
  if (robot_state_.getRobotModel() != state.getRobotModel())
    return;
  robot_state_ = state;
  dataChanged(index(0, 1), index(rowCount() - 1, 1));
}

MotionPlanningFrameJointsWidget::MotionPlanningFrameJointsWidget(MotionPlanningDisplay* display, QWidget* parent)
  : QWidget(parent), ui_(new Ui::MotionPlanningFrameJointsUI()), planning_display_(display)
{
  ui_->setupUi(this);
  // intercept mouse events delivered to joints_view_ to operate "sliders"
  ui_->joints_view_->viewport()->installEventFilter(new JointsWidgetEventFilter(ui_->joints_view_));
  // intercept keyboard events delivered to joints_view_ to operate joints directly
  ui_->joints_view_->installEventFilter(new JointsWidgetEventFilter(ui_->joints_view_));
  ui_->joints_view_->setItemDelegateForColumn(1, new ProgressBarDelegate(this));
  svd_.setThreshold(0.001);
}

MotionPlanningFrameJointsWidget::~MotionPlanningFrameJointsWidget()
{
  delete ui_;
}

void MotionPlanningFrameJointsWidget::clearRobotModel()
{
  ui_->joints_view_->setModel(nullptr);
  start_state_handler_.reset();
  goal_state_handler_.reset();
  start_state_model_.reset();
  goal_state_model_.reset();
}

void MotionPlanningFrameJointsWidget::changePlanningGroup(
    const std::string& group_name, const robot_interaction::InteractionHandlerPtr& start_state_handler,
    const robot_interaction::InteractionHandlerPtr& goal_state_handler)
{
  // release previous models (if any)
  clearRobotModel();
  // create new models
  start_state_handler_ = start_state_handler;
  goal_state_handler_ = goal_state_handler;
  start_state_model_ = std::make_unique<JMGItemModel>(*start_state_handler_->getState(), group_name, this);
  goal_state_model_ = std::make_unique<JMGItemModel>(*goal_state_handler_->getState(), group_name, this);

  // forward model updates to the PlanningDisplay
  connect(start_state_model_.get(), &JMGItemModel::dataChanged, this, [this]() {
    if (!ignore_state_changes_)
      planning_display_->setQueryStartState(start_state_model_->getRobotState());
  });
  connect(goal_state_model_.get(), &JMGItemModel::dataChanged, this, [this]() {
    if (!ignore_state_changes_)
      planning_display_->setQueryGoalState(goal_state_model_->getRobotState());
  });

  // show the goal state by default
  setActiveModel(goal_state_model_.get());
  updateNullspaceSliders();
}

void MotionPlanningFrameJointsWidget::queryStartStateChanged()
{
  if (!start_state_model_ || !start_state_handler_)
    return;
  ignore_state_changes_ = true;
  start_state_model_->updateRobotState(*start_state_handler_->getState());
  ignore_state_changes_ = false;
  setActiveModel(start_state_model_.get());
  updateNullspaceSliders();
}

void MotionPlanningFrameJointsWidget::queryGoalStateChanged()
{
  if (!goal_state_model_ || !goal_state_handler_)
    return;
  ignore_state_changes_ = true;
  goal_state_model_->updateRobotState(*goal_state_handler_->getState());
  ignore_state_changes_ = false;
  setActiveModel(goal_state_model_.get());
  updateNullspaceSliders();
}

void MotionPlanningFrameJointsWidget::setActiveModel(JMGItemModel* model)
{
  ui_->joints_view_->setModel(model);
  ui_->joints_view_label_->setText(
      QString("Group joints of %1 state").arg(model == start_state_model_.get() ? "start" : "goal"));
}

void MotionPlanningFrameJointsWidget::triggerUpdate(JMGItemModel* model)
{
  if (model == start_state_model_.get())
    planning_display_->setQueryStartState(model->getRobotState());
  else
    planning_display_->setQueryGoalState(model->getRobotState());
}

// Find matching key vector in columns of haystack and return the best-aligned column index.
// Only consider available indexes (> 0). If no match is found, return largest available index.
// sign becomes -1 if key has changed sign compared to the matching haystack column.
static Eigen::Index findMatching(const Eigen::VectorXd& key, const Eigen::MatrixXd& haystack,
                                 const Eigen::VectorXi& available, double& sign)
{
  Eigen::Index result = available.array().maxCoeff();
  double best_match = 0.0;
  for (unsigned int i = 0; i < available.rows(); ++i)
  {
    int index = available[i];
    if (index < 0)  // index already taken
      continue;
    if (index >= haystack.cols())
      return result;
    double match = haystack.col(available[i]).transpose() * key;
    double abs_match = std::abs(match);
    if (abs_match > 0.5 && abs_match > best_match)
    {
      best_match = abs_match;
      result = index;
      sign = match > 0 ? 1.0 : -1.0;
    }
  }
  return result;
}

void MotionPlanningFrameJointsWidget::updateNullspaceSliders()
{
  JMGItemModel* model = dynamic_cast<JMGItemModel*>(ui_->joints_view_->model());
  std::size_t i = 0;
  if (model && model->getJointModelGroup() && model->getJointModelGroup()->isChain())
  {
    model->getRobotState().updateLinkTransforms();
    Eigen::MatrixXd jacobian;
    if (!model->getRobotState().getJacobian(model->getJointModelGroup(),
                                            model->getJointModelGroup()->getLinkModels().back(),
                                            Eigen::Vector3d::Zero(), jacobian, false))
      goto cleanup;

    svd_.compute(jacobian, Eigen::ComputeFullV);
    Eigen::Index rank = svd_.rank();
    std::size_t ns_dim = svd_.cols() - rank;
    Eigen::MatrixXd ns(svd_.cols(), ns_dim);
    Eigen::VectorXi available(ns_dim);
    for (std::size_t j = 0; j < ns_dim; ++j)
      available[j] = j;

    ns_sliders_.reserve(ns_dim);
    // create/unhide sliders
    for (; i < ns_dim; ++i)
    {
      if (i >= ns_sliders_.size())
        ns_sliders_.push_back(createNSSlider(i + 1));
      ns_sliders_[i]->show();

      // Find matching null-space basis vector in previous nullspace_
      double sign = 1.0;
      const Eigen::VectorXd& current = svd_.matrixV().col(rank + i);
      Eigen::Index index = findMatching(current, nullspace_, available, sign);
      ns.col(index).noalias() = sign * current;
      available[index] = -1;  // mark index as taken
    }
    nullspace_ = ns;
  }

cleanup:
  if (i == 0)
    nullspace_.resize(0, 0);

  // show/hide dummy slider
  ui_->dummy_ns_slider_->setVisible(i == 0);

  // hide remaining sliders
  for (; i < ns_sliders_.size(); ++i)
    ns_sliders_[i]->hide();
}

QSlider* MotionPlanningFrameJointsWidget::createNSSlider(int i)
{
  JogSlider* slider = new JogSlider(this);
  slider->setOrientation(Qt::Horizontal);
  slider->setMaximum(0.1);
  slider->setToolTip(QString("Nullspace dim #%1").arg(i));
  ui_->nullspace_layout_->addWidget(slider);
  connect(slider, SIGNAL(triggered(double)), this, SLOT(jogNullspace(double)));
  return slider;
}

void MotionPlanningFrameJointsWidget::jogNullspace(double value)
{
  if (value == 0)
    return;

  std::size_t index = std::find(ns_sliders_.begin(), ns_sliders_.end(), sender()) - ns_sliders_.begin();
  if (static_cast<int>(index) >= nullspace_.cols())
    return;

  JMGItemModel* model = dynamic_cast<JMGItemModel*>(ui_->joints_view_->model());
  if (!model)
    return;

  Eigen::VectorXd values;
  model->getRobotState().copyJointGroupPositions(model->getJointModelGroup(), values);
  values += value * nullspace_.col(index);
  model->getRobotState().setJointGroupPositions(model->getJointModelGroup(), values);
  model->getRobotState().harmonizePositions(model->getJointModelGroup());
  triggerUpdate(model);
}

namespace
{
void paintProgressBar(QPainter* painter, QStyle* style, const QString& text, float value, float min, float max,
                      const QRect& rect)
{
  QStyleOptionProgressBar opt;
  opt.rect = rect;
  opt.minimum = 0;
  opt.maximum = 1000;
  opt.progress = 1000. * (value - min) / (max - min);
  opt.text = text;
  opt.textAlignment = Qt::AlignCenter;
  opt.textVisible = true;
  style->drawControl(QStyle::CE_ProgressBar, &opt, painter);
}
}  // namespace

void ProgressBarDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
  // copied from QStyledItemDelegate::paint
  QStyle* style = option.widget ? option.widget->style() : QApplication::style();
  QStyleOptionViewItem style_option = option;
  initStyleOption(&style_option, index);

  if (index.column() == 1)
  {
    QVariant joint_type = index.data(JointTypeRole);
    double value = index.data().toDouble();
    if (joint_type.isValid())
    {
      switch (joint_type.toInt())
      {
        case moveit::core::JointModel::REVOLUTE:
          style_option.text = option.locale.toString(value * 180 / M_PI, 'f', 0).append("°");
          break;
        case moveit::core::JointModel::PRISMATIC:
          style_option.text = option.locale.toString(value, 'f', 3).append("m");
          break;
        default:
          break;
      }
    }

    QVariant vbounds = index.data(VariableBoundsRole);
    if (vbounds.isValid())
    {
      QPointF bounds = vbounds.toPointF();
      const float min = bounds.x();
      const float max = bounds.y();
      paintProgressBar(painter, style, style_option.text, value, min, max, style_option.rect);
      return;
    }
  }

  style->drawControl(QStyle::CE_ItemViewItem, &style_option, painter, option.widget);
}

QWidget* ProgressBarDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                                           const QModelIndex& index) const

{
  auto editor = QStyledItemDelegate::createEditor(parent, option, index);
  if (auto spinbox = qobject_cast<QDoubleSpinBox*>(editor))
  {
    bool is_revolute = (index.data(ProgressBarDelegate::JointTypeRole).toInt() == moveit::core::JointModel::REVOLUTE);
    spinbox->setSuffix(is_revolute ? "°" : "m");
  }
  return editor;
}

JointsWidgetEventFilter::JointsWidgetEventFilter(QAbstractItemView* view) : QObject(view)
{
}

bool JointsWidgetEventFilter::eventFilter(QObject* /*target*/, QEvent* event)
{
  if ((event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseMove) &&
      static_cast<QMouseEvent*>(event)->buttons() & Qt::LeftButton)
  {
    QAbstractItemView* view = qobject_cast<QAbstractItemView*>(parent());
    if (event->type() == QEvent::MouseButtonPress)
    {  // start dragging the "slider"
      QModelIndex index = view->indexAt(static_cast<QMouseEvent*>(event)->pos());
      QVariant vbounds = index.data(ProgressBarDelegate::VariableBoundsRole);
      view->setCurrentIndex(index);
      if (!index.isValid() || !(index.flags() & Qt::ItemIsEditable))
        return false;
      if (!vbounds.isValid())
      {
        view->edit(index);
        return false;
      }

      active_ = index;
      const QRect& rect = view->visualRect(active_);
      pmin_ = rect.x();
      pmax_ = rect.x() + rect.width();
      if (vbounds.isValid())
      {
        QPointF bounds = vbounds.toPointF();
        jmin_ = bounds.x();
        jmax_ = bounds.y();
      }
      bool is_revolute = (index.data(ProgressBarDelegate::JointTypeRole).toInt() == moveit::core::JointModel::REVOLUTE);
      if (is_revolute)
      {
        jmin_ *= 180. / M_PI;
        jmax_ *= 180. / M_PI;
      }
    }
    else if (!active_.isValid())
      return false;

    float v = jmin_ + (static_cast<QMouseEvent*>(event)->x() - pmin_) * (jmax_ - jmin_) / (pmax_ - pmin_);
    view->model()->setData(active_, v, Qt::EditRole);
    return true;  // event handled
  }
  else if (event->type() == QEvent::MouseButtonRelease && static_cast<QMouseEvent*>(event)->button() == Qt::LeftButton)
    active_ = QModelIndex();

  else if (event->type() == QEvent::KeyPress)
  {
    QKeyEvent* key_event = static_cast<QKeyEvent*>(event);
    if (key_event->key() != Qt::Key_Left && key_event->key() != Qt::Key_Right)
      return false;  // only react to KeyLeft / KeyRight events

    QAbstractItemView* view = qobject_cast<QAbstractItemView*>(parent());
    QModelIndex index = view->currentIndex();
    index = index.sibling(index.row(), 1);

    if (key_event->type() == QEvent::KeyPress && key_event->modifiers() == Qt::NoModifier &&
        index.flags() & Qt::ItemIsEditable)
    {
      if (!key_event->isAutoRepeat())  // first key press: initialize delta_ from joint type and direction
      {
        delta_ = key_event->key() == Qt::Key_Left ? -0.1f : 0.1f;
        if (index.data(ProgressBarDelegate::JointTypeRole).toInt() == moveit::core::JointModel::PRISMATIC)
          delta_ *= 0.01;
      }
      else  // increase delta in a multiplicative fashion when holding down the key
      {
        delta_ *= 1.1;
      }
    }

    float current = index.data(Qt::EditRole).toFloat();
    view->model()->setData(index, current + delta_, Qt::EditRole);
    return true;
  }
  return false;
}

JogSlider::JogSlider(QWidget* parent) : QSlider(parent)
{
  setTimerInterval(50);
  setResolution(1000);
  setMaximum(1.0);
}

void JogSlider::setTimerInterval(int ms)
{
  timer_interval_ = ms;
}

void JogSlider::setResolution(unsigned int resolution)
{
  QSlider::setRange(-resolution, +resolution);
}

void JogSlider::setMaximum(double value)
{
  maximum_ = value;
}

void JogSlider::timerEvent(QTimerEvent* event)
{
  QSlider::timerEvent(event);
  if (event->timerId() == timer_id_)
    triggered(value());
}

void JogSlider::mousePressEvent(QMouseEvent* event)
{
  QSlider::mousePressEvent(event);
  timer_id_ = startTimer(timer_interval_);
}

void JogSlider::mouseReleaseEvent(QMouseEvent* event)
{
  killTimer(timer_id_);
  QSlider::mouseReleaseEvent(event);
  setValue(0);
}

}  // namespace moveit_rviz_plugin
