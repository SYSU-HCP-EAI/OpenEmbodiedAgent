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

#include "navigation_widget.h"
#include <QApplication>
#include <QPainter>
#include <QScrollBar>
#include <QStandardItemModel>
#include <iostream>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// CLASS
// ******************************************************************************************

NavigationWidget::NavigationWidget(QWidget* parent) : QListView(parent)
{
  setItemDelegate(new NavDelegate(this));
  setEditTriggers(QAbstractItemView::NoEditTriggers);

  // setAttribute(Qt::WA_MacShowFocusRect, false);

  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Hard code width and height
  setMaximumWidth(160);
  setMinimumWidth(160);
  setMinimumHeight(300);

  verticalScrollBar()->setPageStep(3);
  verticalScrollBar()->setSingleStep(1);

  model_ = new QStandardItemModel(this);
  setModel(model_);
}

void NavigationWidget::setNavs(const QList<QString>& navs)
{
  setModel(nullptr);
  model_->clear();

  for (const QString& nav : navs)
  {
    QStandardItem* item = new QStandardItem();
    item->setData(QVariant::fromValue(nav), Qt::DisplayRole);
    item->setFlags(Qt::NoItemFlags);
    model_->appendRow(item);
  }

  setModel(model_);
}

void NavigationWidget::setEnabled(const int& index, bool enabled)
{
  if (enabled)
    model_->item(index)->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsDragEnabled |
                                  Qt::ItemIsDropEnabled | Qt::ItemIsEnabled);
  else
    model_->item(index)->setFlags(Qt::NoItemFlags);
}

void NavigationWidget::setSelected(const int& index)
{
  // First make sure item is enabled
  setEnabled(index, true);

  // Select one box from column 0, row index
  QModelIndex top = model_->index(index, 0, QModelIndex());
  QModelIndex bottom = model_->index(index, 0, QModelIndex());

  QItemSelection selection(top, bottom);
  selectionModel()->reset();  // set them all to deselected
  selectionModel()->select(selection, QItemSelectionModel::Select);
}

// ******************************************************************************************
// CLASS
// ******************************************************************************************

NavDelegate::NavDelegate(QObject* parent) : QStyledItemDelegate(parent)
{
}

QSize NavDelegate::sizeHint(const QStyleOptionViewItem& option, const QModelIndex& /*index*/) const
{
  return QSize(option.rect.width(), 45);
}

void NavDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
  const bool is_selected = option.state & QStyle::State_Selected;
  const QPalette& palette = QApplication::palette();

  QString nav_name = displayText(index.data(), option.locale);

  painter->save();

  // draw background gradient
  QLinearGradient background_gradient(option.rect.topLeft(), option.rect.bottomLeft());
  if (is_selected)
  {
    background_gradient.setColorAt(0, palette.color(QPalette::Highlight).lighter(125));
    background_gradient.setColorAt(1, palette.color(QPalette::Highlight));
    painter->fillRect(option.rect, QBrush(background_gradient));
  }
  else
  {
    background_gradient.setColorAt(0, palette.color(QPalette::Light));
    background_gradient.setColorAt(1, palette.color(QPalette::Light).darker(105));
    painter->fillRect(option.rect, QBrush(background_gradient));
  }

  if (!is_selected)  // draw shadow
  {
    painter->setPen(palette.color(QPalette::Button));
    painter->drawLine(option.rect.topLeft(), option.rect.topRight());
    painter->setPen(palette.color(QPalette::Light));
    const QPoint offset(0, 1);
    painter->drawLine(option.rect.topLeft() + offset, option.rect.topRight() + offset);
  }

  QRect text_rect(option.rect.x() + 10, option.rect.y(), option.rect.width() - 10, option.rect.height());
  QFont text_font(painter->font());
  text_font.setPixelSize(14);  // Set font size
  painter->setFont(text_font);

  // Font color
  if (is_selected)
    painter->setPen(palette.color(QPalette::HighlightedText));
  else if (!option.state.testFlag(QStyle::State_Enabled))
    painter->setPen(palette.color(QPalette::Dark));
  else
    painter->setPen(palette.color(QPalette::ButtonText));

  painter->drawText(text_rect, Qt::AlignLeft | Qt::AlignVCenter, nav_name);

  painter->restore();
}
}  // namespace moveit_setup_assistant
