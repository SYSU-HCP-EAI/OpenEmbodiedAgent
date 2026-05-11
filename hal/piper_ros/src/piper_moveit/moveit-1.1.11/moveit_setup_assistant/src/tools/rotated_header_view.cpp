/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CITEC, Bielefeld University
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

#include "rotated_header_view.h"
#include <QPainter>

namespace moveit_setup_assistant
{
RotatedHeaderView::RotatedHeaderView(Qt::Orientation orientation, QWidget* parent) : QHeaderView(orientation, parent)
{
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
  setSectionsClickable(true);
  setSectionResizeMode(Fixed);
#else
  setClickable(true);
  setResizeMode(Fixed);
#endif
  setDefaultSectionSize(27);
}

void RotatedHeaderView::paintSection(QPainter* painter, const QRect& rect, int logicalIndex) const
{
  if (orientation() == Qt::Vertical)
  {
    QHeaderView::paintSection(painter, rect, logicalIndex);
    return;
  }

  painter->save();
  // rotate about (x,y)
  painter->translate(rect.x(), rect.y());
  painter->rotate(-90);
  painter->translate(-rect.height(), 0);
  QHeaderView::paintSection(painter, QRect(0, 0, rect.height(), rect.width()), logicalIndex);
  painter->restore();
}

QSize RotatedHeaderView::sectionSizeFromContents(int logicalIndex) const
{
  if (orientation() == Qt::Vertical)
    return QHeaderView::sectionSizeFromContents(logicalIndex);

  Q_ASSERT(logicalIndex >= 0);

  ensurePolished();

  // use SizeHintRole
  QVariant variant = model()->headerData(logicalIndex, Qt::Vertical, Qt::SizeHintRole);
  if (variant.isValid())
    return qvariant_cast<QSize>(variant);

  // otherwise use the contents
  QStyleOptionHeader opt;
  initStyleOption(&opt);
  opt.section = logicalIndex;
  QVariant var = model()->headerData(logicalIndex, orientation(), Qt::FontRole);
  QFont fnt;
  if (var.isValid() && var.canConvert<QFont>())
    fnt = qvariant_cast<QFont>(var);
  else
    fnt = font();
  fnt.setBold(true);
  opt.fontMetrics = QFontMetrics(fnt);
  opt.text = model()->headerData(logicalIndex, orientation(), Qt::DisplayRole).toString();
  variant = model()->headerData(logicalIndex, orientation(), Qt::DecorationRole);
  opt.icon = qvariant_cast<QIcon>(variant);
  if (opt.icon.isNull())
    opt.icon = qvariant_cast<QPixmap>(variant);
  QSize size = style()->sizeFromContents(QStyle::CT_HeaderSection, &opt, QSize(), this);
  if (isSortIndicatorShown())
  {
    int margin = style()->pixelMetric(QStyle::PM_HeaderMargin, &opt, this);
    if (orientation() == Qt::Horizontal)
      size.rwidth() += size.height() + margin;
    else
      size.rheight() += size.width() + margin;
  }
  return QSize(size.height(), size.width());
}

int RotatedHeaderView::sectionSizeHint(int logicalIndex) const
{
  if (isSectionHidden(logicalIndex))
    return 0;
  if (logicalIndex < 0 || logicalIndex >= count())
    return -1;
  QSize size;
  QVariant value = model()->headerData(logicalIndex, orientation(), Qt::SizeHintRole);
  if (value.isValid())
    size = qvariant_cast<QSize>(value);
  else
    size = sectionSizeFromContents(logicalIndex);
  int hint = size.height();
  return qMax(minimumSectionSize(), hint);
}
}  // namespace moveit_setup_assistant
