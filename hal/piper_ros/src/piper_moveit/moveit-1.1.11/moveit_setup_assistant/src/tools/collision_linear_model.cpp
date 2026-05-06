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

#include "collision_linear_model.h"
#include "collision_matrix_model.h"

#include <QItemSelection>
#include <QPainter>
#include <cmath>

CollisionLinearModel::CollisionLinearModel(CollisionMatrixModel* src, QObject* parent) : QAbstractProxyModel(parent)
{
  setSourceModel(src);
}
CollisionLinearModel::~CollisionLinearModel()
{
  delete sourceModel();
}

QModelIndex CollisionLinearModel::mapFromSource(const QModelIndex& sourceIndex) const
{
  // map (row,column) index to linear index k
  // http://stackoverflow.com/questions/27086195/linear-index-upper-triangular-matrix
  int r = sourceIndex.row(), c = sourceIndex.column();
  int n = this->sourceModel()->columnCount();
  if (r == c)
    return QModelIndex();  // main diagonal elements are invalid
  if (r > c)               // only consider upper triagonal matrix
    std::swap(r, c);       // swap r,c if below diagonal

  int k = (n * (n - 1) / 2) - (n - r) * ((n - r) - 1) / 2 + c - r - 1;
  return index(k, 2);
}

QModelIndex CollisionLinearModel::mapToSource(const QModelIndex& proxyIndex) const
{
  // map linear index k to (row, column)
  // http://stackoverflow.com/questions/27086195/linear-index-upper-triangular-matrix
  int n = sourceModel()->columnCount();
  int k = proxyIndex.row();  // linear (row) index
  int r = n - 2 - (int)(sqrt(-8 * k + 4 * n * (n - 1) - 7) / 2.0 - 0.5);
  int c = k + r + 1 - n * (n - 1) / 2 + (n - r) * ((n - r) - 1) / 2;
  return sourceModel()->index(r, c);
}

int CollisionLinearModel::rowCount(const QModelIndex& /*parent*/) const
{
  int n = this->sourceModel()->rowCount();
  return (n * (n - 1) / 2);
}

int CollisionLinearModel::columnCount(const QModelIndex& /*parent*/) const
{
  return 4;
}

QModelIndex CollisionLinearModel::index(int row, int column, const QModelIndex& /*parent*/) const
{
  return createIndex(row, column);
}

QModelIndex CollisionLinearModel::parent(const QModelIndex& /*child*/) const
{
  return QModelIndex();
}

QVariant CollisionLinearModel::data(const QModelIndex& index, int role) const
{
  QModelIndex src_index = this->mapToSource(index);
  switch (index.column())
  {
    case 0:  // link name 1
      if (role != Qt::DisplayRole)
        return QVariant();
      else
        return this->sourceModel()->headerData(src_index.row(), Qt::Horizontal, Qt::DisplayRole);
    case 1:  // link name 2
      if (role != Qt::DisplayRole)
        return QVariant();
      return this->sourceModel()->headerData(src_index.column(), Qt::Vertical, Qt::DisplayRole);
    case 2:  // checkbox
      if (role != Qt::CheckStateRole)
        return QVariant();
      else
        return this->sourceModel()->data(src_index, Qt::CheckStateRole);
    case 3:  // reason
      if (role != Qt::DisplayRole)
        return QVariant();
      else
        return this->sourceModel()->data(src_index, Qt::ToolTipRole);
  }
  return QVariant();
}

bool CollisionLinearModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if (role == Qt::CheckStateRole)
  {
    QModelIndex src_index = this->mapToSource(index);
    if (sourceModel()->setData(src_index, value, role))
    {
      int r = index.row();
      Q_EMIT dataChanged(this->index(r, 2), this->index(r, 3));  // reason changed too
      return true;
    }
  }
  return false;  // reject all other changes
}

void CollisionLinearModel::setEnabled(const QItemSelection& selection, bool value)
{
  for (const auto idx : selection.indexes())
  {
    if (idx.column() != 2)  // only consider checkbox indexes
      continue;
    setData(idx, value ? Qt::Checked : Qt::Unchecked, Qt::CheckStateRole);
  }
}

Qt::ItemFlags CollisionLinearModel::flags(const QModelIndex& index) const
{
  if (index.column() == 2)
    return Qt::ItemIsUserCheckable | QAbstractItemModel::flags(index);
  else
    return QAbstractItemModel::flags(index);
}

QVariant CollisionLinearModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (role != Qt::DisplayRole)
    return QVariant();

  if (orientation == Qt::Horizontal)
  {
    switch (section)
    {
      case 0:
        return "Link A";
      case 1:
        return "Link B";
      case 2:
        return "Disabled";
      case 3:
        return "Reason to Disable";
    }
  }
  else if (orientation == Qt::Vertical)
  {
    return section + 1;
  }
  return QVariant();
}

SortFilterProxyModel::SortFilterProxyModel(QObject* parent) : QSortFilterProxyModel(parent), show_all_(false)
{
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
  connect(this, SIGNAL(sourceModelChanged()), this, SLOT(initSorting()));
#endif

  // by default: sort by link A (col 0), then link B (col 1)
  sort_columns_ << 0 << 1;
  sort_orders_ << Qt::AscendingOrder << Qt::AscendingOrder;
}

QVariant SortFilterProxyModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (role == Qt::DisplayRole && orientation == Qt::Vertical)
    return section + 1;  // simply enumerate rows
  else
    return QSortFilterProxyModel::headerData(section, orientation, role);
}

void SortFilterProxyModel::setEnabled(const QItemSelection& selection, bool value)
{
  static_cast<CollisionLinearModel*>(sourceModel())->setEnabled(mapSelectionToSource(selection), value);
}

void SortFilterProxyModel::initSorting()
{
  int cols = sourceModel()->columnCount();
  int prev_size = sort_columns_.size();
  sort_columns_.resize(cols);
  sort_orders_.resize(cols);

  // initialize new entries to -1
  for (int i = prev_size, end = sort_columns_.size(); i < end; ++i)
    sort_columns_[i] = -1;
}

void SortFilterProxyModel::setShowAll(bool show_all)
{
  if (show_all_ == show_all)
    return;

  show_all_ = show_all;
  invalidateFilter();
}

bool SortFilterProxyModel::filterAcceptsRow(int source_row, const QModelIndex& source_parent) const
{
  CollisionLinearModel* m = qobject_cast<CollisionLinearModel*>(sourceModel());
  if (!(show_all_ || m->data(m->index(source_row, 2), Qt::CheckStateRole) == Qt::Checked))
    return false;  // not accepted due to check state

  const QRegExp regexp = this->filterRegExp();
  if (regexp.isEmpty())
    return true;

  return m->data(m->index(source_row, 0, source_parent), Qt::DisplayRole).toString().contains(regexp) ||
         m->data(m->index(source_row, 1, source_parent), Qt::DisplayRole).toString().contains(regexp);
}

// define a fallback comparison operator for QVariants
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
namespace
{
bool operator<(const QVariant& left, const QVariant& right)
{
  if (left.userType() == QVariant::Type::Int)
    return left.toInt() < right.toInt();
  else
    return left.toString() < right.toString();
}
}  // namespace
#endif

bool SortFilterProxyModel::lessThan(const QModelIndex& src_left, const QModelIndex& src_right) const
{
  int row_left = src_left.row();
  int row_right = src_right.row();
  QAbstractItemModel* m = sourceModel();

  for (int i = 0, end = sort_columns_.size(); i < end && sort_columns_[i] >= 0; ++i)
  {
    int sc = sort_columns_[i];
    int role = sc == 2 ? Qt::CheckStateRole : Qt::DisplayRole;
    QVariant value_left = m->data(m->index(row_left, sc), role);
    QVariant value_right = m->data(m->index(row_right, sc), role);

    if (value_left == value_right)
      continue;

    bool smaller = (value_left < value_right);
    if (sort_orders_[i] == Qt::DescendingOrder)
      smaller = !smaller;
    return smaller;
  }
  return false;
}

void SortFilterProxyModel::sort(int column, Qt::SortOrder order)
{
  beginResetModel();
  if (column < 0)
    initSorting();
  else
  {
    // remember sorting history
    int prev_idx = sort_columns_.indexOf(column);
    if (prev_idx < 0)
      prev_idx = sort_columns_.size() - 1;
    // remove old entries
    sort_columns_.remove(prev_idx);
    sort_orders_.remove(prev_idx);
    // add new entries at front
    sort_columns_.insert(0, column);
    sort_orders_.insert(0, order);
  }
  QSortFilterProxyModel::sort(column, Qt::AscendingOrder);
  endResetModel();
}
