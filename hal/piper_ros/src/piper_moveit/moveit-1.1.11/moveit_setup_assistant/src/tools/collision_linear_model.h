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

#pragma once

#include <QAbstractProxyModel>
#include <QSortFilterProxyModel>
#include <QVector>

#include "collision_matrix_model.h"

class CollisionLinearModel : public QAbstractProxyModel
{
  Q_OBJECT

public:
  CollisionLinearModel(CollisionMatrixModel* src, QObject* parent = nullptr);
  ~CollisionLinearModel() override;

  // reimplement to return the model index in the proxy model that to the sourceIndex from the source model
  QModelIndex mapFromSource(const QModelIndex& sourceIndex) const override;
  // Reimplement this function to return the model index in the source model that corresponds to the proxyIndex in the
  // proxy model
  QModelIndex mapToSource(const QModelIndex& proxyIndex) const override;

  int rowCount(const QModelIndex& parent) const override;
  int columnCount(const QModelIndex& parent) const override;

  QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex& child) const override;
  QVariant data(const QModelIndex& index, int role) const override;

  bool setData(const QModelIndex& index, const QVariant& value, int role) override;
  void setEnabled(const QItemSelection& selection, bool value);

  Qt::ItemFlags flags(const QModelIndex& index) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
};

/** proxy model to allow for sorting of CollisionLinearModel, considering sorting history */
class SortFilterProxyModel : public QSortFilterProxyModel
{
  Q_OBJECT

public:
  SortFilterProxyModel(QObject* parent = nullptr);
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
  void sort(int column, Qt::SortOrder order) override;
  void setShowAll(bool show_all);
  void setEnabled(const QItemSelection& selection, bool value);

protected:
  bool filterAcceptsRow(int source_row, const QModelIndex& source_parent) const override;
  bool lessThan(const QModelIndex& src_left, const QModelIndex& src_right) const override;

private Q_SLOTS:
  void initSorting();

private:
  bool show_all_;
  QVector<int> sort_columns_;  // sorting history
  QVector<int> sort_orders_;   // corresponding sort orders
};
