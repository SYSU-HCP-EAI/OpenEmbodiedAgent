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

#include <algorithm>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QTableView>
#include <QVBoxLayout>

#include "default_collisions_widget.h"
#include "header_widget.h"
#include "../tools/collision_matrix_model.h"
#include "../tools/collision_linear_model.h"
#include "../tools/rotated_header_view.h"
#include <ros/console.h>

namespace
{
// ******************************************************************************************
// Convert LinkPairMap to SRDF
// ******************************************************************************************
void linkPairsToSRDF(const moveit_setup_assistant::LinkPairMap& pairs, srdf::SRDFWriter& srdf)
{
  // reset the data in the SRDF Writer class
  srdf.disabled_collision_pairs_.clear();

  // Create temp disabled collision
  srdf::Model::CollisionPair dc;

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::CollisionPair format
  for (const auto& item : pairs)
  {
    // Only copy those that are actually disabled
    if (item.second.disable_check)
    {
      dc.link1_ = item.first.first;
      dc.link2_ = item.first.second;
      dc.reason_ = moveit_setup_assistant::disabledReasonToString(item.second.reason);
      srdf.disabled_collision_pairs_.push_back(dc);
    }
  }
}
}  // namespace

namespace moveit_setup_assistant
{
// ******************************************************************************************
// User interface for editing the default collision matrix list in an SRDF
// ******************************************************************************************
DefaultCollisionsWidget::DefaultCollisionsWidget(QWidget* parent, const MoveItConfigDataPtr& config_data)
  : SetupScreenWidget(parent), model_(nullptr), selection_model_(nullptr), worker_(nullptr), config_data_(config_data)
{
  // Basic widget container
  layout_ = new QVBoxLayout(this);

  // Top Label Area ------------------------------------------------
  HeaderWidget* header = new HeaderWidget(
      "Optimize Self-Collision Checking",
      "This searches for pairs of robot links that can safely be disabled from collision checking, decreasing motion "
      "planning time. These pairs are disabled when they are always in collision, never in collision, in collision in "
      "the robot's default position, or when the links are adjacent to each other on the kinematic chain. Sampling "
      "density specifies how many random robot positions to check for self collision.",
      this);
  layout_->addWidget(header);

  // Top Button Area -----------------------------------------------
  controls_box_ = new QGroupBox(this);
  layout_->addWidget(controls_box_);
  QVBoxLayout* controls_box_layout = new QVBoxLayout(controls_box_);

  QHBoxLayout* slider_layout = new QHBoxLayout();
  slider_layout->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  controls_box_layout->addLayout(slider_layout);

  // Slider Label
  QLabel* density_left_label = new QLabel(this);
  density_left_label->setText("Sampling Density: Low");
  slider_layout->addWidget(density_left_label);

  // Slider
  density_slider_ = new QSlider(this);
  density_slider_->setTickPosition(QSlider::TicksBelow);
  density_slider_->setMinimum(0);
  density_slider_->setMaximum(99);
  density_slider_->setSingleStep(10);
  density_slider_->setPageStep(50);
  density_slider_->setSliderPosition(9);  // 10,000 is default
  density_slider_->setTickInterval(10);
  density_slider_->setOrientation(Qt::Horizontal);
  slider_layout->addWidget(density_slider_);
  connect(density_slider_, SIGNAL(valueChanged(int)), this, SLOT(changeDensityLabel(int)));

  // Slider Right Label
  QLabel* density_right_label = new QLabel(this);
  density_right_label->setText("High   ");
  slider_layout->addWidget(density_right_label);

  // Slider Value Label
  density_value_label_ = new QLabel(this);
  density_value_label_->setMinimumWidth(50);
  slider_layout->addWidget(density_value_label_);
  changeDensityLabel(density_slider_->value());  // initialize label with value

  QHBoxLayout* buttons_layout = new QHBoxLayout();
  buttons_layout->setAlignment(Qt::AlignRight);
  controls_box_layout->addLayout(buttons_layout);

  // Fraction spin box
  fraction_label_ = new QLabel(this);
  fraction_label_->setText("Min. collisions for \"always\"-colliding pairs:");
  buttons_layout->addWidget(fraction_label_);

  fraction_spinbox_ = new QSpinBox(this);
  fraction_spinbox_->setRange(1, 100);
  fraction_spinbox_->setValue(95);
  fraction_spinbox_->setSuffix("%");
  buttons_layout->addWidget(fraction_spinbox_);

  // Generate Button
  btn_generate_ = new QPushButton(this);
  btn_generate_->setText("&Generate Collision Matrix");
  btn_generate_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  connect(btn_generate_, SIGNAL(clicked()), this, SLOT(startGeneratingCollisionTable()));
  buttons_layout->addWidget(btn_generate_);

  // Progress Bar Area ---------------------------------------------

  // Progress Label
  progress_label_ = new QLabel(this);
  progress_label_->setText("Generating Default Collision Matrix");
  progress_label_->hide();
  layout_->addWidget(progress_label_);

  // Progress Bar
  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide();  // only show when computation begins
  layout_->addWidget(progress_bar_);

  // Table Area --------------------------------------------

  // Table
  collision_table_ = new QTableView(this);
  layout_->addWidget(collision_table_);

  QAction* action;
  action = new QAction(tr("Show"), this);
  header_actions_ << action;
  connect(action, SIGNAL(triggered()), this, SLOT(showSections()));
  action = new QAction(tr("Hide"), this);
  header_actions_ << action;
  connect(action, SIGNAL(triggered()), this, SLOT(hideSections()));
  action = new QAction(tr("Hide others"), this);
  header_actions_ << action;
  connect(action, SIGNAL(triggered()), this, SLOT(hideOtherSections()));
  action = new QAction(tr("Disable by default"), this);
  header_actions_ << action;
  connect(action, &QAction::triggered, this, [this] { setDefaults(true); });
  action = new QAction(tr("Enable by default"), this);
  header_actions_ << action;
  connect(action, &QAction::triggered, this, [this] { setDefaults(false); });

  // Bottom Area ----------------------------------------

  QHBoxLayout* bottom_layout = new QHBoxLayout();
  bottom_layout->setAlignment(Qt::AlignRight);
  layout_->addLayout(bottom_layout);

  // Link Filter QLineEdit
  link_name_filter_ = new QLineEdit(this);
  link_name_filter_->setPlaceholderText("link name filter");
  bottom_layout->addWidget(link_name_filter_);

  // Collision Filter Checkbox
  collision_checkbox_ = new QCheckBox(this);
  collision_checkbox_->setText("show enabled pairs");
  connect(collision_checkbox_, SIGNAL(toggled(bool)), this, SLOT(checkedFilterChanged()));
  bottom_layout->addWidget(collision_checkbox_);

  // View Mode Buttons
  view_mode_buttons_ = new QButtonGroup(this);
  QRadioButton* radio_btn;
  radio_btn = new QRadioButton("linear view");
  bottom_layout->addWidget(radio_btn);
  view_mode_buttons_->addButton(radio_btn, LINEAR_MODE);

  radio_btn = new QRadioButton("matrix view");
  bottom_layout->addWidget(radio_btn);
  view_mode_buttons_->addButton(radio_btn, MATRIX_MODE);
  radio_btn->setChecked(true);
  connect(view_mode_buttons_, SIGNAL(buttonClicked(int)), this, SLOT(loadCollisionTable()));

  // Revert Button
  btn_revert_ = new QPushButton(this);
  btn_revert_->setText("&Revert");
  btn_revert_->setToolTip("Revert current changes to collision matrix");
  btn_revert_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  btn_revert_->setDisabled(true);
  connect(btn_revert_, SIGNAL(clicked()), this, SLOT(revertChanges()));
  bottom_layout->addWidget(btn_revert_);

  setLayout(layout_);
  setWindowTitle("Default Collision Matrix");

  collision_table_->installEventFilter(this);
}

DefaultCollisionsWidget::~DefaultCollisionsWidget()
{
  delete model_;
}

// ******************************************************************************************
// start thread generating the collision table
// ******************************************************************************************
void DefaultCollisionsWidget::startGeneratingCollisionTable()
{
  // Disable controls on form
  disableControls(true);
  btn_revert_->setEnabled(true);  // allow to interrupt and revert

  // create a MonitorThread running generateCollisionTable() in a worker thread and monitoring the progress
  worker_ = new MonitorThread([this](unsigned int* progress) { generateCollisionTable(progress); }, progress_bar_);
  connect(worker_, SIGNAL(finished()), this, SLOT(finishGeneratingCollisionTable()));
  worker_->start();  // start after having finished() signal connected
}

// ******************************************************************************************
// cleanup after worker_ thread has finished
// ******************************************************************************************
void DefaultCollisionsWidget::finishGeneratingCollisionTable()
{
  if (worker_->canceled())
    return;

  // Load the results into the GUI
  loadCollisionTable();

  // Hide the progress bar
  disableControls(false);  // enable everything else

  config_data_->changes |= MoveItConfigData::COLLISIONS;
  worker_->deleteLater();
  worker_ = nullptr;
}

// ******************************************************************************************
// The worker function to compute the collision matrix
// ******************************************************************************************
void DefaultCollisionsWidget::generateCollisionTable(unsigned int* collision_progress)
{
  unsigned int num_trials = density_slider_->value() * 1000 + 1000;  // scale to trials amount
  double min_frac = (double)fraction_spinbox_->value() / 100.0;

  const bool verbose = true;  // Output benchmarking and statistics
  const bool include_never_colliding = true;

  // clear previously loaded collision matrix entries
  config_data_->getPlanningScene()->getAllowedCollisionMatrixNonConst().clear();

  // Find the default collision matrix - all links that are allowed to collide
  auto link_pairs = moveit_setup_assistant::computeDefaultCollisions(
      config_data_->getPlanningScene(), collision_progress, include_never_colliding, num_trials, min_frac, verbose);
  linkPairsToSRDF(link_pairs, *wip_srdf_);
  // Update collision_matrix for robot pose's use
  config_data_->loadAllowedCollisionMatrix(*wip_srdf_);

  // End the progress bar loop
  *collision_progress = 100;

  ROS_INFO_STREAM("Thread complete " << link_pairs.size());
}

// ******************************************************************************************
// Displays data in the link_pairs_ data structure into a QtTableWidget
// ******************************************************************************************
void DefaultCollisionsWidget::loadCollisionTable()
{
  CollisionMatrixModel* matrix_model = new CollisionMatrixModel(
      wip_srdf_, config_data_->getPlanningScene()->getRobotModel()->getLinkModelNamesWithCollisionGeometry());
  QAbstractItemModel* model;

  if (view_mode_buttons_->checkedId() == MATRIX_MODE)
  {
    model = matrix_model;
  }
  else
  {
    CollisionLinearModel* linear_model = new CollisionLinearModel(matrix_model);
    SortFilterProxyModel* sorted_model = new SortFilterProxyModel();
    sorted_model->setShowAll(collision_checkbox_->checkState() == Qt::Checked);
    model = sorted_model;
    sorted_model->setSourceModel(linear_model);
    // ensure deletion of underlying models with model
    linear_model->setParent(sorted_model);
    matrix_model->setParent(linear_model);
  }
  connect(link_name_filter_, SIGNAL(textChanged(QString)), model, SLOT(setFilterRegExp(QString)));
  QMetaObject::invokeMethod(model, "setFilterRegExp", Q_ARG(QString, link_name_filter_->text()));

  collision_table_->setModel(model);
  // delete old and remember new model
  delete model_;
  model_ = model;

  // delete old and fetch new selection model
  delete selection_model_;
  selection_model_ = collision_table_->selectionModel();

  QHeaderView *horizontal_header, *vertical_header;

  // activate some model-specific settings
  if (view_mode_buttons_->checkedId() == MATRIX_MODE)
  {
    connect(selection_model_, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
            SLOT(previewSelectedMatrix(QModelIndex)));

    collision_table_->setSelectionBehavior(QAbstractItemView::SelectItems);
    collision_table_->setSelectionMode(QAbstractItemView::ExtendedSelection);

    collision_table_->setHorizontalHeader(horizontal_header = new RotatedHeaderView(Qt::Horizontal, this));
    collision_table_->setVerticalHeader(vertical_header = new RotatedHeaderView(Qt::Vertical, this));
    collision_table_->setSortingEnabled(false);

    collision_checkbox_->hide();
    horizontal_header->setVisible(true);
    vertical_header->setVisible(true);

    horizontal_header->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(horizontal_header, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(showHeaderContextMenu(QPoint)));
    vertical_header->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(vertical_header, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(showHeaderContextMenu(QPoint)));
  }
  else
  {
    connect(selection_model_, SIGNAL(currentChanged(QModelIndex, QModelIndex)), this,
            SLOT(previewSelectedLinear(QModelIndex)));

    collision_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    collision_table_->setSelectionMode(QAbstractItemView::ExtendedSelection);

    collision_table_->setHorizontalHeader(horizontal_header = new QHeaderView(Qt::Horizontal, this));
    collision_table_->setVerticalHeader(vertical_header = new QHeaderView(Qt::Vertical, this));
    collision_table_->sortByColumn(0, Qt::AscendingOrder);
    collision_table_->setSortingEnabled(true);

    collision_checkbox_->show();
    horizontal_header->setVisible(true);
    vertical_header->setVisible(true);
    horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

    vertical_header->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(vertical_header, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(showHeaderContextMenu(QPoint)));

#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
    horizontal_header->setSectionsClickable(true);
    vertical_header->setSectionsClickable(true);
#else
    horizontal_header->setClickable(true);
    vertical_header->setClickable(true);
#endif
  }

// notice changes to the model
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
  connect(model_, SIGNAL(dataChanged(QModelIndex, QModelIndex, QVector<int>)), this,
          SLOT(collisionsChanged(QModelIndex)));
#else
  connect(model_, SIGNAL(dataChanged(QModelIndex, QModelIndex)), this, SLOT(collisionsChanged(QModelIndex)));
#endif
}

void DefaultCollisionsWidget::collisionsChanged(const QModelIndex& index)
{
  btn_revert_->setEnabled(true);  // enable revert button

  if (!index.isValid())
    return;
  // Hm. For some reason, QTableView doesn't change selection if we click a checkbox
  bool linear_mode = (view_mode_buttons_->checkedId() == LINEAR_MODE);
  const QItemSelection& selection = selection_model_->selection();
  if ((linear_mode && !selection.contains(index)) ||   // in linear mode: index not in selection
      (!linear_mode && !(selection.contains(index) ||  // in matrix mode: index or symmetric index not in selection
                         selection.contains(model_->index(index.column(), index.row())))))
  {
    QItemSelectionModel::SelectionFlags flags = QItemSelectionModel::Select | QItemSelectionModel::Current;
    if (linear_mode)
      flags |= QItemSelectionModel::Rows;
    selection_model_->setCurrentIndex(index, flags);
  }
}

void DefaultCollisionsWidget::showHeaderContextMenu(const QPoint& p)
{
  // This method might be triggered from either of the headers
  QPoint global;
  if (sender() == collision_table_->verticalHeader())
  {
    clicked_section_ = collision_table_->verticalHeader()->logicalIndexAt(p);
    clicked_headers_ = Qt::Vertical;
    global = collision_table_->verticalHeader()->mapToGlobal(p);
  }
  else if (sender() == collision_table_->horizontalHeader())
  {
    clicked_section_ = collision_table_->horizontalHeader()->logicalIndexAt(p);
    clicked_headers_ = Qt::Horizontal;
    global = collision_table_->horizontalHeader()->mapToGlobal(p);
  }
  else
  {
    clicked_section_ = -1;
    clicked_headers_ = Qt::Horizontal | Qt::Vertical;
  }

  QMenu menu;
  if (clicked_section_ < 0)
    menu.addAction(header_actions_.at(0));  // only 'show' action
  else
    menu.addActions(header_actions_);
  menu.exec(global);

  clicked_headers_ = {};
  clicked_section_ = -1;
}

QList<int> DefaultCollisionsWidget::selectedSections(QHeaderView*& header) const
{
  QList<int> list;
  if (clicked_headers_ == Qt::Horizontal)
  {
    header = collision_table_->horizontalHeader();
    for (const QModelIndex& index : selection_model_->selectedColumns())
      if (!header->isSectionHidden(index.column()))
        list << index.column();
  }
  else if (clicked_headers_ == Qt::Vertical)
  {
    header = collision_table_->verticalHeader();
    for (const QModelIndex& index : selection_model_->selectedRows())
      if (!header->isSectionHidden(index.row()))
        list << index.row();
  }
  // if somewhere else than the selection was clicked, only consider this row/column
  if (!list.contains(clicked_section_))
    return { clicked_section_ };

  return list;
}

void DefaultCollisionsWidget::hideSections()
{
  QHeaderView* header;
  auto list = selectedSections(header);

  for (auto index : list)
    header->setSectionHidden(index, true);
}

void DefaultCollisionsWidget::hideOtherSections()
{
  QHeaderView* header;
  auto selected = selectedSections(header);

  for (std::size_t index = 0, end = header->count(); index != end; ++index)
    if (!selected.contains(index))
      header->setSectionHidden(index, true);
}

void DefaultCollisionsWidget::showSections()
{
  if (clicked_section_ < 0)  // show all
  {
    if (clicked_headers_.testFlag(Qt::Horizontal))  // show all columns
      showSections(collision_table_->horizontalHeader(), { 0, model_->columnCount() - 1 });

    if (clicked_headers_.testFlag(Qt::Vertical))  // show all rows
      showSections(collision_table_->verticalHeader(), { 0, model_->rowCount() - 1 });

    return;
  }

  QHeaderView* header;
  QList<int> list = selectedSections(header);
  showSections(header, list);
}

void DefaultCollisionsWidget::showSections(QHeaderView* header, const QList<int>& logicalIndexes)
{
  if (logicalIndexes.size() < 2)
    return;
  int prev = 0;
  for (int next = 1, end = logicalIndexes.size(); next != end; prev = next, ++next)
  {
    for (int index = logicalIndexes[prev], index_end = logicalIndexes[next]; index <= index_end; ++index)
      header->setSectionHidden(index, false);
  }
}

void DefaultCollisionsWidget::setDefaults(bool disabled)
{
  QHeaderView* header;
  QList<int> list = selectedSections(header);
  auto m = collision_table_->model();

  for (auto index : list)
  {
    bool changed = false;
    if (disabled)
    {
      const auto& name = m->headerData(index, Qt::Horizontal, Qt::DisplayRole).toString().toStdString();
      // add name to no_default_collision_links_ (if not yet in there)
      auto& links = wip_srdf_->no_default_collision_links_;
      if (std::find(links.begin(), links.end(), name) == links.end())
      {
        links.push_back(name);
        changed = true;
      }
      // remove-erase disabled pairs that are redundant now
      auto& pairs = wip_srdf_->disabled_collision_pairs_;
      auto last = std::remove_if(pairs.begin(), pairs.end(),
                                 [&name](const auto& p) { return p.link1_ == name || p.link2_ == name; });
      changed |= last != pairs.end();
      pairs.erase(last, pairs.end());
    }
    else
    {
      const auto& name = m->headerData(index, Qt::Horizontal, Qt::DisplayRole).toString().toStdString();
      // remove-erase name from no_default_collision_links_
      auto& links = wip_srdf_->no_default_collision_links_;
      {
        auto last = std::remove(links.begin(), links.end(), name);
        changed |= last != links.end();
        links.erase(last, links.end());
      }

      // remove explicitly enabled pairs
      auto& pairs = wip_srdf_->enabled_collision_pairs_;
      auto last = std::remove_if(pairs.begin(), pairs.end(), [&name, &links](const auto& p) {
        return (p.link1_ == name && std::find(links.begin(), links.end(), p.link2_) == links.end()) ||
               (p.link2_ == name && std::find(links.begin(), links.end(), p.link1_) == links.end());
      });
      pairs.erase(last, pairs.end());
    }
    if (changed)
      btn_revert_->setEnabled(true);
  }
}

void DefaultCollisionsWidget::revertChanges()
{
  *wip_srdf_ = *config_data_->srdf_;
  loadCollisionTable();
  btn_revert_->setEnabled(false);  // no changes to revert
}

bool DefaultCollisionsWidget::eventFilter(QObject* object, QEvent* event)
{
  if (object != collision_table_)
    return false;  // leave event unhandled

  if (event->type() == QEvent::Enter)
  {
    // grab focus as soon as mouse enters to allow for <space> to work in all cases
    collision_table_->setFocus();
    return false;
  }
  else if (event->type() == QEvent::KeyPress)
  {
    QKeyEvent* key_event = static_cast<QKeyEvent*>(event);
    if (key_event->key() != Qt::Key_Space)
      return false;

    toggleSelection(selection_model_->selection());
    return true;  // no need for further processing
  }

  return false;
}

void DefaultCollisionsWidget::toggleSelection(QItemSelection selection)
{
  // remove hidden rows / columns from selection
  int rows = model_->rowCount();
  int cols = model_->columnCount();
  for (int r = 0; r != rows; ++r)
  {
    if (collision_table_->isRowHidden(r))
      selection.merge(QItemSelection(model_->index(r, 0), model_->index(r, cols - 1)), QItemSelectionModel::Deselect);
  }
  for (int c = 0; c != cols; ++c)
  {
    if (collision_table_->isColumnHidden(c))
      selection.merge(QItemSelection(model_->index(0, c), model_->index(rows - 1, c)), QItemSelectionModel::Deselect);
  }

  // set all selected items to inverse value of current item
  const QModelIndex& cur_idx = selection_model_->currentIndex();
  if (view_mode_buttons_->checkedId() == MATRIX_MODE)
  {
    QModelIndex input_index;
    if (cur_idx.flags() & Qt::ItemIsUserCheckable)
      input_index = cur_idx;  // if current index is checkable, this serves as input
    else
    {  // search for first checkable index in selection that can serve as input
      for (const auto idx : selection.indexes())
      {
        if (idx.flags() & Qt::ItemIsUserCheckable)
        {
          input_index = idx;
          break;
        }
      }
      if (!input_index.isValid())
        return;  // no valid selection
    }

    bool current = model_->data(input_index, Qt::CheckStateRole) == Qt::Checked;
    CollisionMatrixModel* m = static_cast<CollisionMatrixModel*>(model_);
    m->setEnabled(selection, !current);
  }
  else
  {
    bool current = model_->data(model_->index(cur_idx.row(), 2), Qt::CheckStateRole) == Qt::Checked;
    SortFilterProxyModel* m = static_cast<SortFilterProxyModel*>(model_);
    m->setEnabled(selection, !current);
  }
}

// ******************************************************************************************
// GUI func for showing sampling density amount
// ******************************************************************************************
void DefaultCollisionsWidget::changeDensityLabel(int value)
{
  density_value_label_->setText(QString::number(value * 1000 + 1000));  //.append(" samples") );
}

// ******************************************************************************************
// Helper function to disable parts of GUI during computation
// ******************************************************************************************
void DefaultCollisionsWidget::disableControls(bool disable)
{
  controls_box_->setDisabled(disable);
  collision_table_->setDisabled(disable);

  if (disable)
  {
    progress_bar_->show();  // only show when computation begins
    progress_label_->show();
  }
  else
  {
    progress_label_->hide();
    progress_bar_->hide();
  }

  QApplication::processEvents();  // allow the progress bar to be shown
}

// ******************************************************************************************
// Changes the table to show or hide collisions that are not disabled (that have collision checking enabled)
// ******************************************************************************************
void DefaultCollisionsWidget::checkedFilterChanged()
{
  SortFilterProxyModel* m = qobject_cast<SortFilterProxyModel*>(model_);
  m->setShowAll(collision_checkbox_->checkState() == Qt::Checked);
}

// ******************************************************************************************
// Preview whatever element is selected
// ******************************************************************************************
void DefaultCollisionsWidget::previewSelectedMatrix(const QModelIndex& index)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  if (!index.isValid())
    return;

  // normalize index
  int r = index.row();
  int c = index.column();
  if (r == c)
    return;
  if (r > c)
    std::swap(r, c);

  // Highlight link pair
  const QString& first_link = model_->headerData(r, Qt::Vertical, Qt::DisplayRole).toString();
  const QString& second_link = model_->headerData(c, Qt::Horizontal, Qt::DisplayRole).toString();
  uint check_state = model_->data(index, Qt::CheckStateRole).toUInt();

  QColor color = (check_state == Qt::Checked) ? QColor(0, 255, 0) : QColor(255, 0, 0);
  Q_EMIT highlightLink(first_link.toStdString(), color);
  Q_EMIT highlightLink(second_link.toStdString(), color);
}

void DefaultCollisionsWidget::previewSelectedLinear(const QModelIndex& index)
{
  // Unhighlight all links
  Q_EMIT unhighlightAll();

  if (!index.isValid())
    return;

  // Highlight link pair
  const QString& first_link = model_->data(model_->index(index.row(), 0), Qt::DisplayRole).toString();
  const QString& second_link = model_->data(model_->index(index.row(), 1), Qt::DisplayRole).toString();
  uint check_state = model_->data(model_->index(index.row(), 2), Qt::CheckStateRole).toUInt();

  QColor color = (check_state == Qt::Checked) ? QColor(0, 255, 0) : QColor(255, 0, 0);
  Q_EMIT highlightLink(first_link.toStdString(), color);
  Q_EMIT highlightLink(second_link.toStdString(), color);
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void DefaultCollisionsWidget::focusGiven()
{
  // srdf backup
  wip_srdf_ = std::make_shared<srdf::SRDFWriter>(*config_data_->srdf_);

  // Load the data to the table
  loadCollisionTable();

  // Enable the table
  disableControls(false);
  btn_revert_->setEnabled(false);  // no changes to revert
}

bool DefaultCollisionsWidget::focusLost()
{
  if (worker_)
  {
    if (QMessageBox::No == QMessageBox::question(this, "Collision Matrix Generation",
                                                 "Collision Matrix Generation is still active. Cancel computation?",
                                                 QMessageBox::Yes | QMessageBox::No, QMessageBox::No))
      return false;
    worker_->cancel();
    worker_->wait();
  }
  *config_data_->srdf_ = *wip_srdf_;

  return true;
}

moveit_setup_assistant::MonitorThread::MonitorThread(const boost::function<void(unsigned int*)>& f,
                                                     QProgressBar* progress_bar)
  : progress_(0), canceled_(false)
{
  // start worker thread
  worker_ = boost::thread([f, progress_ptr = &progress_] { f(progress_ptr); });
  // connect progress bar for updates
  if (progress_bar)
    connect(this, SIGNAL(progress(int)), progress_bar, SLOT(setValue(int)));
}

void moveit_setup_assistant::MonitorThread::run()
{
  // loop until worker thread is finished or cancel is requested
  while (!canceled_ && progress_ < 100)
  {
    Q_EMIT progress(progress_);
    QThread::msleep(100);  // sleep 100ms
  }

  // cancel worker thread
  if (canceled_)
    worker_.interrupt();

  worker_.join();

  progress_ = 100;
  Q_EMIT progress(progress_);
}

}  // namespace moveit_setup_assistant
