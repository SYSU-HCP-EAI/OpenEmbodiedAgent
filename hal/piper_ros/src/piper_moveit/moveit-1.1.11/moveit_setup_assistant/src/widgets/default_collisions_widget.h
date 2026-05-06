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

#pragma once
#include <QThread>
class QAbstractItemModel;
class QAction;
class QButtonGroup;
class QCheckBox;
class QGroupBox;
class QHeaderView;
class QItemSelection;
class QItemSelectionModel;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QRadioButton;
class QSlider;
class QSpinBox;
class QTableView;
class QVBoxLayout;

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>
#include <boost/function/function_fwd.hpp>
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>
#endif

#include "setup_screen_widget.h"  // a base class for screens in the setup assistant

namespace moveit_setup_assistant
{
class MonitorThread;

/**
 * \brief User interface for editing the default collision matrix list in an SRDF
 */
class DefaultCollisionsWidget : public SetupScreenWidget
{
  Q_OBJECT

public:
  enum ViewMode
  {
    MATRIX_MODE = 0,
    LINEAR_MODE = 1
  };

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * \brief User interface for editing the default collision matrix list in an SRDF
   * \param urdf_file String srdf file location. It will create a new file or will edit an existing one
   */
  DefaultCollisionsWidget(QWidget* parent, const MoveItConfigDataPtr& config_data);
  ~DefaultCollisionsWidget() override;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * \brief start generating collision matrix in a worker thread
   */
  void startGeneratingCollisionTable();
  /**
   * \brief finish generating collision matrix after worker thread has finished
   */
  void finishGeneratingCollisionTable();

  /**
   * \brief GUI func for showing sampling density amount
   * \param value Sampling density
   */
  void changeDensityLabel(int value);

  /**
   * \brief Update view and data model for the link_pairs data structure
   */
  void loadCollisionTable();

  /**
   * \brief Change filter settings to show/hide enabled collisions
   */
  void checkedFilterChanged();

  /**
   * \brief Collision model changed
   */
  void collisionsChanged(const QModelIndex& index);

  /**
   * \brief Revert current changes to collision matrix
   */
  void revertChanges();

  /**
   * \brief Called when current row has changed
   */
  void previewSelectedMatrix(const QModelIndex& index);
  void previewSelectedLinear(const QModelIndex& index);

  /**
   * \brief Called when setup assistant navigation switches to this screen
   */
  void focusGiven() override;

  /**
   * \brief Called when setup assistant navigation switches away from this screen
   */
  bool focusLost() override;

  void showHeaderContextMenu(const QPoint& p);
  void hideSections();
  void hideOtherSections();
  void showSections();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QLabel* page_title_;
  QTableView* collision_table_;
  QAbstractItemModel* model_;
  QItemSelectionModel* selection_model_;
  QVBoxLayout* layout_;
  QLabel* density_value_label_;
  QSlider* density_slider_;
  QPushButton* btn_generate_;
  QGroupBox* controls_box_;
  QProgressBar* progress_bar_;
  QLabel* progress_label_;
  QLineEdit* link_name_filter_;
  QCheckBox* collision_checkbox_;
  QLabel* fraction_label_;
  QSpinBox* fraction_spinbox_;
  QPushButton* btn_revert_;
  QButtonGroup* view_mode_buttons_;

  QList<QAction*> header_actions_;    // context actions for header sections
  Qt::Orientations clicked_headers_;  // remember which header section activated context actions
  int clicked_section_;               // remember which header section activated context actions

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  MonitorThread* worker_;

  /// Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data_;
  /// Working copy of SRDF config
  srdf::SRDFWriterPtr wip_srdf_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /**
   * \brief The thread that is called to allow the GUI to update. Calls an external function to do calcs
   * \param collision_progress A shared pointer between 3 threads to allow progress bar to update. See declaration
   * location for more details and warning.
   */
  void generateCollisionTable(unsigned int* collision_progress);

  /**
   * \brief Helper function to disable parts of GUI during computation
   * \param disable A command
   */
  void disableControls(bool disable);

  /** Allow toggling of all checkboxes in selection by filtering <space> keypresses */
  bool eventFilter(QObject* object, QEvent* event) override;

  /** Return list of selected sections */
  QList<int> selectedSections(QHeaderView*& header) const;

  /** Show header's sections in logicalIndexes and everything in between */
  void showSections(QHeaderView* header, const QList<int>& logicalIndexes);

  /** Enable/Disable selected sections by default */
  void setDefaults(bool enabled);

  /** Toggle enabled status of selection */
  void toggleSelection(QItemSelection selection);
};

/** QThread to monitor progress of a boost::thread */
class MonitorThread : public QThread
{
  Q_OBJECT

public:
  MonitorThread(const boost::function<void(unsigned int*)>& f, QProgressBar* progress_bar = nullptr);
  void run() override;
  void cancel()
  {
    canceled_ = true;
  }
  bool canceled() const
  {
    return canceled_;
  }

Q_SIGNALS:
  void progress(int /*_t1*/);

private:
  boost::thread worker_;
  unsigned int progress_;
  bool canceled_;
};
}  // namespace moveit_setup_assistant
