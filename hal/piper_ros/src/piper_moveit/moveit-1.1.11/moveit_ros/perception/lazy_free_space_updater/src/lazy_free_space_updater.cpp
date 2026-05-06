/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/lazy_free_space_updater/lazy_free_space_updater.h>
#include <ros/console.h>

namespace occupancy_map_monitor
{
static const std::string LOGNAME = "lazy_free_space_updater";

LazyFreeSpaceUpdater::LazyFreeSpaceUpdater(const collision_detection::OccMapTreePtr& tree, unsigned int max_batch_size)
  : tree_(tree)
  , running_(true)
  , max_batch_size_(max_batch_size)
  , max_sensor_delta_(1e-3)  // 1mm
  , process_occupied_cells_set_(nullptr)
  , process_model_cells_set_(nullptr)
  , update_thread_([this] { lazyUpdateThread(); })
  , process_thread_([this] { processThread(); })
{
}

LazyFreeSpaceUpdater::~LazyFreeSpaceUpdater()
{
  running_ = false;
  {
    boost::unique_lock<boost::mutex> _(update_cell_sets_lock_);
    update_condition_.notify_one();
  }
  {
    boost::unique_lock<boost::mutex> _(cell_process_lock_);
    process_condition_.notify_one();
  }
  update_thread_.join();
  process_thread_.join();
}

void LazyFreeSpaceUpdater::pushLazyUpdate(octomap::KeySet* occupied_cells, octomap::KeySet* model_cells,
                                          const octomap::point3d& sensor_origin)
{
  ROS_DEBUG_NAMED(LOGNAME, "Pushing %lu occupied cells and %lu model cells for lazy updating...",
                  (long unsigned int)occupied_cells->size(), (long unsigned int)model_cells->size());
  boost::mutex::scoped_lock _(update_cell_sets_lock_);
  occupied_cells_sets_.push_back(occupied_cells);
  model_cells_sets_.push_back(model_cells);
  sensor_origins_.push_back(sensor_origin);
  update_condition_.notify_one();
}

void LazyFreeSpaceUpdater::pushBatchToProcess(OcTreeKeyCountMap* occupied_cells, octomap::KeySet* model_cells,
                                              const octomap::point3d& sensor_origin)
{
  // this is basically a queue of size 1. if this function is called repeatedly without any work being done by
  // processThread(),
  // data can be lost; this is intentional, to avoid spending too much time clearing the octomap
  if (cell_process_lock_.try_lock())
  {
    process_occupied_cells_set_ = occupied_cells;
    process_model_cells_set_ = model_cells;
    process_sensor_origin_ = sensor_origin;
    process_condition_.notify_one();
    cell_process_lock_.unlock();
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "Previous batch update did not complete. Ignoring set of cells to be freed.");
    delete occupied_cells;
    delete model_cells;
  }
}

void LazyFreeSpaceUpdater::processThread()
{
  const float lg_0 = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
  const float lg_miss = tree_->getProbMissLog();

  octomap::KeyRay key_ray1, key_ray2;
  OcTreeKeyCountMap free_cells1, free_cells2;

  while (running_)
  {
    free_cells1.clear();
    free_cells2.clear();

    boost::unique_lock<boost::mutex> ulock(cell_process_lock_);
    while (!process_occupied_cells_set_ && running_)
      process_condition_.wait(ulock);

    if (!running_)
      break;

    ROS_DEBUG_NAMED(LOGNAME,
                    "Begin processing batched update: marking free cells due to %lu occupied cells and %lu model cells",
                    (long unsigned int)process_occupied_cells_set_->size(),
                    (long unsigned int)process_model_cells_set_->size());

    ros::WallTime start = ros::WallTime::now();
    tree_->lockRead();

#pragma omp sections
    {
#pragma omp section
      {
        /* compute the free cells along each ray that ends at an occupied cell */
        for (std::pair<const octomap::OcTreeKey, unsigned int>& it : *process_occupied_cells_set_)
          if (tree_->computeRayKeys(process_sensor_origin_, tree_->keyToCoord(it.first), key_ray1))
            for (octomap::OcTreeKey& jt : key_ray1)
              free_cells1[jt] += it.second;
      }

#pragma omp section
      {
        /* compute the free cells along each ray that ends at a model cell */
        for (const octomap::OcTreeKey& it : *process_model_cells_set_)
          if (tree_->computeRayKeys(process_sensor_origin_, tree_->keyToCoord(it), key_ray2))
            for (octomap::OcTreeKey& jt : key_ray2)
              free_cells2[jt]++;
      }
    }

    tree_->unlockRead();

    for (std::pair<const octomap::OcTreeKey, unsigned int>& it : *process_occupied_cells_set_)
    {
      free_cells1.erase(it.first);
      free_cells2.erase(it.first);
    }

    for (const octomap::OcTreeKey& it : *process_model_cells_set_)
    {
      free_cells1.erase(it);
      free_cells2.erase(it);
    }
    ROS_DEBUG_NAMED(LOGNAME, "Marking %lu cells as free...",
                    (long unsigned int)(free_cells1.size() + free_cells2.size()));

    tree_->lockWrite();

    try
    {
      // set the logodds to the minimum for the cells that are part of the model
      for (const octomap::OcTreeKey& it : *process_model_cells_set_)
        tree_->updateNode(it, lg_0);

      /* mark free cells only if not seen occupied in this cloud */
      for (std::pair<const octomap::OcTreeKey, unsigned int>& it : free_cells1)
        tree_->updateNode(it.first, it.second * lg_miss);
      for (std::pair<const octomap::OcTreeKey, unsigned int>& it : free_cells2)
        tree_->updateNode(it.first, it.second * lg_miss);
    }
    catch (...)
    {
      ROS_ERROR_NAMED(LOGNAME, "Internal error while updating octree");
    }
    tree_->unlockWrite();
    tree_->triggerUpdateCallback();

    ROS_DEBUG_NAMED(LOGNAME, "Marked free cells in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);

    delete process_occupied_cells_set_;
    process_occupied_cells_set_ = nullptr;
    delete process_model_cells_set_;
    process_model_cells_set_ = nullptr;
  }
}

void LazyFreeSpaceUpdater::lazyUpdateThread()
{
  OcTreeKeyCountMap* occupied_cells_set = nullptr;
  octomap::KeySet* model_cells_set = nullptr;
  octomap::point3d sensor_origin;
  unsigned int batch_size = 0;

  while (running_)
  {
    boost::unique_lock<boost::mutex> ulock(update_cell_sets_lock_);
    while (occupied_cells_sets_.empty() && running_)
      update_condition_.wait(ulock);

    if (!running_)
      break;

    if (batch_size == 0)
    {
      occupied_cells_set = new OcTreeKeyCountMap();
      octomap::KeySet* s = occupied_cells_sets_.front();
      occupied_cells_sets_.pop_front();
      for (const octomap::OcTreeKey& it : *s)
        (*occupied_cells_set)[it]++;
      delete s;
      model_cells_set = model_cells_sets_.front();
      model_cells_sets_.pop_front();
      sensor_origin = sensor_origins_.front();
      sensor_origins_.pop_front();
      batch_size++;
    }

    while (!occupied_cells_sets_.empty())
    {
      if ((sensor_origins_.front() - sensor_origin).norm() > max_sensor_delta_)
      {
        ROS_DEBUG_NAMED(LOGNAME, "Pushing %u sets of occupied/model cells to free cells update thread (origin changed)",
                        batch_size);
        pushBatchToProcess(occupied_cells_set, model_cells_set, sensor_origin);
        batch_size = 0;
        break;
      }
      sensor_origins_.pop_front();

      octomap::KeySet* add_occ = occupied_cells_sets_.front();
      for (const octomap::OcTreeKey& it : *add_occ)
        (*occupied_cells_set)[it]++;
      occupied_cells_sets_.pop_front();
      delete add_occ;
      octomap::KeySet* mod_occ = model_cells_sets_.front();
      model_cells_set->insert(mod_occ->begin(), mod_occ->end());
      model_cells_sets_.pop_front();
      delete mod_occ;
      batch_size++;
    }

    if (batch_size >= max_batch_size_)
    {
      ROS_DEBUG_NAMED(LOGNAME, "Pushing %u sets of occupied/model cells to free cells update thread", batch_size);
      pushBatchToProcess(occupied_cells_set, model_cells_set, sensor_origin);
      occupied_cells_set = nullptr;
      batch_size = 0;
    }
  }
}
}  // namespace occupancy_map_monitor
