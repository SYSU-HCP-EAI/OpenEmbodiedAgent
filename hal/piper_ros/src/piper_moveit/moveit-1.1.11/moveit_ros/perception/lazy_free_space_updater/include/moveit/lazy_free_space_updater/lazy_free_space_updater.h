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

#pragma once

#include <moveit/collision_detection/occupancy_map.h>
#include <boost/thread.hpp>
#include <deque>
#include <unordered_map>

namespace occupancy_map_monitor
{
class LazyFreeSpaceUpdater
{
public:
  LazyFreeSpaceUpdater(const collision_detection::OccMapTreePtr& tree, unsigned int max_batch_size = 10);
  ~LazyFreeSpaceUpdater();

  void pushLazyUpdate(octomap::KeySet* occupied_cells, octomap::KeySet* model_cells,
                      const octomap::point3d& sensor_origin);

private:
#ifdef __APPLE__
  typedef std::unordered_map<octomap::OcTreeKey, unsigned int, octomap::OcTreeKey::KeyHash> OcTreeKeyCountMap;
#elif __cplusplus >= 201103L
  typedef std::unordered_map<octomap::OcTreeKey, unsigned int, octomap::OcTreeKey::KeyHash> OcTreeKeyCountMap;
#else
  typedef std::tr1::unordered_map<octomap::OcTreeKey, unsigned int, octomap::OcTreeKey::KeyHash> OcTreeKeyCountMap;
#endif

  void pushBatchToProcess(OcTreeKeyCountMap* occupied_cells, octomap::KeySet* model_cells,
                          const octomap::point3d& sensor_origin);

  void lazyUpdateThread();
  void processThread();

  collision_detection::OccMapTreePtr tree_;
  bool running_;
  std::size_t max_batch_size_;
  double max_sensor_delta_;

  std::deque<octomap::KeySet*> occupied_cells_sets_;
  std::deque<octomap::KeySet*> model_cells_sets_;
  std::deque<octomap::point3d> sensor_origins_;
  boost::condition_variable update_condition_;
  boost::mutex update_cell_sets_lock_;

  OcTreeKeyCountMap* process_occupied_cells_set_;
  octomap::KeySet* process_model_cells_set_;
  octomap::point3d process_sensor_origin_;
  boost::condition_variable process_condition_;
  boost::mutex cell_process_lock_;

  boost::thread update_thread_;
  boost::thread process_thread_;
};
}  // namespace occupancy_map_monitor
