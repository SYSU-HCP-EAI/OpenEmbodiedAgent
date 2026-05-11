/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Mark Moll */

#include <boost/filesystem/fstream.hpp>
#include <numeric>

#include <moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.h>

namespace cached_ik_kinematics_plugin
{
IKCache::IKCache()
{
  // set distance function for nearest-neighbor queries
  ik_nn_.setDistanceFunction([](const IKEntry* entry1, const IKEntry* entry2) {
    double dist = 0.;
    for (unsigned int i = 0; i < entry1->first.size(); ++i)
      dist += entry1->first[i].distance(entry2->first[i]);
    return dist;
  });
}

IKCache::~IKCache()
{
  if (!ik_cache_.empty())
    saveCache();
}

void IKCache::initializeCache(const std::string& robot_id, const std::string& group_name, const std::string& cache_name,
                              const unsigned int num_joints, const Options& opts)
{
  // read ROS parameters
  max_cache_size_ = opts.max_cache_size;
  ik_cache_.reserve(max_cache_size_);
  min_pose_distance_ = opts.min_pose_distance;
  min_config_distance2_ = opts.min_joint_config_distance;
  min_config_distance2_ *= min_config_distance2_;
  std::string cached_ik_path = opts.cached_ik_path;

  // use mutex lock for rest of initialization
  std::lock_guard<std::mutex> slock(lock_);
  // determine cache file name
  boost::filesystem::path prefix(!cached_ik_path.empty() ? cached_ik_path : boost::filesystem::current_path());
  // create cache directory if necessary
  boost::filesystem::create_directories(prefix);

  cache_file_name_ = prefix / (robot_id + group_name + "_" + cache_name + "_" + std::to_string(max_cache_size_) + "_" +
                               std::to_string(min_pose_distance_) + "_" +
                               std::to_string(std::sqrt(min_config_distance2_)) + ".ikcache");

  ik_cache_.clear();
  ik_nn_.clear();
  last_saved_cache_size_ = 0;
  if (boost::filesystem::exists(cache_file_name_))
  {
    // read cache
    boost::filesystem::ifstream cache_file(cache_file_name_, std::ios_base::binary | std::ios_base::in);
    cache_file.read((char*)&last_saved_cache_size_, sizeof(unsigned int));
    unsigned int num_dofs;
    cache_file.read((char*)&num_dofs, sizeof(unsigned int));
    unsigned int num_tips;
    cache_file.read((char*)&num_tips, sizeof(unsigned int));

    ROS_INFO_NAMED("cached_ik", "Found %d IK solutions for a %d-dof system with %d end effectors in %s",
                   last_saved_cache_size_, num_dofs, num_tips, cache_file_name_.string().c_str());

    unsigned int position_size = 3 * sizeof(tf2Scalar);
    unsigned int orientation_size = 4 * sizeof(tf2Scalar);
    unsigned int pose_size = position_size + orientation_size;
    unsigned int config_size = num_dofs * sizeof(double);
    unsigned int offset_conf = pose_size * num_tips;
    unsigned int bufsize = offset_conf + config_size;
    char* buffer = new char[bufsize];
    IKEntry entry;
    entry.first.resize(num_tips);
    entry.second.resize(num_dofs);
    ik_cache_.reserve(last_saved_cache_size_);

    for (unsigned i = 0; i < last_saved_cache_size_; ++i)
    {
      unsigned int j = 0;
      cache_file.read(buffer, bufsize);
      for (auto& pose : entry.first)
      {
        memcpy(&pose.position[0], buffer + j * pose_size, position_size);
        memcpy(&pose.orientation[0], buffer + j * pose_size + position_size, orientation_size);
        ++j;
      }
      memcpy(&entry.second[0], buffer + offset_conf, config_size);
      ik_cache_.push_back(entry);
    }
    ROS_INFO_NAMED("cached_ik", "freeing buffer");
    delete[] buffer;
    ROS_INFO_NAMED("cached_ik", "freed buffer");
    std::vector<IKEntry*> ik_entry_ptrs(last_saved_cache_size_);
    for (unsigned int i = 0; i < last_saved_cache_size_; ++i)
      ik_entry_ptrs[i] = &ik_cache_[i];
    ik_nn_.add(ik_entry_ptrs);
  }

  num_joints_ = num_joints;

  ROS_INFO_NAMED("cached_ik", "cache file %s initialized!", cache_file_name_.string().c_str());
}

double IKCache::configDistance2(const std::vector<double>& config1, const std::vector<double>& config2) const
{
  double dist = 0., diff;
  for (unsigned int i = 0; i < config1.size(); ++i)
  {
    diff = config1[i] - config2[i];
    dist += diff * diff;
  }
  return dist;
}

const IKCache::IKEntry& IKCache::getBestApproximateIKSolution(const Pose& pose) const
{
  if (ik_cache_.empty())
  {
    static IKEntry dummy = std::make_pair(std::vector<Pose>(1, pose), std::vector<double>(num_joints_, 0.));
    return dummy;
  }
  IKEntry query = std::make_pair(std::vector<Pose>(1, pose), std::vector<double>());
  return *ik_nn_.nearest(&query);
}

const IKCache::IKEntry& IKCache::getBestApproximateIKSolution(const std::vector<Pose>& poses) const
{
  if (ik_cache_.empty())
  {
    static IKEntry dummy = std::make_pair(poses, std::vector<double>(num_joints_, 0.));
    return dummy;
  }
  IKEntry query = std::make_pair(poses, std::vector<double>());
  return *ik_nn_.nearest(&query);
}

void IKCache::updateCache(const IKEntry& nearest, const Pose& pose, const std::vector<double>& config) const
{
  if (ik_cache_.size() < ik_cache_.capacity() && (nearest.first[0].distance(pose) > min_pose_distance_ ||
                                                  configDistance2(nearest.second, config) > min_config_distance2_))
  {
    std::lock_guard<std::mutex> slock(lock_);
    ik_cache_.emplace_back(std::vector<Pose>(1u, pose), config);
    ik_nn_.add(&ik_cache_.back());
    if (ik_cache_.size() >= last_saved_cache_size_ + 500u || ik_cache_.size() == max_cache_size_)
      saveCache();
  }
}

void IKCache::updateCache(const IKEntry& nearest, const std::vector<Pose>& poses,
                          const std::vector<double>& config) const
{
  if (ik_cache_.size() < ik_cache_.capacity())
  {
    bool add_to_cache = configDistance2(nearest.second, config) > min_config_distance2_;
    if (!add_to_cache)
    {
      double dist = 0.;
      for (unsigned int i = 0; i < poses.size(); ++i)
      {
        dist += nearest.first[i].distance(poses[i]);
        if (dist > min_pose_distance_)
        {
          add_to_cache = true;
          break;
        }
      }
    }
    if (add_to_cache)
    {
      std::lock_guard<std::mutex> slock(lock_);
      ik_cache_.emplace_back(poses, config);
      ik_nn_.add(&ik_cache_.back());
      if (ik_cache_.size() >= last_saved_cache_size_ + 500u || ik_cache_.size() == max_cache_size_)
        saveCache();
    }
  }
}

void IKCache::saveCache() const
{
  if (cache_file_name_.empty())
    ROS_ERROR_NAMED("cached_ik", "can't save cache before initialization");

  ROS_INFO_NAMED("cached_ik", "writing %ld IK solutions to %s", ik_cache_.size(), cache_file_name_.string().c_str());

  boost::filesystem::ofstream cache_file(cache_file_name_, std::ios_base::binary | std::ios_base::out);
  unsigned int position_size = 3 * sizeof(tf2Scalar);
  unsigned int orientation_size = 4 * sizeof(tf2Scalar);
  unsigned int pose_size = position_size + orientation_size;
  unsigned int num_tips = ik_cache_[0].first.size();
  unsigned int config_size = ik_cache_[0].second.size() * sizeof(double);
  unsigned int offset_conf = num_tips * pose_size;
  unsigned int bufsize = offset_conf + config_size;
  char* buffer = new char[bufsize];

  // write number of IK entries and size of each configuration first
  last_saved_cache_size_ = ik_cache_.size();
  cache_file.write((char*)&last_saved_cache_size_, sizeof(unsigned int));
  unsigned int sz = ik_cache_[0].second.size();
  cache_file.write((char*)&sz, sizeof(unsigned int));
  cache_file.write((char*)&num_tips, sizeof(unsigned int));
  for (const auto& entry : ik_cache_)
  {
    for (unsigned int i = 0; i < num_tips; ++i)
    {
      memcpy(buffer + i * pose_size, &entry.first[i].position[0], position_size);
      memcpy(buffer + i * pose_size + position_size, &entry.first[i].orientation[0], orientation_size);
    }
    memcpy(buffer + offset_conf, &entry.second[0], config_size);
    cache_file.write(buffer, bufsize);
  }
  delete[] buffer;
}

void IKCache::verifyCache(kdl_kinematics_plugin::KDLKinematicsPlugin& fk) const
{
  const std::vector<std::string>& tip_names(fk.getTipFrames());
  std::vector<geometry_msgs::Pose> poses(tip_names.size());
  double error, max_error = 0.;

  for (const auto& entry : ik_cache_)
  {
    fk.getPositionFK(tip_names, entry.second, poses);
    error = 0.;
    for (unsigned int i = 0; i < poses.size(); ++i)
      error += entry.first[i].distance(poses[i]);
    if (!poses.empty())
      error /= (double)poses.size();
    if (error > max_error)
      max_error = error;
    if (error > 1e-4)
      ROS_ERROR_NAMED("cached_ik", "Cache entry is invalid, error = %g", error);
  }
  ROS_INFO_NAMED("cached_ik", "Max. error in cache entries is %g", max_error);
}

IKCache::Pose::Pose(const geometry_msgs::Pose& pose)
{
  position.setX(pose.position.x);
  position.setY(pose.position.y);
  position.setZ(pose.position.z);
  orientation = tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

double IKCache::Pose::distance(const Pose& pose) const
{
  return (position - pose.position).length() + (orientation.angleShortestPath(pose.orientation));
}

IKCacheMap::IKCacheMap(const std::string& robot_description, const std::string& group_name, unsigned int num_joints)
  : robot_description_(robot_description), group_name_(group_name), num_joints_(num_joints)
{
}

IKCacheMap::~IKCacheMap()
{
  for (auto& cache : *this)
    delete cache.second;
}

const IKCache::IKEntry& IKCacheMap::getBestApproximateIKSolution(const std::vector<std::string>& fixed,
                                                                 const std::vector<std::string>& active,
                                                                 const std::vector<Pose>& poses) const
{
  auto key(getKey(fixed, active));
  auto it = find(key);
  if (it != end())
    return it->second->getBestApproximateIKSolution(poses);
  else
  {
    static IKEntry dummy = std::make_pair(poses, std::vector<double>(num_joints_, 0.));
    return dummy;
  }
}

void IKCacheMap::updateCache(const IKEntry& nearest, const std::vector<std::string>& fixed,
                             const std::vector<std::string>& active, const std::vector<Pose>& poses,
                             const std::vector<double>& config)
{
  auto key(getKey(fixed, active));
  auto it = find(key);
  if (it != end())
    it->second->updateCache(nearest, poses, config);
  else
  {
    value_type val = std::make_pair(key, nullptr);
    auto it = insert(val).first;
    it->second = new IKCache;
    it->second->initializeCache(robot_description_, group_name_, key, num_joints_);
  }
}

std::string IKCacheMap::getKey(const std::vector<std::string>& fixed, const std::vector<std::string>& active) const
{
  std::string key;
  std::accumulate(fixed.begin(), fixed.end(), key);
  key += '_';
  std::accumulate(active.begin(), active.end(), key);
  return key;
}
}  // namespace cached_ik_kinematics_plugin
