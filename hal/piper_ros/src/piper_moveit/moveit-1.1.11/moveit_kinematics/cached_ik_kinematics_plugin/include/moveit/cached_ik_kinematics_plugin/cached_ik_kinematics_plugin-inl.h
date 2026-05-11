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

#include <chrono>
#include <cstdlib>

namespace cached_ik_kinematics_plugin
{
template <class KinematicsPlugin>
CachedIKKinematicsPlugin<KinematicsPlugin>::CachedIKKinematicsPlugin()
{
}

template <class KinematicsPlugin>
CachedIKKinematicsPlugin<KinematicsPlugin>::~CachedIKKinematicsPlugin()
{
}

template <class KinematicsPlugin>
void CachedIKKinematicsPlugin<KinematicsPlugin>::initCache(const std::string& robot_id, const std::string& group_name,
                                                           const std::string& cache_name)
{
  IKCache::Options opts;
  int max_cache_size;  // rosparam can't handle unsigned int
  kinematics::KinematicsBase::lookupParam("max_cache_size", max_cache_size, static_cast<int>(opts.max_cache_size));
  opts.max_cache_size = max_cache_size;
  kinematics::KinematicsBase::lookupParam("min_pose_distance", opts.min_pose_distance, 1.0);
  kinematics::KinematicsBase::lookupParam("min_joint_config_distance", opts.min_joint_config_distance, 1.0);
  kinematics::KinematicsBase::lookupParam<std::string>("cached_ik_path", opts.cached_ik_path, "");

  cache_.initializeCache(robot_id, group_name, cache_name, KinematicsPlugin::getJointNames().size(), opts);

  // for debugging purposes:
  // kdl_kinematics_plugin::KDLKinematicsPlugin fk;
  // fk.initialize(robot_description, group_name, base_frame, tip_frame, search_discretization);
  // cache_.verifyCache(fk);
}

template <class KinematicsPlugin>
bool CachedMultiTipIKKinematicsPlugin<KinematicsPlugin>::initialize(const moveit::core::RobotModel& robot_model,
                                                                    const std::string& group_name,
                                                                    const std::string& base_frame,
                                                                    const std::vector<std::string>& tip_frames,
                                                                    double search_discretization)
{
  // call initialize method of wrapped class
  if (!KinematicsPlugin::initialize(robot_model, group_name, base_frame, tip_frames, search_discretization))
    return false;

  std::string cache_name = base_frame;
  std::accumulate(tip_frames.begin(), tip_frames.end(), cache_name);
  CachedIKKinematicsPlugin<KinematicsPlugin>::cache_.initializeCache(robot_model.getName(), group_name, cache_name,
                                                                     KinematicsPlugin::getJointNames().size());
  return true;
}

template <class KinematicsPlugin>
bool CachedMultiTipIKKinematicsPlugin<KinematicsPlugin>::initialize(const std::string& robot_description,
                                                                    const std::string& group_name,
                                                                    const std::string& base_frame,
                                                                    const std::vector<std::string>& tip_frames,
                                                                    double search_discretization)
{
  // call initialize method of wrapped class
  if (!KinematicsPlugin::initialize(robot_description, group_name, base_frame, tip_frames, search_discretization))
    return false;

  std::string cache_name = base_frame;
  std::accumulate(tip_frames.begin(), tip_frames.end(), cache_name);
  CachedIKKinematicsPlugin<KinematicsPlugin>::cache_.initializeCache(robot_description, group_name, cache_name,
                                                                     KinematicsPlugin::getJointNames().size());
  return true;
}

template <class KinematicsPlugin>
bool CachedIKKinematicsPlugin<KinematicsPlugin>::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                                               const std::vector<double>& ik_seed_state,
                                                               std::vector<double>& solution,
                                                               moveit_msgs::MoveItErrorCodes& error_code,
                                                               const KinematicsQueryOptions& options) const
{
  Pose pose(ik_pose);
  const IKEntry& nearest = cache_.getBestApproximateIKSolution(pose);
  bool solution_found = KinematicsPlugin::getPositionIK(ik_pose, nearest.second, solution, error_code, options) ||
                        KinematicsPlugin::getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
  if (solution_found)
    cache_.updateCache(nearest, pose, solution);
  return solution_found;
}

template <class KinematicsPlugin>
bool CachedIKKinematicsPlugin<KinematicsPlugin>::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                                  const std::vector<double>& ik_seed_state,
                                                                  double timeout, std::vector<double>& solution,
                                                                  moveit_msgs::MoveItErrorCodes& error_code,
                                                                  const KinematicsQueryOptions& options) const
{
  std::chrono::time_point<std::chrono::system_clock> start(std::chrono::system_clock::now());
  Pose pose(ik_pose);
  const IKEntry& nearest = cache_.getBestApproximateIKSolution(pose);
  bool solution_found =
      KinematicsPlugin::searchPositionIK(ik_pose, nearest.second, timeout, solution, error_code, options);
  if (!solution_found)
  {
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    solution_found =
        KinematicsPlugin::searchPositionIK(ik_pose, ik_seed_state, diff.count(), solution, error_code, options);
  }
  if (solution_found)
    cache_.updateCache(nearest, pose, solution);
  return solution_found;
}

template <class KinematicsPlugin>
bool CachedIKKinematicsPlugin<KinematicsPlugin>::searchPositionIK(
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& consistency_limits, std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code, const KinematicsQueryOptions& options) const
{
  std::chrono::time_point<std::chrono::system_clock> start(std::chrono::system_clock::now());
  Pose pose(ik_pose);
  const IKEntry& nearest = cache_.getBestApproximateIKSolution(pose);
  bool solution_found = KinematicsPlugin::searchPositionIK(ik_pose, nearest.second, timeout, consistency_limits,
                                                           solution, error_code, options);
  if (!solution_found)
  {
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    solution_found = KinematicsPlugin::searchPositionIK(ik_pose, ik_seed_state, diff.count(), consistency_limits,
                                                        solution, error_code, options);
  }
  if (solution_found)
    cache_.updateCache(nearest, pose, solution);
  return solution_found;
}

template <class KinematicsPlugin>
bool CachedIKKinematicsPlugin<KinematicsPlugin>::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                                  const std::vector<double>& ik_seed_state,
                                                                  double timeout, std::vector<double>& solution,
                                                                  const IKCallbackFn& solution_callback,
                                                                  moveit_msgs::MoveItErrorCodes& error_code,
                                                                  const KinematicsQueryOptions& options) const
{
  std::chrono::time_point<std::chrono::system_clock> start(std::chrono::system_clock::now());
  Pose pose(ik_pose);
  const IKEntry& nearest = cache_.getBestApproximateIKSolution(pose);
  bool solution_found = KinematicsPlugin::searchPositionIK(ik_pose, nearest.second, timeout, solution,
                                                           solution_callback, error_code, options);
  if (!solution_found)
  {
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    solution_found = KinematicsPlugin::searchPositionIK(ik_pose, ik_seed_state, diff.count(), solution,
                                                        solution_callback, error_code, options);
  }
  if (solution_found)
    cache_.updateCache(nearest, pose, solution);
  return solution_found;
}

template <class KinematicsPlugin>
bool CachedIKKinematicsPlugin<KinematicsPlugin>::searchPositionIK(
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code, const KinematicsQueryOptions& options) const
{
  std::chrono::time_point<std::chrono::system_clock> start(std::chrono::system_clock::now());
  Pose pose(ik_pose);
  const IKEntry& nearest = cache_.getBestApproximateIKSolution(pose);
  bool solution_found = KinematicsPlugin::searchPositionIK(ik_pose, nearest.second, timeout, consistency_limits,
                                                           solution, solution_callback, error_code, options);
  if (!solution_found)
  {
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    solution_found = KinematicsPlugin::searchPositionIK(ik_pose, ik_seed_state, diff.count(), consistency_limits,
                                                        solution, solution_callback, error_code, options);
  }
  if (solution_found)
    cache_.updateCache(nearest, pose, solution);
  return solution_found;
}

template <class KinematicsPlugin>
bool CachedMultiTipIKKinematicsPlugin<KinematicsPlugin>::searchPositionIK(
    const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code, const KinematicsQueryOptions& options,
    const moveit::core::RobotState* context_state) const
{
  std::chrono::time_point<std::chrono::system_clock> start(std::chrono::system_clock::now());
  std::vector<Pose> poses(ik_poses.size());
  for (unsigned int i = 0; i < poses.size(); ++i)
    poses[i] = Pose(ik_poses[i]);
  const IKEntry& nearest = CachedIKKinematicsPlugin<KinematicsPlugin>::cache_.getBestApproximateIKSolution(poses);
  bool solution_found =
      KinematicsPlugin::searchPositionIK(ik_poses, nearest.second, timeout, consistency_limits, solution,
                                         solution_callback, error_code, options, context_state);
  if (!solution_found)
  {
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    solution_found =
        KinematicsPlugin::searchPositionIK(ik_poses, ik_seed_state, diff.count(), consistency_limits, solution,
                                           solution_callback, error_code, options, context_state);
  }

  if (solution_found)
    CachedIKKinematicsPlugin<KinematicsPlugin>::cache_.updateCache(nearest, poses, solution);
  return solution_found;
}
}  // namespace cached_ik_kinematics_plugin
