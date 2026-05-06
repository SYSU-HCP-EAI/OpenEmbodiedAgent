/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Sachin Chitta
 *********************************************************************/

#pragma once

#include <kinematics_cache/kinematics_cache.h>
#include <planning_models/robot_model.h>
#include <pluginlib/class_loader.hpp>

namespace kinematics_cache_ros
{
class KinematicsCacheROS : public kinematics_cache::KinematicsCache
{
public:
  /** @class
   *  @brief An implementation of a cache for fast kinematics lookups
   *  This class inherits from KinematicsCache and provides an easy way of initializing the cache using ROS
   */
  KinematicsCacheROS(){};

  /** @brief Initialization function
   *  @param opt A set of options for setting up the cache
   */
  bool init(const kinematics_cache::KinematicsCache::Options& opt, const std::string& kinematics_solver_name,
            const std::string& group_name, const std::string& base_frame, const std::string& tip_frame,
            double search_discretization);

private:
  kinematics::KinematicsBase* kinematics_solver_; /** An instance of a kinematics solver needed by this class */

  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_; /** A loader needed to load
                                                                                              the instance of a
                                                                                              kinematics solver */

  planning_models::RobotModelPtr kinematic_model_; /** A kinematics model */
};
}  // namespace kinematics_cache_ros
