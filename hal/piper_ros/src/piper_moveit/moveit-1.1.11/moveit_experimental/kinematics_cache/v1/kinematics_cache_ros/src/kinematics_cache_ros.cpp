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

#include <kinematics_cache_ros/kinematics_cache_ros.h>

// ROS
#include <boost/shared_ptr.hpp>

// MoveIt
#include <planning_models/kinematic_state.h>
#include <rdf_loader/rdf_loader.h>
#include <urdf_interface/model.h>
#include <urdf/model.h>
#include <srdf/model.h>

namespace kinematics_cache_ros
{
bool KinematicsCacheROS::init(const kinematics_cache::KinematicsCache::Options& opt,
                              const std::string& kinematics_solver_name, const std::string& group_name,
                              const std::string& base_frame, const std::string& tip_frame, double search_discretization)
{
  kinematics_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>("kinematics_base", "kinematics::"
                                                                                              "KinematicsBase");

  try
  {
    kinematics_solver_ = kinematics_loader_->createClassInstance(kinematics_solver_name);
  }
  catch (pluginlib::PluginlibException& ex)  // handle the class failing to load
  {
    ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
    return false;
  }

  if (!kinematics_solver_->initialize(group_name, base_frame, tip_frame, search_discretization))
  {
    ROS_ERROR("Could not initialize solver");
    return false;
  }

  rdf_loader::RDFLoader rdf_loader;
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();
  kinematic_model_ = std::make_shared<planning_models::RobotModel>(urdf_model, srdf);

  if (!initialize((kinematics::KinematicsBaseConstPtr&)kinematics_solver_,
                  (planning_models::RobotModelConstPtr&)kinematic_model_, opt))
  {
    return false;
  }

  return true;
}
}  // namespace kinematics_cache_ros
