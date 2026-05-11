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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

namespace robot_model_loader
{
MOVEIT_CLASS_FORWARD(RobotModelLoader);  // Defines RobotModelLoaderPtr, ConstPtr, WeakPtr... etc

/** @class RobotModelLoader */
class RobotModelLoader
{
public:
  /** @brief Structure that encodes the options to be passed to the RobotModelLoader constructor */
  struct Options
  {
    Options(const std::string& robot_description = "robot_description")
      : robot_description_(robot_description), load_kinematics_solvers_(true)
    {
    }

    Options(const std::string& urdf_string, const std::string& srdf_string)
      : urdf_string_(urdf_string), srdf_string_(srdf_string), load_kinematics_solvers_(true)
    {
    }

    /**  @brief The string name corresponding to the ROS param where the URDF is loaded; Using the same parameter name
       plus the "_planning" suffix, additional configuration can be specified (e.g., additional joint limits).
         Loading from the param server is attempted only if loading from string fails. */
    std::string robot_description_;

    /** @brief The string content of the URDF and SRDF documents. Loading from string is attempted only if loading from
     * XML fails */
    std::string urdf_string_, srdf_string_;

    /** @brief Flag indicating whether the kinematics solvers should be loaded as well, using specified ROS parameters
     */
    bool load_kinematics_solvers_;
  };

  /** @brief Default constructor */
  RobotModelLoader(const Options& opt = Options());

  RobotModelLoader(const std::string& robot_description, bool load_kinematics_solvers = true);

  ~RobotModelLoader();

  /** @brief Get the constructed planning_models::RobotModel */
  const moveit::core::RobotModelPtr& getModel() const
  {
    return model_;
  }

  /** @brief Get the resolved parameter name for the robot description */
  const std::string& getRobotDescription() const
  {
    return rdf_loader_->getRobotDescription();
  }

  /** @brief Get the parsed URDF model*/
  const urdf::ModelInterfaceSharedPtr& getURDF() const
  {
    return rdf_loader_->getURDF();
  }

  /** @brief Get the parsed SRDF model*/
  const srdf::ModelSharedPtr& getSRDF() const
  {
    return rdf_loader_->getSRDF();
  }

  /** @brief Get the instance of rdf_loader::RDFLoader that was used to load the robot description */
  const rdf_loader::RDFLoaderPtr& getRDFLoader() const
  {
    return rdf_loader_;
  }

  /** \brief Get the kinematics solvers plugin loader.
      \note This instance needs to be kept in scope, otherwise kinematics solver plugins may get unloaded. */
  const kinematics_plugin_loader::KinematicsPluginLoaderPtr& getKinematicsPluginLoader() const
  {
    return kinematics_loader_;
  }

  /** @brief Load the kinematics solvers into the kinematic model. This is done by default, unless disabled explicitly
   * by the options passed to the constructor */
  void loadKinematicsSolvers(const kinematics_plugin_loader::KinematicsPluginLoaderPtr& kloader =
                                 kinematics_plugin_loader::KinematicsPluginLoaderPtr());

private:
  void configure(const Options& opt);

  moveit::core::RobotModelPtr model_;
  rdf_loader::RDFLoaderPtr rdf_loader_;
  kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_loader_;
};
}  // namespace robot_model_loader
