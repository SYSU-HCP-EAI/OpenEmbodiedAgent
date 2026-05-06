/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include "pilz_industrial_motion_planner/limits_container.h"

#include <memory>
#include <vector>

#include <moveit/planning_interface/planning_interface.h>

#include "pilz_industrial_motion_planner/limits_container.h"

namespace pilz_industrial_motion_planner
{
/**
 * @brief Base class for all PlanningContextLoaders.
 * Since planning_interface::PlanningContext has a non empty ctor,
 * classes derived from it can not be plugins.
 * This class serves as base class for wrappers.
 */
class PlanningContextLoader
{
public:
  PlanningContextLoader();
  virtual ~PlanningContextLoader();

  /// Return the algorithm the loader uses
  virtual std::string getAlgorithm() const;

  /**
   * @brief Sets the robot model that can be passed to the planning context
   * @param model The robot model
   * @return false if could not be set
   */
  virtual bool setModel(const moveit::core::RobotModelConstPtr& model);

  /**
   * @brief Sets limits the planner can pass to the contexts
   * @param limits container of limits, no guarantee to contain the limits for
   * all joints of the model
   * @return true if limits could be set
   */
  virtual bool setLimits(const pilz_industrial_motion_planner::LimitsContainer& limits);

  /**
   * @brief Return the planning context
   * @param planning_context
   * @param name context name
   * @param group name of the planning group
   * @return true on success, false otherwise
   */
  virtual bool loadContext(planning_interface::PlanningContextPtr& planning_context, const std::string& name,
                           const std::string& group) const = 0;

protected:
  /**
   * @brief Return the planning context of type T
   * @param planning_context
   * @param name context name
   * @param group name of the planning group
   * @return true on success, false otherwise
   */
  template <typename T>
  bool loadContext(planning_interface::PlanningContextPtr& planning_context, const std::string& name,
                   const std::string& group) const;

protected:
  /// Name of the algorithm
  std::string alg_;

  /// True if limits are set
  bool limits_set_;

  /// Limits to be used during planning
  pilz_industrial_motion_planner::LimitsContainer limits_;

  /// True if model is set
  bool model_set_;

  /// The robot model
  moveit::core::RobotModelConstPtr model_;
};

typedef boost::shared_ptr<PlanningContextLoader> PlanningContextLoaderPtr;
typedef boost::shared_ptr<const PlanningContextLoader> PlanningContextLoaderConstPtr;

template <typename T>
bool PlanningContextLoader::loadContext(planning_interface::PlanningContextPtr& planning_context,
                                        const std::string& name, const std::string& group) const
{
  if (limits_set_ && model_set_)
  {
    planning_context = std::make_shared<T>(name, group, model_, limits_);
    return true;
  }
  else
  {
    if (!limits_set_)
    {
      ROS_ERROR_STREAM("Limits are not defined. Cannot load planning context. "
                       "Call setLimits loadContext");
    }
    if (!model_set_)
    {
      ROS_ERROR_STREAM("Robot model was not set");
    }
    return false;
  }
}

}  // namespace pilz_industrial_motion_planner
