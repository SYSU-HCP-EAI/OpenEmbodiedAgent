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

#pragma once

#include <sbpl/headers.h>
#include <planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <sbpl_interface/sbpl_interface.h>
#include <boost/thread.hpp>

namespace sbpl_interface
{
class SBPLMetaInterface
{
public:
  SBPLMetaInterface(const planning_models::RobotModelConstPtr& robot_model);
  virtual ~SBPLMetaInterface()
  {
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res);

  const PlanningStatistics& getLastPlanningStatistics() const
  {
    return last_planning_statistics_;
  }

protected:
  void runSolver(bool use_first, const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res,
                 const PlanningParameters& params);

  boost::mutex planner_done_mutex_;
  boost::condition_variable planner_done_condition_;
  bool first_ok_;
  bool first_done_;
  bool second_ok_;
  bool second_done_;

  boost::shared_ptr<sbpl_interface::SBPLInterface> sbpl_interface_first_;
  boost::shared_ptr<sbpl_interface::SBPLInterface> sbpl_interface_second_;

  PlanningStatistics last_planning_statistics_;
};
}  // namespace sbpl_interface
