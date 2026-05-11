/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
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

/* Author: Robert Haschke */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/kinematic_constraints/utils.h>
#include <class_loader/class_loader.hpp>
#include <ros/ros.h>

namespace default_planner_request_adapters
{
class ResolveConstraintFrames : public planning_request_adapter::PlanningRequestAdapter
{
public:
  ResolveConstraintFrames() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& /*nh*/) override
  {
  }

  std::string getDescription() const override
  {
    return "Resolve constraint frames to robot links";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    ROS_DEBUG("Running '%s'", getDescription().c_str());
    planning_interface::MotionPlanRequest modified = req;
    kinematic_constraints::resolveConstraintFrames(planning_scene->getCurrentState(), modified.path_constraints);
    for (moveit_msgs::Constraints& constraint : modified.goal_constraints)
      kinematic_constraints::resolveConstraintFrames(planning_scene->getCurrentState(), constraint);
    return planner(planning_scene, modified, res);
  }
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::ResolveConstraintFrames,
                            planning_request_adapter::PlanningRequestAdapter);
