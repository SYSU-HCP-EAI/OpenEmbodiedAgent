/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Omid Heidari
   Desc:   TrajOpt planning plugin
*/

#include <moveit/planning_interface/planning_interface.h>

#include "moveit/collision_detection_fcl/collision_detector_allocator_fcl.h"

#include <class_loader/class_loader.hpp>

#include "trajopt_interface/trajopt_planning_context.h"

namespace trajopt_interface
{
class TrajOptPlannerManager : public planning_interface::PlannerManager
{
public:
  TrajOptPlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override
  {
    ROS_INFO_STREAM(" ======================================= initialize gets called, ns: " << ns);

    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);
    std::string trajopt_ns = ns.empty() ? "trajopt" : ns + "/trajopt";

    for (const std::string& gpName : model->getJointModelGroupNames())
    {
      ROS_INFO(" ======================================= group name: %s, robot model: %s", gpName.c_str(),
               model->getName().c_str());
      planning_contexts_[gpName] =
          TrajOptPlanningContextPtr(new TrajOptPlanningContext("trajopt_planning_context", gpName, model, nh_));
    }

    return true;
  }

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "TrajOpt";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("trajopt");
  }

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const override
  {
    ROS_INFO(" ======================================= getPlanningContext() is called ");
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      ROS_ERROR("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      ROS_ERROR("No planning scene supplied as input");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // create PlanningScene using hybrid collision detector
    planning_scene::PlanningScenePtr ps = planning_scene->diff();

    // set FCL for the collision
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(), true);

    // retrieve and configure existing context
    const TrajOptPlanningContextPtr& context = planning_contexts_.at(req.group_name);

    ROS_INFO(" ======================================= context is made ");

    context->setPlanningScene(ps);
    context->setMotionPlanRequest(req);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    return context;
  }

private:
  ros::NodeHandle nh_;

protected:
  std::map<std::string, TrajOptPlanningContextPtr> planning_contexts_;
};

}  // namespace trajopt_interface

// register the TrajOptPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(trajopt_interface::TrajOptPlannerManager, planning_interface::PlannerManager);
