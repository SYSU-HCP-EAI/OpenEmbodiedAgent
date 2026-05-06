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

#include <ros/ros.h>

#include <planning_interface/planning_interface.h>
#include <planning_scene/planning_scene.h>
#include <planning_models/robot_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <sbpl_interface/sbpl_interface.h>

#include <pluginlib/class_list_macros.hpp>

namespace sbpl_interface_ros
{
class SBPLPlanner : public planning_interface::Planner
{
public:
  void init(const planning_models::RobotModelConstPtr& model)
  {
    ros::NodeHandle nh;
    display_bfs_publisher_ = nh.advertise<visualization_msgs::Marker>("planning_components_visualization", 10, true);
    sbpl_interface_ = std::make_shared<sbpl_interface::SBPLInterface>(model);
  }

  bool canServiceRequest(const moveit_msgs::GetMotionPlan::Request& req,
                         planning_interface::PlannerCapability& capabilities) const
  {
    // TODO: this is a dummy implementation
    //      capabilities.dummy = false;
    return true;
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res) const
  {
    sbpl_interface::PlanningParameters params;
    params.use_bfs_ = false;
    bool solve_ok = sbpl_interface_->solve(planning_scene, req, res, params);
    return solve_ok;
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::MotionPlanDetailedResponse& res) const
  {
    sbpl_interface::PlanningParameters params;
    params.use_bfs_ = false;

    moveit_msgs::GetMotionPlan::Response res2;
    if (sbpl_interface_->solve(planning_scene, req, res2, params))
    {
      res.trajectory_start = res2.trajectory_start;
      res.trajectory.push_back(res2.trajectory);
      res.description.push_back("plan");
      res.processing_time.push_back(res2.planning_time);
      return true;
    }
    else
      return false;
  }

  std::string getDescription() const
  {
    return "SBPL";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const
  {
    algs.resize(1);
    algs[0] = "SBPL";
  }

  void terminate() const
  {
    // TODO - make interruptible
  }

private:
  ros::Publisher display_bfs_publisher_;
  boost::shared_ptr<sbpl_interface::SBPLInterface> sbpl_interface_;
};

}  // namespace sbpl_interface_ros

PLUGINLIB_EXPORT_CLASS(sbpl_interface_ros::SBPLPlanner, planning_interface::Planner);
