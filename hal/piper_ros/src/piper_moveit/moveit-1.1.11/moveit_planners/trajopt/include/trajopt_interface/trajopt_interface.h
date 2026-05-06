/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, LLC.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Omid Heidari */
#pragma once

#include <ros/ros.h>
#include <trajopt_sco/sco_common.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include "problem_description.h"

namespace trajopt_interface
{
MOVEIT_CLASS_FORWARD(TrajOptInterface);  // Defines TrajOptInterfacePtr, ConstPtr, WeakPtr... etc

class TrajOptInterface
{
public:
  TrajOptInterface(const ros::NodeHandle& nh = ros::NodeHandle("~"));

  const sco::BasicTrustRegionSQPParameters& getParams() const
  {
    return params_;
  }

  bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req, moveit_msgs::MotionPlanDetailedResponse& res);

protected:
  /** @brief Configure everything using the param server */
  void setTrajOptParams(sco::BasicTrustRegionSQPParameters& param);
  void setDefaultTrajOPtParams();
  void setProblemInfoParam(ProblemInfo& problem_info);
  void setJointPoseTermInfoParams(JointPoseTermInfoPtr& jp, std::string name);
  trajopt::DblVec extractStartJointValues(const planning_interface::MotionPlanRequest& req,
                                          const std::vector<std::string>& group_joint_names);

  ros::NodeHandle nh_;  /// The ROS node handle
  sco::BasicTrustRegionSQPParameters params_;
  std::vector<sco::Optimizer::Callback> optimizer_callbacks_;
  TrajOptProblemPtr trajopt_problem_;
  std::string name_;
};

void callBackFunc(sco::OptProb* opt_prob, sco::OptResults& opt_res);
}  // namespace trajopt_interface
