/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, PickNik LLC
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

/* Author: Dave Coleman
   Desc:   Simple utility to see all the collision objects in a planning scene, including attached
*/

#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string LOGNAME = "print_planning_scene_info";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "print_model_info_to_console");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ROS_INFO_STREAM_NAMED(LOGNAME, "Getting planning scene info to print");

  // Create planning scene monitor
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Planning scene not configured");
    return 1;
  }

  psm.requestPlanningSceneState(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE);

  psm.getPlanningScene()->printKnownObjects();

  return 0;
}
