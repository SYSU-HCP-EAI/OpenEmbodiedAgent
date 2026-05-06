/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_planning_scene", ros::init_options::AnonymousName);

  // decide whether to publish the full scene
  bool full_scene = false;

  // the index of the argument with the filename
  int filename_index = 1;
  if (argc > 2)
    if (strncmp(argv[1], "--scene", 7) == 0)
    {
      full_scene = true;
      filename_index = 2;
    }

  if (argc > 1)
  {
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    ros::Publisher pub_scene;
    if (full_scene)
      pub_scene = nh.advertise<moveit_msgs::PlanningScene>(
          planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC, 1);
    else
      pub_scene = nh.advertise<moveit_msgs::PlanningSceneWorld>(
          planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, 1);

    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "robot_description";
    opt.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoaderPtr rml(new robot_model_loader::RobotModelLoader(opt));
    planning_scene::PlanningScene ps(rml->getModel());

    std::ifstream f(argv[filename_index]);
    if (ps.loadGeometryFromStream(f))
    {
      ROS_INFO("Publishing geometry from '%s' ...", argv[filename_index]);
      moveit_msgs::PlanningScene ps_msg;
      ps.getPlanningSceneMsg(ps_msg);
      ps_msg.is_diff = true;

      ros::WallDuration dt(0.5);
      unsigned int attempts = 0;
      while (pub_scene.getNumSubscribers() < 1 && ++attempts < 100)
        dt.sleep();

      if (full_scene)
        pub_scene.publish(ps_msg);
      else
        pub_scene.publish(ps_msg.world);

      ros::Duration(1).sleep();
    }
  }
  else
    ROS_WARN("A filename was expected as argument. That file should be a text representation of the geometry in a "
             "planning scene.");

  ros::shutdown();
  return 0;
}
