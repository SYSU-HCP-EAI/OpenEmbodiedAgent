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

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

void demoPick(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::Grasp> grasps;
  for (std::size_t i = 0; i < 20; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::Grasp g;
    g.grasp_pose = p;
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.direction.header = p.header;
    g.pre_grasp_approach.min_distance = 0.2;
    g.pre_grasp_approach.desired_distance = 0.4;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.27;
    g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 1;

    g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 0;

    grasps.push_back(g);
  }
  group.pick("bubu", std::move(grasps));
}

void demoPlace(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;
  for (std::size_t i = 0; i < 20; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    moveit_msgs::PlaceLocation g;
    g.place_pose = p;
    g.pre_place_approach.direction.vector.x = 1.0;
    g.post_place_retreat.direction.vector.z = 1.0;
    g.post_place_retreat.direction.header = p.header;
    g.pre_place_approach.min_distance = 0.2;
    g.pre_place_approach.desired_distance = 0.4;
    g.post_place_retreat.min_distance = 0.1;
    g.post_place_retreat.desired_distance = 0.27;

    g.post_place_posture.joint_names.resize(1, "r_gripper_joint");
    g.post_place_posture.points.resize(1);
    g.post_place_posture.points[0].positions.resize(1);
    g.post_place_posture.points[0].positions[0] = 0;

    loc.push_back(g);
  }
  group.place("bubu", std::move(loc));
}

void attachObject()
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group(argc > 1 ? argv[1] : "right_arm");
  demoPlace(group);

  ros::Duration(2).sleep();

  return 0;
}
