/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Felix von Drigalski, Jacob Aas, Tyler Weaver
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
 *   * Neither the name of OMRON SINIC X or PickNik Robotics nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Felix von Drigalski, Jacob Aas, Tyler Weaver, Boston Cleek */

/* This integration test is based on the tutorial for using subframes
 * https://ros-planning.github.io/moveit_tutorials/doc/subframes/subframes_tutorial.html
 */

// C++
#include <vector>
#include <map>

// ROS
#include <ros/ros.h>

// The Testing Framework and Utils
#include <gtest/gtest.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

constexpr double EPSILON = 1e-2;
constexpr double Z_OFFSET = 0.05;
constexpr double PLANNING_TIME_S = 30.0;

const double TAU = 2 * M_PI;  // One turn (360°) in radians

// Function copied from tutorial
// a small helper function to create our planning requests and move the robot.
bool moveToCartPose(const geometry_msgs::PoseStamped& pose, moveit::planning_interface::MoveGroupInterface& group,
                    const std::string& end_effector_link)
{
  group.clearPoseTargets();
  group.setEndEffectorLink(end_effector_link);
  group.setStartStateToCurrentState();
  group.setPoseTarget(pose);

  moveit::planning_interface::MoveGroupInterface::Plan myplan;
  if (group.plan(myplan) && group.execute(myplan))
  {
    return true;
  }

  ROS_WARN("Failed to perform motion.");
  return false;
}

// Function copied from subframes tutorial
// This helper function creates two objects and publishes them to the PlanningScene: a box and a cylinder.
// The box spawns in front of the gripper, the cylinder at the tip of the gripper, as if it had been grasped.
void spawnCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  double z_offset_box = .25;  // The z-axis points away from the gripper
  double z_offset_cylinder = .1;

  moveit_msgs::CollisionObject box;
  box.id = "box";
  box.header.frame_id = "panda_hand";
  box.pose.position.z = z_offset_box;
  box.pose.orientation.w = 1.0;  // Neutral orientation

  box.primitives.resize(1);
  box.primitive_poses.resize(1);
  box.primitives[0].type = box.primitives[0].BOX;
  box.primitives[0].dimensions.resize(3);
  box.primitives[0].dimensions[0] = 0.05;
  box.primitives[0].dimensions[1] = 0.1;
  box.primitives[0].dimensions[2] = 0.02;
  box.primitive_poses[0].orientation.w = 1.0;  // Neutral orientation

  box.subframe_names.resize(1);
  box.subframe_poses.resize(1);

  box.subframe_names[0] = "bottom";
  box.subframe_poses[0].position.y = -.05;

  tf2::Quaternion orientation;
  orientation.setRPY(TAU / 4.0, 0, 0);  // 1/4 turn
  box.subframe_poses[0].orientation = tf2::toMsg(orientation);

  // Next, define the cylinder
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "cylinder";
  cylinder.header.frame_id = "panda_hand";
  cylinder.pose.position.z = z_offset_cylinder;
  orientation.setRPY(0, TAU / 4.0, 0);
  cylinder.pose.orientation = tf2::toMsg(orientation);

  cylinder.primitives.resize(1);
  cylinder.primitive_poses.resize(1);
  cylinder.primitives[0].type = box.primitives[0].CYLINDER;
  cylinder.primitives[0].dimensions.resize(2);
  cylinder.primitives[0].dimensions[0] = 0.06;      // height (along x)
  cylinder.primitives[0].dimensions[1] = 0.005;     // radius
  cylinder.primitive_poses[0].orientation.w = 1.0;  // Neutral orientation

  cylinder.subframe_poses.resize(1);
  cylinder.subframe_names.resize(1);
  cylinder.subframe_names[0] = "tip";
  cylinder.subframe_poses[0].position.z = 0.03;
  cylinder.subframe_poses[0].orientation.w = 1.0;  // Neutral orientation

  // Lastly, the objects are published to the PlanningScene. In this tutorial, we publish a box and a cylinder.
  box.operation = moveit_msgs::CollisionObject::ADD;
  cylinder.operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface.applyCollisionObjects({ box, cylinder });
}

TEST(TestPlanUsingSubframes, SubframesTests)
{
  const std::string log_name = "test_plan_using_subframes";
  ros::NodeHandle nh(log_name);

  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(PLANNING_TIME_S);

  spawnCollisionObjects(planning_scene_interface);
  moveit_msgs::AttachedCollisionObject att_coll_object;
  att_coll_object.object.id = "cylinder";
  att_coll_object.link_name = "panda_hand";
  att_coll_object.object.operation = att_coll_object.object.ADD;
  att_coll_object.object.pose.orientation.w = 1.0;
  planning_scene_interface.applyAttachedCollisionObject(att_coll_object);

  tf2::Quaternion target_orientation;
  target_orientation.setRPY(0, TAU / 2.0, TAU / 4.0);
  geometry_msgs::PoseStamped target_pose_stamped;
  target_pose_stamped.pose.orientation = tf2::toMsg(target_orientation);
  target_pose_stamped.pose.position.z = Z_OFFSET;

  ROS_INFO_STREAM_NAMED(log_name, "Moving to bottom of box with cylinder tip");
  target_pose_stamped.header.frame_id = "box/bottom";
  ASSERT_TRUE(moveToCartPose(target_pose_stamped, group, "cylinder/tip"));
  planning_scene_monitor->requestPlanningSceneState();
  {
    planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor);

    // get the tip and box subframe locations in world
    // TODO (felixvd): Get these from the plan's goal state instead, so we don't have to execute the motion in CI
    Eigen::Isometry3d cyl_tip = planning_scene->getFrameTransform("cylinder/tip");
    Eigen::Isometry3d box_subframe = planning_scene->getFrameTransform(target_pose_stamped.header.frame_id);
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(target_pose_stamped.pose, target_pose);

    // expect that they are identical
    std::stringstream ss;
    ss << "target frame: \n" << (box_subframe * target_pose).matrix() << "\ncylinder frame: \n" << cyl_tip.matrix();
    EXPECT_TRUE(cyl_tip.isApprox(box_subframe * target_pose, EPSILON)) << ss.str();

    // Check that robot wrist is where we expect it to be
    Eigen::Isometry3d panda_link = planning_scene->getFrameTransform("panda_link8");
    Eigen::Isometry3d expected_pose = Eigen::Isometry3d(Eigen::Translation3d(0.307, 0.13, 0.44)) *
                                      Eigen::Isometry3d(Eigen::Quaterniond(0.0003809, -0.38303, 0.92373, 0.00028097));

    ss.str("");
    ss << "panda link frame: \n" << panda_link.matrix() << "\nexpected pose: \n" << expected_pose.matrix();
    EXPECT_TRUE(panda_link.isApprox(expected_pose, EPSILON)) << ss.str();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_test_plan_using_subframes");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
