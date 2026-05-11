/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Tyler Weaver
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
 *   * Neither the name of PickNik Robotics nor the
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

/* Author: Tyler Weaver */

/* These integration tests are based on the tutorials for using move_group to do a pick and place:
 * https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html
 */

// C++
#include <vector>

// ROS
#include <ros/ros.h>

// The Testing Framework and Utils
#include <gtest/gtest.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static const std::string PLANNING_GROUP = "panda_arm";
constexpr double PLANNING_TIME_S = 45.0;
constexpr double MAX_VELOCITY_SCALE = 1.0;
constexpr double MAX_ACCELERATION_SCALE = 1.0;

class PickPlaceTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/move_group_pick_place_test");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

    // set velocity and acceleration scaling factors (full speed)
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);

    // allow more time for planning
    move_group_->setPlanningTime(PLANNING_TIME_S);
  }

protected:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

TEST_F(PickPlaceTestFixture, PickPlaceTest)
{
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Create identity rotation quaternion */
  tf2::Quaternion zero_orientation;
  zero_orientation.setRPY(0, 0, 0);
  geometry_msgs::Quaternion zero_orientation_msg = tf2::toMsg(zero_orientation);

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation = zero_orientation_msg;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation = zero_orientation_msg;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation = zero_orientation_msg;

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion grasp_orientation;
  grasp_orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(grasp_orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  /* Add both finger joints of panda robot. */
  grasps[0].pre_grasp_posture.joint_names.resize(2);
  grasps[0].pre_grasp_posture.joint_names[0] = "panda_finger_joint1";
  grasps[0].pre_grasp_posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  grasps[0].pre_grasp_posture.points.resize(1);
  grasps[0].pre_grasp_posture.points[0].positions.resize(2);
  grasps[0].pre_grasp_posture.points[0].positions[0] = 0.04;
  grasps[0].pre_grasp_posture.points[0].positions[1] = 0.04;
  grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  /* Add both finger joints of panda robot and set them as closed. */
  grasps[0].grasp_posture = grasps[0].pre_grasp_posture;
  grasps[0].grasp_posture.points[0].positions[0] = 0.00;
  grasps[0].grasp_posture.points[0].positions[1] = 0.00;

  // Set support surface as table1.
  move_group_->setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  ASSERT_EQ(move_group_->pick("object", grasps), moveit::core::MoveItErrorCode::SUCCESS);

  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion place_orientation;
  place_orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(place_orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  /* Add both finger joints of panda robot. */
  place_location[0].post_place_posture.joint_names.resize(2);
  place_location[0].post_place_posture.joint_names[0] = "panda_finger_joint1";
  place_location[0].post_place_posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  place_location[0].post_place_posture.points.resize(1);
  place_location[0].post_place_posture.points[0].positions.resize(2);
  place_location[0].post_place_posture.points[0].positions[0] = 0.04;
  place_location[0].post_place_posture.points[0].positions[1] = 0.04;
  place_location[0].post_place_posture.points[0].time_from_start = ros::Duration(0.5);

  // Set support surface as table2.
  move_group_->setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  ASSERT_EQ(move_group_->place("object", place_location), moveit::core::MoveItErrorCode::SUCCESS);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_pick_place_test");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
