/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Universitaet Hamburg
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

/* Author: Michael 'v4hn' Goerner */

#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/message_checks.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>

namespace
{
void makeSphere(moveit_msgs::CollisionObject& co)
{
  // set valid primitive
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::SPHERE;
  primitive.dimensions.push_back(/* SPHERE_RADIUS */ 1.0);
  co.primitives.push_back(primitive);
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  co.primitive_poses.push_back(pose);
}
}  // namespace

TEST(PlanningScene, fillInObjectPoseFromPrimitive)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "object_no_pose";
  co.operation = moveit_msgs::CollisionObject::ADD;
  makeSphere(co);
  scene.processCollisionObjectMsg(co);

  Eigen::Isometry3d primitive_pose;
  tf2::fromMsg(co.primitive_poses.at(0), primitive_pose);
  ASSERT_TRUE(scene.knowsFrameTransform(co.id)) << "failed to add object";
  EXPECT_TRUE(scene.getFrameTransform(co.id).isApprox(primitive_pose))
      << "scene did not use only primitive pose as object pose";
}

TEST(PlanningScene, fillInPrimitivePose)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  moveit_msgs::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "object_no_primitive_pose";
  co.operation = moveit_msgs::CollisionObject::ADD;
  makeSphere(co);
  co.pose = co.primitive_poses.at(0);
  co.primitive_poses.resize(0);
  scene.processCollisionObjectMsg(co);

  Eigen::Isometry3d object_pose;
  tf2::fromMsg(co.pose, object_pose);
  ASSERT_TRUE(scene.knowsFrameTransform(co.id)) << "failed to add object";
  EXPECT_TRUE(scene.getFrameTransform(co.id).isApprox(object_pose))
      << "scene did not implicitly fill in identity pose for only primitive";
}

TEST(PlanningScene, rememberMetadataWhenAttached)
{
  moveit::core::RobotModelPtr robot_model(moveit::core::RobotModelBuilder("empty_robot", "base_link").build());
  planning_scene::PlanningScene scene(robot_model);

  // prepare planning scene message to add a colored object
  moveit_msgs::PlanningScene scene_msg;
  scene_msg.robot_model_name = robot_model->getName();
  scene_msg.is_diff = true;
  scene_msg.robot_state.is_diff = true;

  moveit_msgs::CollisionObject co;
  co.header.frame_id = robot_model->getModelFrame();
  co.id = "blue_sphere";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.pose.orientation.w = 1.0;
  makeSphere(co);

  // meta-data 1: object type
  co.type.key = "blue_sphere_type";
  co.type.db = "{'type':'CustomDB'}";
  scene_msg.world.collision_objects.push_back(co);

  // meta-data 2: object color
  moveit_msgs::ObjectColor color;
  color.id = co.id;
  color.color.b = 1.0;
  color.color.a = 1.0;
  scene_msg.object_colors.push_back(color);

  EXPECT_FALSE(scene.hasObjectColor(co.id)) << "scene knows color before adding it(?)";
  EXPECT_FALSE(scene.hasObjectType(co.id)) << "scene knows type before adding it(?)";

  // add object to scene
  scene.usePlanningSceneMsg(scene_msg);

  EXPECT_TRUE(scene.hasObjectColor(co.id)) << "scene failed to add object color";
  EXPECT_EQ(scene.getObjectColor(co.id), color.color) << "scene added wrong object color";
  EXPECT_TRUE(scene.hasObjectType(co.id)) << "scene failed to add object type";
  EXPECT_EQ(scene.getObjectType(co.id), co.type) << "scene added wrong object type";

  // attach object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  aco.object.id = co.id;
  aco.link_name = robot_model->getModelFrame();
  scene.processAttachedCollisionObjectMsg(aco);

  EXPECT_EQ(scene.getObjectColor(co.id), color.color) << "scene forgot object color after it got attached";
  EXPECT_EQ(scene.getObjectType(co.id), co.type) << "scene forgot object type after it got attached";

  // trying to remove object from the scene while it is attached is expected to fail
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  EXPECT_FALSE(scene.processCollisionObjectMsg(co))
      << "scene removed attached object from collision world (although it's not there)";

  // detach again right away
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  scene.processAttachedCollisionObjectMsg(aco);

  EXPECT_EQ(scene.getObjectColor(co.id), color.color) << "scene forgot specified color after attach/detach";
  EXPECT_EQ(scene.getObjectType(co.id), co.type) << "scene forgot specified type after attach/detach";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
