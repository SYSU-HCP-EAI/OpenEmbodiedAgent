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
 *   * Neither the name of the Willow Garage nor the names of its
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

/** \author E. Gil Jones */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_env_distance_field.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <geometric_shapes/shape_operations.h>
#include <urdf_parser/urdf_parser.h>

#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <boost/filesystem.hpp>

#include <ros/package.h>

typedef collision_detection::CollisionEnvDistanceField DefaultCEnvType;

class DistanceFieldCollisionDetectionTester : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("pr2");

    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(robot_model_->getLinkModelNames(), true);

    std::map<std::string, std::vector<collision_detection::CollisionSphere>> link_body_decompositions;
    cenv_ = std::make_shared<DefaultCEnvType>(robot_model_, link_body_decompositions);
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;

  moveit::core::TransformsPtr ftf_;
  moveit::core::TransformsConstPtr ftf_const_;

  collision_detection::CollisionEnvPtr cenv_;

  collision_detection::AllowedCollisionMatrixPtr acm_;
};

TEST_F(DistanceFieldCollisionDetectionTester, DefaultNotInCollision)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = "whole_body";
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_FALSE(res.collision);
}

TEST_F(DistanceFieldCollisionDetectionTester, ChangeTorsoPosition)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res1;
  collision_detection::CollisionResult res2;

  req.group_name = "right_arm";
  cenv_->checkSelfCollision(req, res1, robot_state, *acm_);
  std::map<std::string, double> torso_val;
  torso_val["torso_lift_joint"] = .15;
  robot_state.setVariablePositions(torso_val);
  robot_state.update();
  cenv_->checkSelfCollision(req, res1, robot_state, *acm_);
  cenv_->checkSelfCollision(req, res1, robot_state, *acm_);
}

TEST_F(DistanceFieldCollisionDetectionTester, LinksInCollision)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res1;
  collision_detection::CollisionResult res2;
  collision_detection::CollisionResult res3;
  // req.contacts = true;
  // req.max_contacts = 100;
  req.group_name = "whole_body";

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation().x() = .01;

  robot_state.updateStateWithLinkAt("base_link", Eigen::Isometry3d::Identity());
  robot_state.updateStateWithLinkAt("base_bellow_link", offset);

  acm_->setEntry("base_link", "base_bellow_link", false);
  cenv_->checkSelfCollision(req, res1, robot_state, *acm_);
  ASSERT_TRUE(res1.collision);

  acm_->setEntry("base_link", "base_bellow_link", true);
  cenv_->checkSelfCollision(req, res2, robot_state, *acm_);
  ASSERT_FALSE(res2.collision);

  robot_state.updateStateWithLinkAt("r_gripper_palm_link", Eigen::Isometry3d::Identity());
  robot_state.updateStateWithLinkAt("l_gripper_palm_link", offset);

  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);
  cenv_->checkSelfCollision(req, res3, robot_state, *acm_);
  ASSERT_TRUE(res3.collision);
}

TEST_F(DistanceFieldCollisionDetectionTester, ContactReporting)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 1;
  req.group_name = "whole_body";

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation().x() = .01;

  robot_state.updateStateWithLinkAt("base_link", Eigen::Isometry3d::Identity());
  robot_state.updateStateWithLinkAt("base_bellow_link", offset);

  robot_state.updateStateWithLinkAt("r_gripper_palm_link", Eigen::Isometry3d::Identity());
  robot_state.updateStateWithLinkAt("l_gripper_palm_link", offset);

  acm_->setEntry("base_link", "base_bellow_link", false);
  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);

  collision_detection::CollisionResult res;
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);
  EXPECT_EQ(res.contacts.size(), 1u);
  EXPECT_EQ(res.contacts.begin()->second.size(), 1u);

  res.clear();
  req.max_contacts = 2;
  req.max_contacts_per_pair = 1;
  //  req.verbose = true;
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);
  EXPECT_EQ(res.contact_count, 2u);
  EXPECT_EQ(res.contacts.begin()->second.size(), 1u);

  res.contacts.clear();
  res.contact_count = 0;

  req.max_contacts = 10;
  req.max_contacts_per_pair = 2;
  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(robot_model_->getLinkModelNames(), false);
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);
  EXPECT_LE(res.contacts.size(), 10u);
  EXPECT_LE(res.contact_count, 10u);
}

TEST_F(DistanceFieldCollisionDetectionTester, ContactPositions)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 1;
  req.group_name = "whole_body";

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  Eigen::Isometry3d pos1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pos2 = Eigen::Isometry3d::Identity();

  pos1.translation().x() = 5.0;
  pos2.translation().x() = 5.01;

  robot_state.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  robot_state.updateStateWithLinkAt("l_gripper_palm_link", pos2);

  acm_->setEntry("r_gripper_palm_link", "l_gripper_palm_link", false);

  collision_detection::CollisionResult res;
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);
  ASSERT_EQ(res.contacts.size(), 1u);
  ASSERT_EQ(res.contacts.begin()->second.size(), 1u);

  for (collision_detection::CollisionResult::ContactMap::const_iterator it = res.contacts.begin();
       it != res.contacts.end(); it++)
  {
    EXPECT_NEAR(it->second[0].pos.x(), 5.0, .33);
  }

  pos1 = Eigen::Isometry3d(Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  pos2 = Eigen::Isometry3d(Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond(0.965, 0.0, 0.258, 0.0));

  robot_state.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  robot_state.updateStateWithLinkAt("l_gripper_palm_link", pos2);

  collision_detection::CollisionResult res2;
  cenv_->checkSelfCollision(req, res2, robot_state, *acm_);
  ASSERT_TRUE(res2.collision);
  ASSERT_EQ(res2.contacts.size(), 1u);
  ASSERT_EQ(res2.contacts.begin()->second.size(), 1u);

  for (collision_detection::CollisionResult::ContactMap::const_iterator it = res2.contacts.begin();
       it != res2.contacts.end(); it++)
  {
    EXPECT_NEAR(it->second[0].pos.x(), 3.0, 0.33);
  }

  pos1 = Eigen::Isometry3d(Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  pos2 = Eigen::Isometry3d(Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond(M_PI / 4.0, 0.0, M_PI / 4.0, 0.0));

  robot_state.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  robot_state.updateStateWithLinkAt("l_gripper_palm_link", pos2);

  collision_detection::CollisionResult res3;
  cenv_->checkSelfCollision(req, res2, robot_state, *acm_);
  ASSERT_FALSE(res3.collision);
}

TEST_F(DistanceFieldCollisionDetectionTester, AttachedBodyTester)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  req.group_name = "right_arm";

  acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(robot_model_->getLinkModelNames(), true);

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();

  Eigen::Isometry3d pos1 = Eigen::Isometry3d::Identity();
  pos1.translation().x() = 1.0;

  robot_state.updateStateWithLinkAt("r_gripper_palm_link", pos1);
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_FALSE(res.collision);

  shapes::Shape* shape = new shapes::Box(.25, .25, .25);
  cenv_->getWorld()->addToObject("box", shapes::ShapeConstPtr(shape), pos1);

  res = collision_detection::CollisionResult();
  cenv_->checkRobotCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);

  // deletes shape
  cenv_->getWorld()->removeObject("box");

  const auto identity = Eigen::Isometry3d::Identity();
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Isometry3d poses;
  shapes.push_back(shapes::ShapeConstPtr(new shapes::Box(.25, .25, .25)));
  poses.push_back(identity);
  std::set<std::string> touch_links;
  trajectory_msgs::JointTrajectory empty_state;

  robot_state.attachBody(std::make_unique<moveit::core::AttachedBody>(
      robot_state.getLinkModel("r_gripper_palm_link"), "box", identity, shapes, poses, touch_links, empty_state));

  res = collision_detection::CollisionResult();
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);

  // deletes shape
  robot_state.clearAttachedBody("box");

  touch_links.insert("r_gripper_palm_link");
  shapes[0] = std::make_shared<shapes::Box>(.1, .1, .1);

  robot_state.attachBody(std::make_unique<moveit::core::AttachedBody>(
      robot_state.getLinkModel("r_gripper_palm_link"), "box", identity, shapes, poses, touch_links, empty_state));

  res = collision_detection::CollisionResult();
  cenv_->checkSelfCollision(req, res, robot_state, *acm_);
  // ASSERT_FALSE(res.collision);

  pos1.translation().x() = 1.01;
  shapes::Shape* coll = new shapes::Box(.1, .1, .1);
  cenv_->getWorld()->addToObject("coll", shapes::ShapeConstPtr(coll), pos1);
  res = collision_detection::CollisionResult();
  cenv_->checkRobotCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);

  acm_->setEntry("coll", "r_gripper_palm_link", true);
  res = collision_detection::CollisionResult();
  cenv_->checkRobotCollision(req, res, robot_state, *acm_);
  ASSERT_TRUE(res.collision);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
