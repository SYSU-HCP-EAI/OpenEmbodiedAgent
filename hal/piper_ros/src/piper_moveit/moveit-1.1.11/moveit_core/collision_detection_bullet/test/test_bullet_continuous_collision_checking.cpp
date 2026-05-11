/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <moveit/collision_detection_bullet/bullet_integration/bullet_cast_bvh_manager.h>
#include <moveit/collision_detection_bullet/bullet_integration/bullet_discrete_bvh_manager.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

namespace cb = collision_detection_bullet;

/** \brief Brings the panda robot in user defined home position */
inline void setToHome(moveit::core::RobotState& panda_state)
{
  panda_state.setToDefaultValues();
  double joint2 = -0.785;
  double joint4 = -2.356;
  double joint6 = 1.571;
  double joint7 = 0.785;
  panda_state.setJointPositions("panda_joint2", &joint2);
  panda_state.setJointPositions("panda_joint4", &joint4);
  panda_state.setJointPositions("panda_joint6", &joint6);
  panda_state.setJointPositions("panda_joint7", &joint7);
  panda_state.update();
}

class BulletCollisionDetectionTester : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    robot_model_ok_ = static_cast<bool>(robot_model_);

    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(*robot_model_->getSRDF());

    cenv_ = std::make_shared<collision_detection::CollisionEnvBullet>(robot_model_);

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

    setToHome(*robot_state_);
  }

  void TearDown() override
  {
  }

protected:
  bool robot_model_ok_;

  moveit::core::RobotModelPtr robot_model_;

  collision_detection::CollisionEnvPtr cenv_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  moveit::core::RobotStatePtr robot_state_;
};

void addCollisionObjects(cb::BulletCastBVHManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(1, 1, 1));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  cb::AlignedVector<Eigen::Isometry3d> obj1_poses;
  std::vector<cb::CollisionObjectType> obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(cb::CollisionObjectType::USE_SHAPE_TYPE);

  cb::CollisionObjectWrapperPtr cow_1(new cb::CollisionObjectWrapper(
      "static_box_link", collision_detection::BodyType::WORLD_OBJECT, obj1_shapes, obj1_poses, obj1_types));
  checker.addCollisionObject(cow_1);

  ////////////////////////////
  // Add moving box to checker
  ////////////////////////////
  shapes::ShapePtr moving_box(new shapes::Box(0.2, 0.2, 0.2));
  Eigen::Isometry3d moving_box_pose;

  moving_box_pose.setIdentity();
  moving_box_pose.translation() = Eigen::Vector3d(0.5, -0.5, 0);

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  cb::AlignedVector<Eigen::Isometry3d> obj2_poses;
  std::vector<cb::CollisionObjectType> obj2_types;
  obj2_shapes.push_back(moving_box);
  obj2_poses.push_back(moving_box_pose);
  obj2_types.push_back(cb::CollisionObjectType::USE_SHAPE_TYPE);

  cb::CollisionObjectWrapperPtr cow_2(new cb::CollisionObjectWrapper(
      "moving_box_link", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes, obj2_poses, obj2_types));
  checker.addCollisionObject(cow_2);
}

void addCollisionObjectsMesh(cb::BulletCastBVHManager& checker)
{
  ////////////////////////////
  // Add static box to checker
  ////////////////////////////
  shapes::ShapePtr static_box(new shapes::Box(0.3, 0.3, 0.3));
  Eigen::Isometry3d static_box_pose;
  static_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  cb::AlignedVector<Eigen::Isometry3d> obj1_poses;
  std::vector<cb::CollisionObjectType> obj1_types;
  obj1_shapes.push_back(static_box);
  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(cb::CollisionObjectType::USE_SHAPE_TYPE);

  cb::CollisionObjectWrapperPtr cow_1(new cb::CollisionObjectWrapper(
      "static_box_link", collision_detection::BodyType::WORLD_OBJECT, obj1_shapes, obj1_poses, obj1_types));
  checker.addCollisionObject(cow_1);
  ////////////////////////////
  // Add moving mesh to checker
  ////////////////////////////

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  cb::AlignedVector<Eigen::Isometry3d> obj2_poses;
  std::vector<cb::CollisionObjectType> obj2_types;

  obj1_poses.push_back(static_box_pose);
  obj1_types.push_back(cb::CollisionObjectType::USE_SHAPE_TYPE);

  Eigen::Isometry3d s_pose;
  s_pose.setIdentity();

  std::string kinect = "package://moveit_resources_panda_description/meshes/collision/hand.stl";
  auto s = std::shared_ptr<shapes::Shape>{ shapes::createMeshFromResource(kinect) };
  obj2_shapes.push_back(s);
  obj2_types.push_back(cb::CollisionObjectType::CONVEX_HULL);
  obj2_poses.push_back(s_pose);

  cb::CollisionObjectWrapperPtr cow_2(new cb::CollisionObjectWrapper(
      "moving_box_link", collision_detection::BodyType::WORLD_OBJECT, obj2_shapes, obj2_poses, obj2_types));
  checker.addCollisionObject(cow_2);
}

void runTest(cb::BulletCastBVHManager& checker, collision_detection::CollisionResult& result,
             std::vector<collision_detection::Contact>& result_vector, Eigen::Isometry3d& start_pos,
             Eigen::Isometry3d& end_pos)
{
  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.setActiveCollisionObjects({ "moving_box_link" });
  checker.setContactDistanceThreshold(0.1);

  // Set the collision object transforms
  checker.setCollisionObjectsTransform("static_box_link", Eigen::Isometry3d::Identity());
  checker.setCastCollisionObjectsTransform("moving_box_link", start_pos, end_pos);

  // Perform collision check
  collision_detection::CollisionRequest request;
  request.contacts = true;
  // cb::ContactResultMap result;
  checker.contactTest(result, request, nullptr, false);

  for (const auto& contacts_all : result.contacts)
  {
    for (const auto& contact : contacts_all.second)
    {
      result_vector.push_back(contact);
    }
  }
}

// TODO(j-petit): Add continuous to continuous collision checking
/** \brief Continuous self collision checks are not supported yet by the bullet integration */
TEST_F(BulletCollisionDetectionTester, DISABLED_ContinuousCollisionSelf)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  moveit::core::RobotState state1(robot_model_);
  moveit::core::RobotState state2(robot_model_);

  setToHome(state1);
  double joint2 = 0.15;
  double joint4 = -3.0;
  double joint5 = -0.8;
  double joint7 = -0.785;
  state1.setJointPositions("panda_joint2", &joint2);
  state1.setJointPositions("panda_joint4", &joint4);
  state1.setJointPositions("panda_joint5", &joint5);
  state1.setJointPositions("panda_joint7", &joint7);
  state1.update();

  cenv_->checkSelfCollision(req, res, state1, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  setToHome(state2);
  double joint_5_other = 0.8;
  state2.setJointPositions("panda_joint2", &joint2);
  state2.setJointPositions("panda_joint4", &joint4);
  state2.setJointPositions("panda_joint5", &joint_5_other);
  state2.setJointPositions("panda_joint7", &joint7);
  state2.update();

  cenv_->checkSelfCollision(req, res, state2, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  ROS_INFO_STREAM("Continous to continous collisions are not supported yet, therefore fail here.");
  ASSERT_TRUE(res.collision);
  res.clear();
}

/** \brief Two similar robot poses are used as start and end pose of a continuous collision check. */
TEST_F(BulletCollisionDetectionTester, ContinuousCollisionWorld)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 10;
  collision_detection::CollisionResult res;

  moveit::core::RobotState state1(robot_model_);
  moveit::core::RobotState state2(robot_model_);

  setToHome(state1);
  state1.update();

  setToHome(state2);
  double joint_2{ 0.05 };
  double joint_4{ -1.6 };
  state2.setJointPositions("panda_joint2", &joint_2);
  state2.setJointPositions("panda_joint4", &joint_4);
  state2.update();

  cenv_->checkRobotCollision(req, res, state1, state2, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  // Adding the box which is not in collision with the individual states but with the casted one.
  shapes::Shape* shape = new shapes::Box(0.1, 0.1, 0.1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0.43;
  pos.translation().y() = 0;
  pos.translation().z() = 0.55;
  cenv_->getWorld()->addToObject("box", shape_ptr, pos);

  cenv_->checkRobotCollision(req, res, state1, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  cenv_->checkRobotCollision(req, res, state2, *acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  cenv_->checkRobotCollision(req, res, state1, state2, *acm_);
  ASSERT_TRUE(res.collision);
  ASSERT_EQ(res.contact_count, 4u);
  // test contact types
  for (auto& contact_pair : res.contacts)
  {
    for (collision_detection::Contact& contact : contact_pair.second)
    {
      collision_detection::BodyType contact_type1 = contact.body_name_1 == "box" ?
                                                        collision_detection::BodyType::WORLD_OBJECT :
                                                        collision_detection::BodyType::ROBOT_LINK;
      collision_detection::BodyType contact_type2 = contact.body_name_2 == "box" ?
                                                        collision_detection::BodyType::WORLD_OBJECT :
                                                        collision_detection::BodyType::ROBOT_LINK;
      ASSERT_EQ(contact.body_type_1, contact_type1);
      ASSERT_EQ(contact.body_type_2, contact_type2);
    }
  }
  res.clear();
}

TEST(ContinuousCollisionUnit, BulletCastBVHCollisionBoxBoxUnit)
{
  collision_detection::CollisionResult result;
  std::vector<collision_detection::Contact> result_vector;

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation().x() = -2;
  end_pos.setIdentity();
  end_pos.translation().x() = 2;

  cb::BulletCastBVHManager checker;
  addCollisionObjects(checker);
  runTest(checker, result, result_vector, start_pos, end_pos);

  ASSERT_TRUE(result.collision);
  EXPECT_NEAR(result_vector[0].depth, -0.6, 0.001);
  EXPECT_NEAR(result_vector[0].percent_interpolation, 0.6, 0.001);
}

TEST(ContinuousCollisionUnit, BulletCastMeshVsBox)
{
  cb::BulletCastBVHManager checker;
  addCollisionObjectsMesh(checker);

  Eigen::Isometry3d start_pos, end_pos;
  start_pos.setIdentity();
  start_pos.translation().x() = -1.9;
  end_pos.setIdentity();
  end_pos.translation().x() = 1.9;

  collision_detection::CollisionResult result;
  std::vector<collision_detection::Contact> result_vector;

  runTest(checker, result, result_vector, start_pos, end_pos);

  ASSERT_TRUE(result.collision);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
