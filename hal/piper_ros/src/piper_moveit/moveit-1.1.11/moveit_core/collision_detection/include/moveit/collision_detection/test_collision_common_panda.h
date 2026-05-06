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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection/collision_detector_allocator.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

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

template <class CollisionAllocatorType>
class CollisionDetectorPandaTest : public testing::Test
{
public:
  std::shared_ptr<CollisionAllocatorType> value_;

protected:
  void SetUp() override
  {
    value_ = std::make_shared<CollisionAllocatorType>();
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    robot_model_ok_ = static_cast<bool>(robot_model_);

    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(*robot_model_->getSRDF());

    cenv_ = value_->allocateEnv(robot_model_);

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    setToHome(*robot_state_);
  }

  void TearDown() override
  {
  }

  bool robot_model_ok_;

  moveit::core::RobotModelPtr robot_model_;

  collision_detection::CollisionEnvPtr cenv_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  moveit::core::RobotStatePtr robot_state_;
};

TYPED_TEST_CASE_P(CollisionDetectorPandaTest);

/** \brief Correct setup testing. */
TYPED_TEST_P(CollisionDetectorPandaTest, InitOK)
{
  ASSERT_TRUE(this->robot_model_ok_);
}

/** \brief Tests the default values specified in the SRDF if they are collision free. */
TYPED_TEST_P(CollisionDetectorPandaTest, DefaultNotInCollision)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  this->cenv_->checkSelfCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_FALSE(res.collision);
}

/** \brief A configuration where the robot should collide with itself. */
TYPED_TEST_P(CollisionDetectorPandaTest, LinksInCollision)
{
  // Sets the joints into a colliding configuration
  double joint2 = 0.15;
  double joint4 = -3.0;
  this->robot_state_->setJointPositions("panda_joint2", &joint2);
  this->robot_state_->setJointPositions("panda_joint4", &joint4);
  this->robot_state_->update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  this->cenv_->checkSelfCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_TRUE(res.collision);
}

/** \brief Adding obstacles to the world which are tested against the robot. Simple cases. */
TYPED_TEST_P(CollisionDetectorPandaTest, RobotWorldCollision_1)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  shapes::Shape* shape = new shapes::Box(.1, .1, .1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos1 = Eigen::Isometry3d::Identity();
  pos1.translation().z() = 0.3;
  this->cenv_->getWorld()->addToObject("box", pos1, shape_ptr, Eigen::Isometry3d::Identity());

  this->cenv_->checkSelfCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_TRUE(res.collision);
  res.clear();

  this->cenv_->getWorld()->moveObject("box", pos1);
  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_TRUE(res.collision);
  res.clear();

  this->cenv_->getWorld()->moveObject("box", pos1);
  ASSERT_FALSE(res.collision);
}

/** \brief Adding obstacles to the world which are tested against the robot. */
TYPED_TEST_P(CollisionDetectorPandaTest, RobotWorldCollision_2)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.max_contacts = 10;
  req.contacts = true;
  req.verbose = true;

  shapes::Shape* shape = new shapes::Box(.4, .4, .4);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos1 = Eigen::Isometry3d::Identity();
  pos1.translation().z() = 0.3;
  this->cenv_->getWorld()->addToObject("box", pos1, shape_ptr, Eigen::Isometry3d::Identity());

  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_TRUE(res.collision);
  ASSERT_GE(res.contact_count, 3u);
  res.clear();
}

/** \brief Tests the padding through expanding the link geometry in such a way that a collision occurs. */
TYPED_TEST_P(CollisionDetectorPandaTest, PaddingTest)
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 10;
  collision_detection::CollisionResult res;

  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_FALSE(res.collision);
  res.clear();

  // Adding the box right in front of the robot hand
  shapes::Shape* shape = new shapes::Box(0.1, 0.1, 0.1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0.43;
  pos.translation().y() = 0;
  pos.translation().z() = 0.55;
  this->cenv_->getWorld()->addToObject("box", pos, shape_ptr, Eigen::Isometry3d::Identity());

  this->cenv_->setLinkPadding("panda_hand", 0.08);
  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_TRUE(res.collision);
  res.clear();

  this->cenv_->setLinkPadding("panda_hand", 0.0);
  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_FALSE(res.collision);
}

/** \brief Tests the distance reporting with the robot itself */
TYPED_TEST_P(CollisionDetectorPandaTest, DistanceSelf)
{
  collision_detection::CollisionRequest req;
  req.distance = true;
  collision_detection::CollisionResult res;
  this->cenv_->checkSelfCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_FALSE(res.collision);
  EXPECT_NEAR(res.distance, 0.022, 0.001);
}

TYPED_TEST_P(CollisionDetectorPandaTest, DistanceWorld)
{
  collision_detection::CollisionRequest req;
  req.distance = true;
  collision_detection::CollisionResult res;

  // Adding the box right in front of the robot hand
  shapes::Shape* shape = new shapes::Box(0.1, 0.1, 0.1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0.43;
  pos.translation().y() = 0;
  pos.translation().z() = 0.55;
  this->cenv_->getWorld()->addToObject("box", pos, shape_ptr, Eigen::Isometry3d::Identity());

  this->cenv_->setLinkPadding("panda_hand", 0.0);
  this->cenv_->checkRobotCollision(req, res, *this->robot_state_, *this->acm_);
  ASSERT_FALSE(res.collision);
  EXPECT_NEAR(res.distance, 0.029, 0.01);
}

template <class CollisionAllocatorType>
class DistanceCheckPandaTest : public CollisionDetectorPandaTest<CollisionAllocatorType>
{
};

TYPED_TEST_CASE_P(DistanceCheckPandaTest);

TYPED_TEST_P(DistanceCheckPandaTest, DistanceSingle)
{
  std::set<const moveit::core::LinkModel*> active_components{ this->robot_model_->getLinkModel("panda_hand") };
  collision_detection::DistanceRequest req;
  req.type = collision_detection::DistanceRequestTypes::SINGLE;
  req.active_components_only = &active_components;
  req.enable_signed_distance = true;

  random_numbers::RandomNumberGenerator rng(0x47110815);
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < 10; ++i)
  {
    collision_detection::DistanceResult res;

    shapes::ShapeConstPtr shape(new shapes::Cylinder(rng.uniform01(), rng.uniform01()));
    Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
    pose.translation() =
        Eigen::Vector3d(rng.uniformReal(0.1, 2.0), rng.uniformReal(0.1, 2.0), rng.uniformReal(1.2, 1.7));
    double quat[4];
    rng.quaternion(quat);
    pose.linear() = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();

    this->cenv_->getWorld()->addToObject("collection", Eigen::Isometry3d::Identity(), shape, pose);
    this->cenv_->getWorld()->removeObject("object");
    this->cenv_->getWorld()->addToObject("object", pose, shape, Eigen::Isometry3d::Identity());

    this->cenv_->distanceRobot(req, res, *this->robot_state_);
    auto& distances1 = res.distances[std::pair<std::string, std::string>("collection", "panda_hand")];
    auto& distances2 = res.distances[std::pair<std::string, std::string>("object", "panda_hand")];
    ASSERT_EQ(distances1.size(), 1u) << "no distance reported for collection/panda_hand";
    ASSERT_EQ(distances2.size(), 1u) << "no distance reported for object/panda_hand";

    double collection_distance = distances1[0].distance;
    min_distance = std::min(min_distance, distances2[0].distance);
    ASSERT_NEAR(collection_distance, min_distance, 1e-5)
        << "distance to collection is greater than distance to minimum of individual objects after " << i << " rounds";
  }
}

// FCL < 0.6 completely fails the DistancePoints test, so we have two test
// suites, one with and one without the test.
template <class CollisionAllocatorType>
class DistanceFullPandaTest : public DistanceCheckPandaTest<CollisionAllocatorType>
{
};

TYPED_TEST_CASE_P(DistanceFullPandaTest);

/** \brief Tests if nearest points are computed correctly. */
TYPED_TEST_P(DistanceFullPandaTest, DistancePoints)
{
  // Adding the box right in front of the robot hand
  shapes::Box* shape = new shapes::Box(0.1, 0.1, 0.1);
  shapes::ShapeConstPtr shape_ptr(shape);

  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };
  pos.translation().x() = 0.43;
  pos.translation().y() = 0;
  pos.translation().z() = 0.55;
  this->cenv_->getWorld()->addToObject("box", shape_ptr, pos);

  collision_detection::DistanceRequest req;
  req.acm = &*this->acm_;
  req.type = collision_detection::DistanceRequestTypes::SINGLE;
  req.enable_nearest_points = true;
  req.max_contacts_per_body = 1;

  collision_detection::DistanceResult res;
  this->cenv_->distanceRobot(req, res, *this->robot_state_);

  // Checks a particular point is inside the box
  auto check_in_box = [&](const Eigen::Vector3d& p) {
    Eigen::Vector3d in_box = pos.inverse() * p;

    constexpr double eps = 1e-5;
    EXPECT_LE(std::abs(in_box.x()), shape->size[0] + eps);
    EXPECT_LE(std::abs(in_box.y()), shape->size[1] + eps);
    EXPECT_LE(std::abs(in_box.z()), shape->size[2] + eps);
  };

  // Check that all points reported on "box" are actually on the box and not
  // on the robot
  for (auto& distance : res.distances)
  {
    for (auto& pair : distance.second)
    {
      if (pair.link_names[0] == "box")
        check_in_box(pair.nearest_points[0]);
      else if (pair.link_names[1] == "box")
        check_in_box(pair.nearest_points[1]);
      else
        ADD_FAILURE() << "Unrecognized link names";
    }
  }
}

REGISTER_TYPED_TEST_CASE_P(CollisionDetectorPandaTest, InitOK, DefaultNotInCollision, LinksInCollision,
                           RobotWorldCollision_1, RobotWorldCollision_2, PaddingTest, DistanceSelf, DistanceWorld);

REGISTER_TYPED_TEST_CASE_P(DistanceCheckPandaTest, DistanceSingle);

REGISTER_TYPED_TEST_CASE_P(DistanceFullPandaTest, DistancePoints);
