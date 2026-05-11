/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <gtest/gtest.h>

#include <map>
#include <math.h>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pilz_industrial_motion_planner/cartesian_trajectory.h"
#include "pilz_industrial_motion_planner/cartesian_trajectory_point.h"
#include "pilz_industrial_motion_planner/limits_container.h"
#include "pilz_industrial_motion_planner/trajectory_functions.h"
#include "test_utils.h"

#define _USE_MATH_DEFINES

static constexpr double EPSILON{ 1.0e-6 };
static constexpr double IK_SEED_OFFSET{ 0.1 };
static constexpr double L0{ 0.2604 };  // Height of foot
static constexpr double L1{ 0.3500 };  // Height of first connector
static constexpr double L2{ 0.3070 };  // Height of second connector
static constexpr double L3{ 0.0840 };  // Distance last joint to flange

const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

// parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string GROUP_TIP_LINK_NAME("group_tip_link");
const std::string ROBOT_TCP_LINK_NAME("tcp_link");
const std::string IK_FAST_LINK_NAME("ik_fast_link");
const std::string RANDOM_TEST_NUMBER("random_test_number");

/**
 * @brief test fixtures base class
 */
class TrajectoryFunctionsTestBase : public testing::TestWithParam<std::string>
{
protected:
  /**
   * @brief Create test scenario for trajectory functions
   *
   */
  void SetUp() override;

  /**
   * @brief check if two transformations are close
   * @param pose1
   * @param pose2
   * @param epsilon
   * @return
   */
  bool tfNear(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2, const double& epsilon);

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(GetParam()).getModel() };
  planning_scene::PlanningSceneConstPtr planning_scene_{ new planning_scene::PlanningScene(robot_model_) };

  // test parameters from parameter server
  std::string planning_group_, group_tip_link_, tcp_link_, ik_fast_link_;
  int random_test_number_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> zero_state_;

  // random seed
  boost::uint32_t random_seed_{ 100 };
  random_numbers::RandomNumberGenerator rng_{ random_seed_ };
};

void TrajectoryFunctionsTestBase::SetUp()
{
  // parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(GROUP_TIP_LINK_NAME, group_tip_link_));
  ASSERT_TRUE(ph_.getParam(ROBOT_TCP_LINK_NAME, tcp_link_));
  ASSERT_TRUE(ph_.getParam(IK_FAST_LINK_NAME, ik_fast_link_));
  ASSERT_TRUE(ph_.getParam(RANDOM_TEST_NUMBER, random_test_number_));

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, tcp_link_);

  // initialize the zero state configurationg and test joint state
  joint_names_ = robot_model_->getJointModelGroup(planning_group_)->getActiveJointModelNames();
  for (const auto& joint_name : joint_names_)
  {
    zero_state_[joint_name] = 0.0;
  }
}

bool TrajectoryFunctionsTestBase::tfNear(const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2,
                                         const double& epsilon)
{
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 4; ++j)
    {
      if (fabs(pose1(i, j) - pose2(i, j)) > fabs(epsilon))
        return false;
    }
  return true;
}

/**
 * @brief Parametrized class for tests with and without gripper.
 */
class TrajectoryFunctionsTestFlangeAndGripper : public TrajectoryFunctionsTestBase
{
};

/**
 * @brief Parametrized class for tests, that only run with a gripper
 */
class TrajectoryFunctionsTestOnlyGripper : public TrajectoryFunctionsTestBase
{
};

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_SUITE_P(InstantiationName, TrajectoryFunctionsTestFlangeAndGripper,
                         ::testing::Values(PARAM_MODEL_NO_GRIPPER_NAME, PARAM_MODEL_WITH_GRIPPER_NAME));

// Instantiate the test cases for robot model with a gripper
INSTANTIATE_TEST_SUITE_P(InstantiationName, TrajectoryFunctionsTestOnlyGripper,
                         ::testing::Values(PARAM_MODEL_WITH_GRIPPER_NAME));

/**
 * @brief Test the forward kinematics function with simple robot poses for robot
 * tip link
 * using robot model without gripper.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, TipLinkFK)
{
  Eigen::Isometry3d tip_pose;
  std::map<std::string, double> test_state = zero_state_;
  EXPECT_TRUE(pilz_industrial_motion_planner::computeLinkFK(robot_model_, group_tip_link_, test_state, tip_pose));
  EXPECT_NEAR(tip_pose(0, 3), 0, EPSILON);
  EXPECT_NEAR(tip_pose(1, 3), 0, EPSILON);
  EXPECT_NEAR(tip_pose(2, 3), L0 + L1 + L2 + L3, EPSILON);

  test_state[joint_names_.at(1)] = M_PI_2;
  EXPECT_TRUE(pilz_industrial_motion_planner::computeLinkFK(robot_model_, group_tip_link_, test_state, tip_pose));
  EXPECT_NEAR(tip_pose(0, 3), L1 + L2 + L3, EPSILON);
  EXPECT_NEAR(tip_pose(1, 3), 0, EPSILON);
  EXPECT_NEAR(tip_pose(2, 3), L0, EPSILON);

  test_state[joint_names_.at(1)] = -M_PI_2;
  test_state[joint_names_.at(2)] = M_PI_2;
  EXPECT_TRUE(pilz_industrial_motion_planner::computeLinkFK(robot_model_, group_tip_link_, test_state, tip_pose));
  EXPECT_NEAR(tip_pose(0, 3), -L1, EPSILON);
  EXPECT_NEAR(tip_pose(1, 3), 0, EPSILON);
  EXPECT_NEAR(tip_pose(2, 3), L0 - L2 - L3, EPSILON);

  // wrong link name
  std::string link_name = "wrong_link_name";
  EXPECT_FALSE(pilz_industrial_motion_planner::computeLinkFK(robot_model_, link_name, test_state, tip_pose));
}

/**
 * @brief Test the inverse kinematics directly through ikfast solver
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIKSolver)
{
  // Load solver
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group_);
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();

  // robot state
  robot_state::RobotState rstate(robot_model_);

  while (random_test_number_ > 0)
  {
    // sample random robot state
    rstate.setToRandomPositions(jmg, rng_);
    rstate.update();
    geometry_msgs::Pose pose_expect = tf2::toMsg(rstate.getFrameTransform(ik_fast_link_));

    // prepare inverse kinematics
    std::vector<geometry_msgs::Pose> ik_poses;
    ik_poses.push_back(pose_expect);
    std::vector<double> ik_seed, ik_expect, ik_actual;
    for (const auto& joint_name : jmg->getActiveJointModelNames())
    {
      ik_expect.push_back(rstate.getVariablePosition(joint_name));
      if (rstate.getVariablePosition(joint_name) > 0)
        ik_seed.push_back(rstate.getVariablePosition(joint_name) - IK_SEED_OFFSET);
      else
        ik_seed.push_back(rstate.getVariablePosition(joint_name) + IK_SEED_OFFSET);
    }

    std::vector<std::vector<double>> ik_solutions;
    kinematics::KinematicsResult ik_result;
    moveit_msgs::MoveItErrorCodes err_code;
    kinematics::KinematicsQueryOptions options = kinematics::KinematicsQueryOptions();

    // compute all ik solutions
    EXPECT_TRUE(solver->getPositionIK(ik_poses, ik_seed, ik_solutions, ik_result, options));

    // compute one ik solution
    EXPECT_TRUE(solver->getPositionIK(pose_expect, ik_seed, ik_actual, err_code));

    ASSERT_EQ(ik_expect.size(), ik_actual.size());

    for (std::size_t i = 0; i < ik_expect.size(); ++i)
    {
      EXPECT_NEAR(ik_actual.at(i), ik_expect.at(i), 4 * IK_SEED_OFFSET);
    }

    --random_test_number_;
  }
}

/**
 * @brief Test the inverse kinematics using RobotState class (setFromIK) using
 * robot model
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIKRobotState)
{
  // robot state
  robot_state::RobotState rstate(robot_model_);
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group_);

  while (random_test_number_ > 0)
  {
    // sample random robot state
    rstate.setToRandomPositions(jmg, rng_);

    Eigen::Isometry3d pose_expect = rstate.getFrameTransform(tcp_link_);

    // copy the random state and set ik seed
    std::map<std::string, double> ik_seed, ik_expect;
    for (const auto& joint_name : joint_names_)
    {
      ik_expect[joint_name] = rstate.getVariablePosition(joint_name);
      if (rstate.getVariablePosition(joint_name) > 0)
        ik_seed[joint_name] = rstate.getVariablePosition(joint_name) - IK_SEED_OFFSET;
      else
        ik_seed[joint_name] = rstate.getVariablePosition(joint_name) + IK_SEED_OFFSET;
    }

    rstate.setVariablePositions(ik_seed);
    rstate.update();

    // compute the ik
    std::map<std::string, double> ik_actual;

    EXPECT_TRUE(rstate.setFromIK(robot_model_->getJointModelGroup(planning_group_), pose_expect, tcp_link_));

    for (const auto& joint_name : joint_names_)
    {
      ik_actual[joint_name] = rstate.getVariablePosition(joint_name);
    }

    // compare ik solution and expected value
    for (const auto& joint_pair : ik_actual)
    {
      EXPECT_NEAR(joint_pair.second, ik_expect.at(joint_pair.first), 4 * IK_SEED_OFFSET);
    }

    // compute the pose from ik_solution
    rstate.setVariablePositions(ik_actual);
    rstate.update();
    Eigen::Isometry3d pose_actual = rstate.getFrameTransform(tcp_link_);

    EXPECT_TRUE(tfNear(pose_expect, pose_actual, EPSILON));

    --random_test_number_;
  }
}

/**
 * @brief Test the wrapper function to compute inverse kinematics using robot
 * model
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testComputePoseIK)
{
  // robot state
  robot_state::RobotState rstate(robot_model_);

  const std::string frame_id = robot_model_->getModelFrame();
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group_);

  while (random_test_number_ > 0)
  {
    // sample random robot state
    rstate.setToRandomPositions(jmg, rng_);

    Eigen::Isometry3d pose_expect = rstate.getFrameTransform(tcp_link_);

    // copy the random state and set ik seed
    std::map<std::string, double> ik_seed, ik_expect;
    for (const auto& joint_name : robot_model_->getJointModelGroup(planning_group_)->getActiveJointModelNames())
    {
      ik_expect[joint_name] = rstate.getVariablePosition(joint_name);
      if (rstate.getVariablePosition(joint_name) > 0)
      {
        ik_seed[joint_name] = rstate.getVariablePosition(joint_name) - IK_SEED_OFFSET;
      }
      else
      {
        ik_seed[joint_name] = rstate.getVariablePosition(joint_name) + IK_SEED_OFFSET;
      }
    }

    // compute the ik
    std::map<std::string, double> ik_actual;
    EXPECT_TRUE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, tcp_link_, pose_expect,
                                                              frame_id, ik_seed, ik_actual, false));

    // compare ik solution and expected value
    for (const auto& joint_pair : ik_actual)
    {
      EXPECT_NEAR(joint_pair.second, ik_expect.at(joint_pair.first), 4 * IK_SEED_OFFSET);
    }

    --random_test_number_;
  }
}

/**
 * @brief Test computePoseIK for invalid group_name
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testComputePoseIKInvalidGroupName)
{
  const std::string frame_id = robot_model_->getModelFrame();
  Eigen::Isometry3d pose_expect;

  std::map<std::string, double> ik_seed;

  // compute the ik
  std::map<std::string, double> ik_actual;
  EXPECT_FALSE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, "InvalidGroupName", tcp_link_,
                                                             pose_expect, frame_id, ik_seed, ik_actual, false));
}

/**
 * @brief Test computePoseIK for invalid link_name
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testComputePoseIKInvalidLinkName)
{
  const std::string frame_id = robot_model_->getModelFrame();
  Eigen::Isometry3d pose_expect;

  std::map<std::string, double> ik_seed;

  // compute the ik
  std::map<std::string, double> ik_actual;
  EXPECT_FALSE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, "WrongLink", pose_expect,
                                                             frame_id, ik_seed, ik_actual, false));
}

/**
 * @brief Test computePoseIK for invalid frame_id
 *
 * Currently only robot_model_->getModelFrame() == frame_id
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testComputePoseIKInvalidFrameId)
{
  Eigen::Isometry3d pose_expect;

  std::map<std::string, double> ik_seed;

  // compute the ik
  std::map<std::string, double> ik_actual;
  EXPECT_FALSE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, tcp_link_, pose_expect,
                                                             "InvalidFrameId", ik_seed, ik_actual, false));
}

/**
 * @brief Test if activated self collision for a pose that would be in self
 * collision without the check results in a
 * valid ik solution.
 */
TEST_P(TrajectoryFunctionsTestOnlyGripper, testComputePoseIKSelfCollisionForValidPosition)
{
  const std::string frame_id = robot_model_->getModelFrame();
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group_);

  // create seed
  std::vector<double> ik_seed_states = { -0.553, 0.956, 1.758, 0.146, -1.059, 1.247 };
  auto joint_names = jmg->getActiveJointModelNames();

  std::map<std::string, double> ik_seed;
  for (unsigned int i = 0; i < ik_seed_states.size(); ++i)
  {
    ik_seed[joint_names[i]] = ik_seed_states[i];
  }

  // create expected pose
  geometry_msgs::Pose pose;
  pose.position.x = -0.454;
  pose.position.y = -0.15;
  pose.position.z = 0.431;
  pose.orientation.y = 0.991562;
  pose.orientation.w = -0.1296328;
  Eigen::Isometry3d pose_expect;
  normalizeQuaternion(pose.orientation);
  tf2::fromMsg(pose, pose_expect);

  // compute the ik without self collision check and expect the resulting pose
  // to be in self collission.
  std::map<std::string, double> ik_actual1;
  EXPECT_TRUE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, tcp_link_, pose_expect,
                                                            frame_id, ik_seed, ik_actual1, false));

  robot_state::RobotState rstate(robot_model_);
  planning_scene::PlanningScene rscene(robot_model_);

  std::vector<double> ik_state;
  std::transform(ik_actual1.begin(), ik_actual1.end(), std::back_inserter(ik_state),
                 [](const auto& pair) { return pair.second; });

  rstate.setJointGroupPositions(jmg, ik_state);
  rstate.update();

  collision_detection::CollisionRequest collision_req;
  collision_req.group_name = jmg->getName();
  collision_detection::CollisionResult collision_res;

  rscene.checkSelfCollision(collision_req, collision_res, rstate);

  EXPECT_TRUE(collision_res.collision);

  // compute the ik with collision detection activated and expect the resulting
  // pose to be without self collision.
  std::map<std::string, double> ik_actual2;
  EXPECT_TRUE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, tcp_link_, pose_expect,
                                                            frame_id, ik_seed, ik_actual2, true));

  std::vector<double> ik_state2;
  std::transform(ik_actual2.begin(), ik_actual2.end(), std::back_inserter(ik_state2),
                 [](const auto& pair) { return pair.second; });
  rstate.setJointGroupPositions(jmg, ik_state2);
  rstate.update();

  collision_detection::CollisionResult collision_res2;
  rscene.checkSelfCollision(collision_req, collision_res2, rstate);

  EXPECT_FALSE(collision_res2.collision);
}

/**
 * @brief Test if self collision is considered by using a pose that always has
 * self collision.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testComputePoseIKSelfCollisionForInvalidPose)
{
  // robot state
  robot_state::RobotState rstate(robot_model_);

  const std::string frame_id = robot_model_->getModelFrame();
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group_);

  // create seed
  std::map<std::string, double> ik_seed;
  for (const auto& joint_name : jmg->getActiveJointModelNames())
  {
    ik_seed[joint_name] = 0;
  }

  // create goal
  std::vector<double> ik_goal = { 0, 2.3, -2.3, 0, 0, 0 };

  rstate.setJointGroupPositions(jmg, ik_goal);

  Eigen::Isometry3d pose_expect = rstate.getFrameTransform(tcp_link_);

  // compute the ik with disabled collision check
  std::map<std::string, double> ik_actual;
  EXPECT_TRUE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, tcp_link_, pose_expect,
                                                            frame_id, ik_seed, ik_actual, false));

  // compute the ik with enabled collision check
  EXPECT_FALSE(pilz_industrial_motion_planner::computePoseIK(planning_scene_, planning_group_, tcp_link_, pose_expect,
                                                             frame_id, ik_seed, ik_actual, true));
}

/**
 * @brief Check that function VerifySampleJointLimits() returns 'false' in case
 * of very small sample duration.
 *
 * Test Sequence:
 *    1. Call function with very small sample duration.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testVerifySampleJointLimitsWithSmallDuration)
{
  const std::map<std::string, double> position_last, velocity_last, position_current;
  double duration_last{ 0.0 };
  const pilz_industrial_motion_planner::JointLimitsContainer joint_limits;

  double duration_current = 10e-7;

  EXPECT_FALSE(pilz_industrial_motion_planner::verifySampleJointLimits(position_last, velocity_last, position_current,
                                                                       duration_last, duration_current, joint_limits));
}

/**
 * @brief Check that function VerifySampleJointLimits() returns 'false' in case
 * of a velocity violation.
 *
 * Test Sequence:
 *    1. Call function with a velocity violation.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testVerifySampleJointLimitsVelocityViolation)
{
  const std::string test_joint_name{ "joint" };

  std::map<std::string, double> position_last{ { test_joint_name, 2.0 } };
  std::map<std::string, double> position_current{ { test_joint_name, 10.0 } };
  std::map<std::string, double> velocity_last;
  double duration_current{ 1.0 };
  double duration_last{ 0.0 };
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits;

  JointLimit test_joint_limits;
  // Calculate the max allowed velocity in such a way that it is always smaller
  // than the current velocity.
  test_joint_limits.max_velocity =
      ((position_current.at(test_joint_name) - position_last.at(test_joint_name)) / duration_current) - 1.0;
  test_joint_limits.has_velocity_limits = true;
  joint_limits.addLimit(test_joint_name, test_joint_limits);

  EXPECT_FALSE(pilz_industrial_motion_planner::verifySampleJointLimits(position_last, velocity_last, position_current,
                                                                       duration_last, duration_current, joint_limits));
}

/**
 * @brief Check that function VerifySampleJointLimits() returns 'false' in case
 * of a acceleration violation.
 *
 * Test Sequence:
 *    1. Call function with a acceleration violation.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testVerifySampleJointLimitsAccelerationViolation)
{
  const std::string test_joint_name{ "joint" };

  double duration_current = 1.0;
  double duration_last = 1.0;

  std::map<std::string, double> position_last{ { test_joint_name, 2.0 } };
  std::map<std::string, double> position_current{ { test_joint_name, 20.0 } };
  double velocity_current =
      ((position_current.at(test_joint_name) - position_last.at(test_joint_name)) / duration_current);
  std::map<std::string, double> velocity_last{ { test_joint_name, 9.0 } };
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits;

  JointLimit test_joint_limits;
  // Calculate the max allowed velocity in such a way that it is always bigger
  // than the current velocity.
  test_joint_limits.max_velocity = velocity_current + 1.0;
  test_joint_limits.has_velocity_limits = true;

  double acceleration_current =
      (velocity_current - velocity_last.at(test_joint_name)) / (duration_last + duration_current) * 2;
  // Calculate the max allowed acceleration in such a way that it is always
  // smaller than the current acceleration.
  test_joint_limits.max_acceleration = acceleration_current - 1.0;
  test_joint_limits.has_acceleration_limits = true;

  joint_limits.addLimit(test_joint_name, test_joint_limits);

  EXPECT_FALSE(pilz_industrial_motion_planner::verifySampleJointLimits(position_last, velocity_last, position_current,
                                                                       duration_last, duration_current, joint_limits));
}

/**
 * @brief Check that function VerifySampleJointLimits() returns 'false' in case
 * of a deceleration violation.
 *
 * Test Sequence:
 *    1. Call function with a deceleration violation.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testVerifySampleJointLimitsDecelerationViolation)
{
  const std::string test_joint_name{ "joint" };

  double duration_current = 1.0;
  double duration_last = 1.0;

  std::map<std::string, double> position_last{ { test_joint_name, 20.0 } };
  std::map<std::string, double> position_current{ { test_joint_name, 2.0 } };
  double velocity_current =
      ((position_current.at(test_joint_name) - position_last.at(test_joint_name)) / duration_current);
  std::map<std::string, double> velocity_last{ { test_joint_name, 19.0 } };
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits;

  JointLimit test_joint_limits;
  // Calculate the max allowed velocity in such a way that it is always bigger
  // than the current velocity.
  test_joint_limits.max_velocity = fabs(velocity_current) + 1.0;
  test_joint_limits.has_velocity_limits = true;

  double acceleration_current =
      (velocity_current - velocity_last.at(test_joint_name)) / (duration_last + duration_current) * 2;
  // Calculate the max allowed deceleration in such a way that it is always
  // bigger than the current acceleration.
  test_joint_limits.max_deceleration = acceleration_current + 1.0;
  test_joint_limits.has_deceleration_limits = true;

  joint_limits.addLimit(test_joint_name, test_joint_limits);

  EXPECT_FALSE(pilz_industrial_motion_planner::verifySampleJointLimits(position_last, velocity_last, position_current,
                                                                       duration_last, duration_current, joint_limits));
}

/**
 * @brief Check that function generateJointTrajectory() returns 'false' if
 * a joint trajectory cannot be computed from a cartesian trajectory.
 *
 * Please note: Both function variants are tested in this test.
 *
 * Test Sequence:
 *    1. Call function with a cartesian trajectory which cannot be transformed
 * into a joint trajectory by using
 *        an invalid group_name.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testGenerateJointTrajectoryWithInvalidCartesianTrajectory)
{
  // Create random test trajectory
  // Note: 'path' is deleted by KDL::Trajectory_Segment
  KDL::Path_RoundedComposite* path =
      new KDL::Path_RoundedComposite(0.2, 0.01, new KDL::RotationalInterpolation_SingleAxis());
  path->Add(KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(-1, 0, 0)));
  path->Finish();
  // Note: 'velprof' is deleted by KDL::Trajectory_Segment
  KDL::VelocityProfile* vel_prof = new KDL::VelocityProfile_Trap(0.5, 0.1);
  vel_prof->SetProfile(0, path->PathLength());
  KDL::Trajectory_Segment kdl_trajectory(path, vel_prof);

  pilz_industrial_motion_planner::JointLimitsContainer joint_limits;
  std::string group_name{ "invalid_group_name" };
  std::map<std::string, double> initial_joint_position;
  double sampling_time{ 0.1 };
  trajectory_msgs::JointTrajectory joint_trajectory;
  moveit_msgs::MoveItErrorCodes error_code;
  bool check_self_collision{ false };

  EXPECT_FALSE(pilz_industrial_motion_planner::generateJointTrajectory(
      planning_scene_, joint_limits, kdl_trajectory, group_name, tcp_link_, initial_joint_position, sampling_time,
      joint_trajectory, error_code, check_self_collision));

  std::map<std::string, double> initial_joint_velocity;

  pilz_industrial_motion_planner::CartesianTrajectory cart_traj;
  cart_traj.group_name = group_name;
  cart_traj.link_name = tcp_link_;
  pilz_industrial_motion_planner::CartesianTrajectoryPoint cart_traj_point;
  cart_traj.points.push_back(cart_traj_point);

  EXPECT_FALSE(pilz_industrial_motion_planner::generateJointTrajectory(
      planning_scene_, joint_limits, cart_traj, group_name, tcp_link_, initial_joint_position, initial_joint_velocity,
      joint_trajectory, error_code, check_self_collision));
}

/**
 * @brief Check that function determineAndCheckSamplingTime() returns 'false' if
 * both of the needed vectors have an incorrect vector size.
 *
 *
 * Test Sequence:
 *    1. Call function with vectors of incorrect size.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testDetermineAndCheckSamplingTimeInvalidVectorSize)
{
  robot_trajectory::RobotTrajectoryPtr first_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
  robot_trajectory::RobotTrajectoryPtr second_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
  double epsilon{ 0.0 };
  double sampling_time{ 0.0 };

  robot_state::RobotState rstate(robot_model_);
  first_trajectory->insertWayPoint(0, rstate, 0.1);
  second_trajectory->insertWayPoint(0, rstate, 0.1);

  EXPECT_FALSE(pilz_industrial_motion_planner::determineAndCheckSamplingTime(first_trajectory, second_trajectory,
                                                                             epsilon, sampling_time));
}

/**
 * @brief Check that function determineAndCheckSamplingTime() returns 'true' if
 * sampling time is correct.
 *
 *
 * Test Sequence:
 *    1. Call function with trajectories which do NOT violate sampling time.
 *
 * Expected Results:
 *    1. Function returns 'true'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testDetermineAndCheckSamplingTimeCorrectSamplingTime)
{
  robot_trajectory::RobotTrajectoryPtr first_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
  robot_trajectory::RobotTrajectoryPtr second_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
  double epsilon{ 0.0001 };
  double sampling_time{ 0.0 };
  double expected_sampling_time{ 0.1 };

  robot_state::RobotState rstate(robot_model_);
  first_trajectory->insertWayPoint(0, rstate, expected_sampling_time);
  first_trajectory->insertWayPoint(1, rstate, expected_sampling_time);

  second_trajectory->insertWayPoint(0, rstate, expected_sampling_time);
  second_trajectory->insertWayPoint(1, rstate, expected_sampling_time);
  second_trajectory->insertWayPoint(2, rstate, expected_sampling_time);

  EXPECT_TRUE(pilz_industrial_motion_planner::determineAndCheckSamplingTime(first_trajectory, second_trajectory,
                                                                            epsilon, sampling_time));
  EXPECT_EQ(expected_sampling_time, sampling_time);
}

/**
 * @brief Check that function determineAndCheckSamplingTime() returns 'false' if
 * sampling time is violated.
 *
 *
 * Test Sequence:
 *    1. Call function with trajectories which violate sampling time.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testDetermineAndCheckSamplingTimeViolateSamplingTime)
{
  robot_trajectory::RobotTrajectoryPtr first_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
  robot_trajectory::RobotTrajectoryPtr second_trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
  double epsilon{ 0.0001 };
  double sampling_time{ 0.0 };
  double expected_sampling_time{ 0.1 };

  robot_state::RobotState rstate(robot_model_);
  first_trajectory->insertWayPoint(0, rstate, expected_sampling_time);
  first_trajectory->insertWayPoint(1, rstate, expected_sampling_time);
  first_trajectory->insertWayPoint(2, rstate, expected_sampling_time);
  // Violate sampling time
  first_trajectory->insertWayPoint(2, rstate, expected_sampling_time + 1.0);
  first_trajectory->insertWayPoint(3, rstate, expected_sampling_time);

  second_trajectory->insertWayPoint(0, rstate, expected_sampling_time);
  second_trajectory->insertWayPoint(1, rstate, expected_sampling_time);
  second_trajectory->insertWayPoint(2, rstate, expected_sampling_time);
  second_trajectory->insertWayPoint(3, rstate, expected_sampling_time);

  EXPECT_FALSE(pilz_industrial_motion_planner::determineAndCheckSamplingTime(first_trajectory, second_trajectory,
                                                                             epsilon, sampling_time));
  EXPECT_EQ(expected_sampling_time, sampling_time);
}

/**
 * @brief Check that function isRobotStateEqual() returns 'false' if
 * the positions of the robot states are not equal.
 *
 *
 * Test Sequence:
 *    1. Call function with robot states with different positions.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIsRobotStateEqualPositionUnequal)
{
  robot_state::RobotState rstate_1 = robot_state::RobotState(robot_model_);
  robot_state::RobotState rstate_2 = robot_state::RobotState(robot_model_);

  double default_joint_position[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupPositions(planning_group_, default_joint_position);
  // Ensure that the joint positions of both robot states are different
  default_joint_position[0] = default_joint_position[0] + 70.0;
  rstate_2.setJointGroupPositions(planning_group_, default_joint_position);

  double epsilon{ 0.0001 };
  EXPECT_FALSE(pilz_industrial_motion_planner::isRobotStateEqual(rstate_1, rstate_2, planning_group_, epsilon));
}

/**
 * @brief Check that function isRobotStateEqual() returns 'false' if
 * the velocity of the robot states are not equal.
 *
 *
 * Test Sequence:
 *    1. Call function with robot states with different velocities.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIsRobotStateEqualVelocityUnequal)
{
  robot_state::RobotState rstate_1 = robot_state::RobotState(robot_model_);
  robot_state::RobotState rstate_2 = robot_state::RobotState(robot_model_);

  // Ensure that the joint positions of both robot state are equal
  double default_joint_position[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupPositions(planning_group_, default_joint_position);
  rstate_2.setJointGroupPositions(planning_group_, default_joint_position);

  double default_joint_velocity[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupVelocities(planning_group_, default_joint_velocity);
  // Ensure that the joint velocites of both robot states are different
  default_joint_velocity[1] = default_joint_velocity[1] + 10.0;
  rstate_2.setJointGroupVelocities(planning_group_, default_joint_velocity);

  double epsilon{ 0.0001 };
  EXPECT_FALSE(pilz_industrial_motion_planner::isRobotStateEqual(rstate_1, rstate_2, planning_group_, epsilon));
}

/**
 * @brief Check that function isRobotStateEqual() returns 'false' if
 * the acceleration of the robot states are not equal.
 *
 *
 * Test Sequence:
 *    1. Call function with robot states with different acceleration.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIsRobotStateEqualAccelerationUnequal)
{
  robot_state::RobotState rstate_1 = robot_state::RobotState(robot_model_);
  robot_state::RobotState rstate_2 = robot_state::RobotState(robot_model_);

  // Ensure that the joint positions of both robot state are equal
  double default_joint_position[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupPositions(planning_group_, default_joint_position);
  rstate_2.setJointGroupPositions(planning_group_, default_joint_position);

  // Ensure that the joint velocities of both robot state are equal
  double default_joint_velocity[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupVelocities(planning_group_, default_joint_velocity);
  rstate_2.setJointGroupVelocities(planning_group_, default_joint_velocity);

  double default_joint_acceleration[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupAccelerations(planning_group_, default_joint_acceleration);
  // Ensure that the joint accelerations of both robot states are different
  default_joint_acceleration[1] = default_joint_acceleration[1] + 10.0;
  rstate_2.setJointGroupAccelerations(planning_group_, default_joint_acceleration);

  double epsilon{ 0.0001 };
  EXPECT_FALSE(pilz_industrial_motion_planner::isRobotStateEqual(rstate_1, rstate_2, planning_group_, epsilon));
}

/**
 * @brief Check that function isRobotStateStationary() returns 'false' if
 * the joint velocities are not equal to zero.
 *
 *
 * Test Sequence:
 *    1. Call function with robot state with joint velocities != 0.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIsRobotStateStationaryVelocityUnequal)
{
  robot_state::RobotState rstate_1 = robot_state::RobotState(robot_model_);

  // Ensure that the joint velocities are NOT zero
  double default_joint_velocity[6] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupVelocities(planning_group_, default_joint_velocity);

  double epsilon{ 0.0001 };
  EXPECT_FALSE(pilz_industrial_motion_planner::isRobotStateStationary(rstate_1, planning_group_, epsilon));
}

/**
 * @brief Check that function isRobotStateStationary() returns 'false' if
 * the joint acceleration are not equal to zero.
 *
 *
 * Test Sequence:
 *    1. Call function with robot state with joint acceleration != 0.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryFunctionsTestFlangeAndGripper, testIsRobotStateStationaryAccelerationUnequal)
{
  robot_state::RobotState rstate_1 = robot_state::RobotState(robot_model_);

  // Ensure that the joint velocities are zero
  double default_joint_velocity[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupVelocities(planning_group_, default_joint_velocity);

  // Ensure that the joint acceleration are NOT zero
  double default_joint_acceleration[6] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  rstate_1.setJointGroupAccelerations(planning_group_, default_joint_acceleration);

  double epsilon{ 0.0001 };
  EXPECT_FALSE(pilz_industrial_motion_planner::isRobotStateStationary(rstate_1, planning_group_, epsilon));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_trajectory_functions");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
