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

#include <memory>

#include <gtest/gtest.h>

#include "pilz_industrial_motion_planner/joint_limits_aggregator.h"
#include "pilz_industrial_motion_planner/trajectory_generator_ptp.h"
#include "test_utils.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <pluginlib/class_loader.h>

// parameters for parameterized tests
const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

// parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");
const std::string JOINT_VELOCITY_TOLERANCE("joint_velocity_tolerance");
const std::string JOINT_ACCELERATION_TOLERANCE("joint_acceleration_tolerance");
const std::string POSE_TRANSFORM_MATRIX_NORM_TOLERANCE("pose_norm_tolerance");

using namespace pilz_industrial_motion_planner;

class TrajectoryGeneratorPTPTest : public testing::TestWithParam<std::string>
{
protected:
  /**
   * @brief Create test fixture for ptp trajectory generator
   *
   */
  void SetUp() override;

  /**
   * @brief check the resulted joint trajectory
   * @param trajectory
   * @param req
   * @param joint_limits
   * @return
   */
  bool checkTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                       const planning_interface::MotionPlanRequest& req, const JointLimitsContainer& joint_limits);

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(GetParam()).getModel() };
  planning_scene::PlanningSceneConstPtr planning_scene_{ new planning_scene::PlanningScene(robot_model_) };

  // trajectory generator
  std::unique_ptr<TrajectoryGenerator> ptp_;

  // test parameters from parameter server
  LimitsContainer planner_limits_;
  std::string planning_group_, target_link_;
  double joint_position_tolerance_, joint_velocity_tolerance_, joint_acceleration_tolerance_, pose_norm_tolerance_;
};

void TrajectoryGeneratorPTPTest::SetUp()
{
  // get parameters from parameter server
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph_.getParam(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_VELOCITY_TOLERANCE, joint_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_ACCELERATION_TOLERANCE, joint_acceleration_tolerance_));
  ASSERT_TRUE(ph_.getParam(POSE_TRANSFORM_MATRIX_NORM_TOLERANCE, pose_norm_tolerance_));

  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // create the limits container
  JointLimitsContainer joint_limits;
  for (const auto& jmg : robot_model_->getJointModelGroups())
  {
    std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
    JointLimit joint_limit;
    joint_limit.max_position = 3.124;
    joint_limit.min_position = -3.124;
    joint_limit.has_velocity_limits = true;
    joint_limit.max_velocity = 1;
    joint_limit.has_acceleration_limits = true;
    joint_limit.max_acceleration = 0.5;
    joint_limit.has_deceleration_limits = true;
    joint_limit.max_deceleration = -1;
    for (const auto& joint_name : joint_names)
    {
      joint_limits.addLimit(joint_name, joint_limit);
    }
  }

  // create the trajectory generator
  planner_limits_.setJointLimits(joint_limits);
  ptp_ = std::make_unique<TrajectoryGeneratorPTP>(robot_model_, planner_limits_, planning_group_);
  ASSERT_NE(nullptr, ptp_);
}

bool TrajectoryGeneratorPTPTest::checkTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                                 const planning_interface::MotionPlanRequest& req,
                                                 const JointLimitsContainer& joint_limits)
{
  return (testutils::isTrajectoryConsistent(trajectory) &&
          testutils::isGoalReached(trajectory, req.goal_constraints.front().joint_constraints,
                                   joint_position_tolerance_, joint_velocity_tolerance_) &&
          testutils::isPositionBounded(trajectory, joint_limits) &&
          testutils::isVelocityBounded(trajectory, joint_limits) &&
          testutils::isAccelerationBounded(trajectory, joint_limits));
}

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST(TrajectoryGeneratorPTPTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<PtpVelocityProfileSyncFailed> pvpsf_ex{ new PtpVelocityProfileSyncFailed("") };
    EXPECT_EQ(pvpsf_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<PtpNoIkSolutionForGoalPose> pnisfgp_ex{ new PtpNoIkSolutionForGoalPose("") };
    EXPECT_EQ(pnisfgp_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
  }
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_SUITE_P(InstantiationName, TrajectoryGeneratorPTPTest,
                         ::testing::Values(PARAM_MODEL_NO_GRIPPER_NAME, PARAM_MODEL_WITH_GRIPPER_NAME));
/**
 * @brief Construct a TrajectoryGeneratorPTP with no limits given
 */
TEST_P(TrajectoryGeneratorPTPTest, noLimits)
{
  LimitsContainer planner_limits;
  EXPECT_THROW(TrajectoryGeneratorPTP(this->robot_model_, planner_limits, planning_group_),
               TrajectoryGeneratorInvalidLimitsException);
}

/**
 * @brief Send an empty request, define res.trajectory_
 *
 *  - Test Sequence:
 *    1. Create request, define a trajectory in the result
 *    2. assign at least one joint limit will all required limits
 *
 *  - Expected Results:
 *    1. the res.trajectory_ should be cleared (contain no waypoints)
 */
TEST_P(TrajectoryGeneratorPTPTest, emptyRequest)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;

  robot_trajectory::RobotTrajectoryPtr trajectory(
      new robot_trajectory::RobotTrajectory(this->robot_model_, planning_group_));
  robot_state::RobotState state(this->robot_model_);
  trajectory->addPrefixWayPoint(state, 0);
  res.trajectory_ = trajectory;

  EXPECT_FALSE(res.trajectory_->empty());

  EXPECT_FALSE(ptp_->generate(planning_scene_, req, res));

  EXPECT_TRUE(res.trajectory_->empty());
}

/**
 * @brief Construct a TrajectoryGeneratorPTP with missing velocity limits
 */
TEST_P(TrajectoryGeneratorPTPTest, missingVelocityLimits)
{
  LimitsContainer planner_limits;

  JointLimitsContainer joint_limits;
  auto joint_models = robot_model_->getActiveJointModels();
  JointLimit joint_limit;
  joint_limit.has_velocity_limits = false;
  joint_limit.has_acceleration_limits = true;
  joint_limit.max_deceleration = -1;
  joint_limit.has_deceleration_limits = true;
  for (const auto& joint_model : joint_models)
  {
    ASSERT_TRUE(joint_limits.addLimit(joint_model->getName(), joint_limit))
        << "Failed to add the limits for joint " << joint_model->getName();
  }

  planner_limits.setJointLimits(joint_limits);
  EXPECT_THROW(TrajectoryGeneratorPTP(this->robot_model_, planner_limits, planning_group_),
               TrajectoryGeneratorInvalidLimitsException);
}

/**
 * @brief Construct a TrajectoryGeneratorPTP missing deceleration limits
 */
TEST_P(TrajectoryGeneratorPTPTest, missingDecelerationimits)
{
  LimitsContainer planner_limits;

  JointLimitsContainer joint_limits;
  const auto& joint_models = robot_model_->getActiveJointModels();
  JointLimit joint_limit;
  joint_limit.has_velocity_limits = true;
  joint_limit.has_acceleration_limits = true;
  joint_limit.has_deceleration_limits = false;
  for (const auto& joint_model : joint_models)
  {
    ASSERT_TRUE(joint_limits.addLimit(joint_model->getName(), joint_limit))
        << "Failed to add the limits for joint " << joint_model->getName();
  }

  planner_limits.setJointLimits(joint_limits);
  EXPECT_THROW(TrajectoryGeneratorPTP(this->robot_model_, planner_limits, planning_group_),
               TrajectoryGeneratorInvalidLimitsException);
}

/**
 * @brief test the constructor when insufficient limits are given
 *  - Test Sequence:
 *    1. assign joint limits without acc and dec
 *    2. assign at least one joint limit per group with all required limits
 *
 *  - Expected Results:
 *    1. the constructor throws an exception of type
 * TrajectoryGeneratorInvalidLimitsException
 *    2. the constructor throws no exception
 */
TEST_P(TrajectoryGeneratorPTPTest, testInsufficientLimit)
{
  /**********/
  /* Step 1 */
  /**********/
  const auto& joint_models = robot_model_->getActiveJointModels();
  ASSERT_TRUE(joint_models.size());

  // joint limit with insufficient limits (no acc/dec limits)
  JointLimit insufficient_limit;
  insufficient_limit.has_position_limits = true;
  insufficient_limit.max_position = 2.5;
  insufficient_limit.min_position = -2.5;
  insufficient_limit.has_velocity_limits = true;
  insufficient_limit.max_velocity = 1.256;
  insufficient_limit.has_acceleration_limits = false;
  insufficient_limit.has_deceleration_limits = false;
  JointLimitsContainer insufficient_joint_limits;
  for (const auto& joint_model : joint_models)
  {
    ASSERT_TRUE(insufficient_joint_limits.addLimit(joint_model->getName(), insufficient_limit))
        << "Failed to add the limits for joint " << joint_model->getName();
  }
  LimitsContainer insufficient_planner_limits;
  insufficient_planner_limits.setJointLimits(insufficient_joint_limits);

  EXPECT_THROW(
      {
        std::unique_ptr<TrajectoryGeneratorPTP> ptp_error(
            new TrajectoryGeneratorPTP(robot_model_, insufficient_planner_limits, planning_group_));
      },
      TrajectoryGeneratorInvalidLimitsException);

  /**********/
  /* Step 2 */
  /**********/
  // joint limit with sufficient limits
  JointLimit sufficient_limit;
  sufficient_limit.has_position_limits = true;
  sufficient_limit.max_position = 2.356;
  sufficient_limit.min_position = -2.356;
  sufficient_limit.has_velocity_limits = true;
  sufficient_limit.max_velocity = 1;
  sufficient_limit.has_acceleration_limits = true;
  sufficient_limit.max_acceleration = 0.5;
  sufficient_limit.has_deceleration_limits = true;
  sufficient_limit.max_deceleration = -1;
  JointLimitsContainer sufficient_joint_limits;
  // fill joint limits container, such that it contains one sufficient limit and
  // all others are insufficient
  for (const auto& jmg : robot_model_->getJointModelGroups())
  {
    const auto& joint_names{ jmg->getActiveJointModelNames() };
    ASSERT_FALSE(joint_names.empty());
    ASSERT_TRUE(sufficient_joint_limits.addLimit(joint_names.front(), sufficient_limit))
        << "Failed to add the limits for joint " << joint_names.front();

    for (auto it = std::next(joint_names.begin()); it != joint_names.end(); ++it)
    {
      ASSERT_TRUE(sufficient_joint_limits.addLimit((*it), insufficient_limit))
          << "Failed to add the limits for joint " << (*it);
    }
  }
  LimitsContainer sufficient_planner_limits;
  sufficient_planner_limits.setJointLimits(sufficient_joint_limits);

  EXPECT_NO_THROW({
    std::unique_ptr<TrajectoryGeneratorPTP> ptp_no_error(
        new TrajectoryGeneratorPTP(robot_model_, sufficient_planner_limits, planning_group_));
  });
}

/**
 * @brief test the ptp trajectory generator of Cartesian space goal
 */
TEST_P(TrajectoryGeneratorPTPTest, testCartesianGoal)
{
  //***************************************
  //*** prepare the motion plan request ***
  //***************************************
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);

  // cartesian goal pose
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 0.65;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(target_link_, pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  //****************************************
  //*** test robot model without gripper ***
  //****************************************
  ASSERT_TRUE(ptp_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  if (!res_msg.trajectory.joint_trajectory.points.empty())
  {
    EXPECT_TRUE(checkTrajectory(res_msg.trajectory.joint_trajectory, req, planner_limits_.getJointLimitContainer()));
  }
  else
  {
    FAIL() << "Received empty trajectory.";
  }

  // check goal pose
  EXPECT_TRUE(testutils::isGoalReached(robot_model_, res_msg.trajectory.joint_trajectory, req, pose_norm_tolerance_));
}

/**
 * @brief Check that missing a link_name in position or orientation constraints
 * is detected
 */
TEST_P(TrajectoryGeneratorPTPTest, testCartesianGoalMissingLinkNameConstraints)
{
  //***************************************
  //*** prepare the motion plan request ***
  //***************************************
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);

  // cartesian goal pose
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 0.65;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(target_link_, pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  planning_interface::MotionPlanRequest req_no_position_constaint_link_name = req;
  req_no_position_constaint_link_name.goal_constraints.front().position_constraints.front().link_name = "";
  ASSERT_FALSE(ptp_->generate(planning_scene_, req_no_position_constaint_link_name, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

  planning_interface::MotionPlanRequest req_no_orientation_constaint_link_name = req;
  req_no_orientation_constaint_link_name.goal_constraints.front().orientation_constraints.front().link_name = "";
  ASSERT_FALSE(ptp_->generate(planning_scene_, req_no_orientation_constaint_link_name, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief test the ptp trajectory generator of invalid Cartesian space goal
 */
TEST_P(TrajectoryGeneratorPTPTest, testInvalidCartesianGoal)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 2.5;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(target_link_, pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  ASSERT_FALSE(ptp_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
  EXPECT_EQ(res.trajectory_, nullptr);
}

/**
 * @brief test the ptp trajectory generator of joint space goal which is close
 * enough to the start which does not need
 * to plan the trajectory
 */
TEST_P(TrajectoryGeneratorPTPTest, testJointGoalAlreadyReached)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);
  ASSERT_TRUE(robot_model_->getJointModelGroup(planning_group_)->getActiveJointModelNames().size())
      << "No link exists in the planning group.";

  moveit_msgs::Constraints gc;
  moveit_msgs::JointConstraint jc;
  jc.joint_name = robot_model_->getJointModelGroup(planning_group_)->getActiveJointModelNames().front();
  jc.position = 0.0;
  gc.joint_constraints.push_back(jc);
  req.goal_constraints.push_back(gc);

  // TODO lin and circ has different settings
  ASSERT_TRUE(ptp_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  EXPECT_EQ(1u, res_msg.trajectory.joint_trajectory.points.size());
}

/**
 * @brief test scaling factor
 * with zero start velocity
 */
TEST_P(TrajectoryGeneratorPTPTest, testScalingFactor)
{
  // create ptp generator with different limits
  JointLimit joint_limit;
  JointLimitsContainer joint_limits;

  // set the joint limits
  joint_limit.has_position_limits = true;
  joint_limit.max_position = 2.967;
  joint_limit.min_position = -2.967;
  joint_limit.has_velocity_limits = true;
  joint_limit.max_velocity = 2;
  joint_limit.has_acceleration_limits = true;
  joint_limit.max_acceleration = 1.5;
  joint_limit.has_deceleration_limits = true;
  joint_limit.max_deceleration = -3;
  joint_limits.addLimit("prbt_joint_1", joint_limit);
  joint_limit.max_position = 2.530;
  joint_limit.min_position = -2.530;
  joint_limits.addLimit("prbt_joint_2", joint_limit);
  joint_limit.max_position = 2.356;
  joint_limit.min_position = -2.356;
  joint_limits.addLimit("prbt_joint_3", joint_limit);
  joint_limit.max_position = 2.967;
  joint_limit.min_position = -2.967;
  joint_limits.addLimit("prbt_joint_4", joint_limit);
  joint_limit.max_position = 2.967;
  joint_limit.min_position = -2.967;
  joint_limits.addLimit("prbt_joint_5", joint_limit);
  joint_limit.max_position = 3.132;
  joint_limit.min_position = -3.132;
  joint_limits.addLimit("prbt_joint_6", joint_limit);
  // add gripper limit such that generator does not complain about missing limit
  joint_limits.addLimit("prbt_gripper_finger_left_joint", joint_limit);

  pilz_industrial_motion_planner::LimitsContainer planner_limits;
  planner_limits.setJointLimits(joint_limits);

  // create the generator with new limits
  ptp_ = std::make_unique<TrajectoryGeneratorPTP>(robot_model_, planner_limits, planning_group_);

  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);
  req.start_state.joint_state.position[2] = 0.1;
  moveit_msgs::Constraints gc;
  moveit_msgs::JointConstraint jc;
  jc.joint_name = "prbt_joint_1";
  jc.position = 1.5;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_3";
  jc.position = 2.1;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_6";
  jc.position = 3.0;
  gc.joint_constraints.push_back(jc);
  req.goal_constraints.push_back(gc);
  req.max_velocity_scaling_factor = 0.5;
  req.max_acceleration_scaling_factor = 1.0 / 3.0;

  ASSERT_TRUE(ptp_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  EXPECT_TRUE(checkTrajectory(res_msg.trajectory.joint_trajectory, req, planner_limits_.getJointLimitContainer()));

  // trajectory duration
  EXPECT_NEAR(4.5, res.trajectory_->getWayPointDurationFromStart(res.trajectory_->getWayPointCount()),
              joint_acceleration_tolerance_);

  // way point at 1s
  int index;
  index = testutils::getWayPointIndex(res.trajectory_, 1.0);
  // joint_1
  EXPECT_NEAR(0.125, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(1.0 / 6.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2],
              joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);
  // other joints
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].positions[4], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[4], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[4], joint_acceleration_tolerance_);

  // way point at 2s
  index = testutils::getWayPointIndex(res.trajectory_, 2.0);
  // joint_1
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(2.0 / 3.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  // other joints
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[1], joint_acceleration_tolerance_);

  // way point at 3s
  index = testutils::getWayPointIndex(res.trajectory_, 3.0);
  // joint_1
  EXPECT_NEAR(1, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(4.0 / 3.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2], joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(2.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);
  // other joints
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[3], joint_acceleration_tolerance_);

  // way point at 4s
  index = testutils::getWayPointIndex(res.trajectory_, 4.0);
  // joint_1
  EXPECT_NEAR(2.875 / 2.0, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(-0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(5.75 / 3.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(-2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2],
              joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(2.875, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(-1.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);

  // way point at 4.5s
  index = testutils::getWayPointIndex(res.trajectory_, 4.5);
  // joint_1
  EXPECT_NEAR(1.5, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(2.1, res_msg.trajectory.joint_trajectory.points[index].positions[2], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(3.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
}

/**
 * @brief test the ptp trajectory generator of joint space goal
 * with (almost) zero start velocity
 */
TEST_P(TrajectoryGeneratorPTPTest, testJointGoalAndAlmostZeroStartVelocity)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);
  req.start_state.joint_state.position[2] = 0.1;

  // Set velocity to all 1e-16
  req.start_state.joint_state.velocity = std::vector<double>(req.start_state.joint_state.position.size(), 1e-16);

  moveit_msgs::Constraints gc;
  moveit_msgs::JointConstraint jc;
  jc.joint_name = "prbt_joint_1";
  jc.position = 1.5;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_3";
  jc.position = 2.1;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_6";
  jc.position = 3.0;
  gc.joint_constraints.push_back(jc);
  req.goal_constraints.push_back(gc);

  ASSERT_TRUE(ptp_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  EXPECT_TRUE(checkTrajectory(res_msg.trajectory.joint_trajectory, req, planner_limits_.getJointLimitContainer()));

  // trajectory duration
  EXPECT_NEAR(4.5, res.trajectory_->getWayPointDurationFromStart(res.trajectory_->getWayPointCount()),
              joint_acceleration_tolerance_);

  // way point at 1s
  int index;
  index = testutils::getWayPointIndex(res.trajectory_, 1.0);
  // joint_1
  EXPECT_NEAR(0.125, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(1.0 / 6.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2],
              joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);
  // other joints
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].positions[4], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[4], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[4], joint_acceleration_tolerance_);

  // way point at 2s
  index = testutils::getWayPointIndex(res.trajectory_, 2.0);
  // joint_1
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(2.0 / 3.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  // other joints
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[1], joint_acceleration_tolerance_);

  // way point at 3s
  index = testutils::getWayPointIndex(res.trajectory_, 3.0);
  // joint_1
  EXPECT_NEAR(1, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(4.0 / 3.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2], joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(2.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);
  // other joints
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[3], joint_acceleration_tolerance_);

  // way point at 4s
  index = testutils::getWayPointIndex(res.trajectory_, 4.0);
  // joint_1
  EXPECT_NEAR(2.875 / 2.0, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(-0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(5.75 / 3.0 + 0.1, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(-2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2],
              joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(2.875, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(-1.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);

  // way point at 4.5s
  index = testutils::getWayPointIndex(res.trajectory_, 4.5);
  // joint_1
  EXPECT_NEAR(1.5, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(2.1, res_msg.trajectory.joint_trajectory.points[index].positions[2], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(3.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);

  // Check that velocity at the end is all zero
  EXPECT_TRUE(std::all_of(res_msg.trajectory.joint_trajectory.points.back().velocities.cbegin(),
                          res_msg.trajectory.joint_trajectory.points.back().velocities.cend(),
                          [this](double v) { return std::fabs(v) < this->joint_velocity_tolerance_; }));

  // Check that acceleration at the end is all zero
  EXPECT_TRUE(std::all_of(res_msg.trajectory.joint_trajectory.points.back().accelerations.cbegin(),
                          res_msg.trajectory.joint_trajectory.points.back().accelerations.cend(),
                          [this](double v) { return std::fabs(v) < this->joint_acceleration_tolerance_; }));
}

/**
 * @brief test the ptp_ trajectory generator of joint space goal
 * with zero start velocity
 */
TEST_P(TrajectoryGeneratorPTPTest, testJointGoalNoStartVel)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req;
  testutils::createDummyRequest(robot_model_, planning_group_, req);
  req.start_state.joint_state.position[4] = 0.3;
  req.start_state.joint_state.position[2] = 0.11;

  moveit_msgs::Constraints gc;
  moveit_msgs::JointConstraint jc;

  jc.joint_name = "prbt_joint_1";
  jc.position = 1.5;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_2";
  jc.position = -1.5;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_3";
  jc.position = 2.11;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_4";
  jc.position = -2.0;
  gc.joint_constraints.push_back(jc);
  jc.joint_name = "prbt_joint_6";
  jc.position = 3.0;
  gc.joint_constraints.push_back(jc);
  req.goal_constraints.push_back(gc);

  ASSERT_TRUE(ptp_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  EXPECT_TRUE(checkTrajectory(res_msg.trajectory.joint_trajectory, req, planner_limits_.getJointLimitContainer()));

  // trajectory duration
  EXPECT_NEAR(4.5, res.trajectory_->getWayPointDurationFromStart(res.trajectory_->getWayPointCount()),
              joint_position_tolerance_);

  // way point at 0s
  // joint_1
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].velocities[0], joint_velocity_tolerance_);
  // joint_2
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].velocities[1], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(0.11, res_msg.trajectory.joint_trajectory.points[0].positions[2], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].velocities[2], joint_velocity_tolerance_);
  // joint_4
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].velocities[3], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[0].velocities[5], joint_velocity_tolerance_);

  // way point at 1s
  int index;
  index = testutils::getWayPointIndex(res.trajectory_, 1.0);
  // joint_1
  EXPECT_NEAR(0.125, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_2
  EXPECT_NEAR(-0.125, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(-0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  EXPECT_NEAR(-0.25, res_msg.trajectory.joint_trajectory.points[index].accelerations[1], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(1.0 / 6.0 + 0.11, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2],
              joint_acceleration_tolerance_);
  // joint_4
  EXPECT_NEAR(-1.0 / 6.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(-1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  EXPECT_NEAR(-1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[3],
              joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);

  // way point at 2s
  index = testutils::getWayPointIndex(res.trajectory_, 2.0);
  // joint_1
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  // joint_2
  EXPECT_NEAR(-0.5, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(-0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(2.0 / 3.0 + 0.11, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  // joint_4
  EXPECT_NEAR(-2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(-2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);

  // way point at 3s
  index = testutils::getWayPointIndex(res.trajectory_, 3.0);
  // joint_1
  EXPECT_NEAR(1, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_2
  EXPECT_NEAR(-1, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(-0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[1], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(4.0 / 3.0 + 0.11, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2], joint_acceleration_tolerance_);
  // joint_4
  EXPECT_NEAR(-4.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(-2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[3], joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(2.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(1.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);

  // way point at 4s
  index = testutils::getWayPointIndex(res.trajectory_, 4.0);
  // joint_1
  EXPECT_NEAR(2.875 / 2.0, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  EXPECT_NEAR(-0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[0], joint_acceleration_tolerance_);
  // joint_2
  EXPECT_NEAR(-2.875 / 2.0, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(-0.25, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].accelerations[1], joint_acceleration_tolerance_);
  // joint_3
  EXPECT_NEAR(5.75 / 3.0 + 0.11, res_msg.trajectory.joint_trajectory.points[index].positions[2],
              joint_position_tolerance_);
  EXPECT_NEAR(1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  EXPECT_NEAR(-2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[2],
              joint_acceleration_tolerance_);
  // joint_4
  EXPECT_NEAR(-5.75 / 3.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(-1.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  EXPECT_NEAR(2.0 / 3.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[3],
              joint_acceleration_tolerance_);
  // joint_6
  EXPECT_NEAR(2.875, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.5, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);
  EXPECT_NEAR(-1.0, res_msg.trajectory.joint_trajectory.points[index].accelerations[5], joint_acceleration_tolerance_);

  // way point at 4.5s
  index = testutils::getWayPointIndex(res.trajectory_, 4.5);
  // joint_1
  EXPECT_NEAR(1.5, res_msg.trajectory.joint_trajectory.points[index].positions[0], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[0], joint_velocity_tolerance_);
  // joint_2
  EXPECT_NEAR(-1.5, res_msg.trajectory.joint_trajectory.points[index].positions[1], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[1], joint_velocity_tolerance_);
  // joint_3
  EXPECT_NEAR(2.11, res_msg.trajectory.joint_trajectory.points[index].positions[2], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[2], joint_velocity_tolerance_);
  // joint_4
  EXPECT_NEAR(-2.0, res_msg.trajectory.joint_trajectory.points[index].positions[3], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[3], joint_velocity_tolerance_);
  // joint_6
  EXPECT_NEAR(3.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);
  EXPECT_NEAR(0.0, res_msg.trajectory.joint_trajectory.points[index].velocities[5], joint_velocity_tolerance_);

  // Check last point
  EXPECT_NEAR(3.0, res_msg.trajectory.joint_trajectory.points[index].positions[5], joint_position_tolerance_);

  // Check that velocity at the end is all zero
  EXPECT_TRUE(std::all_of(res_msg.trajectory.joint_trajectory.points.back().velocities.cbegin(),
                          res_msg.trajectory.joint_trajectory.points.back().velocities.cend(),
                          [this](double v) { return std::fabs(v) < this->joint_velocity_tolerance_; }));

  // Check that acceleration at the end is all zero
  EXPECT_TRUE(std::all_of(res_msg.trajectory.joint_trajectory.points.back().accelerations.cbegin(),
                          res_msg.trajectory.joint_trajectory.points.back().accelerations.cend(),
                          [this](double v) { return std::fabs(v) < this->joint_acceleration_tolerance_; }));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_trajectory_generator_ptp");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
