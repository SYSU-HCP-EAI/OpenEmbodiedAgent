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
#include "pilz_industrial_motion_planner/trajectory_generator_lin.h"
#include "pilz_industrial_motion_planner_testutils/command_types_typedef.h"
#include "pilz_industrial_motion_planner_testutils/xml_testdata_loader.h"
#include "test_utils.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/console.h>

const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

// parameters from parameter server
const std::string TEST_DATA_FILE_NAME("testdata_file_name");
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string TARGET_LINK_HCD("target_link_hand_computed_data");
const std::string RANDOM_TEST_TRIAL_NUM("random_trial_number");
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");
const std::string JOINT_VELOCITY_TOLERANCE("joint_velocity_tolerance");
const std::string POSE_TRANSFORM_MATRIX_NORM_TOLERANCE("pose_norm_tolerance");
const std::string ROTATION_AXIS_NORM_TOLERANCE("rot_axis_norm_tolerance");
const std::string VELOCITY_SCALING_FACTOR("velocity_scaling_factor");
const std::string OTHER_TOLERANCE("other_tolerance");

using namespace pilz_industrial_motion_planner;
using namespace pilz_industrial_motion_planner_testutils;

/**
 * @brief Parameterized unittest of trajectory generator LIN to enable tests
 * against
 * different robot models.The parameter is the name of robot model parameter on
 * the
 * ros parameter server.
 */
class TrajectoryGeneratorLINTest : public testing::TestWithParam<std::string>
{
protected:
  /**
   * @brief Create test scenario for lin trajectory generator
   *
   */
  void SetUp() override;

  bool checkLinResponse(const planning_interface::MotionPlanRequest& req,
                        const planning_interface::MotionPlanResponse& res);

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(GetParam()).getModel() };
  planning_scene::PlanningSceneConstPtr planning_scene_{ new planning_scene::PlanningScene(robot_model_) };

  // lin trajectory generator using model without gripper
  std::unique_ptr<TrajectoryGenerator> lin_;
  // test data provider
  std::unique_ptr<pilz_industrial_motion_planner_testutils::TestdataLoader> tdp_;

  // test parameters from parameter server
  std::string planning_group_, target_link_hcd_, test_data_file_name_;
  int random_trial_num_;
  double joint_position_tolerance_, joint_velocity_tolerance_, pose_norm_tolerance_, rot_axis_norm_tolerance_,
      velocity_scaling_factor_, other_tolerance_;
  LimitsContainer planner_limits_;
};

void TrajectoryGeneratorLINTest::SetUp()
{
  // get the parameters
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(TARGET_LINK_HCD, target_link_hcd_));
  ASSERT_TRUE(ph_.getParam(RANDOM_TEST_TRIAL_NUM, random_trial_num_));
  ASSERT_TRUE(ph_.getParam(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_VELOCITY_TOLERANCE, joint_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(POSE_TRANSFORM_MATRIX_NORM_TOLERANCE, pose_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(ROTATION_AXIS_NORM_TOLERANCE, rot_axis_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(VELOCITY_SCALING_FACTOR, velocity_scaling_factor_));
  ASSERT_TRUE(ph_.getParam(OTHER_TOLERANCE, other_tolerance_));

  testutils::checkRobotModel(robot_model_, planning_group_, target_link_hcd_);

  // load the test data provider
  tdp_ = std::make_unique<pilz_industrial_motion_planner_testutils::XmlTestdataLoader>(test_data_file_name_);
  ASSERT_NE(nullptr, tdp_) << "Failed to load test data by provider.";

  tdp_->setRobotModel(robot_model_);

  // create the limits container
  // TODO, move this also into test data set
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
      pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(ph_,
                                                                                 robot_model_->getActiveJointModels());
  CartesianLimit cart_limits;
  cart_limits.setMaxRotationalVelocity(0.5 * M_PI);
  cart_limits.setMaxTranslationalAcceleration(2);
  cart_limits.setMaxTranslationalDeceleration(2);
  cart_limits.setMaxTranslationalVelocity(1);
  planner_limits_.setJointLimits(joint_limits);
  planner_limits_.setCartesianLimits(cart_limits);

  // initialize the LIN trajectory generator
  lin_ = std::make_unique<TrajectoryGeneratorLIN>(robot_model_, planner_limits_, planning_group_);
  ASSERT_NE(nullptr, lin_) << "Failed to create LIN trajectory generator.";
}

bool TrajectoryGeneratorLINTest::checkLinResponse(const planning_interface::MotionPlanRequest& req,
                                                  const planning_interface::MotionPlanResponse& res)
{
  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  if (!testutils::isGoalReached(robot_model_, res_msg.trajectory.joint_trajectory, req, pose_norm_tolerance_))
  {
    return false;
  }

  if (!testutils::checkCartesianLinearity(robot_model_, res_msg.trajectory.joint_trajectory, req, pose_norm_tolerance_,
                                          rot_axis_norm_tolerance_))
  {
    return false;
  }

  if (!testutils::checkJointTrajectory(res_msg.trajectory.joint_trajectory, planner_limits_.getJointLimitContainer()))
  {
    return false;
  }

  return true;
}

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST(TrajectoryGeneratorLINTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<LinTrajectoryConversionFailure> ltcf_ex{ new LinTrajectoryConversionFailure("") };
    EXPECT_EQ(ltcf_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<JointNumberMismatch> jnm_ex{ new JointNumberMismatch("") };
    EXPECT_EQ(jnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<LinJointMissingInStartState> ljmiss_ex{ new LinJointMissingInStartState("") };
    EXPECT_EQ(ljmiss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<LinInverseForGoalIncalculable> lifgi_ex{ new LinInverseForGoalIncalculable("") };
    EXPECT_EQ(lifgi_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
  }
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_SUITE_P(InstantiationName, TrajectoryGeneratorLINTest,
                         ::testing::Values(PARAM_MODEL_NO_GRIPPER_NAME, PARAM_MODEL_WITH_GRIPPER_NAME));

/**
 * @brief test the lin planner with invalid motion plan request which has non
 * zero start velocity
 */
TEST_P(TrajectoryGeneratorLINTest, nonZeroStartVelocity)
{
  planning_interface::MotionPlanRequest req{ tdp_->getLinJoint("lin2").toRequest() };

  // add non-zero velocity in the start state
  req.start_state.joint_state.velocity.push_back(1.0);

  // try to generate the result
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(lin_->generate(planning_scene_, req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief test the lin planner with joint space goal
 */
TEST_P(TrajectoryGeneratorLINTest, jointSpaceGoal)
{
  planning_interface::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(planning_scene_, lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief test the lin planner with joint space goal with start velocity almost
 * zero
 */
TEST_P(TrajectoryGeneratorLINTest, jointSpaceGoalNearZeroStartVelocity)
{
  planning_interface::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  // Set velocity near zero
  lin_joint_req.start_state.joint_state.velocity =
      std::vector<double>(lin_joint_req.start_state.joint_state.position.size(), 1e-16);

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(planning_scene_, lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief test the lin planner with Cartesian goal
 */
TEST_P(TrajectoryGeneratorLINTest, cartesianSpaceGoal)
{
  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_cart_req{ tdp_->getLinCart("lin2").toRequest() };

  // generate lin trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(planning_scene_, lin_cart_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_cart_req, res));
}

/**
 * @brief test the trapezoid shape of the planning trajectory in Cartesian space
 *
 * The test checks translational path for a trapezoid velocity profile.
 * Due to the way the acceleration is calculated 1 or 2 intermediate points
 * occur that are neither
 * acceleration, constant or deceleration.
 */
TEST_P(TrajectoryGeneratorLINTest, cartesianTrapezoidProfile)
{
  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  /// +++++++++++++++++++++++
  /// + plan LIN trajectory +
  /// +++++++++++++++++++++++
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(planning_scene_, lin_joint_req, res, 0.01));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);

  ASSERT_TRUE(testutils::checkCartesianTranslationalPath(res.trajectory_, target_link_hcd_));

  // check last point for vel=acc=0
  for (size_t idx = 0; idx < res.trajectory_->getLastWayPointPtr()->getVariableCount(); ++idx)
  {
    EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableVelocity(idx), other_tolerance_);
    EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableAcceleration(idx), other_tolerance_);
  }
}

/**
 * @brief Check that lin planner returns 'false' if
 * calculated lin trajectory violates velocity/acceleration or deceleration
 * limits.
 *
 *
 * Test Sequence:
 *    1. Call function with lin request violating velocity/acceleration or
 * deceleration limits.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryGeneratorLINTest, LinPlannerLimitViolation)
{
  LinJoint lin{ tdp_->getLinJoint("lin2") };
  lin.setAccelerationScale(1.01);

  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(lin_->generate(planning_scene_, lin.toRequest(), res));
}

/**
 * @brief test joint linear movement with discontinuities in joint space
 *
 * This will violate joint velocity/acceleration limits.
 *
 * Test Sequence:
 *    1. Generate lin trajectory which is discontinuous in joint space.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryGeneratorLINTest, LinPlannerDiscontinuousJointTraj)
{
  LinJoint lin{ tdp_->getLinJoint("lin2") };
  // Alter goal joint configuration (represents the same cartesian pose, but
  // does not fit together with start config)
  lin.getGoalConfiguration().setJoint(1, 1.63);
  lin.getGoalConfiguration().setJoint(2, 0.96);
  lin.getGoalConfiguration().setJoint(4, -2.48);
  lin.setVelocityScale(1.0);
  lin.setAccelerationScale(1.0);

  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(lin_->generate(planning_scene_, lin.toRequest(), res));
}

/**
 * @brief test joint linear movement with equal goal and start
 *
 * Test Sequence:
 *    1. Call function with lin request start = goal
 *
 * Expected Results:
 *    1. trajectory generation is successful.
 */
TEST_P(TrajectoryGeneratorLINTest, LinStartEqualsGoal)
{
  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  moveit::core::RobotState start_state(robot_model_);
  jointStateToRobotState(lin_joint_req.start_state.joint_state, start_state);

  for (auto& joint_constraint : lin_joint_req.goal_constraints.at(0).joint_constraints)
  {
    joint_constraint.position = start_state.getVariablePosition(joint_constraint.joint_name);
  }

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(planning_scene_, lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief Checks that constructor throws an exception if no limits are given.
 *
 * Test Sequence:
 *    1. Call Ctor without set limits.
 *
 * Expected Results:
 *    1. Ctor throws exception.
 */
TEST_P(TrajectoryGeneratorLINTest, CtorNoLimits)
{
  pilz_industrial_motion_planner::LimitsContainer planner_limits;

  EXPECT_THROW(pilz_industrial_motion_planner::TrajectoryGeneratorLIN(robot_model_, planner_limits, planning_group_),
               pilz_industrial_motion_planner::TrajectoryGeneratorInvalidLimitsException);
}

/**
 * @brief Checks that generate() function returns 'false' if called with an
 * incorrect number of joints.
 *
 * Test Sequence:
 *    1. Call functions with incorrect number of joints.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryGeneratorLINTest, IncorrectJointNumber)
{
  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req{ tdp_->getLinJoint("lin2").toRequest() };

  // Ensure that request consists of an incorrect number of joints.
  lin_joint_req.goal_constraints.front().joint_constraints.pop_back();

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(lin_->generate(planning_scene_, lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief test invalid motion plan request with incomplete start state and
 * cartesian goal
 */
TEST_P(TrajectoryGeneratorLINTest, cartGoalIncompleteStartState)
{
  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_cart_req{ tdp_->getLinCart("lin2").toRequest() };
  EXPECT_GT(lin_cart_req.start_state.joint_state.name.size(), 1u);
  lin_cart_req.start_state.joint_state.name.resize(1);
  lin_cart_req.start_state.joint_state.position.resize(1);  // prevent failing check for equal sizes

  // generate lin trajectory
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(lin_->generate(planning_scene_, lin_cart_req, res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief Set a frame id in goal constraint with cartesian goal on both position
 * and orientation constraints
 */
TEST_P(TrajectoryGeneratorLINTest, cartGoalFrameIdBothConstraints)
{
  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_cart_req{ tdp_->getLinCart("lin2").toRequest() };

  lin_cart_req.goal_constraints.front().position_constraints.front().header.frame_id = robot_model_->getModelFrame();
  lin_cart_req.goal_constraints.front().orientation_constraints.front().header.frame_id = robot_model_->getModelFrame();

  // generate lin trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(planning_scene_, lin_cart_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_cart_req, res));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_trajectory_generator_lin");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
