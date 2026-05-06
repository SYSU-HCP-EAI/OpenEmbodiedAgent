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
#include <string>

#include <gtest/gtest.h>

#include <functional>
#include <ros/ros.h>
#include <ros/time.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <tf2_eigen/tf2_eigen.h>

#include <pilz_industrial_motion_planner_testutils/gripper.h>
#include <pilz_industrial_motion_planner_testutils/lin.h>
#include <pilz_industrial_motion_planner_testutils/sequence.h>
#include <pilz_industrial_motion_planner_testutils/xml_testdata_loader.h>

#include "test_utils.h"

#include "pilz_industrial_motion_planner/command_list_manager.h"
#include "pilz_industrial_motion_planner/tip_frame_getter.h"

const std::string ROBOT_DESCRIPTION_STR{ "robot_description" };
const std::string EMPTY_VALUE{ "" };

const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using testutils::hasStrictlyIncreasingTime;
using namespace pilz_industrial_motion_planner;
using namespace pilz_industrial_motion_planner_testutils;

static std::string createManipulatorJointName(const size_t& joint_number)
{
  return std::string("prbt_joint_") + std::to_string(joint_number + 1);
}

static std::string createGripperJointName(const size_t& joint_number)
{
  switch (joint_number)
  {
    case 0:
      return "prbt_gripper_finger_left_joint";
    default:
      break;
  }
  throw std::runtime_error("Could not create gripper joint name");
}

class IntegrationTestCommandListManager : public testing::Test
{
protected:
  void SetUp() override;

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_STR).getModel() };
  std::shared_ptr<pilz_industrial_motion_planner::CommandListManager> manager_;
  planning_scene::PlanningScenePtr scene_;
  planning_pipeline::PlanningPipelinePtr pipeline_;

  std::unique_ptr<pilz_industrial_motion_planner_testutils::TestdataLoader> data_loader_;
};

void IntegrationTestCommandListManager::SetUp()
{
  // get necessary parameters
  if (!robot_model_)
  {
    FAIL() << "Robot model could not be loaded.";
  }

  std::string test_data_file_name;
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name));

  // load the test data provider
  data_loader_ =
      std::make_unique<pilz_industrial_motion_planner_testutils::XmlTestdataLoader>(test_data_file_name, robot_model_);
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  // Define and set the current scene and manager test object
  manager_ = std::make_shared<pilz_industrial_motion_planner::CommandListManager>(ph_, robot_model_);
  scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, ph_);
}

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 *
 */
TEST_F(IntegrationTestCommandListManager, TestExceptionErrorCodeMapping)
{
  std::shared_ptr<NegativeBlendRadiusException> nbr_ex{ new NegativeBlendRadiusException("") };
  EXPECT_EQ(nbr_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  std::shared_ptr<LastBlendRadiusNotZeroException> lbrnz_ex{ new LastBlendRadiusNotZeroException("") };
  EXPECT_EQ(lbrnz_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  std::shared_ptr<StartStateSetException> sss_ex{ new StartStateSetException("") };
  EXPECT_EQ(sss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);

  std::shared_ptr<OverlappingBlendRadiiException> obr_ex{ new OverlappingBlendRadiiException("") };
  EXPECT_EQ(obr_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  std::shared_ptr<PlanningPipelineException> pp_ex{ new PlanningPipelineException("") };
  EXPECT_EQ(pp_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
}

/**
 * @brief Tests the concatenation of three motion commands.
 *
 * Test Sequence:
 *    1. Generate request with three trajectories and zero blend radius.
 *    2. Generate request with first trajectory and zero blend radius.
 *    3. Generate request with second trajectory and zero blend radius.
 *    4. Generate request with third trajectory and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *    2. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *    3. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *    4. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *       resulting duration in step1 is approximately step2 + step3 + step4
 */
TEST_F(IntegrationTestCommandListManager, concatThreeSegments)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  ASSERT_GE(seq.size(), 3u);
  seq.erase(3, seq.size());
  seq.setAllBlendRadiiToZero();

  RobotTrajCont res123_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res123_vec.size(), 1u);
  EXPECT_GT(res123_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res123_vec.front())) << "Time steps not strictly positively increasing";

  ROS_INFO("step 2: only first segment");
  moveit_msgs::MotionSequenceRequest req_1;
  req_1.items.resize(1);
  req_1.items.at(0).req = seq.getCmd(0).toRequest();
  req_1.items.at(0).blend_radius = 0.;
  RobotTrajCont res1_vec{ manager_->solve(scene_, pipeline_, req_1) };
  EXPECT_EQ(res1_vec.size(), 1u);
  EXPECT_GT(res1_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res1_vec.front()->getFirstWayPoint().getVariableCount(),
            req_1.items.at(0).req.start_state.joint_state.name.size());

  ROS_INFO("step 3: only second segment");
  moveit_msgs::MotionSequenceRequest req_2;
  req_2.items.resize(1);
  req_2.items.at(0).req = seq.getCmd(1).toRequest();
  req_2.items.at(0).blend_radius = 0.;
  RobotTrajCont res2_vec{ manager_->solve(scene_, pipeline_, req_2) };
  EXPECT_EQ(res2_vec.size(), 1u);
  EXPECT_GT(res2_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res2_vec.front()->getFirstWayPoint().getVariableCount(),
            req_2.items.at(0).req.start_state.joint_state.name.size());

  ROS_INFO("step 4: only third segment");
  moveit_msgs::MotionSequenceRequest req_3;
  req_3.items.resize(1);
  req_3.items.at(0).req = seq.getCmd(2).toRequest();
  req_3.items.at(0).blend_radius = 0.;
  RobotTrajCont res3_vec{ manager_->solve(scene_, pipeline_, req_3) };
  EXPECT_EQ(res3_vec.size(), 1u);
  EXPECT_GT(res3_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res3_vec.front()->getFirstWayPoint().getVariableCount(),
            req_3.items.at(0).req.start_state.joint_state.name.size());

  // durations for the different segments
  auto t1_2_3 = res123_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount() - 1);
  auto t1 = res1_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount() - 1);
  auto t2 = res2_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount() - 1);
  auto t3 = res3_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount() - 1);
  ROS_DEBUG_STREAM("total time: " << t1_2_3 << " t1:" << t1 << " t2:" << t2 << " t3:" << t3);
  EXPECT_LT(fabs((t1_2_3 - t1 - t2 - t3)), 0.4);
}

/**
 * @brief Tests if times are strictly increasing with selective blending
 *
 * Test Sequence:
 *    1. Generate request with three trajectories where only the first has a
 * blend radius.
 *    1. Generate request with three trajectories where only the second has a
 * blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly increasing in time
 *    2. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly increasing in time
 */
TEST_F(IntegrationTestCommandListManager, concatSegmentsSelectiveBlending)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  ASSERT_GE(seq.size(), 3u);
  seq.erase(3, seq.size());
  seq.setAllBlendRadiiToZero();
  seq.setBlendRadius(0, 0.1);
  RobotTrajCont res1{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res1.size(), 1u);
  EXPECT_GT(res1.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res1.front())) << "Time steps not strictly positively increasing";

  seq.setAllBlendRadiiToZero();
  seq.setBlendRadius(1, 0.1);
  RobotTrajCont res2{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res2.size(), 1u);
  EXPECT_GT(res2.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res2.front())) << "Time steps not strictly positively increasing";
}

/**
 * @brief Tests the concatenation of two ptp commands
 *
 * Test Sequence:
 *    1. Generate request with two PTP trajectories and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive.
 */
TEST_F(IntegrationTestCommandListManager, concatTwoPtpSegments)
{
  Sequence seq{ data_loader_->getSequence("PtpPtpSequence") };
  ASSERT_GE(seq.size(), 2u);
  seq.setAllBlendRadiiToZero();

  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_vec.size(), 1u);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res_vec.front()));
}

/**
 * @brief Tests the concatenation of ptp and a lin command
 *
 * Test Sequence:
 *    1. Generate request with PTP and LIN trajectory and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive.
 */
TEST_F(IntegrationTestCommandListManager, concatPtpAndLinSegments)
{
  Sequence seq{ data_loader_->getSequence("PtpLinSequence") };
  ASSERT_GE(seq.size(), 2u);
  seq.setAllBlendRadiiToZero();

  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_vec.size(), 1u);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res_vec.front()));
}

/**
 * @brief Tests the concatenation of a lin and a ptp command
 *
 * Test Sequence:
 *    1. Generate request with LIN and PTP trajectory and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive.
 */
TEST_F(IntegrationTestCommandListManager, concatLinAndPtpSegments)
{
  Sequence seq{ data_loader_->getSequence("LinPtpSequence") };
  ASSERT_GE(seq.size(), 2u);
  seq.setAllBlendRadiiToZero();

  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_vec.size(), 1u);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res_vec.front()));
}

/**
 * @brief Tests the blending of motion commands
 *
 *  - Test Sequence:
 *    1. Generate request with two trajectories and request blending.
 *
 *  - Expected Results:
 *    1. blending is successful, result trajectory is not empty
 */
TEST_F(IntegrationTestCommandListManager, blendTwoSegments)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  ASSERT_EQ(seq.size(), 2u);
  moveit_msgs::MotionSequenceRequest req{ seq.toRequest() };
  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, req) };
  EXPECT_EQ(res_vec.size(), 1u);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res_vec.front()->getFirstWayPoint().getVariableCount(),
            req.items.at(0).req.start_state.joint_state.name.size());

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("my_planned_path", 1);
  ros::Duration duration(1.0);  // wait to notify possible subscribers
  duration.sleep();

  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::RobotTrajectory rob_traj_msg;
  res_vec.front()->getRobotTrajectoryMsg(rob_traj_msg);
  display_trajectory.trajectory.push_back(rob_traj_msg);
  pub.publish(display_trajectory);
}

// ------------------
// FAILURE cases
// ------------------

/**
 * @brief Tests sending an empty blending request.
 *
 * Test Sequence:
 *    1. Generate empty request and request blending.
 *
 * Expected Results:
 *    1. blending is successful, result trajectory container is empty
 */
TEST_F(IntegrationTestCommandListManager, emptyList)
{
  moveit_msgs::MotionSequenceRequest empty_list;
  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, empty_list) };
  EXPECT_TRUE(res_vec.empty());
}

/**
 * @brief Checks that exception is thrown if first goal is not reachable.
 *
 * Test Sequence:
 *    1. Generate request with first goal out of workspace.
 *
 * Expected Results:
 *    1. Exception is thrown.
 */
TEST_F(IntegrationTestCommandListManager, firstGoalNotReachable)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  ASSERT_GE(seq.size(), 2u);
  LinCart& lin{ seq.getCmd<LinCart>(0) };
  lin.getGoalConfiguration().getPose().position.y = 2700;
  EXPECT_THROW(manager_->solve(scene_, pipeline_, seq.toRequest()), PlanningPipelineException);
}

/**
 * @brief Checks that exception is thrown if second goal has a start state.
 *
 * Test Sequence:
 *    1. Generate request, second goal has an invalid start state set.
 *
 * Expected Results:
 *    1. Exception is thrown.
 */
TEST_F(IntegrationTestCommandListManager, startStateNotFirstGoal)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  ASSERT_GE(seq.size(), 2u);
  const LinCart& lin{ seq.getCmd<LinCart>(0) };
  moveit_msgs::MotionSequenceRequest req{ seq.toRequest() };
  req.items.at(1).req.start_state = lin.getGoalConfiguration().toMoveitMsgsRobotState();
  EXPECT_THROW(manager_->solve(scene_, pipeline_, req), StartStateSetException);
}

/**
 * @brief Checks that exception is thrown in case of blending request with
 * negative blend_radius.
 *
 *  Test Sequence:
 *    1. Generate request, first goal has negative blend_radius.
 *
 *  Expected Results:
 *    1. Exception is thrown.
 */
TEST_F(IntegrationTestCommandListManager, blendingRadiusNegative)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  ASSERT_GE(seq.size(), 2u);
  seq.setBlendRadius(0, -0.3);
  EXPECT_THROW(manager_->solve(scene_, pipeline_, seq.toRequest()), NegativeBlendRadiusException);
}

/**
 * @brief Checks that exception is thrown if last blend radius is not zero.
 *
 *
 * Test Sequence:
 *    1. Generate request, second goal has non-zero blend_radius.
 *
 * Expected Results:
 *    1. Exception is thrown.
 */
TEST_F(IntegrationTestCommandListManager, lastBlendingRadiusNonZero)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  ASSERT_EQ(seq.size(), 2u);
  seq.setBlendRadius(1, 0.03);
  EXPECT_THROW(manager_->solve(scene_, pipeline_, seq.toRequest()), LastBlendRadiusNotZeroException);
}

/**
 * @brief Checks that exception is thrown if blend radius is greater than the
 * segment.
 *
 * Test Sequence:
 *    1. Generate request with huge blending radius, so that trajectories are
 *       completely inside
 *
 * Expected Results:
 *    2. Exception is thrown.
 */
TEST_F(IntegrationTestCommandListManager, blendRadiusGreaterThanSegment)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  ASSERT_GE(seq.size(), 2u);
  seq.setBlendRadius(0, 42.0);
  EXPECT_THROW(manager_->solve(scene_, pipeline_, seq.toRequest()), BlendingFailedException);
}

/**
 * @brief Checks that exception is thrown if two consecutive blend radii
 * overlap.
 *
 * Test Sequence:
 *    1. Generate request with three trajectories
 *    2. Increase second blend radius, so that the radii overlap
 *
 * Expected Results:
 *    1. blending succeeds, result trajectory is not empty
 *    2. Exception is thrown.
 */
TEST_F(IntegrationTestCommandListManager, blendingRadiusOverlapping)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  ASSERT_GE(seq.size(), 3u);
  seq.erase(3, seq.size());

  RobotTrajCont res_valid_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_valid_vec.size(), 1u);
  EXPECT_GT(res_valid_vec.front()->getWayPointCount(), 0u);

  // calculate distance from first to second goal
  const PtpJointCart& ptp{ seq.getCmd<PtpJointCart>(0) };
  const CircInterimCart& circ{ seq.getCmd<CircInterimCart>(1) };
  Eigen::Isometry3d p1, p2;
  tf2::fromMsg(ptp.getGoalConfiguration().getPose(), p1);
  tf2::fromMsg(circ.getGoalConfiguration().getPose(), p2);
  auto distance = (p2.translation() - p1.translation()).norm();

  seq.setBlendRadius(1, (distance - seq.getBlendRadius(0) + 0.01));  // overlapping radii
  EXPECT_THROW(manager_->solve(scene_, pipeline_, seq.toRequest()), OverlappingBlendRadiiException);
}

/**
 * @brief Tests if the planned execution time scales correctly with the number
 * of repetitions.
 *
 * Test Sequence:
 *    1. Generate trajectory and save calculated execution time.
 *    2. Generate request with repeated path along the points from Test Step 1
 *      (repeated two times).
 *
 * Expected Results:
 *    1. Blending succeeds, result trajectory is not empty.
 *    2. Blending succeeds, planned execution time should be approx N times
 *       the single planned execution time.
 */
TEST_F(IntegrationTestCommandListManager, TestExecutionTime)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  ASSERT_GE(seq.size(), 2u);
  RobotTrajCont res_single_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_single_vec.size(), 1u);
  EXPECT_GT(res_single_vec.front()->getWayPointCount(), 0u);

  moveit_msgs::MotionSequenceRequest req{ seq.toRequest() };
  // Create large request by making copies of the original sequence commands
  // and adding them to the end of the original sequence.
  const size_t n{ req.items.size() };
  for (size_t i = 0; i < n; ++i)
  {
    moveit_msgs::MotionSequenceItem item{ req.items.at(i) };
    if (i == 0)
    {
      // Remove start state because only the first request
      // is allowed to have a start state in a sequence.
      item.req.start_state = moveit_msgs::RobotState();
    }
    req.items.push_back(item);
  }

  RobotTrajCont res_n_vec{ manager_->solve(scene_, pipeline_, req) };
  EXPECT_EQ(res_n_vec.size(), 1u);
  EXPECT_GT(res_n_vec.front()->getWayPointCount(), 0u);

  const double trajectory_time_1 =
      res_single_vec.front()->getWayPointDurationFromStart(res_single_vec.front()->getWayPointCount() - 1);
  const double trajectory_time_n =
      res_n_vec.front()->getWayPointDurationFromStart(res_n_vec.front()->getWayPointCount() - 1);
  double multiplicator = req.items.size() / n;
  EXPECT_LE(trajectory_time_n, trajectory_time_1 * multiplicator);
  EXPECT_GE(trajectory_time_n, trajectory_time_1 * multiplicator * 0.5);
}

/**
 * @brief Tests if it possible to send requests which contain more than
 * one group.
 *
 * Please note: This test is still quite trivial. It does not check the
 * "correctness" of a calculated trajectory. It only checks that for each
 * group and group change there exists a calculated trajectory.
 *
 */
TEST_F(IntegrationTestCommandListManager, TestDifferentGroups)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequenceWithGripper") };
  ASSERT_GE(seq.size(), 1u);
  // Count the number of group changes in the given sequence
  unsigned int num_groups{ 1 };
  std::string last_group_name{ seq.getCmd(0).getPlanningGroup() };
  for (size_t i = 1; i < seq.size(); ++i)
  {
    if (seq.getCmd(i).getPlanningGroup() != last_group_name)
    {
      ++num_groups;
      last_group_name = seq.getCmd(i).getPlanningGroup();
    }
  }

  RobotTrajCont res_single_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_single_vec.size(), num_groups);

  for (const auto& res : res_single_vec)
  {
    EXPECT_GT(res->getWayPointCount(), 0u);
  }
}

/**
 * @brief Checks that no exception is thrown if two gripper commands are
 * blended.
 *
 */
TEST_F(IntegrationTestCommandListManager, TestGripperCmdBlending)
{
  Sequence seq{ data_loader_->getSequence("PureGripperSequence") };
  ASSERT_GE(seq.size(), 2u);
  ASSERT_TRUE(seq.cmdIsOfType<Gripper>(0));
  ASSERT_TRUE(seq.cmdIsOfType<Gripper>(1));

  // Ensure that blending is requested for gripper commands.
  seq.setBlendRadius(0, 1.0);
  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_EQ(res_vec.size(), 1u);
}

/**
 * @brief Tests the execution of a sequence in which each group states a start
 * state only consisting of joints of the corresonding group.
 *
 * Test Sequence:
 *    1. Create sequence request for which each start state only consists of
 *        joints of the corresonding group
 *
 * Expected Results:
 *    1. Trajectory generation is successful, result trajectory is not empty.
 *
 *
 */
TEST_F(IntegrationTestCommandListManager, TestGroupSpecificStartState)
{
  using std::placeholders::_1;

  Sequence seq{ data_loader_->getSequence("ComplexSequenceWithGripper") };
  ASSERT_GE(seq.size(), 4u);
  seq.erase(4, seq.size());

  Gripper& gripper{ seq.getCmd<Gripper>(0) };
  gripper.getStartConfiguration().setCreateJointNameFunc([](size_t n) { return createGripperJointName(n); });
  // By deleting the model we guarantee that the start state only consists
  // of joints of the gripper group without the manipulator
  gripper.getStartConfiguration().clearModel();

  PtpJointCart& ptp{ seq.getCmd<PtpJointCart>(1) };
  ptp.getStartConfiguration().setCreateJointNameFunc([](size_t n) { return createManipulatorJointName(n); });
  // By deleting the model we guarantee that the start state only consists
  // of joints of the manipulator group without the gripper
  ptp.getStartConfiguration().clearModel();

  RobotTrajCont res_vec{ manager_->solve(scene_, pipeline_, seq.toRequest()) };
  EXPECT_GE(res_vec.size(), 1u);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
}

/**
 * @brief Checks that exception is thrown if Tip-Frame is requested for
 * a group without a solver.
 */
TEST_F(IntegrationTestCommandListManager, TestGetSolverTipFrameForSolverlessGroup)
{
  Gripper gripper_cmd{ data_loader_->getGripper("open_gripper") };
  EXPECT_THROW(getSolverTipFrame(robot_model_->getJointModelGroup(gripper_cmd.getPlanningGroup())), NoSolverException);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_command_list_manager");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
