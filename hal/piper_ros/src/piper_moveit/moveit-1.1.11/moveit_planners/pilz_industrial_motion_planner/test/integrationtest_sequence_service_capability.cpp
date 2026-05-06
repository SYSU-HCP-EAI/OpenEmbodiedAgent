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

#include <functional>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <ros/ros.h>

#include <pilz_industrial_motion_planner_testutils/sequence.h>
#include <pilz_industrial_motion_planner_testutils/xml_testdata_loader.h>

#include "moveit_msgs/GetMotionSequence.h"
#include "moveit_msgs/MotionSequenceRequest.h"
#include "pilz_industrial_motion_planner/capability_names.h"

// Parameters from parameter server
const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using namespace pilz_industrial_motion_planner_testutils;

static std::string createJointName(const size_t& joint_number)
{
  return std::string("prbt_joint_") + std::to_string(joint_number + 1);
}

class IntegrationTestSequenceService : public ::testing::Test
{
protected:
  void SetUp() override;

protected:
  ros::NodeHandle ph_{ "~" };
  ros::ServiceClient client_;
  robot_model::RobotModelPtr robot_model_;

  std::string test_data_file_name_;
  TestdataLoaderUPtr data_loader_;
};

void IntegrationTestSequenceService::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  robot_model_loader::RobotModelLoader model_loader;
  robot_model_ = model_loader.getModel();

  data_loader_ = std::make_unique<XmlTestdataLoader>(test_data_file_name_, robot_model_);
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  ASSERT_TRUE(ros::service::waitForService(pilz_industrial_motion_planner::SEQUENCE_SERVICE_NAME, ros::Duration(10)))
      << "Service not available.";
  ros::NodeHandle nh;  // connect to service in global namespace, not in ph_
  client_ = nh.serviceClient<moveit_msgs::GetMotionSequence>(pilz_industrial_motion_planner::SEQUENCE_SERVICE_NAME);
}

/**
 * @brief Test behavior of service when empty sequence is sent.
 *
 *  Test Sequence:
 *    1. Generate empty request and call sequence service.
 *    2. Evaluate the result.
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command is successful, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestSendingOfEmptySequence)
{
  moveit_msgs::MotionSequenceRequest empty_list;

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = empty_list;

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.response.error_code.val) << "Planning failed.";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests that invalid (differing) group names are detected.
 *
 * Test Sequence:
 *    1. Generate request, first request has invalid group_name +  Call sequence
 * service.
 *    2. Invalidate first request (change group_name) and send goal for planning
 * and execution.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestDifferingGroupNames)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  MotionCmd& cmd{ seq.getCmd(1) };
  cmd.setPlanningGroup("WrongGroupName");

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME, srv.response.response.error_code.val)
      << "Planning should have failed but did not.";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests that negative blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with negative blend_radius +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestNegativeBlendRadius)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  seq.setBlendRadius(0, -1.0);

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, srv.response.response.error_code.val)
      << "Planning should have failed but did not.";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests that overlapping blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with overlapping blend radii +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestOverlappingBlendRadii)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  seq.setBlendRadius(0, 10 * seq.getBlendRadius(0));

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, srv.response.response.error_code.val)
      << "Incorrect error code";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests that too large blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with too large blend radii +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestTooLargeBlendRadii)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  seq.erase(2, seq.size());
  seq.setBlendRadius(0, 10 * seq.getBlendRadius(seq.size() - 2));

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::FAILURE, srv.response.response.error_code.val) << "Incorrect error code";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests behavior of service when sequence with invalid second
 * start state is sent.
 *
 *  Test Sequence:
 *    1. Generate request (second goal has invalid start state) +  Call sequence
 * service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestSecondTrajInvalidStartState)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  moveit_msgs::MotionSequenceRequest req_list{ seq.toRequest() };

  // Set start state
  using std::placeholders::_1;
  JointConfiguration config{ "MyGroupName", { -1., 2., -3., 4., -5., 0. }, [](size_t n) { return createJointName(n); } };
  req_list.items[1].req.start_state.joint_state = config.toSensorMsg();

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = req_list;

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE, srv.response.response.error_code.val)
      << "Incorrect error code.";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests behavior of service when sequence with invalid first goal
 * is sent.
 *
 *  Test Sequence:
 *    1. Generate request with first goal out of workspace +  Call sequence
 * service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestFirstGoalNotReachable)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  PtpJointCart& cmd{ seq.getCmd<PtpJointCart>(0) };
  cmd.getGoalConfiguration().getPose().position.y = 27;

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION, srv.response.response.error_code.val)
      << "Incorrect error code.";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests that incorrect link_names are detected.
 *
 * Test Sequence:
 *    1. Create sequence and send it.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestInvalidLinkName)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  seq.setAllBlendRadiiToZero();

  // Invalidate link name
  CircInterimCart& circ{ seq.getCmd<CircInterimCart>(1) };
  circ.getGoalConfiguration().setLinkName("InvalidLinkName");

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_NE(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.response.error_code.val) << "Incorrect error code.";
  EXPECT_TRUE(srv.response.response.planned_trajectories.empty());
}

/**
 * @brief Tests the execution of a sequence with more than two commands.
 *
 * Test Sequence:
 *    1. Call service with serveral requests.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command succeeds, result trajectory is not empty.
 */
TEST_F(IntegrationTestSequenceService, TestLargeRequest)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };
  moveit_msgs::MotionSequenceRequest req{ seq.toRequest() };
  // Make copy of sequence commands and add them to the end of sequence.
  // Create large request by making copies of the original sequence commands
  // and adding them to the end of the original sequence.
  size_t n{ req.items.size() };
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

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = req;

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(srv.response.response.planned_trajectories.size(), 1u);
  EXPECT_GT(srv.response.response.planned_trajectories.front().joint_trajectory.points.size(), 0u)
      << "Trajectory should contain points.";
}

/**
 * @brief Tests the execution of a sequence command (without blending)
 * consisting of most of the possible command type combination.
 *
 * Test Sequence:
 *    1. Create sequence goal and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceService, TestComplexSequenceWithoutBlending)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };

  seq.setAllBlendRadiiToZero();

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(srv.response.response.planned_trajectories.size(), 1u);
  EXPECT_GT(srv.response.response.planned_trajectories.front().joint_trajectory.points.size(), 0u)
      << "Trajectory should contain points.";
}

/**
 * @brief Tests the execution of a sequence command (with blending)
 * consisting of most of the possible command type combination.
 *
 * Test Sequence:
 *    1. Create sequence goal and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceService, TestComplexSequenceWithBlending)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };

  moveit_msgs::GetMotionSequence srv;
  srv.request.request = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, srv.response.response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(srv.response.response.planned_trajectories.size(), 1u);
  EXPECT_GT(srv.response.response.planned_trajectories.front().joint_trajectory.points.size(), 0u)
      << "Trajectory should contain points.";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_sequence_service_capability");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
