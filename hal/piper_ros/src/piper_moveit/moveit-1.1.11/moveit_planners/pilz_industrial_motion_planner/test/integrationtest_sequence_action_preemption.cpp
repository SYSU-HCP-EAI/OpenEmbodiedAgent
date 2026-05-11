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

#include <chrono>
#include <condition_variable>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/JointConstraint.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <pilz_industrial_motion_planner_testutils/async_test.h>

#include <pilz_industrial_motion_planner_testutils/checks.h>
#include <pilz_industrial_motion_planner_testutils/sequence.h>
#include <pilz_industrial_motion_planner_testutils/xml_testdata_loader.h>

#include "moveit_msgs/MoveGroupSequenceAction.h"

static constexpr double WAIT_FOR_RESULT_TIME_OUT{ 5. };          // seconds
static constexpr double TIME_BEFORE_CANCEL_GOAL{ 1.0 };          // seconds
static constexpr double WAIT_FOR_ACTION_SERVER_TIME_OUT{ 10. };  // seconds

const std::string SEQUENCE_ACTION_NAME("/sequence_move_group");

// Parameters from parameter server
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");

// events for callback tests
const std::string GOAL_SUCCEEDED_EVENT = "GOAL_SUCCEEDED";
const std::string SERVER_IDLE_EVENT = "SERVER_IDLE";

const std::string TEST_DATA_FILE_NAME("testdata_file_name");
const std::string GROUP_NAME("group_name");

using namespace pilz_industrial_motion_planner_testutils;

class IntegrationTestSequenceAction : public testing::Test, public testing::AsyncTest
{
protected:
  void SetUp() override;

public:
  MOCK_METHOD0(active_callback, void());
  MOCK_METHOD1(feedback_callback, void(const moveit_msgs::MoveGroupSequenceFeedbackConstPtr& feedback));
  MOCK_METHOD2(done_callback, void(const actionlib::SimpleClientGoalState& state,
                                   const moveit_msgs::MoveGroupSequenceResultConstPtr& result));

protected:
  ros::NodeHandle ph_{ "~" };
  actionlib::SimpleActionClient<moveit_msgs::MoveGroupSequenceAction> ac_{ ph_, SEQUENCE_ACTION_NAME, true };
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  robot_model_loader::RobotModelLoader model_loader_;
  robot_model::RobotModelPtr robot_model_;
  double joint_position_tolerance_;

  std::string test_data_file_name_;
  std::string group_name_;
  TestdataLoaderUPtr data_loader_;

  //! The configuration at which the robot stays at the beginning of each test.
  JointConfiguration start_config;
};

void IntegrationTestSequenceAction::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));
  ph_.param<std::string>(GROUP_NAME, group_name_, "manipulator");

  robot_model_ = model_loader_.getModel();

  data_loader_ = std::make_unique<XmlTestdataLoader>(test_data_file_name_, robot_model_);
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  // wait for action server
  ASSERT_TRUE(ac_.waitForServer(ros::Duration(WAIT_FOR_ACTION_SERVER_TIME_OUT))) << "Action server is not active.";

  // move to default position
  start_config = data_loader_->getJoints("ZeroPose", group_name_);
  robot_state::RobotState robot_state{ start_config.toRobotState() };

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(start_config.getGroupName());
  move_group_->setPlannerId("PTP");
  move_group_->setGoalTolerance(joint_position_tolerance_);
  move_group_->setJointValueTarget(robot_state);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->move();

  ASSERT_TRUE(isAtExpectedPosition(robot_state, *(move_group_->getCurrentState()), joint_position_tolerance_));
}

/**
 * @brief Tests that goal can be cancelled.
 *
 * Test Sequence:
 *    1. Send goal for planning and execution.
 *    2. Cancel goal before it finishes.
 *
 * Expected Results:
 *    1. Goal is sent to the action server.
 *    2. Goal is cancelled. Execution stops.
 */
TEST_F(IntegrationTestSequenceAction, TestCancellingOfGoal)
{
  Sequence seq{ data_loader_->getSequence("ComplexSequence") };

  moveit_msgs::MoveGroupSequenceGoal seq_goal;
  seq_goal.request = seq.toRequest();

  ac_.sendGoal(seq_goal);
  // wait for 1 second
  ros::Duration(TIME_BEFORE_CANCEL_GOAL).sleep();

  ac_.cancelGoal();
  ac_.waitForResult(ros::Duration(WAIT_FOR_RESULT_TIME_OUT));

  moveit_msgs::MoveGroupSequenceResultConstPtr res = ac_.getResult();
  EXPECT_EQ(res->response.error_code.val, moveit_msgs::MoveItErrorCodes::PREEMPTED)
      << "Error code should be preempted.";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_sequence_action_preemption");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
