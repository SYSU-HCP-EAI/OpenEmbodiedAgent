/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer */

#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

namespace ompl_interface_testing
{
/** \brief Robot independent test class setup
 *
 * The desired robot tests can be impelmented in a derived class in a generic way.
 *
 * It is implemented this way to avoid the ros specific test framework
 * outside moveit_ros.
 *
 * (This is an (uglier) alternative to using the rostest framework
 * and reading the robot settings from the parameter server.
 * Then we have several rostest launch files that load the parameters
 * for a specific robot and run the same compiled tests for all robots.)
 *
 * based on
 * https://stackoverflow.com/questions/38207346/specify-constructor-arguments-for-a-google-test-fixture/38218657
 * (answer by PiotrNycz)
 *
 *   --- example.cpp ---
 *
 *   #include <gtest/gtest.h>
 *   #include "load_test_robot.h"
 *
 *   class GenericTests : public ompl_interface_testing::LoadTestRobot, public testing::Test
 *   {
 *   public:
 *     GenericTests(const std::string& robot_name, const std::string& group_name)
 *        : LoadTestRobot(robot_name, group_name) { }
 *     void SetUp() override { }
 *     void TearDown() override  { }
 *
 *     void someTest()
 *     {
 *       // use robot_model_, robot_state_, .. here
 *       EXPECT_TRUE(true);
 *     }
 *   };
 *
 *   // now you can run `someTest()` on different robots:
 *
 *   class PandaTest : public GenericTests
 *   {
 *   protected:
 *     PandaTest() : GenericTests("panda", "panda_arm") { }
 *   };
 *
 *   TEST_F(PandaTest, someTest)
 *   {
 *     someTest();
 *   }
 *
 *   --- end example.cpp ---
 *
 *
 * */
class LoadTestRobot
{
protected:
  LoadTestRobot(const std::string& robot_name, const std::string& group_name)
    : group_name_(group_name), robot_name_(robot_name)
  {
    // load robot
    robot_model_ = moveit::core::loadTestingRobotModel(robot_name_);
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    joint_model_group_ = robot_state_->getJointModelGroup(group_name_);

    // extract useful parameters for tests
    num_dofs_ = joint_model_group_->getVariableCount();
    ee_link_name_ = joint_model_group_->getLinkModelNames().back();
    base_link_name_ = robot_model_->getRootLinkName();

    ROS_INFO_STREAM("Created test robot named: " << robot_name_ << " for planning group " << group_name_);
  }

protected:
  const std::string group_name_;
  const std::string robot_name_;

  moveit::core::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  std::size_t num_dofs_;
  std::string base_link_name_;
  std::string ee_link_name_;
};
}  // namespace ompl_interface_testing
