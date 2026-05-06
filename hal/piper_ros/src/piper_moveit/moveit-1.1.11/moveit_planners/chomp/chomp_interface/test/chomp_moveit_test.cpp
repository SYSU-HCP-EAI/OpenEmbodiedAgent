/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Heriot-Watt University, Edinburgh Centre for Robotics
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
 *   * Neither the name of Willow Garage nor the names of its
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

/// \author Bence Magyar

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class CHOMPMoveitTest : public ::testing::Test
{
public:
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_;

public:
  CHOMPMoveitTest() : move_group_("arm")
  {
  }
};

// TEST CASES
TEST_F(CHOMPMoveitTest, jointSpaceGoodGoal)
{
  move_group_.setStartState(*(move_group_.getCurrentState()));
  move_group_.setJointValueTarget(std::vector<double>({ 1.0, 1.0 }));

  moveit::core::MoveItErrorCode error_code = move_group_.plan(my_plan_);
  EXPECT_GT(my_plan_.trajectory_.joint_trajectory.points.size(), 0u);
  EXPECT_EQ(error_code.val, moveit::core::MoveItErrorCode::SUCCESS);
}

TEST_F(CHOMPMoveitTest, jointSpaceBadGoal)
{
  move_group_.setStartState(*(move_group_.getCurrentState()));
  // joint2 is limited to [-PI/2, PI/2]
  move_group_.setJointValueTarget(std::vector<double>({ 100.0, 2 * M_PI / 3.0 }));

  moveit::core::MoveItErrorCode error_code = move_group_.plan(my_plan_);
  EXPECT_EQ(error_code.val, moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE);
}

TEST_F(CHOMPMoveitTest, cartesianGoal)
{
  move_group_.setStartState(*(move_group_.getCurrentState()));
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 10000.;
  target_pose1.position.y = 10000.;
  target_pose1.position.z = 10000.;
  move_group_.setPoseTarget(target_pose1);

  moveit::core::MoveItErrorCode error_code = move_group_.plan(my_plan_);
  // CHOMP doesn't support Cartesian-space goals at the moment
  EXPECT_EQ(error_code.val, moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS);
}

TEST_F(CHOMPMoveitTest, noStartState)
{
  move_group_.setJointValueTarget(std::vector<double>({ 0.2, 0.2 }));

  moveit::core::MoveItErrorCode error_code = move_group_.plan(my_plan_);
  EXPECT_EQ(error_code.val, moveit::core::MoveItErrorCode::SUCCESS);
}

TEST_F(CHOMPMoveitTest, collisionAtEndOfPath)
{
  move_group_.setStartState(*(move_group_.getCurrentState()));
  move_group_.setJointValueTarget(std::vector<double>({ M_PI / 2.0, 0 }));

  moveit::core::MoveItErrorCode error_code = move_group_.plan(my_plan_);
  EXPECT_EQ(error_code.val, moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "chomp_moveit_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
