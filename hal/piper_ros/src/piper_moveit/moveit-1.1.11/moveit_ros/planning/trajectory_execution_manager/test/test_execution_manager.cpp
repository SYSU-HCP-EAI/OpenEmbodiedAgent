/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Cristian C. Beltran
   Desc: Test the TrajectoryExecutionManager with MoveitCpp
*/

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Main class
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
// Msgs
#include <geometry_msgs/PointStamped.h>

namespace moveit_cpp
{
class MoveItCppTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle();
    moveit_cpp_ptr = std::make_shared<MoveItCpp>(nh_);
    trajectory_execution_manager_ptr = moveit_cpp_ptr->getTrajectoryExecutionManager();

    traj1.joint_trajectory.joint_names.push_back("panda_joint1");
    traj1.joint_trajectory.points.resize(1);
    traj1.joint_trajectory.points[0].positions.push_back(0.0);

    traj2 = traj1;
    traj2.joint_trajectory.joint_names.push_back("panda_joint2");
    traj2.joint_trajectory.points[0].positions.push_back(1.0);
    traj2.multi_dof_joint_trajectory.joint_names.push_back("panda_joint3");
    traj2.multi_dof_joint_trajectory.points.resize(1);
    traj2.multi_dof_joint_trajectory.points[0].transforms.resize(1);
  }

protected:
  ros::NodeHandle nh_;
  MoveItCppPtr moveit_cpp_ptr;
  PlanningComponentPtr planning_component_ptr;
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_ptr;
  moveit_msgs::RobotTrajectory traj1;
  moveit_msgs::RobotTrajectory traj2;
};

TEST_F(MoveItCppTest, EnsureActiveControllersForJointsTest)
{
  ASSERT_TRUE(trajectory_execution_manager_ptr->ensureActiveControllersForJoints({ "panda_joint1" }));
}

TEST_F(MoveItCppTest, ensureActiveControllerTest)
{
  ASSERT_TRUE(trajectory_execution_manager_ptr->ensureActiveController("fake_panda_arm_controller"));
}

TEST_F(MoveItCppTest, ExecuteEmptySetOfTrajectoriesTest)
{
  // execute with empty set of trajectories
  trajectory_execution_manager_ptr->execute();
  auto last_execution_status = trajectory_execution_manager_ptr->waitForExecution();
  ASSERT_EQ(last_execution_status, moveit_controller_manager::ExecutionStatus::SUCCEEDED);
}

TEST_F(MoveItCppTest, PushExecuteAndWaitTest)
{
  ASSERT_TRUE(trajectory_execution_manager_ptr->push(traj1));
  ASSERT_TRUE(trajectory_execution_manager_ptr->push(traj2));
  traj1.multi_dof_joint_trajectory = traj2.multi_dof_joint_trajectory;
  ASSERT_TRUE(trajectory_execution_manager_ptr->push(traj1));
  auto last_execution_status = trajectory_execution_manager_ptr->executeAndWait();
  ASSERT_EQ(last_execution_status, moveit_controller_manager::ExecutionStatus::SUCCEEDED);
}

}  // namespace moveit_cpp

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_execution_manager");

  int result = RUN_ALL_TESTS();

  return result;
}
