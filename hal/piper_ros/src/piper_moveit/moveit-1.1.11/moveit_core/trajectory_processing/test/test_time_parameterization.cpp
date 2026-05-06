/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
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

/* Author: Ken Anderson */

#include <gtest/gtest.h>
#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/utils/robot_model_test_utils.h>

// Static variables used in all tests
moveit::core::RobotModelConstPtr RMODEL = moveit::core::loadTestingRobotModel("pr2");
robot_trajectory::RobotTrajectory TRAJECTORY(RMODEL, "right_arm");

// Initialize one-joint, 3 points exactly the same.
int initRepeatedPointTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const int num = 3;
  unsigned i;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED("trajectory_processing", "Need to set the group");
    return -1;
  }
  // leave initial velocity/acceleration unset
  const std::vector<int>& idx = group->getVariableIndexList();
  moveit::core::RobotState state(trajectory.getRobotModel());

  trajectory.clear();
  for (i = 0; i < num; i++)
  {
    state.setVariablePosition(idx[0], 1.0);
    trajectory.addSuffixWayPoint(state, 0.0);
  }

  return 0;
}

// Initialize one-joint, straight-line trajectory
int initStraightTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const int num = 10;
  const double max = 2.0;
  unsigned i;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED("trajectory_processing", "Need to set the group");
    return -1;
  }
  // leave initial velocity/acceleration unset
  const std::vector<int>& idx = group->getVariableIndexList();
  moveit::core::RobotState state(trajectory.getRobotModel());

  trajectory.clear();
  for (i = 0; i < num; i++)
  {
    state.setVariablePosition(idx[0], i * max / num);
    trajectory.addSuffixWayPoint(state, 0.0);
  }

  // leave final velocity/acceleration unset
  state.setVariablePosition(idx[0], max);
  trajectory.addSuffixWayPoint(state, 0.0);

  return 0;
}

void printTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  const std::vector<int>& idx = group->getVariableIndexList();
  unsigned int count = trajectory.getWayPointCount();

  std::cout << "trajectory length is " << trajectory.getWayPointDurationFromStart(count - 1) << " seconds."
            << std::endl;
  std::cout << "  Trajectory Points" << std::endl;
  for (unsigned i = 0; i < count; i++)
  {
    moveit::core::RobotStatePtr point = trajectory.getWayPointPtr(i);
    printf("  waypoint %2d time %6.2f pos %6.2f vel %6.2f acc %6.2f ", i, trajectory.getWayPointDurationFromStart(i),
           point->getVariablePosition(idx[0]), point->getVariableVelocity(idx[0]),
           point->getVariableAcceleration(idx[0]));
    if (i > 0)
    {
      moveit::core::RobotStatePtr prev = trajectory.getWayPointPtr(i - 1);
      printf("jrk %6.2f",
             (point->getVariableAcceleration(idx[0]) - prev->getVariableAcceleration(idx[0])) /
                 (trajectory.getWayPointDurationFromStart(i) - trajectory.getWayPointDurationFromStart(i - 1)));
    }
    printf("\n");
  }
}

TEST(TestTimeParameterization, TestIterativeParabolic)
{
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  EXPECT_EQ(initStraightTrajectory(TRAJECTORY), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(TRAJECTORY));
  std::cout << "IterativeParabolicTimeParameterization  took " << (ros::WallTime::now() - wt).toSec() << std::endl;
  printTrajectory(TRAJECTORY);
  ASSERT_LT(TRAJECTORY.getWayPointDurationFromStart(TRAJECTORY.getWayPointCount() - 1), 3.0);
}

TEST(TestTimeParameterization, TestIterativeSpline)
{
  trajectory_processing::IterativeSplineParameterization time_parameterization(false);
  EXPECT_EQ(initStraightTrajectory(TRAJECTORY), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(TRAJECTORY));
  std::cout << "IterativeSplineParameterization took " << (ros::WallTime::now() - wt).toSec() << std::endl;
  printTrajectory(TRAJECTORY);
  ASSERT_LT(TRAJECTORY.getWayPointDurationFromStart(TRAJECTORY.getWayPointCount() - 1), 5.0);
}

TEST(TestTimeParameterization, TestIterativeSplineAddPoints)
{
  trajectory_processing::IterativeSplineParameterization time_parameterization(true);
  EXPECT_EQ(initStraightTrajectory(TRAJECTORY), 0);

  ros::WallTime wt = ros::WallTime::now();
  EXPECT_TRUE(time_parameterization.computeTimeStamps(TRAJECTORY));
  std::cout << "IterativeSplineParameterization with added points took " << (ros::WallTime::now() - wt).toSec()
            << std::endl;
  printTrajectory(TRAJECTORY);
  ASSERT_LT(TRAJECTORY.getWayPointDurationFromStart(TRAJECTORY.getWayPointCount() - 1), 5.0);
}

TEST(TestTimeParameterization, TestRepeatedPoint)
{
  trajectory_processing::IterativeSplineParameterization time_parameterization(true);
  EXPECT_EQ(initRepeatedPointTrajectory(TRAJECTORY), 0);

  EXPECT_TRUE(time_parameterization.computeTimeStamps(TRAJECTORY));
  printTrajectory(TRAJECTORY);
  ASSERT_LT(TRAJECTORY.getWayPointDurationFromStart(TRAJECTORY.getWayPointCount() - 1), 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
