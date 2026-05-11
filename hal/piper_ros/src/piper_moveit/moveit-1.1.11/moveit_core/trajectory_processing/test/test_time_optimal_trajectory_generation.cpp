/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/utils/robot_model_test_utils.h>

using trajectory_processing::Path;
using trajectory_processing::TimeOptimalTrajectoryGeneration;
using trajectory_processing::Trajectory;

TEST(time_optimal_trajectory_generation, test1)
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1424.0, 984.999694824219, 2126.0, 0.0;
  waypoints.push_back(waypoint);
  waypoint << 1423.0, 985.000244140625, 2126.0, 0.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.00249, 0.00249, 0.00249, 0.00249;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(40.080256821829849, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1424.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(984.999694824219, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(2126.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(0.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(1423.0, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(985.000244140625, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(2126.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(0.0, trajectory.getPosition(trajectory.getDuration())[3]);
}

TEST(time_optimal_trajectory_generation, test2)
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations, 10.0);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(1922.1418427445944, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1427.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(368.0, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(690.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(452.5, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(533.0, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(951.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(trajectory.getDuration())[3]);
}

TEST(time_optimal_trajectory_generation, test3)
{
  Eigen::VectorXd waypoint(4);
  std::list<Eigen::VectorXd> waypoints;

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  waypoints.push_back(waypoint);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  waypoints.push_back(waypoint);

  Eigen::VectorXd max_velocities(4);
  max_velocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd max_accelerations(4);
  max_accelerations << 0.002, 0.002, 0.002, 0.002;

  Trajectory trajectory(Path(waypoints, 100.0), max_velocities, max_accelerations);
  EXPECT_TRUE(trajectory.isValid());
  EXPECT_DOUBLE_EQ(1919.5597888812974, trajectory.getDuration());

  // Test start matches
  EXPECT_DOUBLE_EQ(1427.0, trajectory.getPosition(0.0)[0]);
  EXPECT_DOUBLE_EQ(368.0, trajectory.getPosition(0.0)[1]);
  EXPECT_DOUBLE_EQ(690.0, trajectory.getPosition(0.0)[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(0.0)[3]);

  // Test end matches
  EXPECT_DOUBLE_EQ(452.5, trajectory.getPosition(trajectory.getDuration())[0]);
  EXPECT_DOUBLE_EQ(533.0, trajectory.getPosition(trajectory.getDuration())[1]);
  EXPECT_DOUBLE_EQ(951.0, trajectory.getPosition(trajectory.getDuration())[2]);
  EXPECT_DOUBLE_EQ(90.0, trajectory.getPosition(trajectory.getDuration())[3]);
}

// Test that totg algorithm doesn't give large acceleration
TEST(time_optimal_trajectory_generation, testLargeAccel)
{
  double path_tolerance = 0.1;
  double resample_dt = 0.1;
  Eigen::VectorXd waypoint(6);
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd max_velocities(6);
  Eigen::VectorXd max_accelerations(6);

  // Waypoints
  // clang-format off
  waypoint << 1.6113056281076339,
             -0.21400163389235427,
             -1.974502599739185,
              9.9653618690354051e-12,
             -1.3810916877429624,
              1.5293902838041467;
  waypoints.push_back(waypoint);

  waypoint << 1.6088016187976597,
             -0.21792862470933924,
             -1.9758628799742952,
              0.00010424017303217738,
             -1.3835690515335755,
              1.5279972853269816;
  waypoints.push_back(waypoint);

  waypoint << 1.5887695443178671,
             -0.24934455124521923,
             -1.9867451218551782,
              0.00093816147756670078,
             -1.4033879618584812,
              1.5168532975096607;
  waypoints.push_back(waypoint);

  waypoint << 1.1647412393815282,
             -0.91434018564402375,
             -2.2170946337498498,
              0.018590164397622583,
             -1.8229041212673529,
              1.2809632867583278;
  waypoints.push_back(waypoint);

  // Max velocities
  max_velocities << 0.89535390627300004,
                    0.89535390627300004,
                    0.79587013890930003,
                    0.92022484811399996,
                    0.82074108075029995,
                    1.3927727430915;
  // Max accelerations
  max_accelerations << 0.82673490883799994,
                       0.78539816339699997,
                       0.60883578557700002,
                       3.2074759432319997,
                       1.4398966328939999,
                       4.7292792634680003;
  // clang-format on

  Trajectory parameterized(Path(waypoints, path_tolerance), max_velocities, max_accelerations, 0.001);

  ASSERT_TRUE(parameterized.isValid());

  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt);
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);

    ASSERT_EQ(acceleration.size(), 6);
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(acceleration(i), 0.0, 100.0) << "Invalid acceleration at position " << sample_count << "\n";
  }
}

TEST(time_optimal_trajectory_generation, testPluginAPI)
{
  constexpr auto robot_name{ "panda" };
  constexpr auto group_name{ "panda_arm" };

  auto robot_model = moveit::core::loadTestingRobotModel(robot_name);
  ASSERT_TRUE((bool)robot_model) << "Failed to load robot model" << robot_name;
  auto group = robot_model->getJointModelGroup(group_name);
  ASSERT_TRUE((bool)group) << "Failed to load joint model group " << group_name;
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state.setToDefaultValues();

  robot_trajectory::RobotTrajectory trajectory(robot_model, group);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ 0.0, -3.5, 1.4, -1.2, -1.0, -0.2, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ 0.0, -3.5, 1.4, -1.2, -1.0, -0.2, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);
  waypoint_state.setJointGroupPositions(group, std::vector<double>{ -0.5, -3.52, 1.35, -2.51, -0.88, 0.63, 0.0 });
  trajectory.addSuffixWayPoint(waypoint_state, 0.1);

  // Number TOTG iterations
  constexpr size_t totg_iterations = 10;

  // Test computing the dynamics repeatedly with the same totg instance
  moveit_msgs::RobotTrajectory first_trajectory_msg_start, first_trajectory_msg_end;
  {
    robot_trajectory::RobotTrajectory test_trajectory(trajectory, true /* deep copy */);

    // Test if the trajectory was copied correctly
    ASSERT_EQ(test_trajectory.getDuration(), trajectory.getDuration());
    moveit::core::JointBoundsVector test_bounds = test_trajectory.getRobotModel()->getActiveJointModelsBounds();
    moveit::core::JointBoundsVector original_bounds = trajectory.getRobotModel()->getActiveJointModelsBounds();
    ASSERT_EQ(test_bounds.size(), original_bounds.size());
    for (size_t bound_idx = 0; bound_idx < test_bounds.at(0)->size(); ++bound_idx)
    {
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).max_velocity_, original_bounds.at(0)->at(bound_idx).max_velocity_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).min_velocity_, original_bounds.at(0)->at(bound_idx).min_velocity_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).velocity_bounded_,
                original_bounds.at(0)->at(bound_idx).velocity_bounded_);

      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).max_acceleration_,
                original_bounds.at(0)->at(bound_idx).max_acceleration_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).min_acceleration_,
                original_bounds.at(0)->at(bound_idx).min_acceleration_);
      ASSERT_EQ(test_bounds.at(0)->at(bound_idx).acceleration_bounded_,
                original_bounds.at(0)->at(bound_idx).acceleration_bounded_);
    }
    ASSERT_EQ(test_trajectory.getWayPointDurationFromPrevious(1), trajectory.getWayPointDurationFromPrevious(1));

    TimeOptimalTrajectoryGeneration totg;
    ASSERT_TRUE(totg.computeTimeStamps(test_trajectory)) << "Failed to compute time stamps";

    test_trajectory.getRobotTrajectoryMsg(first_trajectory_msg_start);

    // Iteratively recompute timestamps with same totg instance
    for (size_t i = 0; i < totg_iterations; ++i)
    {
      bool totg_success = totg.computeTimeStamps(test_trajectory);
      EXPECT_TRUE(totg_success) << "Failed to compute time stamps with a same TOTG instance in iteration " << i;
    }

    test_trajectory.getRobotTrajectoryMsg(first_trajectory_msg_end);
  }

  // Test computing the dynamics repeatedly with one TOTG instance per call
  moveit_msgs::RobotTrajectory second_trajectory_msg_start, second_trajectory_msg_end;
  {
    robot_trajectory::RobotTrajectory test_trajectory(trajectory, true /* deep copy */);

    {
      TimeOptimalTrajectoryGeneration totg;
      ASSERT_TRUE(totg.computeTimeStamps(test_trajectory)) << "Failed to compute time stamps";
    }

    test_trajectory.getRobotTrajectoryMsg(second_trajectory_msg_start);

    // Iteratively recompute timestamps with new totg instances
    for (size_t i = 0; i < totg_iterations; ++i)
    {
      TimeOptimalTrajectoryGeneration totg;
      bool totg_success = totg.computeTimeStamps(test_trajectory);
      EXPECT_TRUE(totg_success) << "Failed to compute time stamps with a new TOTG instance in iteration " << i;
    }

    test_trajectory.getRobotTrajectoryMsg(second_trajectory_msg_end);
  }

  // Make sure trajectories produce equal waypoints independent of TOTG instances
  ASSERT_EQ(first_trajectory_msg_start, second_trajectory_msg_start);
  ASSERT_EQ(first_trajectory_msg_end, second_trajectory_msg_end);

  // Iterate on the original trajectory again
  moveit_msgs::RobotTrajectory third_trajectory_msg_end;

  {
    TimeOptimalTrajectoryGeneration totg;
    ASSERT_TRUE(totg.computeTimeStamps(trajectory)) << "Failed to compute time stamps";
  }

  for (size_t i = 0; i < totg_iterations; ++i)
  {
    TimeOptimalTrajectoryGeneration totg;
    bool totg_success = totg.computeTimeStamps(trajectory);
    ASSERT_TRUE(totg_success) << "Failed to compute timestamps on a new TOTG instance in iteration " << i;
  }

  // Compare with previous work
  trajectory.getRobotTrajectoryMsg(third_trajectory_msg_end);

  // Make sure trajectories produce equal waypoints independent of TOTG instances
  ASSERT_EQ(first_trajectory_msg_end, third_trajectory_msg_end);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
