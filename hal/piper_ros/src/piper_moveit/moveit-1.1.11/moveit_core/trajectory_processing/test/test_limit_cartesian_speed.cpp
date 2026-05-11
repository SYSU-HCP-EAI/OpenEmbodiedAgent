/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Ken Anderson
 *  Copyright (c) 2020, Benjamin Scholz
 *  Copyright (c) 2021, Thies Oelerich
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
 *   * Neither the name of the authors nor the names of other
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

/* Authors: Benjamin Scholz, Thies Oelerich, based off test_time_parameterization.cpp by Ken Anderson */

#include <gtest/gtest.h>
#include <fstream>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/limit_cartesian_speed.h>
#include <moveit/utils/robot_model_test_utils.h>

// Static variables used in all tests
moveit::core::RobotModelConstPtr RMODEL = moveit::core::loadTestingRobotModel("panda");
robot_trajectory::RobotTrajectory TRAJECTORY(RMODEL, "panda_arm");

// Vector of distances between waypoints
std::vector<double> WAYPOINT_DISTANCES;

// Name of logger
const char* LOGGER_NAME = "trajectory_processing";

// Initialize one-joint, straight-line trajectory
bool initStraightTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const robot_model::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED(LOGGER_NAME, "Need to set the group");
    return false;
  }
  // Get state of the robot
  moveit::core::RobotState state(trajectory.getRobotModel());

  trajectory.clear();
  // Initial waypoint
  // Cartesian Position (in panda_link0 frame):
  // [X: 0.30701957005161057, Y: 0, Z: 0.5902504197143968]
  state.setVariablePosition("panda_joint1", 0);
  state.setVariablePosition("panda_joint2", -0.785);
  state.setVariablePosition("panda_joint3", 0);
  state.setVariablePosition("panda_joint4", -2.356);
  state.setVariablePosition("panda_joint5", 0);
  state.setVariablePosition("panda_joint6", 1.571);
  state.setVariablePosition("panda_joint7", 0.785);
  trajectory.addSuffixWayPoint(state, 0.0);

  // First waypoint (+0.3 m in X direction)
  // Cartesian Position (in panda_link0 frame):
  // [X: 0.6070218670533757, Y: 0, Z: 0.5902504197143968]
  state.setVariablePosition("panda_joint1", 0.00011058924053135735);
  state.setVariablePosition("panda_joint2", 0.15980591412916012);
  state.setVariablePosition("panda_joint3", -0.000269206763021151);
  state.setVariablePosition("panda_joint4", -1.4550637907602342);
  state.setVariablePosition("panda_joint5", 0.0006907805230834268);
  state.setVariablePosition("panda_joint6", 1.61442119426705);
  state.setVariablePosition("panda_joint7", 0.7845267455355481);
  trajectory.addSuffixWayPoint(state, 0.0);
  WAYPOINT_DISTANCES.push_back(0.3);

  // Second waypoint (+0.3 m in Y direction)
  // Cartesian Position (in panda_link0 frame):
  // [X: 0.6070218670533757, Y: 0.3, Z: 0.5902504197143968]
  state.setVariablePosition("panda_joint1", 0.32516555661705315);
  state.setVariablePosition("panda_joint2", 0.4668669802969372);
  state.setVariablePosition("panda_joint3", 0.20650832887601522);
  state.setVariablePosition("panda_joint4", -1.0166745094262262);
  state.setVariablePosition("panda_joint5", -0.0931020003693296);
  state.setVariablePosition("panda_joint6", 1.4764599578787032);
  state.setVariablePosition("panda_joint7", 1.2855361917975465);
  trajectory.addSuffixWayPoint(state, 0.0);
  WAYPOINT_DISTANCES.push_back(0.3);

  // Third waypoint (-0.3 m in Z direction)
  // Cartesian Position (in panda_link0 frame):
  // [X: 0.6070218670533757, Y: 0.3, Z: 0.29026011026061577]
  state.setVariablePosition("panda_joint1", 0.1928958411545848);
  state.setVariablePosition("panda_joint2", 0.5600654280773957);
  state.setVariablePosition("panda_joint3", 0.31117191776899084);
  state.setVariablePosition("panda_joint4", -1.6747509079656924);
  state.setVariablePosition("panda_joint5", -0.20206061876786893);
  state.setVariablePosition("panda_joint6", 2.2024820844782385);
  state.setVariablePosition("panda_joint7", 1.3635216856419043);
  trajectory.addSuffixWayPoint(state, 0.0);
  WAYPOINT_DISTANCES.push_back(0.3);

  return true;
}

void printTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const robot_model::JointModelGroup* group = trajectory.getGroup();
  const std::vector<int>& idx = group->getVariableIndexList();
  unsigned int count = trajectory.getWayPointCount();

  ROS_INFO_STREAM_NAMED(LOGGER_NAME,
                        "trajectory length is " << trajectory.getWayPointDurationFromStart(count - 1) << " seconds.");
  for (unsigned i = 0; i < count; i++)
  {
    robot_state::RobotStatePtr point = trajectory.getWayPointPtr(i);
    ROS_INFO_STREAM_NAMED(LOGGER_NAME, "Waypoint " << i << " time " << trajectory.getWayPointDurationFromStart(i)
                                                   << " pos " << point->getVariablePosition(idx[0]) << " vel "
                                                   << point->getVariableVelocity(idx[0]) << " acc "
                                                   << point->getVariableAcceleration(idx[0]));

    if (i > 0)
    {
      robot_state::RobotStatePtr prev = trajectory.getWayPointPtr(i - 1);
      ROS_INFO_STREAM_NAMED(
          LOGGER_NAME,
          "jrk " << (point->getVariableAcceleration(idx[0]) - prev->getVariableAcceleration(idx[0])) /
                        (trajectory.getWayPointDurationFromStart(i) - trajectory.getWayPointDurationFromStart(i - 1)));
    }
  }
}

TEST(TestCartesianSpeed, TestCartesianEndEffectorSpeed)
{
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  EXPECT_EQ(initStraightTrajectory(TRAJECTORY), true);
  const char* end_effector_link = "panda_link8";

  EXPECT_TRUE(time_parameterization.computeTimeStamps(TRAJECTORY));
  trajectory_processing::limitMaxCartesianLinkSpeed(TRAJECTORY, 0.01, end_effector_link);
  printTrajectory(TRAJECTORY);
  size_t num_waypoints = TRAJECTORY.getWayPointCount();
  robot_state::RobotStatePtr kinematic_state = TRAJECTORY.getFirstWayPointPtr();
  Eigen::Isometry3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector_link);
  Eigen::Isometry3d next_end_effector_state;
  double euclidean_distance = 0.0;

  // Check average speed of the total trajectory by exact calculations
  for (size_t i = 0; i < num_waypoints - 1; i++)
  {
    kinematic_state = TRAJECTORY.getWayPointPtr(i + 1);
    next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector_link);
    euclidean_distance += (next_end_effector_state.translation() - current_end_effector_state.translation()).norm();
    current_end_effector_state = next_end_effector_state;
  }
  double avg_speed = euclidean_distance / TRAJECTORY.getWayPointDurationFromStart(num_waypoints);
  ASSERT_NEAR(0.01, avg_speed, 2e-4);

  // Check average speed between waypoints using the information about the hand
  // designed waypoints
  for (size_t i = 1; i < num_waypoints; i++)
  {
    double current_avg_speed = WAYPOINT_DISTANCES[i - 1] / TRAJECTORY.getWayPointDurationFromPrevious(i);
    ASSERT_NEAR(0.01, current_avg_speed, 2e-4);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
