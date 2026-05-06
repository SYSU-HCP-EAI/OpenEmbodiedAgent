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

#include <gtest/gtest.h>

#include "pilz_industrial_motion_planner/velocity_profile_atrap.h"

// Modultest Level1 of Class VelocityProfileATrap
#define EPSILON 1.0e-10

TEST(ATrapTest, Test_SetProfile1)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // can reach the maximal velocity
  vp.SetProfile(3, 35);

  EXPECT_NEAR(vp.Duration(), 11.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 4.0, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 7.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(4.5), 17.0, EPSILON);
  EXPECT_NEAR(vp.Vel(4.5), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(4.5), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(7), 27.0, EPSILON);
  EXPECT_NEAR(vp.Vel(7), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(7), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(9), 33.0, EPSILON);
  EXPECT_NEAR(vp.Vel(9), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(9), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(11), 35.0, EPSILON);
  EXPECT_NEAR(vp.Vel(11), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(11), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(12), 35.0, EPSILON);
  EXPECT_NEAR(vp.Vel(12), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(12), 0.0, EPSILON);
}

TEST(ATrapTest, Test_SetProfile2)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(6, 2, 1.5);

  // just arrive the maximal velocity
  vp.SetProfile(5, 26);

  EXPECT_NEAR(vp.Duration(), 7.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1.5), 7.25, EPSILON);
  EXPECT_NEAR(vp.Vel(1.5), 3.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1.5), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 14.0, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 6.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(5), 23.0, EPSILON);
  EXPECT_NEAR(vp.Vel(5), 3.0, EPSILON);
  EXPECT_NEAR(vp.Acc(5), -1.5, EPSILON);

  EXPECT_NEAR(vp.Pos(7), 26.0, EPSILON);
  EXPECT_NEAR(vp.Vel(7), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(7), -1.5, EPSILON);

  EXPECT_NEAR(vp.Pos(8), 26.0, EPSILON);
  EXPECT_NEAR(vp.Vel(8), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(8), 0.0, EPSILON);
}

TEST(ATrapTest, Test_SetProfile3)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(6, 2, 1);

  // cannot reach the maximal velocity
  vp.SetProfile(5, 17);

  EXPECT_NEAR(vp.Duration(), 6.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 6.0, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 9.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(4), 15.0, EPSILON);
  EXPECT_NEAR(vp.Vel(4), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(4), -1, EPSILON);

  EXPECT_NEAR(vp.Pos(6), 17.0, EPSILON);
  EXPECT_NEAR(vp.Vel(6), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(6), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(7), 17.0, EPSILON);
  EXPECT_NEAR(vp.Vel(7), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(7), 0.0, EPSILON);
}

TEST(ATrapTest, Test_SetProfile4)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(6, 2, 1);

  // empty profile
  vp.SetProfile(5, 5);

  EXPECT_NEAR(vp.Duration(), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);
}

/**
 * @brief Test Description
 *
 * Test Sequence:
 *   1. Generate two profiles with same specifica
 *   2. Set double EPSILON as duration of the first
 *   3. Set the resulting duration as the duration of the second trajectory
 *
 * Expected Results:
 *   1. -
 *   2. -
 *   3. Both profiles should be the same (checked with testpoints in the middle
 * of each phase
 */
TEST(ATrapTest, Test_SetProfileToLowDuration)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp1 =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);
  pilz_industrial_motion_planner::VelocityProfileATrap vp2 = vp1;

  vp1.SetProfileDuration(3, 35, std::numeric_limits<double>::epsilon());
  double fastest_duration = vp1.Duration();

  vp2.SetProfileDuration(3, 35, fastest_duration);

  EXPECT_TRUE(vp1 == vp2) << "Not equal! Profile 1: \n" << vp1 << "\n Profile 2: " << vp2;
}

/**
 * @brief Define Profile with setProfileAllDurations with to low duration
 *
 * Test Sequence:
 *    1. Define a profile with SetProfile(double, double), this will yield the
 * fastest duration
 *    2. Try to define a profile with setProfileAllDurations with a faster
 * combination of durations
 *
 * Expected Results:
 *    1.
 *    2. Both trajectories should be equal
 */
TEST(ATrapTest, Test_setProfileAllDurationsToLowDuration)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp1 =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);
  pilz_industrial_motion_planner::VelocityProfileATrap vp2 = vp1;

  vp1.SetProfile(3, 35);
  double fastest_duration = vp1.Duration();

  // Trigger Duration()>(3*fastest_duration/4)
  vp2.setProfileAllDurations(3, 35, fastest_duration / 4, fastest_duration / 4, fastest_duration / 4);

  EXPECT_TRUE(vp1 == vp2) << "Not equal! Profile 1: \n" << vp1 << "\n Profile 2: " << vp2;
}

/**
 * @brief Define Profile with setProfileStartVelocity with zero velocity
 *
 * Test Sequence:
 *    1. Define a profile with SetProfile(double, double)
 *    2. Try to define a profile with setProfileStartVelocity with zero velocity
 *
 * Expected Results:
 *    1.
 *    2. Both trajectories should be equal
 */
TEST(ATrapTest, Test_SetProfileZeroStartVelocity)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp1 =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);
  pilz_industrial_motion_planner::VelocityProfileATrap vp2 = vp1;

  vp1.SetProfile(1, 2);

  vp2.setProfileStartVelocity(1, 2, 0);  // <-- Set zero velocity
  EXPECT_TRUE(vp1 == vp2) << "Not equal! Profile 1: \n" << vp1 << "\n Profile 2: " << vp2;
}

TEST(ATrapTest, Test_SetProfileDuration)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // set the duration as twice as the fastest profile
  vp.SetProfileDuration(3, 35, 22.0);

  EXPECT_NEAR(vp.Duration(), 22.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 4.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 1.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), 0.5, EPSILON);

  EXPECT_NEAR(vp.Pos(4), 7.0, EPSILON);
  EXPECT_NEAR(vp.Vel(4), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(4), 0.5, EPSILON);

  EXPECT_NEAR(vp.Pos(9), 17.0, EPSILON);
  EXPECT_NEAR(vp.Vel(9), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(9), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(14), 27.0, EPSILON);
  EXPECT_NEAR(vp.Vel(14), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(14), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(18), 33.0, EPSILON);
  EXPECT_NEAR(vp.Vel(18), 1.0, EPSILON);
  EXPECT_NEAR(vp.Acc(18), -0.25, EPSILON);

  EXPECT_NEAR(vp.Pos(22), 35.0, EPSILON);
  EXPECT_NEAR(vp.Vel(22), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(22), -0.25, EPSILON);

  EXPECT_NEAR(vp.Pos(23), 35.0, EPSILON);
  EXPECT_NEAR(vp.Vel(23), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(23), 0.0, EPSILON);
}

TEST(ATrapTest, Test_setProfileAllDurations1)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // set durations
  EXPECT_TRUE(vp.setProfileAllDurations(3, 35, 3.0, 4.0, 5.0));

  EXPECT_NEAR(vp.Duration(), 12.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 3.0 + 8.0 / 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 8.0 / 3.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), 4.0 / 3.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 9.0, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), 4.0 / 3.0, EPSILON);

  EXPECT_NEAR(vp.Pos(5), 17.0, EPSILON);
  EXPECT_NEAR(vp.Vel(5), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(5), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(7), 25.0, EPSILON);
  EXPECT_NEAR(vp.Vel(7), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(7), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(9), 31.4, EPSILON);
  EXPECT_NEAR(vp.Vel(9), 2.4, EPSILON);
  EXPECT_NEAR(vp.Acc(9), -0.8, EPSILON);

  EXPECT_NEAR(vp.Pos(12), 35.0, EPSILON);
  EXPECT_NEAR(vp.Vel(12), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(12), -0.8, EPSILON);

  EXPECT_NEAR(vp.Pos(13), 35.0, EPSILON);
  EXPECT_NEAR(vp.Vel(13), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(13), 0.0, EPSILON);
}

TEST(ATrapTest, Test_setProfileAllDurations2)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // invalid maximal velocity
  EXPECT_FALSE(vp.setProfileAllDurations(3, 35, 3.0, 3.0, 5.0));
  // invalid acceleration
  EXPECT_FALSE(vp.setProfileAllDurations(3, 35, 1.0, 4.0, 7.0));
  // invalid deceleration
  EXPECT_FALSE(vp.setProfileAllDurations(3, 35, 7.0, 4.0, 1.0));
}

TEST(ATrapTest, Test_setProfileStartVelocity1)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // invalide cases
  EXPECT_FALSE(vp.setProfileStartVelocity(3.0, 5.0, -1.0));

  // only deceleration
  vp.setProfileStartVelocity(3.0, 5.0, 2.0);

  EXPECT_NEAR(vp.Duration(), 2.0, EPSILON);
  EXPECT_NEAR(vp.firstPhaseDuration(), 2.0, EPSILON);
  EXPECT_NEAR(vp.secondPhaseDuration(), 0.0, EPSILON);
  EXPECT_NEAR(vp.thirdPhaseDuration(), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 4.5, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 1.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), 0.0, EPSILON);
}

TEST(ATrapTest, Test_setProfileStartVelocity2)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // deceleration, acceleration and deceleration
  vp.setProfileStartVelocity(3.0, 4.0, 2.0);
  EXPECT_NEAR(vp.Duration(), 2.0 + 3 * sqrt(1.0 / 3.0), EPSILON);
  EXPECT_NEAR(vp.firstPhaseDuration(), 2.0, EPSILON);
  EXPECT_NEAR(vp.secondPhaseDuration(), sqrt(1.0 / 3.0), EPSILON);
  EXPECT_NEAR(vp.thirdPhaseDuration(), 2 * sqrt(1.0 / 3.0), EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 4.5, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 1.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 5.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2.1), 4.99, EPSILON);
  EXPECT_NEAR(vp.Vel(2.1), -0.2, EPSILON);
  EXPECT_NEAR(vp.Acc(2.1), -2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2 + sqrt(1.0 / 3.0)), 5.0 - 1.0 / 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2 + sqrt(1.0 / 3.0)), -2 * sqrt(1.0 / 3.0), EPSILON);
  EXPECT_NEAR(vp.Acc(2 + sqrt(1.0 / 3.0)), -2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2 + 3 * sqrt(1.0 / 3.0) - 0.2), 4.02, EPSILON);
  EXPECT_NEAR(vp.Vel(2 + 3 * sqrt(1.0 / 3.0) - 0.2), -0.2, EPSILON);
  EXPECT_NEAR(vp.Acc(2 + 3 * sqrt(1.0 / 3.0) - 0.2), 1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2 + 3 * sqrt(1.0 / 3.0)), 4.0, EPSILON);
  EXPECT_NEAR(vp.Vel(2 + 3 * sqrt(1.0 / 3.0)), 0, EPSILON);
  EXPECT_NEAR(vp.Acc(2 + 3 * sqrt(1.0 / 3.0)), 1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(5), 4.0, EPSILON);
  EXPECT_NEAR(vp.Vel(5), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(5), 0.0, EPSILON);
}

TEST(ATrapTest, Test_setProfileStartVelocity3)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // acceleration, deceleration
  vp.setProfileStartVelocity(3, 14, 2);
  EXPECT_NEAR(vp.Duration(), 5.0, EPSILON);
  EXPECT_NEAR(vp.firstPhaseDuration(), 1.0, EPSILON);
  EXPECT_NEAR(vp.secondPhaseDuration(), 0.0, EPSILON);
  EXPECT_NEAR(vp.thirdPhaseDuration(), 4.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 6.0, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 9.5, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 3.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 12.0, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(5), 14.0, EPSILON);
  EXPECT_NEAR(vp.Vel(5), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(5), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(6), 14, EPSILON);
  EXPECT_NEAR(vp.Vel(6), 0, EPSILON);
  EXPECT_NEAR(vp.Acc(6), 0, EPSILON);
}

TEST(ATrapTest, Test_setProfileStartVelocity4)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // acceleration, constant, deceleration
  vp.setProfileStartVelocity(3, 14, 2);
  EXPECT_NEAR(vp.Duration(), 5.0, EPSILON);
  EXPECT_NEAR(vp.firstPhaseDuration(), 1.0, EPSILON);
  EXPECT_NEAR(vp.secondPhaseDuration(), 0.0, EPSILON);
  EXPECT_NEAR(vp.thirdPhaseDuration(), 4.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 6.0, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 9.5, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 3.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 12.0, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(5), 14.0, EPSILON);
  EXPECT_NEAR(vp.Vel(5), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(5), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(6), 14, EPSILON);
  EXPECT_NEAR(vp.Vel(6), 0, EPSILON);
  EXPECT_NEAR(vp.Acc(6), 0, EPSILON);
}

TEST(ATrapTest, Test_setProfileStartVelocity5)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // acceleration, constant, deceleration
  vp.setProfileStartVelocity(3, 18, 2);
  EXPECT_NEAR(vp.Duration(), 6.0, EPSILON);
  EXPECT_NEAR(vp.firstPhaseDuration(), 1.0, EPSILON);
  EXPECT_NEAR(vp.secondPhaseDuration(), 1.0, EPSILON);
  EXPECT_NEAR(vp.thirdPhaseDuration(), 4.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 6.0, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), 2.0, EPSILON);

  EXPECT_NEAR(vp.Pos(2), 10, EPSILON);
  EXPECT_NEAR(vp.Vel(2), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(2), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 13.5, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 3.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(4), 16.0, EPSILON);
  EXPECT_NEAR(vp.Vel(4), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(4), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(6), 18.0, EPSILON);
  EXPECT_NEAR(vp.Vel(6), 0.0, EPSILON);
  EXPECT_NEAR(vp.Acc(6), -1.0, EPSILON);

  EXPECT_NEAR(vp.Pos(7), 18, EPSILON);
  EXPECT_NEAR(vp.Vel(7), 0, EPSILON);
  EXPECT_NEAR(vp.Acc(7), 0, EPSILON);
}

TEST(ATrapTest, Test_setProfileStartVelocity6)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 2, 1);

  // acceleration, constant, deceleration
  vp.setProfileStartVelocity(3, 15, 4);
  EXPECT_NEAR(vp.Duration(), 5.0, EPSILON);
  EXPECT_NEAR(vp.firstPhaseDuration(), 0.0, EPSILON);
  EXPECT_NEAR(vp.secondPhaseDuration(), 1.0, EPSILON);
  EXPECT_NEAR(vp.thirdPhaseDuration(), 4.0, EPSILON);

  EXPECT_NEAR(vp.Pos(-1), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(-1), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(-1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(0), 3.0, EPSILON);
  EXPECT_NEAR(vp.Vel(0), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(0), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(1), 7.0, EPSILON);
  EXPECT_NEAR(vp.Vel(1), 4.0, EPSILON);
  EXPECT_NEAR(vp.Acc(1), 0.0, EPSILON);

  EXPECT_NEAR(vp.Pos(3), 13, EPSILON);
  EXPECT_NEAR(vp.Vel(3), 2.0, EPSILON);
  EXPECT_NEAR(vp.Acc(3), -1.0, EPSILON);
}

/**
 * @brief Check that the clone function returns a equal profile
 *
 * Note: Definitions other than setProfileAllDurations could fail due to numeric
 * noise
 */
TEST(ATrapTest, Test_Clone)
{
  pilz_industrial_motion_planner::VelocityProfileATrap vp =
      pilz_industrial_motion_planner::VelocityProfileATrap(4, 1, 1);
  vp.setProfileAllDurations(0, 10, 10, 10, 10);
  pilz_industrial_motion_planner::VelocityProfileATrap* vp_clone =
      static_cast<pilz_industrial_motion_planner::VelocityProfileATrap*>(vp.Clone());
  EXPECT_EQ(vp, *vp_clone);
  delete vp_clone;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
