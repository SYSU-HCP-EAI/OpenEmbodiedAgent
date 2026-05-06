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

#include "pilz_industrial_motion_planner/joint_limits_container.h"
#include "pilz_industrial_motion_planner/joint_limits_extension.h"

using namespace pilz_industrial_motion_planner;

class JointLimitsContainerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    JointLimit lim1;
    lim1.has_position_limits = true;
    lim1.min_position = -2;
    lim1.max_position = 2;
    lim1.has_acceleration_limits = true;
    lim1.max_acceleration = 3;  //<- Expected for common_limit_.max_acceleration

    JointLimit lim2;
    lim2.has_position_limits = true;
    lim2.min_position = -1;  //<- Expected for common_limit_.min_position
    lim2.max_position = 1;   //<- Expected for common_limit_.max_position
    lim2.has_deceleration_limits = true;
    lim2.max_deceleration = -5;  //<- Expected for common_limit_.max_deceleration

    JointLimit lim3;
    lim3.has_velocity_limits = true;
    lim3.max_velocity = 10;

    JointLimit lim4;
    lim4.has_position_limits = true;
    lim4.min_position = -1;
    lim4.max_position = 1;
    lim4.has_acceleration_limits = true;
    lim4.max_acceleration = 400;
    lim4.has_deceleration_limits = false;
    lim4.max_deceleration = -1;

    JointLimit lim5;
    lim5.has_position_limits = true;
    lim5.min_position = -1;
    lim5.max_position = 1;
    lim5.has_acceleration_limits = false;
    lim5.max_acceleration = 1;

    JointLimit lim6;
    lim6.has_velocity_limits = true;
    lim6.max_velocity = 2;  //<- Expected for common_limit_.max_velocity
    lim6.has_deceleration_limits = true;
    lim6.max_deceleration = -100;

    container_.addLimit("joint1", lim1);
    container_.addLimit("joint2", lim2);
    container_.addLimit("joint3", lim3);
    container_.addLimit("joint4", lim4);
    container_.addLimit("joint5", lim5);
    container_.addLimit("joint6", lim6);

    common_limit_ = container_.getCommonLimit();
  }

  pilz_industrial_motion_planner::JointLimitsContainer container_;
  JointLimit common_limit_;
};

/**
 * @brief Check postion
 */
TEST_F(JointLimitsContainerTest, CheckPositionUnification)
{
  EXPECT_EQ(-1, common_limit_.min_position);
  EXPECT_EQ(1, common_limit_.max_position);
}

/**
 * @brief Check velocity
 */
TEST_F(JointLimitsContainerTest, CheckVelocityUnification)
{
  EXPECT_EQ(2, common_limit_.max_velocity);
}

/**
 * @brief Check acceleration
 */
TEST_F(JointLimitsContainerTest, CheckAccelerationUnification)
{
  EXPECT_EQ(3, common_limit_.max_acceleration);
}

/**
 * @brief Check deceleration
 */
TEST_F(JointLimitsContainerTest, CheckDecelerationUnification)
{
  EXPECT_EQ(-5, common_limit_.max_deceleration);
}

/**
 * @brief Check AddLimit for positive and null deceleration
 */
TEST_F(JointLimitsContainerTest, CheckAddLimitDeceleration)
{
  JointLimit lim_invalid1;
  lim_invalid1.has_deceleration_limits = true;
  lim_invalid1.max_deceleration = 0;

  JointLimit lim_invalid2;
  lim_invalid2.has_deceleration_limits = true;
  lim_invalid2.max_deceleration = 1;

  JointLimit lim_valid;
  lim_valid.has_deceleration_limits = true;
  lim_valid.max_deceleration = -1;

  pilz_industrial_motion_planner::JointLimitsContainer container;

  EXPECT_EQ(false, container.addLimit("joint_invalid1", lim_invalid1));
  EXPECT_EQ(false, container.addLimit("joint_invalid2", lim_invalid2));
  EXPECT_EQ(true, container.addLimit("joint_valid", lim_valid));
}

/**
 * @brief Check AddLimit for already contained limit
 */
TEST_F(JointLimitsContainerTest, CheckAddLimitAlreadyContained)
{
  JointLimit lim_valid;
  lim_valid.has_deceleration_limits = true;
  lim_valid.max_deceleration = -1;

  pilz_industrial_motion_planner::JointLimitsContainer container;
  ASSERT_TRUE(container.addLimit("joint_valid", lim_valid));
  EXPECT_FALSE(container.addLimit("joint_valid", lim_valid));
}

/**
 * @brief An uninitialized container should not have any limits set.
 */
TEST_F(JointLimitsContainerTest, CheckEmptyContainer)
{
  pilz_industrial_motion_planner::JointLimitsContainer container;
  JointLimit limits = container.getCommonLimit();
  EXPECT_FALSE(limits.has_position_limits);
  EXPECT_FALSE(limits.has_velocity_limits);
  EXPECT_FALSE(limits.has_acceleration_limits);
}

/**
 * @brief empty position limits for first joint, second one should be returned
 */
TEST_F(JointLimitsContainerTest, FirstPositionEmpty)
{
  JointLimit lim1;

  JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -1;  //<- Expected for common_limit_.min_position
  lim2.max_position = 1;   //<- Expected for common_limit_.max_position

  pilz_industrial_motion_planner::JointLimitsContainer container;
  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  JointLimit limits = container.getCommonLimit();
  EXPECT_TRUE(limits.has_position_limits);
  EXPECT_EQ(1, limits.max_position);
  EXPECT_EQ(-1, limits.min_position);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
