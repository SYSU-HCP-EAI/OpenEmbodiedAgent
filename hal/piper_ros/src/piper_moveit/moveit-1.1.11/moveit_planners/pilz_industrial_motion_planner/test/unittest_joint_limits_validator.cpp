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

#include "pilz_industrial_motion_planner/joint_limits_extension.h"

#include "pilz_industrial_motion_planner/joint_limits_validator.h"

using namespace pilz_industrial_motion_planner;

class JointLimitsValidatorTest : public ::testing::Test
{
};

/**
 * @brief Check postion equality
 */
TEST_F(JointLimitsValidatorTest, CheckPositionEquality)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in min_position detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMinPosition)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 1;

  JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -2;
  lim2.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(false, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in max_position detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxPosition1)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 2;

  JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -1;
  lim2.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(false, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in max_position detection (using 3 limits)
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxPosition2)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 2;

  JointLimit lim2;
  lim2.has_position_limits = true;
  lim2.min_position = -1;
  lim2.max_position = 1;

  JointLimit lim3;
  lim3.has_position_limits = true;
  lim3.min_position = -1;
  lim3.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(false, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in has_position_limits false detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityHasPositionLimits)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_position_limits = true;
  lim1.min_position = -1;
  lim1.max_position = 1;

  JointLimit lim2;
  lim2.has_position_limits = false;
  lim2.min_position = -1;
  lim2.max_position = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(false, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// VELOCITY
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

/**
 * @brief Check velocity equality
 */
TEST_F(JointLimitsValidatorTest, CheckVelocityEquality)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_velocity_limits = true;
  lim1.max_velocity = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_velocity inequality
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxVelocity1)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_velocity_limits = true;
  lim1.max_velocity = 1;

  JointLimit lim2;
  lim2.has_velocity_limits = true;
  lim2.max_velocity = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_velocity inequality (using 3Limits)
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxVelocity2)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_velocity_limits = true;
  lim1.max_velocity = 1;

  JointLimit lim2;
  lim2.has_velocity_limits = true;
  lim2.max_velocity = 2;

  JointLimit lim3;
  lim3.has_velocity_limits = true;
  lim3.max_velocity = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check postion inequality in has_position_limits false detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityHasVelocityLimits)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_velocity_limits = true;

  JointLimit lim2;
  lim2.has_velocity_limits = false;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// ACCELERATION
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

/**
 * @brief Check acceleration equality
 */
TEST_F(JointLimitsValidatorTest, CheckAccelerationEquality)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_acceleration_limits = true;
  lim1.max_acceleration = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_acceleration inequality
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxAcceleration1)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_acceleration_limits = true;
  lim1.max_acceleration = 1;

  JointLimit lim2;
  lim2.has_acceleration_limits = true;
  lim2.max_acceleration = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_acceleration inequality (using 3 Limits)
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityMaxAcceleration2)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_acceleration_limits = true;
  lim1.max_acceleration = 1;

  JointLimit lim2;
  lim2.has_acceleration_limits = true;
  lim2.max_acceleration = 2;

  JointLimit lim3;
  lim3.has_acceleration_limits = true;
  lim3.max_acceleration = 2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check acceleration inequality in has_acceleration_limits false
 * detection
 */
TEST_F(JointLimitsValidatorTest, CheckPositionInEqualityHasAccelerationLimits)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_acceleration_limits = true;

  JointLimit lim2;
  lim2.has_acceleration_limits = false;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// DECELERATION
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

/**
 * @brief Check deceleration equality
 */
TEST_F(JointLimitsValidatorTest, CheckDecelerationEquality)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = 1;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim1);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_deceleration inequality
 */
TEST_F(JointLimitsValidatorTest, CheckInEqualityMaxDeceleration1)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = -1;

  JointLimit lim2;
  lim2.has_deceleration_limits = true;
  lim2.max_deceleration = -2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check max_deceleration inequality
 */
TEST_F(JointLimitsValidatorTest, CheckInEqualityMaxDeceleration2)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = -1;

  JointLimit lim2;
  lim2.has_deceleration_limits = true;
  lim2.max_deceleration = -2;

  JointLimit lim3;
  lim3.has_deceleration_limits = true;
  lim3.max_deceleration = -2;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);
  container.addLimit("joint3", lim3);

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

/**
 * @brief Check deceleration inequality in has_deceleration_limits false
 * detection
 */
TEST_F(JointLimitsValidatorTest, CheckInEqualityHasDecelerationLimits)
{
  JointLimitsContainer container;

  JointLimit lim1;
  lim1.has_deceleration_limits = true;
  lim1.max_deceleration = -1;

  JointLimit lim2;
  lim2.has_deceleration_limits = false;

  container.addLimit("joint1", lim1);
  container.addLimit("joint2", lim2);

  ASSERT_EQ(2u, container.getCount());

  EXPECT_EQ(true, JointLimitsValidator::validateAllPositionLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllVelocityLimitsEqual(container));
  EXPECT_EQ(true, JointLimitsValidator::validateAllAccelerationLimitsEqual(container));
  EXPECT_EQ(false, JointLimitsValidator::validateAllDecelerationLimitsEqual(container));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
