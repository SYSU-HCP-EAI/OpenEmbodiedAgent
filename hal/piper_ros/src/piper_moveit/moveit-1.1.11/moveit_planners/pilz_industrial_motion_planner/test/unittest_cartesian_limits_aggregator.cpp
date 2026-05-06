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

#include <ros/ros.h>

#include "pilz_industrial_motion_planner/cartesian_limit.h"
#include "pilz_industrial_motion_planner/cartesian_limits_aggregator.h"

/**
 * @brief Unittest of the CartesianLimitsAggregator class
 */
class CartesianLimitsAggregator : public ::testing::Test
{
};

/**
 * @brief Check if only velocity is set
 */
TEST_F(CartesianLimitsAggregator, OnlyVelocity)
{
  ros::NodeHandle nh("~/vel_only");

  pilz_industrial_motion_planner::CartesianLimit limit =
      pilz_industrial_motion_planner::CartesianLimitsAggregator::getAggregatedLimits(nh);
  EXPECT_TRUE(limit.hasMaxTranslationalVelocity());
  EXPECT_EQ(limit.getMaxTranslationalVelocity(), 10);
  EXPECT_FALSE(limit.hasMaxTranslationalAcceleration());
  EXPECT_FALSE(limit.hasMaxTranslationalDeceleration());
  EXPECT_FALSE(limit.hasMaxRotationalVelocity());
}

/**
 * @brief Check if all values are set correctly
 */
TEST_F(CartesianLimitsAggregator, AllValues)
{
  ros::NodeHandle nh("~/all");

  pilz_industrial_motion_planner::CartesianLimit limit =
      pilz_industrial_motion_planner::CartesianLimitsAggregator::getAggregatedLimits(nh);
  EXPECT_TRUE(limit.hasMaxTranslationalVelocity());
  EXPECT_EQ(limit.getMaxTranslationalVelocity(), 1);

  EXPECT_TRUE(limit.hasMaxTranslationalAcceleration());
  EXPECT_EQ(limit.getMaxTranslationalAcceleration(), 2);

  EXPECT_TRUE(limit.hasMaxTranslationalDeceleration());
  EXPECT_EQ(limit.getMaxTranslationalDeceleration(), -3);

  EXPECT_TRUE(limit.hasMaxRotationalVelocity());
  EXPECT_EQ(limit.getMaxRotationalVelocity(), 4);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_cartesian_limits_aggregator");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
