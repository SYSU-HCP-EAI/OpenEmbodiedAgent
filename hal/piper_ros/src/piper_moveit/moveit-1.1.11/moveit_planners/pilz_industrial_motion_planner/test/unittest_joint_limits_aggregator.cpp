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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_industrial_motion_planner/joint_limits_aggregator.h"
#include "pilz_industrial_motion_planner/joint_limits_extension.h"
#include "pilz_industrial_motion_planner/joint_limits_interface_extension.h"

/**
 * @brief Unittest of the JointLimitsAggregator class
 */
class JointLimitsAggregator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle node_handle("~");

    // Load robot module
    robot_model_loader::RobotModelLoader::Options opt("robot_description");
    model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(opt);
    robot_model_ = model_loader_->getModel();

    return;
  }

  /// The robot model loader
  robot_model_loader::RobotModelLoaderPtr model_loader_;

  /// The robot model
  robot_model::RobotModelConstPtr robot_model_;
};

/**
 * @brief Check for that the size of the map and the size of the given joint
 * models is equal
 */
TEST_F(JointLimitsAggregator, ExpectedMapSize)
{
  ros::NodeHandle nh("~");

  pilz_industrial_motion_planner::JointLimitsContainer container =
      pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(nh,
                                                                                 robot_model_->getActiveJointModels());

  EXPECT_EQ(robot_model_->getActiveJointModels().size(), container.getCount());
}

/**
 * @brief Check that the value in the parameter server correctly overrides the
 * position(if within limits)
 */
TEST_F(JointLimitsAggregator, CorrectOverwriteByParamterPosition)
{
  ros::NodeHandle nh("~/valid_1");

  pilz_industrial_motion_planner::JointLimitsContainer container =
      pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(nh,
                                                                                 robot_model_->getActiveJointModels());

  for (std::pair<std::string, pilz_industrial_motion_planner::JointLimit> lim : container)
  {
    // Check for the overwrite
    if (lim.first == "prbt_joint_1")
    {
      EXPECT_EQ(2, container.getLimit(lim.first).max_position);
      EXPECT_EQ(-2, container.getLimit(lim.first).min_position);
    }
    // Check that nothing else changed
    else
    {
      EXPECT_EQ(robot_model_->getJointModel(lim.first)->getVariableBounds()[0].min_position_,
                container.getLimit(lim.first).min_position);
      EXPECT_EQ(robot_model_->getJointModel(lim.first)->getVariableBounds()[0].max_position_,
                container.getLimit(lim.first).max_position);
    }
  }
}

/**
 * @brief Check that the value in the parameter server correctly overrides the
 * velocity(if within limits)
 */
TEST_F(JointLimitsAggregator, CorrectOverwriteByParamterVelocity)
{
  ros::NodeHandle nh("~/valid_1");

  pilz_industrial_motion_planner::JointLimitsContainer container =
      pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(nh,
                                                                                 robot_model_->getActiveJointModels());

  for (std::pair<std::string, pilz_industrial_motion_planner::JointLimit> lim : container)
  {
    // Check that velocity was only changed in joint "prbt_joint_3"
    if (lim.first == "prbt_joint_3")
    {
      EXPECT_EQ(1.1, container.getLimit(lim.first).max_velocity);
    }
    else
    {
      EXPECT_EQ(robot_model_->getJointModel(lim.first)->getVariableBounds()[0].max_velocity_,
                container.getLimit(lim.first).max_velocity);
    }
  }
}

/**
 * @brief Check that the acceleration and deceleration are set properly
 */
TEST_F(JointLimitsAggregator, CorrectSettingAccelerationAndDeceleration)
{
  ros::NodeHandle nh("~/valid_1");

  pilz_industrial_motion_planner::JointLimitsContainer container =
      pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(nh,
                                                                                 robot_model_->getActiveJointModels());

  for (std::pair<std::string, pilz_industrial_motion_planner::JointLimit> lim : container)
  {
    if (lim.first == "prbt_joint_4")
    {
      EXPECT_EQ(5.5, container.getLimit(lim.first).max_acceleration) << lim.first;
      EXPECT_EQ(-5.5, container.getLimit(lim.first).max_deceleration) << lim.first;
    }
    else if (lim.first == "prbt_joint_5")
    {
      EXPECT_EQ(0, container.getLimit(lim.first).max_acceleration) << lim.first;
      EXPECT_EQ(-6.6, container.getLimit(lim.first).max_deceleration) << lim.first;
    }
    else
    {
      EXPECT_EQ(0, container.getLimit(lim.first).max_acceleration) << lim.first;
      EXPECT_EQ(0, container.getLimit(lim.first).max_deceleration) << lim.first;
    }
  }
}

/**
 * @brief Check that position limit violations are detected properly
 */
TEST_F(JointLimitsAggregator, LimitsViolationPosition)
{
  ros::NodeHandle nh_min("~/violate_position_min");

  EXPECT_THROW(pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
                   nh_min, robot_model_->getActiveJointModels()),
               pilz_industrial_motion_planner::AggregationBoundsViolationException);

  ros::NodeHandle nh_max("~/violate_position_max");

  EXPECT_THROW(pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
                   nh_max, robot_model_->getActiveJointModels()),
               pilz_industrial_motion_planner::AggregationBoundsViolationException);
}

/**
 * @brief Check that velocity limit violations are detected properly
 */
TEST_F(JointLimitsAggregator, LimitsViolationVelocity)
{
  ros::NodeHandle nh("~/violate_velocity");

  EXPECT_THROW(pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
                   nh, robot_model_->getActiveJointModels()),
               pilz_industrial_motion_planner::AggregationBoundsViolationException);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_joint_limits_aggregator");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
