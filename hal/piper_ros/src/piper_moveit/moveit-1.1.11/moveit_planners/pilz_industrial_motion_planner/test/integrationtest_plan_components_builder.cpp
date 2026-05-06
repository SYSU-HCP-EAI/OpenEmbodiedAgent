/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Pilz GmbH & Co. KG
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

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "pilz_industrial_motion_planner/plan_components_builder.h"
#include "pilz_industrial_motion_planner/trajectory_blender_transition_window.h"

const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string ROBOT_DESCRIPTION_STR{ "robot_description" };
const std::string EMPTY_VALUE{ "" };

using namespace pilz_industrial_motion_planner;
using namespace pilz_industrial_motion_planner;

class IntegrationTestPlanComponentBuilder : public testing::Test
{
protected:
  void SetUp() override;

protected:
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_STR).getModel() };
  planning_scene::PlanningSceneConstPtr planning_scene_{ new planning_scene::PlanningScene(robot_model_) };

  std::string planning_group_;
};

void IntegrationTestPlanComponentBuilder::SetUp()
{
  if (!robot_model_)
  {
    FAIL() << "Robot model could not be loaded.";
  }

  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
}

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 *
 */
TEST_F(IntegrationTestPlanComponentBuilder, TestExceptionErrorCodeMapping)
{
  std::shared_ptr<NoBlenderSetException> nbs_ex{ new NoBlenderSetException("") };
  EXPECT_EQ(nbs_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  std::shared_ptr<NoTipFrameFunctionSetException> ntffse_ex{ new NoTipFrameFunctionSetException("") };
  EXPECT_EQ(ntffse_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  std::shared_ptr<NoRobotModelSetException> nrms_ex{ new NoRobotModelSetException("") };
  EXPECT_EQ(nrms_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  std::shared_ptr<BlendingFailedException> bf_ex{ new BlendingFailedException("") };
  EXPECT_EQ(bf_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
}

/**
 * @brief Checks that exception is thrown if no robot model is set.
 *
 */
TEST_F(IntegrationTestPlanComponentBuilder, TestModelSet)
{
  robot_trajectory::RobotTrajectoryPtr traj{ new robot_trajectory::RobotTrajectory(robot_model_, planning_group_) };
  PlanComponentsBuilder builder;

  EXPECT_THROW(builder.append(planning_scene_, traj, 1.0), NoRobotModelSetException);
}

/**
 * @brief Checks that exception is thrown if no blender is set.
 *
 */
TEST_F(IntegrationTestPlanComponentBuilder, TestNoBlenderSet)
{
  robot_trajectory::RobotTrajectoryPtr traj{ new robot_trajectory::RobotTrajectory(robot_model_, planning_group_) };
  PlanComponentsBuilder builder;
  builder.setModel(robot_model_);

  builder.append(planning_scene_, traj, 0.0);

  EXPECT_THROW(builder.append(planning_scene_, traj, 1.0), NoBlenderSetException);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_plan_components_builder");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
