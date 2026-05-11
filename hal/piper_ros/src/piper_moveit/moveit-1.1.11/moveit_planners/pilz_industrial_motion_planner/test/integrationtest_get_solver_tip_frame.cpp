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
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>

#include "pilz_industrial_motion_planner/tip_frame_getter.h"

static const std::string ROBOT_DESCRIPTION_PARAM{ "robot_description" };

namespace pilz_industrial_motion_planner
{
class GetSolverTipFrameIntegrationTest : public testing::Test
{
protected:
  void SetUp() override;

protected:
  robot_model::RobotModelConstPtr robot_model_{
    robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM).getModel()
  };
};

void GetSolverTipFrameIntegrationTest::SetUp()
{
  if (!robot_model_)
  {
    FAIL() << "Robot model could not be loaded.";
  }
}

/**
 * @brief Check if hasSolver() can be called successfully for the manipulator
 * group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestHasSolverManipulator)
{
  EXPECT_TRUE(hasSolver(robot_model_->getJointModelGroup("manipulator"))) << "hasSolver returns false for manipulator";
}

/**
 * @brief Check if hasSolver() can be called successfully for the gripper group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestHasSolverGripperGroup)
{
  EXPECT_FALSE(hasSolver(robot_model_->getJointModelGroup("gripper"))) << "hasSolver returns true for gripper";
}

/**
 * @brief Check if getSolverTipFrame() can be called successfully for the
 * manipulator group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestGetTipSolverFrameManipulator)
{
  getSolverTipFrame(robot_model_->getJointModelGroup("manipulator"));
}

/**
 * @brief Check if getSolverTipFrame() fails for gripper group.
 */
TEST_F(GetSolverTipFrameIntegrationTest, TestGetTipSolverFrameGripper)
{
  EXPECT_THROW(getSolverTipFrame(robot_model_->getJointModelGroup("gripper")), NoSolverException);
}

}  // namespace pilz_industrial_motion_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_get_solver_tip_frame");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
