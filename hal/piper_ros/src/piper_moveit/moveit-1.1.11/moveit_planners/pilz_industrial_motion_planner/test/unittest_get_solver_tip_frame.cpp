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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <moveit/robot_model/joint_model_group.h>

#include "pilz_industrial_motion_planner/tip_frame_getter.h"

namespace pilz_industrial_motion_planner
{
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::ReturnRef;

class SolverMock
{
public:
  MOCK_CONST_METHOD0(getTipFrames, const std::vector<std::string>&());
};

class JointModelGroupMock
{
public:
  MOCK_CONST_METHOD0(getSolverInstance, SolverMock const*());
  MOCK_CONST_METHOD0(getName, const std::string&());
};

/**
 * @brief Test fixture for getSolverTipFrame tests.
 */
class GetSolverTipFrameTest : public testing::Test
{
protected:
  void SetUp() override;

protected:
  SolverMock solver_mock_;
  JointModelGroupMock jmg_mock_;
  const std::string group_name_{ "fake_group" };
};

void GetSolverTipFrameTest::SetUp()
{
  EXPECT_CALL(jmg_mock_, getName()).WillRepeatedly(ReturnRef(group_name_));
}

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST_F(GetSolverTipFrameTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<NoSolverException> nse_ex{ new NoSolverException("") };
    EXPECT_EQ(nse_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<MoreThanOneTipFrameException> ex{ new MoreThanOneTipFrameException("") };
    EXPECT_EQ(ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }
}

/**
 * @brief Checks that an exceptions is thrown in case a group has more than
 * one tip frame.
 */
TEST_F(GetSolverTipFrameTest, TestExceptionMoreThanOneTipFrame)
{
  std::vector<std::string> tip_frames{ "fake_tip_frame1", "fake_tip_frame2" };

  EXPECT_CALL(jmg_mock_, getSolverInstance()).Times(AtLeast(1)).WillRepeatedly(Return(&solver_mock_));

  EXPECT_CALL(solver_mock_, getTipFrames()).Times(AtLeast(1)).WillRepeatedly(ReturnRef(tip_frames));

  EXPECT_THROW(getSolverTipFrame(&jmg_mock_), MoreThanOneTipFrameException);
}

/**
 * @brief Checks that an exceptions is thrown in case a group does not
 * possess a solver.
 */
TEST_F(GetSolverTipFrameTest, TestExceptionNoSolver)
{
  EXPECT_CALL(jmg_mock_, getSolverInstance()).WillOnce(Return(nullptr));

  EXPECT_THROW(getSolverTipFrame(&jmg_mock_), NoSolverException);
}

/**
 * @brief Checks that an exceptions is thrown in case a nullptr is
 * specified as JointModelGroup.
 */
TEST_F(GetSolverTipFrameTest, NullptrJointGroup)
{
  moveit::core::JointModelGroup* group = nullptr;
  EXPECT_THROW(hasSolver(group), std::invalid_argument);
}

}  // namespace pilz_industrial_motion_planner

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
