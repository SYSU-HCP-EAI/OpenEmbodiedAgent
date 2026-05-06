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

#include <memory>

#include <gtest/gtest.h>

#include "pilz_industrial_motion_planner/trajectory_generator.h"

using namespace pilz_industrial_motion_planner;

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 */
TEST(TrajectoryGeneratorTest, TestExceptionErrorCodeMapping)
{
  {
    std::shared_ptr<TrajectoryGeneratorInvalidLimitsException> tgil_ex{ new TrajectoryGeneratorInvalidLimitsException(
        "") };
    EXPECT_EQ(tgil_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<VelocityScalingIncorrect> vsi_ex{ new VelocityScalingIncorrect("") };
    EXPECT_EQ(vsi_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<AccelerationScalingIncorrect> asi_ex{ new AccelerationScalingIncorrect("") };
    EXPECT_EQ(asi_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
  }

  {
    std::shared_ptr<UnknownPlanningGroup> upg_ex{ new UnknownPlanningGroup("") };
    EXPECT_EQ(upg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME);
  }

  {
    std::shared_ptr<NoJointNamesInStartState> njniss_ex{ new NoJointNamesInStartState("") };
    EXPECT_EQ(njniss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<SizeMismatchInStartState> smiss_ex{ new SizeMismatchInStartState("") };
    EXPECT_EQ(smiss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<JointsOfStartStateOutOfRange> jofssoor_ex{ new JointsOfStartStateOutOfRange("") };
    EXPECT_EQ(jofssoor_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<NonZeroVelocityInStartState> nzviss_ex{ new NonZeroVelocityInStartState("") };
    EXPECT_EQ(nzviss_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  }

  {
    std::shared_ptr<NotExactlyOneGoalConstraintGiven> neogcg_ex{ new NotExactlyOneGoalConstraintGiven("") };
    EXPECT_EQ(neogcg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<OnlyOneGoalTypeAllowed> oogta_ex{ new OnlyOneGoalTypeAllowed("") };
    EXPECT_EQ(oogta_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<StartStateGoalStateMismatch> ssgsm_ex{ new StartStateGoalStateMismatch("") };
    EXPECT_EQ(ssgsm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<JointConstraintDoesNotBelongToGroup> jcdnbtg_ex{ new JointConstraintDoesNotBelongToGroup("") };
    EXPECT_EQ(jcdnbtg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<JointsOfGoalOutOfRange> jogoor_ex{ new JointsOfGoalOutOfRange("") };
    EXPECT_EQ(jogoor_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<PositionConstraintNameMissing> pcnm_ex{ new PositionConstraintNameMissing("") };
    EXPECT_EQ(pcnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<OrientationConstraintNameMissing> ocnm_ex{ new OrientationConstraintNameMissing("") };
    EXPECT_EQ(ocnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<PositionOrientationConstraintNameMismatch> pocnm_ex{ new PositionOrientationConstraintNameMismatch(
        "") };
    EXPECT_EQ(pocnm_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }

  {
    std::shared_ptr<NoIKSolverAvailable> nisa_ex{ new NoIKSolverAvailable("") };
    EXPECT_EQ(nisa_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
  }

  {
    std::shared_ptr<NoPrimitivePoseGiven> nppg_ex{ new NoPrimitivePoseGiven("") };
    EXPECT_EQ(nppg_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
