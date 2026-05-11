/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <limits>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>

#include <urdf_parser/urdf_parser.h>

#include <ompl/util/Exception.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem/path.hpp>
#include <ros/package.h>

constexpr double EPSILON = std::numeric_limits<double>::epsilon();

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("pr2");
  };

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(LoadPlanningModelsPr2, StateSpace)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec(robot_model_, "whole_body");
  ompl_interface::JointModelStateSpace ss(spec);
  ss.setPlanningVolume(-1, 1, -1, 1, -1, 1);
  ss.setup();
  std::ofstream fout("ompl_interface_test_state_space_diagram1.dot");
  ss.diagram(fout);
  bool passed = false;
  try
  {
    ss.sanityChecks();
    passed = true;
  }
  catch (ompl::Exception& ex)
  {
    ROS_ERROR("Sanity checks did not pass: %s", ex.what());
  }
  EXPECT_TRUE(passed);
}

TEST_F(LoadPlanningModelsPr2, StateSpaces)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec1(robot_model_, "right_arm");
  ompl_interface::JointModelStateSpace ss1(spec1);
  ss1.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec2(robot_model_, "left_arm");
  ompl_interface::JointModelStateSpace ss2(spec2);
  ss2.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec3(robot_model_, "whole_body");
  ompl_interface::JointModelStateSpace ss3(spec3);
  ss3.setup();

  ompl_interface::ModelBasedStateSpaceSpecification spec4(robot_model_, "arms");
  ompl_interface::JointModelStateSpace ss4(spec4);
  ss4.setup();

  std::ofstream fout("ompl_interface_test_state_space_diagram2.dot");
  ompl::base::StateSpace::Diagram(fout);
}

TEST_F(LoadPlanningModelsPr2, StateSpaceCopy)
{
  ompl_interface::ModelBasedStateSpaceSpecification spec(robot_model_, "right_arm");
  ompl_interface::JointModelStateSpace joint_model_state_space(spec);
  joint_model_state_space.setPlanningVolume(-1, 1, -1, 1, -1, 1);
  joint_model_state_space.setup();
  std::ofstream fout("ompl_interface_test_state_space_diagram1.dot");
  joint_model_state_space.diagram(fout);
  bool passed = false;
  try
  {
    joint_model_state_space.sanityChecks();
    passed = true;
  }
  catch (ompl::Exception& ex)
  {
    ROS_ERROR("Sanity checks did not pass: %s", ex.what());
  }
  EXPECT_TRUE(passed);

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToRandomPositions();
  EXPECT_TRUE(robot_state.distance(robot_state) < EPSILON);
  ompl::base::State* state = joint_model_state_space.allocState();
  for (int i = 0; i < 10; ++i)
  {
    moveit::core::RobotState robot_state2(robot_state);
    EXPECT_TRUE(robot_state.distance(robot_state2) < EPSILON);
    joint_model_state_space.copyToOMPLState(state, robot_state);
    robot_state.setToRandomPositions(
        robot_state.getRobotModel()->getJointModelGroup(joint_model_state_space.getJointModelGroupName()));
    std::cout << (robot_state.getGlobalLinkTransform("r_wrist_roll_link").translation() -
                  robot_state2.getGlobalLinkTransform("r_wrist_roll_link").translation())
              << std::endl;
    EXPECT_TRUE(robot_state.distance(robot_state2) > EPSILON);
    joint_model_state_space.copyToRobotState(robot_state, state);
    std::cout << (robot_state.getGlobalLinkTransform("r_wrist_roll_link").translation() -
                  robot_state2.getGlobalLinkTransform("r_wrist_roll_link").translation())
              << std::endl;
    EXPECT_TRUE(robot_state.distance(robot_state2) < EPSILON);
  }

  // repeat the above test with a different method to copy the state to OMPL
  for (int i = 0; i < 10; ++i)
  {
    // create two equal states
    moveit::core::RobotState robot_state2(robot_state);
    EXPECT_LT(robot_state.distance(robot_state2), EPSILON);

    // copy the first state to OMPL as backup (this is where the 'different' method comes into play)
    const moveit::core::JointModelGroup* joint_model_group =
        robot_state.getRobotModel()->getJointModelGroup(joint_model_state_space.getJointModelGroupName());
    std::vector<std::string> joint_model_names = joint_model_group->getActiveJointModelNames();
    for (std::size_t joint_index = 0; joint_index < joint_model_group->getVariableCount(); ++joint_index)
    {
      const moveit::core::JointModel* joint_model = joint_model_group->getJointModel(joint_model_names[joint_index]);
      EXPECT_NE(joint_model, nullptr);
      joint_model_state_space.copyJointToOMPLState(state, robot_state, joint_model, joint_index);
    }

    // and change the joint values of the moveit state, so it is different that robot_state2
    robot_state.setToRandomPositions(
        robot_state.getRobotModel()->getJointModelGroup(joint_model_state_space.getJointModelGroupName()));
    EXPECT_GT(robot_state.distance(robot_state2), EPSILON);

    // copy the backup values in the OMPL state back to the first state,
    // and check if it is still equal to the second
    joint_model_state_space.copyToRobotState(robot_state, state);
    EXPECT_LT(robot_state.distance(robot_state2), EPSILON);
  }

  joint_model_state_space.freeState(state);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
