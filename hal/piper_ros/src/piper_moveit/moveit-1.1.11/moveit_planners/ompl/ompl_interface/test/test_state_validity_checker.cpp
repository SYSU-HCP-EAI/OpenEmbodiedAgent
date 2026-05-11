/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer */
/**
 *    This test checks the basics of a StateValidityChecker:
 *        - Can we create one?
 *        - States inside and outside joint limits.
 *        - States that are in self-collision.
 *        - Position constraints on the robot's end-effector link.
 *
 *    It does not yet test:
 *        - Collision with objects in the environment.
 *        - Orientation constraints, visibility constraints, ...
 *        - A user-specified feasibility function in the planning scene.
 *
 *    The test do show what is minimally required to create a working StateValidityChecker.
 **/

#include "load_test_robot.h"

#include <limits>
#include <ostream>

#include <gtest/gtest.h>

#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ompl/geometric/SimpleSetup.h>

/** \brief This flag sets the verbosity level for the state validity checker. **/
constexpr bool VERBOSE{ false };

constexpr char LOGNAME[] = "test_state_validity_checker";

/** \brief Pretty print std:vectors **/
std::ostream& operator<<(std::ostream& os, const std::vector<double>& v)
{
  os << "( ";
  for (auto value : v)
    os << value << ", ";
  os << " )";
  return os;
}

/** \brief Generic implementation of the tests that can be executed on different robots. **/
class TestStateValidityChecker : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
public:
  TestStateValidityChecker(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot(robot_name, group_name)
  {
  }

  /***************************************************************************
   * START Test implementations
   * ************************************************************************/

  void testConstructor()
  {
    ompl::base::StateValidityCheckerPtr checker =
        std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
  }

  /** This test takes a state that is inside the joint limits and collision free as input. **/
  void testJointLimits(const std::vector<double>& position_in_limits)
  {
    // create a validity checker for this test
    auto checker = std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
    checker->setVerbose(VERBOSE);

    robot_state_->setJointGroupPositions(joint_model_group_, position_in_limits);

    // use a scoped OMPL state so we don't have to call allocState and freeState
    // (as recommended in the OMPL documantion)
    ompl::base::ScopedState<> ompl_state(state_space_);
    state_space_->copyToOMPLState(ompl_state.get(), *robot_state_);

    ROS_DEBUG_STREAM_NAMED(LOGNAME, ompl_state.reals());

    // assume the given position is not in self-collision
    // and there are no collision objects or path constraints so this state should be valid
    EXPECT_TRUE(checker->isValid(ompl_state.get()));

    // move first joint obviously outside any joint limits
    ompl_state->as<ompl_interface::JointModelStateSpace::StateType>()->values[0] = std::numeric_limits<double>::max();
    ompl_state->as<ompl_interface::JointModelStateSpace::StateType>()->clearKnownInformation();

    ROS_DEBUG_STREAM_NAMED(LOGNAME, ompl_state.reals());

    EXPECT_FALSE(checker->isValid(ompl_state.get()));
  }

  /** This test takes a state that is known to be in self-collision and inside the joint limits as input. **/
  void testSelfCollision(const std::vector<double>& position_in_self_collision)
  {
    // create a validity checker for this test
    auto checker = std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
    checker->setVerbose(VERBOSE);

    robot_state_->setJointGroupPositions(joint_model_group_, position_in_self_collision);

    // use a scoped OMPL state so we don't have to call allocState and freeState
    // (as recommended in the OMPL documantion)
    ompl::base::ScopedState<> ompl_state(state_space_);
    state_space_->copyToOMPLState(ompl_state.get(), *robot_state_);

    ROS_DEBUG_STREAM_NAMED(LOGNAME, ompl_state.reals());

    // the given state is known to be in self-collision, we check it here
    EXPECT_FALSE(checker->isValid(ompl_state.get()));

    // but it should respect the joint limits
    EXPECT_TRUE(robot_state_->satisfiesBounds());
  }

  void testPathConstraints(const std::vector<double>& position_in_joint_limits)
  {
    ASSERT_NE(planning_context_, nullptr) << "Initialize planning context before adding path constraints.";

    // set the robot to a known position that is withing the joint limits and collision free
    robot_state_->setJointGroupPositions(joint_model_group_, position_in_joint_limits);

    // create position constraints around the given robot state
    moveit_msgs::Constraints path_constraints;
    Eigen::Isometry3d ee_pose = robot_state_->getGlobalLinkTransform(ee_link_name_);
    path_constraints.name = "test_position_constraints";
    path_constraints.position_constraints.push_back(createPositionConstraint(
        { ee_pose.translation().x(), ee_pose.translation().y(), ee_pose.translation().z() }, { 0.1, 0.1, 0.1 }));

    moveit_msgs::MoveItErrorCodes error_code_not_used;
    ASSERT_TRUE(planning_context_->setPathConstraints(path_constraints, &error_code_not_used));

    auto checker = std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
    checker->setVerbose(VERBOSE);

    // use a scoped OMPL state so we don't have to call allocState and freeState
    // (as recommended in the OMPL documantion)
    ompl::base::ScopedState<> ompl_state(state_space_);
    state_space_->copyToOMPLState(ompl_state.get(), *robot_state_);

    ROS_DEBUG_STREAM_NAMED(LOGNAME, ompl_state.reals());

    EXPECT_TRUE(checker->isValid(ompl_state.get()));

    // move the position constraints away from the curren end-effector position to make it fail
    moveit_msgs::Constraints path_constraints_2(path_constraints);
    path_constraints_2.position_constraints.at(0).constraint_region.primitive_poses.at(0).position.z += 0.2;

    ASSERT_TRUE(planning_context_->setPathConstraints(path_constraints_2, &error_code_not_used));

    // clear the cached validity of the state before checking again,
    // otherwise the path constraints will not be checked.
    ompl_state->as<ompl_interface::JointModelStateSpace::StateType>()->clearKnownInformation();

    EXPECT_FALSE(checker->isValid(ompl_state.get()));
  }

  /***************************************************************************
   * END Test implementation
   * ************************************************************************/

protected:
  void SetUp() override
  {
    // setup all the input we need to create a StateValidityChecker
    setupStateSpace();
    setupPlanningContext();
  };

  void setupStateSpace()
  {
    ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_, group_name_);
    state_space_ = std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
    state_space_->computeLocations();  // this gets normally called in the state space factory
  }

  void setupPlanningContext()
  {
    ASSERT_NE(state_space_, nullptr) << "Initialize state space before creating the planning context.";

    planning_context_spec_.state_space_ = state_space_;
    planning_context_spec_.ompl_simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);
    planning_context_ =
        std::make_shared<ompl_interface::ModelBasedPlanningContext>(group_name_, planning_context_spec_);

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_context_->setPlanningScene(planning_scene_);
    moveit::core::RobotState start_state(robot_model_);
    start_state.setToDefaultValues();
    planning_context_->setCompleteInitialState(start_state);
  }

  /** \brief Helper function to create a position constraint. **/
  moveit_msgs::PositionConstraint createPositionConstraint(std::array<double, 3> position,
                                                           std::array<double, 3> dimensions)
  {
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimensions[0];
    box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimensions[1];
    box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimensions[2];

    geometry_msgs::Pose box_pose;
    box_pose.position.x = position[0];
    box_pose.position.y = position[1];
    box_pose.position.z = position[2];
    box_pose.orientation.w = 1.0;

    moveit_msgs::PositionConstraint position_constraint;
    position_constraint.header.frame_id = base_link_name_;
    position_constraint.link_name = ee_link_name_;
    position_constraint.constraint_region.primitives.push_back(box);
    position_constraint.constraint_region.primitive_poses.push_back(box_pose);

    // set the default weight to avoid warning in test output
    position_constraint.weight = 1.0;

    return position_constraint;
  }

  ompl_interface::ModelBasedStateSpacePtr state_space_;
  ompl_interface::ModelBasedPlanningContextSpecification planning_context_spec_;
  ompl_interface::ModelBasedPlanningContextPtr planning_context_;
  planning_scene::PlanningScenePtr planning_scene_;
};

// /***************************************************************************
//  * Run all tests on the Panda robot
//  * ************************************************************************/
class PandaValidity : public TestStateValidityChecker
{
protected:
  PandaValidity() : TestStateValidityChecker("panda", "panda_arm")
  {
  }
};

TEST_F(PandaValidity, testConstructor)
{
  testConstructor();
}

TEST_F(PandaValidity, testJointLimits)
{
  // use the panda "ready" state from the srdf config
  // we know this state should be within limits and self-collision free
  testJointLimits({ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 });
}

TEST_F(PandaValidity, testSelfCollision)
{
  // the given state has self collision between "hand" and "panda_link2"
  // (I just tried a couple of random states until I found one that collided.)
  testSelfCollision({ 2.31827, -0.169668, 2.5225, -2.98568, -0.36355, 0.808339, 0.0843406 });
}

TEST_F(PandaValidity, testPathConstraints)
{
  // use the panda "ready" state from the srdf config
  // we know this state should be within limits and self-collision free
  testPathConstraints({ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 });
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucTest : public TestStateValidityChecker
{
protected:
  FanucTest() : TestStateValidityChecker("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucTest, createStateValidityChecker)
{
  testConstructor();
}

TEST_F(FanucTest, testJointLimits)
{
  // I assume the Fanucs's zero state is within limits and self-collision free
  testJointLimits({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(FanucTest, testSelfCollision)
{
  // the given state has self collision between "base_link" and "link_5"
  // (I just tried a couple of random states until I found one that collided.)
  testSelfCollision({ -2.95993, -0.682185, -2.43873, -0.939784, 3.0544, 0.882294 });
}

TEST_F(FanucTest, testPathConstraints)
{
  // I assume the Fanucs's zero state is within limits and self-collision free
  testPathConstraints({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
