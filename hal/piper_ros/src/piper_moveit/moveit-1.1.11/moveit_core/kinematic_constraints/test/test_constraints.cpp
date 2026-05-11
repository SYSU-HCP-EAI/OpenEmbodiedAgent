/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <boost/math/constants/constants.hpp>

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("pr2");
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(LoadPlanningModelsPr2, JointConstraintsSimple)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::JointConstraint jc(robot_model_);
  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = "head_pan_joint";
  jcm.position = 0.4;
  jcm.tolerance_above = 0.1;
  jcm.tolerance_below = 0.05;

  EXPECT_TRUE(jc.configure(jcm));
  // weight should have been changed to 1.0
  EXPECT_NEAR(jc.getConstraintWeight(), 1.0, std::numeric_limits<double>::epsilon());

  // tests that the default state is outside the bounds
  // given that the default state is at 0.0
  EXPECT_TRUE(jc.configure(jcm));
  kinematic_constraints::ConstraintEvaluationResult p1 = jc.decide(robot_state);
  EXPECT_FALSE(p1.satisfied);
  EXPECT_NEAR(p1.distance, jcm.position, 1e-6);

  // tests that when we set the state within the bounds
  // the constraint is satisfied
  double jval = 0.41;
  robot_state.setJointPositions(jcm.joint_name, &jval);
  kinematic_constraints::ConstraintEvaluationResult p2 = jc.decide(robot_state);
  EXPECT_TRUE(p2.satisfied);
  EXPECT_NEAR(p2.distance, 0.01, 1e-6);

  // exactly equal to the low bound is fine too
  jval = 0.35;
  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // and so is less than epsilon when there's no other source of error
  //    jvals[jcm.joint_name] = 0.35-std::numeric_limits<double>::epsilon();
  jval = 0.35 - std::numeric_limits<double>::epsilon();

  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // but this is too much
  jval = 0.35 - 3 * std::numeric_limits<double>::epsilon();
  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // negative value makes configuration fail
  jcm.tolerance_below = -0.05;
  EXPECT_FALSE(jc.configure(jcm));

  jcm.tolerance_below = 0.05;
  EXPECT_TRUE(jc.configure(jcm));

  // still satisfied at a slightly different state
  jval = 0.46;
  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // still satisfied at a slightly different state
  jval = 0.501;
  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // still satisfied at a slightly different state
  jval = 0.39;
  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // outside the bounds
  jval = 0.34;
  robot_state.setJointPositions(jcm.joint_name, &jval);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // testing equality
  kinematic_constraints::JointConstraint jc2(robot_model_);
  EXPECT_TRUE(jc2.configure(jcm));
  EXPECT_TRUE(jc2.enabled());
  EXPECT_TRUE(jc.equal(jc2, 1e-12));

  // if name not equal, not equal
  jcm.joint_name = "head_tilt_joint";
  EXPECT_TRUE(jc2.configure(jcm));
  EXPECT_FALSE(jc.equal(jc2, 1e-12));

  // if different, test margin behavior
  jcm.joint_name = "head_pan_joint";
  jcm.position = 0.3;
  EXPECT_TRUE(jc2.configure(jcm));
  EXPECT_FALSE(jc.equal(jc2, 1e-12));
  // exactly equal is still false
  EXPECT_FALSE(jc.equal(jc2, .1));
  EXPECT_TRUE(jc.equal(jc2, .101));

  // no name makes this false
  jcm.joint_name = "";
  jcm.position = 0.4;
  EXPECT_FALSE(jc2.configure(jcm));
  EXPECT_FALSE(jc2.enabled());
  EXPECT_FALSE(jc.equal(jc2, 1e-12));

  // no DOF makes this false
  jcm.joint_name = "base_footprint_joint";
  EXPECT_FALSE(jc2.configure(jcm));

  // clear means not enabled
  jcm.joint_name = "head_pan_joint";
  EXPECT_TRUE(jc2.configure(jcm));
  jc2.clear();
  EXPECT_FALSE(jc2.enabled());
  EXPECT_FALSE(jc.equal(jc2, 1e-12));
}

TEST_F(LoadPlanningModelsPr2, JointConstraintsCont)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::JointConstraint jc(robot_model_);
  moveit_msgs::JointConstraint jcm;

  jcm.joint_name = "l_wrist_roll_joint";
  jcm.position = 0.0;
  jcm.tolerance_above = 0.04;
  jcm.tolerance_below = 0.02;
  jcm.weight = 1.0;

  EXPECT_TRUE(jc.configure(jcm));

  std::map<std::string, double> jvals;

  // state should have zeros, and work
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // within the above tolerance
  jvals[jcm.joint_name] = .03;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // outside the above tolerance
  jvals[jcm.joint_name] = .05;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // inside the below tolerance
  jvals[jcm.joint_name] = -.01;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // ouside the below tolerance
  jvals[jcm.joint_name] = -.03;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // now testing wrap around from positive to negative
  jcm.position = 3.14;
  EXPECT_TRUE(jc.configure(jcm));

  // testing that wrap works
  jvals[jcm.joint_name] = 3.17;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p1 = jc.decide(robot_state);
  EXPECT_TRUE(p1.satisfied);
  EXPECT_NEAR(p1.distance, 0.03, 1e-6);

  // testing that negative wrap works
  jvals[jcm.joint_name] = -3.14;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p2 = jc.decide(robot_state);
  EXPECT_TRUE(p2.satisfied);
  EXPECT_NEAR(p2.distance, 0.003185, 1e-4);

  // over bound testing
  jvals[jcm.joint_name] = 3.19;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // reverses to other direction
  // but still tested using above tolerance
  jvals[jcm.joint_name] = -3.11;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // outside of the bound given the wrap
  jvals[jcm.joint_name] = -3.09;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // lower tolerance testing
  // within bounds
  jvals[jcm.joint_name] = 3.13;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // within outside
  jvals[jcm.joint_name] = 3.11;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // testing the other direction
  jcm.position = -3.14;
  EXPECT_TRUE(jc.configure(jcm));

  // should be governed by above tolerance
  jvals[jcm.joint_name] = -3.11;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // outside upper bound
  jvals[jcm.joint_name] = -3.09;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(jc.decide(robot_state).satisfied);

  // governed by lower bound
  jvals[jcm.joint_name] = 3.13;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // outside lower bound (but would be inside upper)
  jvals[jcm.joint_name] = 3.12;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // testing wrap
  jcm.position = 6.28;
  EXPECT_TRUE(jc.configure(jcm));

  // should wrap to zero
  jvals[jcm.joint_name] = 0.0;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(jc.decide(robot_state).satisfied);

  // should wrap to close and test to be near
  moveit_msgs::JointConstraint jcm2 = jcm;
  jcm2.position = -6.28;
  kinematic_constraints::JointConstraint jc2(robot_model_);
  EXPECT_TRUE(jc.configure(jcm2));
  jc.equal(jc2, .02);
}

TEST_F(LoadPlanningModelsPr2, JointConstraintsMultiDOF)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  kinematic_constraints::JointConstraint jc(robot_model_);
  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = "world_joint";
  jcm.position = 3.14;
  jcm.tolerance_above = 0.1;
  jcm.tolerance_below = 0.05;
  jcm.weight = 1.0;

  // shouldn't work for multi-dof without local name
  EXPECT_FALSE(jc.configure(jcm));

  // this should, and function like any other single joint constraint
  jcm.joint_name = "world_joint/x";
  EXPECT_TRUE(jc.configure(jcm));

  std::map<std::string, double> jvals;
  jvals[jcm.joint_name] = 3.2;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p1 = jc.decide(robot_state);
  EXPECT_TRUE(p1.satisfied);

  jvals[jcm.joint_name] = 3.25;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p2 = jc.decide(robot_state);
  EXPECT_FALSE(p2.satisfied);

  jvals[jcm.joint_name] = -3.14;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p3 = jc.decide(robot_state);
  EXPECT_FALSE(p3.satisfied);

  // theta is continuous
  jcm.joint_name = "world_joint/theta";
  EXPECT_TRUE(jc.configure(jcm));

  jvals[jcm.joint_name] = -3.14;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p4 = jc.decide(robot_state);
  EXPECT_TRUE(p4.satisfied);

  jvals[jcm.joint_name] = 3.25;
  robot_state.setVariablePositions(jvals);
  kinematic_constraints::ConstraintEvaluationResult p5 = jc.decide(robot_state);
  EXPECT_FALSE(p5.satisfied);
}

TEST_F(LoadPlanningModelsPr2, PositionConstraintsFixed)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update(true);
  moveit::core::Transforms tf(robot_model_->getModelFrame());
  kinematic_constraints::PositionConstraint pc(robot_model_);
  moveit_msgs::PositionConstraint pcm;

  // empty certainly means false
  EXPECT_FALSE(pc.configure(pcm, tf));

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;

  // no dimensions, so no valid regions
  EXPECT_FALSE(pc.configure(pcm, tf));

  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.2;

  // no pose, so no valid region
  EXPECT_FALSE(pc.configure(pcm, tf));

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  // intentionally leaving header frame blank to test behavior
  EXPECT_FALSE(pc.configure(pcm, tf));

  pcm.header.frame_id = robot_model_->getModelFrame();
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_FALSE(pc.mobileReferenceFrame());

  EXPECT_TRUE(pc.decide(robot_state).satisfied);

  std::map<std::string, double> jvals;
  jvals["torso_lift_joint"] = 0.4;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_FALSE(pc.decide(robot_state).satisfied);
  EXPECT_TRUE(pc.equal(pc, 1e-12));

  // arbitrary offset that puts it back into the pose range
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = .15;

  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(pc.hasLinkOffset());
  EXPECT_TRUE(pc.decide(robot_state).satisfied);

  pc.clear();
  EXPECT_FALSE(pc.enabled());

  // invalid quaternion results in zero quaternion
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 0.0;

  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(pc.decide(robot_state).satisfied);
}

TEST_F(LoadPlanningModelsPr2, PositionConstraintsMobile)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  moveit::core::Transforms tf(robot_model_->getModelFrame());
  robot_state.update();

  kinematic_constraints::PositionConstraint pc(robot_model_);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;

  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.38 * 2.0;

  pcm.header.frame_id = "r_wrist_roll_link";

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.0;
  pcm.constraint_region.primitive_poses[0].position.y = 0.6;
  pcm.constraint_region.primitive_poses[0].position.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  EXPECT_FALSE(tf.isFixedFrame(pcm.link_name));
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(pc.mobileReferenceFrame());

  EXPECT_TRUE(pc.decide(robot_state).satisfied);

  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  pcm.constraint_region.primitives[0].dimensions.resize(3);
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
  EXPECT_TRUE(pc.configure(pcm, tf));

  std::map<std::string, double> jvals;
  jvals["l_shoulder_pan_joint"] = 0.4;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_TRUE(pc.decide(robot_state).satisfied);
  EXPECT_TRUE(pc.equal(pc, 1e-12));

  jvals["l_shoulder_pan_joint"] = -0.4;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_FALSE(pc.decide(robot_state).satisfied);

  // adding a second constrained region makes this work
  pcm.constraint_region.primitive_poses.resize(2);
  pcm.constraint_region.primitive_poses[1].position.x = 0.0;
  pcm.constraint_region.primitive_poses[1].position.y = 0.1;
  pcm.constraint_region.primitive_poses[1].position.z = 0.0;
  pcm.constraint_region.primitive_poses[1].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[1].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[1].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[1].orientation.w = 1.0;

  pcm.constraint_region.primitives.resize(2);
  pcm.constraint_region.primitives[1].type = shape_msgs::SolidPrimitive::BOX;
  pcm.constraint_region.primitives[1].dimensions.resize(3);
  pcm.constraint_region.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  pcm.constraint_region.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  pcm.constraint_region.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(pc.decide(robot_state, false).satisfied);
}

TEST_F(LoadPlanningModelsPr2, PositionConstraintsEquality)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::PositionConstraint pc(robot_model_);
  kinematic_constraints::PositionConstraint pc2(robot_model_);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;

  pcm.constraint_region.primitives.resize(2);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.38 * 2.0;
  pcm.constraint_region.primitives[1].type = shape_msgs::SolidPrimitive::BOX;
  pcm.constraint_region.primitives[1].dimensions.resize(3);
  pcm.constraint_region.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.0;
  pcm.constraint_region.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
  pcm.constraint_region.primitives[1].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 2.0;

  pcm.header.frame_id = "r_wrist_roll_link";
  pcm.constraint_region.primitive_poses.resize(2);
  pcm.constraint_region.primitive_poses[0].position.x = 0.0;
  pcm.constraint_region.primitive_poses[0].position.y = 0.6;
  pcm.constraint_region.primitive_poses[0].position.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.constraint_region.primitive_poses[1].position.x = 2.0;
  pcm.constraint_region.primitive_poses[1].position.y = 0.0;
  pcm.constraint_region.primitive_poses[1].position.z = 0.0;
  pcm.constraint_region.primitive_poses[1].orientation.w = 1.0;
  pcm.weight = 1.0;

  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(pc2.configure(pcm, tf));

  EXPECT_TRUE(pc.equal(pc2, .001));
  EXPECT_TRUE(pc2.equal(pc, .001));

  // putting regions in different order
  moveit_msgs::PositionConstraint pcm2 = pcm;
  pcm2.constraint_region.primitives[0] = pcm.constraint_region.primitives[1];
  pcm2.constraint_region.primitives[1] = pcm.constraint_region.primitives[0];

  pcm2.constraint_region.primitive_poses[0] = pcm.constraint_region.primitive_poses[1];
  pcm2.constraint_region.primitive_poses[1] = pcm.constraint_region.primitive_poses[0];

  EXPECT_TRUE(pc2.configure(pcm2, tf));
  EXPECT_TRUE(pc.equal(pc2, .001));
  EXPECT_TRUE(pc2.equal(pc, .001));

  // messing with one value breaks it
  pcm2.constraint_region.primitive_poses[0].position.z = .01;
  EXPECT_TRUE(pc2.configure(pcm2, tf));
  EXPECT_FALSE(pc.equal(pc2, .001));
  EXPECT_FALSE(pc2.equal(pc, .001));
  EXPECT_TRUE(pc.equal(pc2, .1));
  EXPECT_TRUE(pc2.equal(pc, .1));

  // adding an identical third shape to the last one is ok
  pcm2.constraint_region.primitive_poses[0].position.z = 0.0;
  pcm2.constraint_region.primitives.resize(3);
  pcm2.constraint_region.primitive_poses.resize(3);
  pcm2.constraint_region.primitives[2] = pcm2.constraint_region.primitives[0];
  pcm2.constraint_region.primitive_poses[2] = pcm2.constraint_region.primitive_poses[0];
  EXPECT_TRUE(pc2.configure(pcm2, tf));
  EXPECT_TRUE(pc.equal(pc2, .001));
  EXPECT_TRUE(pc2.equal(pc, .001));

  // but if we change it, it's not
  pcm2.constraint_region.primitives[2].dimensions[0] = 3.0;
  EXPECT_TRUE(pc2.configure(pcm2, tf));
  EXPECT_FALSE(pc.equal(pc2, .001));
  EXPECT_FALSE(pc2.equal(pc, .001));

  // changing the shape also changes it
  pcm2.constraint_region.primitives[2].dimensions[0] = pcm2.constraint_region.primitives[0].dimensions[0];
  pcm2.constraint_region.primitives[2].type = shape_msgs::SolidPrimitive::SPHERE;
  EXPECT_TRUE(pc2.configure(pcm2, tf));
  EXPECT_FALSE(pc.equal(pc2, .001));
}

TEST_F(LoadPlanningModelsPr2, OrientationConstraintsSimple)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit_msgs::OrientationConstraint ocm;

  EXPECT_FALSE(oc.configure(ocm, tf));

  ocm.link_name = "r_wrist_roll_link";

  // all we currently have to specify is the link name to get a valid constraint
  EXPECT_TRUE(oc.configure(ocm, tf));

  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.mobileReferenceFrame());

  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  ocm.header.frame_id = ocm.link_name;
  EXPECT_TRUE(oc.configure(ocm, tf));

  EXPECT_TRUE(oc.decide(robot_state).satisfied);
  EXPECT_TRUE(oc.equal(oc, 1e-12));
  EXPECT_TRUE(oc.mobileReferenceFrame());

  ASSERT_TRUE(oc.getLinkModel());

  geometry_msgs::Pose p = tf2::toMsg(robot_state.getGlobalLinkTransform(oc.getLinkModel()->getName()));

  ocm.orientation = p.orientation;
  ocm.header.frame_id = robot_model_->getModelFrame();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_TRUE(oc.decide(robot_state).satisfied);

  std::map<std::string, double> jvals;
  jvals["r_wrist_roll_joint"] = .05;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_TRUE(oc.decide(robot_state).satisfied);

  jvals["r_wrist_roll_joint"] = .11;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  // rotation by pi does not wrap to zero
  jvals["r_wrist_roll_joint"] = boost::math::constants::pi<double>();
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_FALSE(oc.decide(robot_state).satisfied);
}

TEST_F(LoadPlanningModelsPr2, VisibilityConstraintsSimple)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::VisibilityConstraint vc(robot_model_);
  moveit_msgs::VisibilityConstraint vcm;

  EXPECT_FALSE(vc.configure(vcm, tf));

  vcm.sensor_pose.header.frame_id = "base_footprint";
  vcm.sensor_pose.pose.position.z = -1.0;
  vcm.sensor_pose.pose.orientation.y = 1.0;

  vcm.target_pose.header.frame_id = "base_footprint";
  vcm.target_pose.pose.position.z = -2.0;
  vcm.target_pose.pose.orientation.y = 0.0;
  vcm.target_pose.pose.orientation.w = 1.0;

  vcm.target_radius = .2;
  vcm.cone_sides = 10;
  vcm.max_view_angle = 0.0;
  vcm.max_range_angle = 0.0;
  vcm.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_Z;
  vcm.weight = 1.0;

  EXPECT_TRUE(vc.configure(vcm, tf));
  // sensor and target are perfectly lined up
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  vcm.max_view_angle = .1;

  // true, even with view angle
  EXPECT_TRUE(vc.configure(vcm, tf));
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  // very slight angle, so still ok
  vcm.target_pose.pose.orientation.y = 0.03;
  vcm.target_pose.pose.orientation.w = sqrt(1 - pow(vcm.target_pose.pose.orientation.y, 2));
  EXPECT_TRUE(vc.configure(vcm, tf));
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  // a little bit more puts it over
  vcm.target_pose.pose.orientation.y = 0.06;
  vcm.target_pose.pose.orientation.w = sqrt(1 - pow(vcm.target_pose.pose.orientation.y, 2));
  EXPECT_TRUE(vc.configure(vcm, tf));
  EXPECT_FALSE(vc.decide(robot_state, true).satisfied);
}

TEST_F(LoadPlanningModelsPr2, VisibilityConstraintsPR2)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::VisibilityConstraint vc(robot_model_);
  moveit_msgs::VisibilityConstraint vcm;

  vcm.sensor_pose.header.frame_id = "narrow_stereo_optical_frame";
  vcm.sensor_pose.pose.position.z = 0.05;
  vcm.sensor_pose.pose.orientation.w = 1.0;

  vcm.target_pose.header.frame_id = "l_gripper_r_finger_tip_link";
  vcm.target_pose.pose.position.z = 0.03;
  vcm.target_pose.pose.orientation.w = 1.0;

  vcm.cone_sides = 10;
  vcm.max_view_angle = 0.0;
  vcm.max_range_angle = 0.0;
  vcm.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_Z;
  vcm.weight = 1.0;

  // false because target radius is 0.0
  EXPECT_FALSE(vc.configure(vcm, tf));

  // this is all fine
  vcm.target_radius = .05;
  EXPECT_TRUE(vc.configure(vcm, tf));
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  // this moves into collision with the cone, and should register false
  std::map<std::string, double> state_values;
  state_values["l_shoulder_lift_joint"] = .5;
  state_values["r_shoulder_pan_joint"] = .5;
  state_values["r_elbow_flex_joint"] = -1.4;
  robot_state.setVariablePositions(state_values);
  robot_state.update();
  EXPECT_FALSE(vc.decide(robot_state, true).satisfied);

  // this moves far enough away that it's fine
  state_values["r_shoulder_pan_joint"] = .4;
  robot_state.setVariablePositions(state_values);
  robot_state.update();
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  // this is in collision with the arm, but now the cone, and should be fine
  state_values["l_shoulder_lift_joint"] = 0;
  state_values["r_shoulder_pan_joint"] = .5;
  state_values["r_elbow_flex_joint"] = -.6;
  robot_state.setVariablePositions(state_values);
  robot_state.update();
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  // this shouldn't matter
  vcm.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_X;
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  robot_state.setToDefaultValues();
  robot_state.update();
  // just hits finger tip
  vcm.target_radius = .01;
  vcm.target_pose.pose.position.z = 0.00;
  vcm.target_pose.pose.position.x = 0.035;
  EXPECT_TRUE(vc.configure(vcm, tf));
  EXPECT_TRUE(vc.decide(robot_state, true).satisfied);

  // larger target means it also hits finger
  vcm.target_radius = .05;
  EXPECT_TRUE(vc.configure(vcm, tf));
  EXPECT_FALSE(vc.decide(robot_state, true).satisfied);
}

TEST_F(LoadPlanningModelsPr2, TestKinematicConstraintSet)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::KinematicConstraintSet kcs(robot_model_);
  EXPECT_TRUE(kcs.empty());

  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = "head_pan_joint";
  jcm.position = 0.4;
  jcm.tolerance_above = 0.1;
  jcm.tolerance_below = 0.05;
  jcm.weight = 1.0;

  // this is a valid constraint
  std::vector<moveit_msgs::JointConstraint> jcv;
  jcv.push_back(jcm);
  EXPECT_TRUE(kcs.add(jcv));

  // but it isn't satisfied in the default state
  EXPECT_FALSE(kcs.decide(robot_state).satisfied);

  // now it is
  std::map<std::string, double> jvals;
  jvals[jcm.joint_name] = 0.41;
  robot_state.setVariablePositions(jvals);
  robot_state.update();
  EXPECT_TRUE(kcs.decide(robot_state).satisfied);

  // adding another constraint for a different joint
  EXPECT_FALSE(kcs.empty());
  kcs.clear();
  EXPECT_TRUE(kcs.empty());
  jcv.push_back(jcm);
  jcv.back().joint_name = "head_tilt_joint";
  EXPECT_TRUE(kcs.add(jcv));

  // now this one isn't satisfied
  EXPECT_FALSE(kcs.decide(robot_state).satisfied);

  // now it is
  jvals[jcv.back().joint_name] = 0.41;
  robot_state.setVariablePositions(jvals);
  EXPECT_TRUE(kcs.decide(robot_state).satisfied);

  // changing one joint outside the bounds makes it unsatisfied
  jvals[jcv.back().joint_name] = 0.51;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(kcs.decide(robot_state).satisfied);

  // one invalid constraint makes the add return false
  kcs.clear();
  jcv.back().joint_name = "no_joint";
  EXPECT_FALSE(kcs.add(jcv));

  // but we can still evaluate it succesfully for the remaining constraint
  EXPECT_TRUE(kcs.decide(robot_state).satisfied);

  // violating the remaining good constraint changes this
  jvals["head_pan_joint"] = 0.51;
  robot_state.setVariablePositions(jvals);
  EXPECT_FALSE(kcs.decide(robot_state).satisfied);
}

TEST_F(LoadPlanningModelsPr2, TestKinematicConstraintSetEquality)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  kinematic_constraints::KinematicConstraintSet kcs(robot_model_);
  kinematic_constraints::KinematicConstraintSet kcs2(robot_model_);

  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = "head_pan_joint";
  jcm.position = 0.4;
  jcm.tolerance_above = 0.1;
  jcm.tolerance_below = 0.05;
  jcm.weight = 1.0;

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.2;

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  pcm.header.frame_id = robot_model_->getModelFrame();

  // this is a valid constraint
  std::vector<moveit_msgs::JointConstraint> jcv;
  jcv.push_back(jcm);
  EXPECT_TRUE(kcs.add(jcv));

  std::vector<moveit_msgs::PositionConstraint> pcv;
  pcv.push_back(pcm);
  EXPECT_TRUE(kcs.add(pcv, tf));

  // now adding in reverse order
  EXPECT_TRUE(kcs2.add(pcv, tf));
  EXPECT_TRUE(kcs2.add(jcv));

  // should be true
  EXPECT_TRUE(kcs.equal(kcs2, .001));
  EXPECT_TRUE(kcs2.equal(kcs, .001));

  // adding another copy of one of the constraints doesn't change anything
  jcv.push_back(jcm);
  EXPECT_TRUE(kcs2.add(jcv));

  EXPECT_TRUE(kcs.equal(kcs2, .001));
  EXPECT_TRUE(kcs2.equal(kcs, .001));

  jcm.joint_name = "head_pan_joint";
  jcm.position = 0.35;
  jcm.tolerance_above = 0.1;
  jcm.tolerance_below = 0.05;
  jcm.weight = 1.0;

  jcv.push_back(jcm);
  EXPECT_TRUE(kcs2.add(jcv));

  EXPECT_FALSE(kcs.equal(kcs2, .001));
  EXPECT_FALSE(kcs2.equal(kcs, .001));

  // but they are within this margin
  EXPECT_TRUE(kcs.equal(kcs2, .1));
  EXPECT_TRUE(kcs2.equal(kcs, .1));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
