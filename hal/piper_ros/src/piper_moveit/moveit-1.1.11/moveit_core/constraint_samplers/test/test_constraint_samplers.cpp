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

/* Author: Ioan Sucan */

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/constraint_samplers/union_constraint_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/constraint_sampler_tools.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <geometric_shapes/shape_operations.h>
#include <visualization_msgs/MarkerArray.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <functional>

#include "pr2_arm_kinematics_plugin.h"

class LoadPlanningModelsPr2 : public testing::Test
{
protected:
  kinematics::KinematicsBasePtr getKinematicsSolverRightArm(const moveit::core::JointModelGroup* /*jmg*/)
  {
    {
      return pr2_kinematics_plugin_right_arm_;
    }
  }

  kinematics::KinematicsBasePtr getKinematicsSolverLeftArm(const moveit::core::JointModelGroup* /*jmg*/)
  {
    {
      return pr2_kinematics_plugin_left_arm_;
    }
  }

  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("pr2");

    pr2_kinematics_plugin_right_arm_ = std::make_shared<pr2_arm_kinematics::PR2ArmKinematicsPlugin>();
    pr2_kinematics_plugin_right_arm_->initialize(*robot_model_, "right_arm", "torso_lift_link", { "r_wrist_roll_link" },
                                                 .01);

    pr2_kinematics_plugin_left_arm_ = std::make_shared<pr2_arm_kinematics::PR2ArmKinematicsPlugin>();
    pr2_kinematics_plugin_left_arm_->initialize(*robot_model_, "left_arm", "torso_lift_link", { "l_wrist_roll_link" },
                                                .01);

    func_right_arm_ = [this](const moveit::core::JointModelGroup* jmg) { return getKinematicsSolverRightArm(jmg); };
    func_left_arm_ = [this](const moveit::core::JointModelGroup* jmg) { return getKinematicsSolverLeftArm(jmg); };

    std::map<std::string, moveit::core::SolverAllocatorFn> allocators;
    allocators["right_arm"] = func_right_arm_;
    allocators["left_arm"] = func_left_arm_;
    allocators["whole_body"] = func_right_arm_;
    allocators["base"] = func_left_arm_;

    robot_model_->setKinematicsAllocators(allocators);

    ps_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  };

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr ps_;
  pr2_arm_kinematics::PR2ArmKinematicsPluginPtr pr2_kinematics_plugin_right_arm_;
  pr2_arm_kinematics::PR2ArmKinematicsPluginPtr pr2_kinematics_plugin_left_arm_;
  moveit::core::SolverAllocatorFn func_right_arm_;
  moveit::core::SolverAllocatorFn func_left_arm_;
};

TEST_F(LoadPlanningModelsPr2, JointConstraintsSamplerSimple)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();

  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();

  kinematic_constraints::JointConstraint jc1(robot_model_);
  moveit_msgs::JointConstraint jcm1;
  // leaving off joint name
  jcm1.position = 0.42;
  jcm1.tolerance_above = 0.01;
  jcm1.tolerance_below = 0.05;
  jcm1.weight = 1.0;
  EXPECT_FALSE(jc1.configure(jcm1));

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);

  constraint_samplers::JointConstraintSampler jcs(ps_, "right_arm");
  // no valid constraints
  EXPECT_FALSE(jcs.configure(js));

  // testing that this does the right thing
  jcm1.joint_name = "r_shoulder_pan_joint";
  EXPECT_TRUE(jc1.configure(jcm1));
  js.push_back(jc1);
  EXPECT_TRUE(jcs.configure(js));
  EXPECT_EQ(jcs.getConstrainedJointCount(), 1u);
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 6u);
  EXPECT_TRUE(jcs.sample(ks, ks, 1));

  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc1.decide(ks).satisfied);
  }

  // redoing the configure leads to 6 unconstrained variables as well
  EXPECT_TRUE(jcs.configure(js));
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 6u);

  kinematic_constraints::JointConstraint jc2(robot_model_);

  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "r_shoulder_pan_joint";
  jcm2.position = 0.54;
  jcm2.tolerance_above = 0.01;
  jcm2.tolerance_below = 0.01;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc2);

  // creating a constraint that conflicts with the other (leaves no sampleable region)
  EXPECT_FALSE(jcs.configure(js));
  EXPECT_FALSE(jcs.sample(ks, ks_const, 1));

  // we can't sample for a different group
  constraint_samplers::JointConstraintSampler jcs2(ps_, "arms");
  jcs2.configure(js);
  EXPECT_FALSE(jcs2.sample(ks, ks_const, 1));

  // not ok to not have any references to joints in this group in the constraints
  constraint_samplers::JointConstraintSampler jcs3(ps_, "left_arm");
  EXPECT_FALSE(jcs3.configure(js));

  // testing that the most restrictive bounds are used
  js.clear();

  jcm1.position = .4;
  jcm1.tolerance_above = .05;
  jcm1.tolerance_below = .05;
  jcm2.position = .4;
  jcm2.tolerance_above = .1;
  jcm2.tolerance_below = .1;
  EXPECT_TRUE(jc1.configure(jcm1));
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc1);
  js.push_back(jc2);

  EXPECT_TRUE(jcs.configure(js));

  // should always be within narrower constraints
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc1.decide(ks).satisfied);
  }

  // too narrow range outside of joint limits
  js.clear();

  jcm1.position = -3.1;
  jcm1.tolerance_above = .05;
  jcm1.tolerance_below = .05;

  // the joint configuration corrects this
  EXPECT_TRUE(jc1.configure(jcm1));
  js.push_back(jc1);
  EXPECT_TRUE(jcs.configure(js));

  // partially overlapping regions will also work
  js.clear();
  jcm1.position = .35;
  jcm1.tolerance_above = .05;
  jcm1.tolerance_below = .05;
  jcm2.position = .45;
  jcm2.tolerance_above = .05;
  jcm2.tolerance_below = .05;
  EXPECT_TRUE(jc1.configure(jcm1));
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc1);
  js.push_back(jc2);

  // leads to a min and max of .4, so all samples should be exactly .4
  EXPECT_TRUE(jcs.configure(js));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    std::map<std::string, double> var_values;
    EXPECT_NEAR(ks.getVariablePosition("r_shoulder_pan_joint"), .4, std::numeric_limits<double>::epsilon());
    EXPECT_TRUE(jc1.decide(ks).satisfied);
    EXPECT_TRUE(jc2.decide(ks).satisfied);
  }

  // this leads to a larger sampleable region
  jcm1.position = .38;
  jcm2.position = .42;
  EXPECT_TRUE(jc1.configure(jcm1));
  EXPECT_TRUE(jc2.configure(jcm2));
  js.push_back(jc1);
  js.push_back(jc2);
  EXPECT_TRUE(jcs.configure(js));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc1.decide(ks).satisfied);
    EXPECT_TRUE(jc2.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, IKConstraintsSamplerSimple)
{
  moveit::core::Transforms& tf = ps_->getTransformsNonConst();

  kinematic_constraints::PositionConstraint pc(robot_model_);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  EXPECT_FALSE(pc.configure(pcm, tf));

  constraint_samplers::IKConstraintSampler ik_bad(ps_, "l_arm");
  EXPECT_FALSE(ik_bad.isValid());

  constraint_samplers::IKConstraintSampler iks(ps_, "left_arm");
  EXPECT_FALSE(iks.configure(constraint_samplers::IKSamplingPose()));
  EXPECT_FALSE(iks.isValid());

  EXPECT_FALSE(iks.configure(constraint_samplers::IKSamplingPose(pc)));

  pcm.header.frame_id = robot_model_->getModelFrame();
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(iks.configure(constraint_samplers::IKSamplingPose(pc)));

  // ik link not in this group
  constraint_samplers::IKConstraintSampler ik_bad_2(ps_, "right_arm");
  EXPECT_FALSE(ik_bad_2.configure(constraint_samplers::IKSamplingPose(pc)));
  EXPECT_FALSE(ik_bad_2.isValid());

  // not the ik link
  pcm.link_name = "l_shoulder_pan_link";
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_FALSE(iks.configure(constraint_samplers::IKSamplingPose(pc)));

  // solver for base doesn't cover group
  constraint_samplers::IKConstraintSampler ik_base(ps_, "base");
  pcm.link_name = "l_wrist_roll_link";
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_FALSE(ik_base.configure(constraint_samplers::IKSamplingPose(pc)));
  EXPECT_FALSE(ik_base.isValid());

  // shouldn't work as no direct constraint solver
  constraint_samplers::IKConstraintSampler ik_arms(ps_, "arms");
  EXPECT_FALSE(iks.isValid());
}

TEST_F(LoadPlanningModelsPr2, OrientationConstraintsSampler)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();
  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  moveit::core::Transforms& tf = ps_->getTransformsNonConst();

  kinematic_constraints::OrientationConstraint oc(robot_model_);
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = ocm.link_name;
  ocm.orientation.x = 0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = 0.5;
  ocm.orientation.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;

  EXPECT_TRUE(oc.configure(ocm, tf));

  bool p1 = oc.decide(ks).satisfied;
  EXPECT_FALSE(p1);

  ocm.header.frame_id = robot_model_->getModelFrame();
  EXPECT_TRUE(oc.configure(ocm, tf));

  constraint_samplers::IKConstraintSampler iks(ps_, "right_arm");
  EXPECT_TRUE(iks.configure(constraint_samplers::IKSamplingPose(oc)));
  for (int t = 0; t < 100; ++t)
  {
    ks.update();
    EXPECT_TRUE(iks.sample(ks, ks_const, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }

  // test another parameterization for orientation constraints
  ocm.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
  EXPECT_TRUE(oc.configure(ocm, tf));

  EXPECT_TRUE(iks.configure(constraint_samplers::IKSamplingPose(oc)));
  for (int t = 0; t < 100; ++t)
  {
    ks.update();
    EXPECT_TRUE(iks.sample(ks, ks_const, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, IKConstraintsSamplerValid)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();
  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  moveit::core::Transforms& tf = ps_->getTransformsNonConst();

  kinematic_constraints::PositionConstraint pc(robot_model_);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.header.frame_id = robot_model_->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  EXPECT_TRUE(pc.configure(pcm, tf));

  kinematic_constraints::OrientationConstraint oc(robot_model_);
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;

  EXPECT_TRUE(oc.configure(ocm, tf));

  constraint_samplers::IKConstraintSampler iks1(ps_, "left_arm");
  EXPECT_TRUE(iks1.configure(constraint_samplers::IKSamplingPose(pc, oc)));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(iks1.sample(ks, ks_const, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }

  constraint_samplers::IKConstraintSampler iks2(ps_, "left_arm");
  EXPECT_TRUE(iks2.configure(constraint_samplers::IKSamplingPose(pc)));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(iks2.sample(ks, ks_const, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
  }

  constraint_samplers::IKConstraintSampler iks3(ps_, "left_arm");
  EXPECT_TRUE(iks3.configure(constraint_samplers::IKSamplingPose(oc)));
  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(iks3.sample(ks, ks_const, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, UnionConstraintSampler)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();

  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  moveit::core::Transforms& tf = ps_->getTransformsNonConst();

  kinematic_constraints::JointConstraint jc1(robot_model_);

  std::map<std::string, double> state_values;

  moveit_msgs::JointConstraint torso_constraint;
  torso_constraint.joint_name = "torso_lift_joint";
  torso_constraint.position = ks.getVariablePosition("torso_lift_joint");
  torso_constraint.tolerance_above = 0.01;
  torso_constraint.tolerance_below = 0.01;
  torso_constraint.weight = 1.0;
  EXPECT_TRUE(jc1.configure(torso_constraint));

  kinematic_constraints::JointConstraint jc2(robot_model_);
  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "r_elbow_flex_joint";
  jcm2.position = ks.getVariablePosition("r_elbow_flex_joint");
  jcm2.tolerance_above = 0.01;
  jcm2.tolerance_below = 0.01;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));

  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

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

  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);

  constraint_samplers::JointConstraintSamplerPtr jcsp(
      new constraint_samplers::JointConstraintSampler(ps_, "arms_and_torso"));
  EXPECT_TRUE(jcsp->configure(js));

  std::vector<kinematic_constraints::JointConstraint> js2;
  js2.push_back(jc2);

  constraint_samplers::JointConstraintSamplerPtr jcsp2(new constraint_samplers::JointConstraintSampler(ps_, "arms"));
  EXPECT_TRUE(jcsp2->configure(js2));

  kinematic_constraints::PositionConstraint pc(robot_model_);
  EXPECT_TRUE(pc.configure(pcm, tf));

  kinematic_constraints::OrientationConstraint oc(robot_model_);
  EXPECT_TRUE(oc.configure(ocm, tf));

  constraint_samplers::IKConstraintSamplerPtr iksp(new constraint_samplers::IKConstraintSampler(ps_, "left_arm"));
  EXPECT_TRUE(iksp->configure(constraint_samplers::IKSamplingPose(pc, oc)));
  EXPECT_TRUE(iksp->isValid());

  std::vector<constraint_samplers::ConstraintSamplerPtr> cspv;
  cspv.push_back(jcsp2);
  cspv.push_back(iksp);
  cspv.push_back(jcsp);

  constraint_samplers::UnionConstraintSampler ucs(ps_, "arms_and_torso", cspv);

  // should have reordered to place whole body first
  constraint_samplers::JointConstraintSampler* jcs =
      dynamic_cast<constraint_samplers::JointConstraintSampler*>(ucs.getSamplers()[0].get());
  EXPECT_TRUE(jcs);
  EXPECT_EQ(jcs->getJointModelGroup()->getName(), "arms_and_torso");

  constraint_samplers::JointConstraintSampler* jcs2 =
      dynamic_cast<constraint_samplers::JointConstraintSampler*>(ucs.getSamplers()[1].get());
  EXPECT_TRUE(jcs2);
  EXPECT_EQ(jcs2->getJointModelGroup()->getName(), "arms");

  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(ucs.sample(ks, ks_const, 100));
    ks.update();
    ks.updateLinkTransforms();  // Returned samples have dirty link transforms.
    ks_const.update();
    EXPECT_TRUE(jc1.decide(ks).satisfied);
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(pc.decide(ks).satisfied);
  }

  // now we add a position constraint on right arm
  pcm.link_name = "r_wrist_roll_link";
  ocm.link_name = "r_wrist_roll_link";
  cspv.clear();

  kinematic_constraints::PositionConstraint pc2(robot_model_);
  EXPECT_TRUE(pc2.configure(pcm, tf));

  kinematic_constraints::OrientationConstraint oc2(robot_model_);
  EXPECT_TRUE(oc2.configure(ocm, tf));

  constraint_samplers::IKConstraintSamplerPtr iksp2(new constraint_samplers::IKConstraintSampler(ps_, "right_arm"));
  EXPECT_TRUE(iksp2->configure(constraint_samplers::IKSamplingPose(pc2, oc2)));
  EXPECT_TRUE(iksp2->isValid());

  // totally disjoint, so should break ties based on alphabetical order
  cspv.clear();
  cspv.push_back(iksp2);
  cspv.push_back(iksp);

  constraint_samplers::UnionConstraintSampler ucs2(ps_, "arms_and_torso", cspv);

  constraint_samplers::IKConstraintSampler* ikcs_test =
      dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs2.getSamplers()[0].get());
  ASSERT_TRUE(ikcs_test);
  EXPECT_EQ(ikcs_test->getJointModelGroup()->getName(), "left_arm");

  // now we make left depends on right, right should stay first
  pcm.link_name = "l_wrist_roll_link";
  ocm.link_name = "l_wrist_roll_link";
  pcm.header.frame_id = "r_wrist_roll_link";
  ocm.header.frame_id = "r_wrist_roll_link";
  EXPECT_TRUE(pc.configure(pcm, tf));
  EXPECT_TRUE(oc.configure(ocm, tf));
  ASSERT_TRUE(iksp->configure(constraint_samplers::IKSamplingPose(pc, oc)));

  cspv.clear();
  cspv.push_back(iksp2);
  cspv.push_back(iksp);

  constraint_samplers::UnionConstraintSampler ucs3(ps_, "arms_and_torso", cspv);

  ikcs_test = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs3.getSamplers()[0].get());
  EXPECT_TRUE(ikcs_test);
  EXPECT_EQ(ikcs_test->getJointModelGroup()->getName(), "right_arm");
}

TEST_F(LoadPlanningModelsPr2, PoseConstraintSamplerManager)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();
  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  kinematic_constraints::PositionConstraint pc(robot_model_);

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.header.frame_id = robot_model_->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;

  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;
  c.position_constraints.push_back(pcm);
  c.orientation_constraints.push_back(ocm);

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", c);
  EXPECT_TRUE(s != nullptr);
  constraint_samplers::IKConstraintSampler* iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(s.get());
  ASSERT_TRUE(iks);
  ASSERT_TRUE(static_cast<bool>(iks->getPositionConstraint()));
  ASSERT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));

  static const int NT = 100;
  int succ = 0;
  for (int t = 0; t < NT; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 100));
    EXPECT_TRUE(iks->getPositionConstraint()->decide(ks).satisfied);
    EXPECT_TRUE(iks->getOrientationConstraint()->decide(ks).satisfied);
    if (s->sample(ks, ks_const, 1))
      succ++;
  }
  ROS_INFO("Success rate for IK Constraint Sampler with position & orientation constraints for one arm: %lf",
           (double)succ / (double)NT);

  // add additional ocm with smaller volume
  ocm.absolute_x_axis_tolerance = 0.1;

  c.orientation_constraints.push_back(ocm);

  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", c);
  EXPECT_TRUE(s != nullptr);

  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(s.get());
  ASSERT_TRUE(iks);
  ASSERT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));
  EXPECT_NEAR(iks->getOrientationConstraint()->getXAxisTolerance(), .1, .0001);
}

TEST_F(LoadPlanningModelsPr2, JointVersusPoseConstraintSamplerManager)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();

  moveit_msgs::Constraints con;
  con.joint_constraints.resize(1);

  con.joint_constraints[0].joint_name = "l_shoulder_pan_joint";
  con.joint_constraints[0].position = 0.54;
  con.joint_constraints[0].tolerance_above = 0.01;
  con.joint_constraints[0].tolerance_below = 0.01;
  con.joint_constraints[0].weight = 1.0;

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "right_arm", con);
  EXPECT_FALSE(static_cast<bool>(s));
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));

  con.joint_constraints.resize(7);

  con.joint_constraints[1].joint_name = "l_shoulder_lift_joint";
  con.joint_constraints[1].position = 0.54;
  con.joint_constraints[1].tolerance_above = 0.01;
  con.joint_constraints[1].tolerance_below = 0.01;
  con.joint_constraints[1].weight = 1.0;

  con.joint_constraints[2].joint_name = "l_upper_arm_roll_joint";
  con.joint_constraints[2].position = 0.54;
  con.joint_constraints[2].tolerance_above = 0.01;
  con.joint_constraints[2].tolerance_below = 0.01;
  con.joint_constraints[2].weight = 1.0;

  con.joint_constraints[3].joint_name = "l_elbow_flex_joint";
  con.joint_constraints[3].position = -0.54;
  con.joint_constraints[3].tolerance_above = 0.01;
  con.joint_constraints[3].tolerance_below = 0.01;
  con.joint_constraints[3].weight = 1.0;

  con.joint_constraints[4].joint_name = "l_forearm_roll_joint";
  con.joint_constraints[4].position = 0.54;
  con.joint_constraints[4].tolerance_above = 0.01;
  con.joint_constraints[4].tolerance_below = 0.01;
  con.joint_constraints[4].weight = 1.0;

  con.joint_constraints[5].joint_name = "l_wrist_flex_joint";
  con.joint_constraints[5].position = -0.54;
  con.joint_constraints[5].tolerance_above = 0.05;
  con.joint_constraints[5].tolerance_below = 0.05;
  con.joint_constraints[5].weight = 1.0;

  // an extra constraint on one link, but this shouldn't change anything
  con.joint_constraints[6].joint_name = "l_wrist_flex_joint";
  con.joint_constraints[6].position = -0.56;
  con.joint_constraints[6].tolerance_above = 0.01;
  con.joint_constraints[6].tolerance_below = 0.01;
  con.joint_constraints[6].weight = 1.0;

  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));

  con.position_constraints.resize(1);

  // intentionally making wrong wrist
  con.position_constraints[0].link_name = "r_wrist_roll_link";
  con.position_constraints[0].target_point_offset.x = 0;
  con.position_constraints[0].target_point_offset.y = 0;
  con.position_constraints[0].target_point_offset.z = 0;
  con.position_constraints[0].constraint_region.primitives.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  con.position_constraints[0].constraint_region.primitives[0].dimensions.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].dimensions[0] = 0.001;

  con.position_constraints[0].header.frame_id = robot_model_->getModelFrame();

  con.position_constraints[0].constraint_region.primitive_poses.resize(1);
  con.position_constraints[0].constraint_region.primitive_poses[0].position.x = 0.55;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.y = 0.2;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.z = 1.25;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.x = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.y = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.z = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1.0;
  con.position_constraints[0].weight = 1.0;

  // this still works, but we should get a JointConstraintSampler
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  constraint_samplers::JointConstraintSampler* jcs =
      dynamic_cast<constraint_samplers::JointConstraintSampler*>(s.get());
  EXPECT_TRUE(jcs);

  con.position_constraints[0].link_name = "l_wrist_roll_link";
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  jcs = dynamic_cast<constraint_samplers::JointConstraintSampler*>(s.get());
  EXPECT_FALSE(jcs);
  constraint_samplers::IKConstraintSampler* iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(s.get());
  EXPECT_FALSE(iks);

  // we should get a union constraint sampler
  constraint_samplers::UnionConstraintSampler* ucs =
      dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  EXPECT_TRUE(ucs);

  con.orientation_constraints.resize(1);

  // again, screwing this up intentionally
  con.orientation_constraints[0].link_name = "r_wrist_roll_link";
  con.orientation_constraints[0].header.frame_id = robot_model_->getModelFrame();
  con.orientation_constraints[0].orientation.x = 0.0;
  con.orientation_constraints[0].orientation.y = 0.0;
  con.orientation_constraints[0].orientation.z = 0.0;
  con.orientation_constraints[0].orientation.w = 1.0;
  con.orientation_constraints[0].absolute_x_axis_tolerance = 0.2;
  con.orientation_constraints[0].absolute_y_axis_tolerance = 0.1;
  con.orientation_constraints[0].absolute_z_axis_tolerance = 0.4;
  con.orientation_constraints[0].weight = 1.0;

  // we still get an IK sampler with just the position constraint
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  ucs = dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  ASSERT_TRUE(ucs);
  jcs = dynamic_cast<constraint_samplers::JointConstraintSampler*>(ucs->getSamplers()[0].get());
  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());

  ASSERT_TRUE(iks);
  ASSERT_TRUE(jcs);
  EXPECT_TRUE(static_cast<bool>(iks->getPositionConstraint()));
  EXPECT_FALSE(iks->getOrientationConstraint());

  con.orientation_constraints[0].link_name = "l_wrist_roll_link";

  // now they both are good
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  ucs = dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());
  ASSERT_TRUE(iks);
  EXPECT_TRUE(static_cast<bool>(iks->getPositionConstraint()));
  EXPECT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));

  // now just the orientation constraint is good
  con.position_constraints[0].link_name = "r_wrist_roll_link";
  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  ASSERT_TRUE(static_cast<bool>(s));
  ucs = dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  iks = dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());
  ASSERT_TRUE(iks);
  EXPECT_FALSE(iks->getPositionConstraint());
  EXPECT_TRUE(static_cast<bool>(iks->getOrientationConstraint()));

  // now if we constraint all the joints, we get a joint constraint sampler
  con.joint_constraints.resize(8);
  con.joint_constraints[7].joint_name = "l_wrist_roll_joint";
  con.joint_constraints[7].position = 0.54;
  con.joint_constraints[7].tolerance_above = 0.01;
  con.joint_constraints[7].tolerance_below = 0.01;
  con.joint_constraints[7].weight = 1.0;

  s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "left_arm", con);
  EXPECT_TRUE(static_cast<bool>(s));
  jcs = dynamic_cast<constraint_samplers::JointConstraintSampler*>(s.get());
  ASSERT_TRUE(jcs);
}

TEST_F(LoadPlanningModelsPr2, MixedJointAndIkSamplerManager)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();
  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  moveit_msgs::Constraints con;
  con.joint_constraints.resize(1);

  con.joint_constraints[0].joint_name = "torso_lift_joint";
  con.joint_constraints[0].position = ks.getVariablePosition("torso_lift_joint");
  con.joint_constraints[0].tolerance_above = 0.01;
  con.joint_constraints[0].tolerance_below = 0.01;
  con.joint_constraints[0].weight = 1.0;

  kinematic_constraints::JointConstraint jc(robot_model_);
  EXPECT_TRUE(jc.configure(con.joint_constraints[0]));

  con.position_constraints.resize(1);

  con.position_constraints[0].link_name = "l_wrist_roll_link";
  con.position_constraints[0].target_point_offset.x = 0;
  con.position_constraints[0].target_point_offset.y = 0;
  con.position_constraints[0].target_point_offset.z = 0;
  con.position_constraints[0].constraint_region.primitives.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  con.position_constraints[0].constraint_region.primitives[0].dimensions.resize(1);
  con.position_constraints[0].constraint_region.primitives[0].dimensions[0] = 0.001;

  con.position_constraints[0].constraint_region.primitive_poses.resize(1);
  con.position_constraints[0].constraint_region.primitive_poses[0].position.x = 0.55;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.y = 0.2;
  con.position_constraints[0].constraint_region.primitive_poses[0].position.z = 1.25;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.x = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.y = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.z = 0.0;
  con.position_constraints[0].constraint_region.primitive_poses[0].orientation.w = 1.0;
  con.position_constraints[0].weight = 1.0;

  con.position_constraints[0].header.frame_id = robot_model_->getModelFrame();

  con.orientation_constraints.resize(1);
  con.orientation_constraints[0].link_name = "l_wrist_roll_link";
  con.orientation_constraints[0].header.frame_id = robot_model_->getModelFrame();
  con.orientation_constraints[0].orientation.x = 0.0;
  con.orientation_constraints[0].orientation.y = 0.0;
  con.orientation_constraints[0].orientation.z = 0.0;
  con.orientation_constraints[0].orientation.w = 1.0;
  con.orientation_constraints[0].absolute_x_axis_tolerance = 0.2;
  con.orientation_constraints[0].absolute_y_axis_tolerance = 0.1;
  con.orientation_constraints[0].absolute_z_axis_tolerance = 0.4;
  con.orientation_constraints[0].weight = 1.0;

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "arms_and_torso", con);

  constraint_samplers::UnionConstraintSampler* ucs =
      dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  ASSERT_TRUE(ucs);

  constraint_samplers::IKConstraintSampler* ikcs_test =
      dynamic_cast<constraint_samplers::IKConstraintSampler*>(ucs->getSamplers()[1].get());
  ASSERT_TRUE(ikcs_test);

  for (int t = 0; t < 1; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 100));
    EXPECT_TRUE(jc.decide(ks).satisfied);
    EXPECT_TRUE(ikcs_test->getPositionConstraint()->decide(ks).satisfied);
    EXPECT_TRUE(ikcs_test->getOrientationConstraint()->decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, SubgroupJointConstraintsSamplerManager)
{
  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();
  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  kinematic_constraints::JointConstraint jc1(robot_model_);
  moveit_msgs::JointConstraint jcm1;
  jcm1.joint_name = "head_pan_joint";
  jcm1.position = 0.42;
  jcm1.tolerance_above = 0.01;
  jcm1.tolerance_below = 0.05;
  jcm1.weight = 1.0;
  EXPECT_TRUE(jc1.configure(jcm1));

  kinematic_constraints::JointConstraint jc2(robot_model_);
  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "l_shoulder_pan_joint";
  jcm2.position = 0.9;
  jcm2.tolerance_above = 0.1;
  jcm2.tolerance_below = 0.05;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));

  kinematic_constraints::JointConstraint jc3(robot_model_);
  moveit_msgs::JointConstraint jcm3;
  jcm3.joint_name = "r_wrist_roll_joint";
  jcm3.position = 0.7;
  jcm3.tolerance_above = 0.14;
  jcm3.tolerance_below = 0.005;
  jcm3.weight = 1.0;
  EXPECT_TRUE(jc3.configure(jcm3));

  kinematic_constraints::JointConstraint jc4(robot_model_);
  moveit_msgs::JointConstraint jcm4;
  jcm4.joint_name = "torso_lift_joint";
  jcm4.position = 0.2;
  jcm4.tolerance_above = 0.09;
  jcm4.tolerance_below = 0.01;
  jcm4.weight = 1.0;
  EXPECT_TRUE(jc4.configure(jcm4));

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);
  js.push_back(jc2);
  js.push_back(jc3);
  js.push_back(jc4);

  constraint_samplers::JointConstraintSampler jcs(ps_, "arms");
  jcs.configure(js);
  EXPECT_EQ(jcs.getConstrainedJointCount(), 2u);
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 12u);

  for (int t = 0; t < 100; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks, ks_const, 1));
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(jc3.decide(ks).satisfied);
  }

  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;

  // no constraints should give no sampler
  constraint_samplers::ConstraintSamplerPtr s0 =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "arms", c);
  EXPECT_TRUE(s0 == nullptr);

  // add the constraints
  c.joint_constraints.push_back(jcm1);
  c.joint_constraints.push_back(jcm2);
  c.joint_constraints.push_back(jcm3);
  c.joint_constraints.push_back(jcm4);

  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "arms", c);
  EXPECT_TRUE(s != nullptr);

  // test the generated sampler
  for (int t = 0; t < 1000; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 1));
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(jc3.decide(ks).satisfied);
  }
}

TEST_F(LoadPlanningModelsPr2, SubgroupPoseConstraintsSampler)
{
  moveit_msgs::Constraints c;

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;

  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

  pcm.header.frame_id = robot_model_->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;
  c.position_constraints.push_back(pcm);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  moveit::core::Transforms& tf = ps_->getTransformsNonConst();
  constraint_samplers::ConstraintSamplerPtr s =
      constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(ps_, "arms", c);
  EXPECT_TRUE(static_cast<bool>(s));
  constraint_samplers::UnionConstraintSampler* ucs =
      dynamic_cast<constraint_samplers::UnionConstraintSampler*>(s.get());
  EXPECT_TRUE(ucs);

  kinematic_constraints::KinematicConstraintSet kset(robot_model_);
  kset.add(c, tf);

  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();
  moveit::core::RobotState ks_const(robot_model_);
  ks_const.setToDefaultValues();
  ks_const.update();

  static const int NT = 100;
  int succ = 0;
  for (int t = 0; t < NT; ++t)
  {
    EXPECT_TRUE(s->sample(ks, ks_const, 1000));
    EXPECT_TRUE(kset.decide(ks).satisfied);
    if (s->sample(ks, ks_const, 1))
      succ++;
  }
  ROS_INFO_NAMED("pr2_arm_kinematics_plugin",
                 "Success rate for IK Constraint Sampler with position & orientation constraints for both arms: %lf",
                 (double)succ / (double)NT);
}

TEST_F(LoadPlanningModelsPr2, JointConstraintsSamplerSeeded)
{
  ros::param::set("~joint_constraint_sampler_random_seed", 12345);
  constraint_samplers::JointConstraintSampler seeded_sampler1(ps_, "right_arm");
  kinematic_constraints::JointConstraint jc(robot_model_);
  moveit_msgs::JointConstraint jcm;
  jcm.position = 0.42;
  jcm.tolerance_above = 0.01;
  jcm.tolerance_below = 0.05;
  jcm.weight = 1.0;
  jcm.joint_name = "r_shoulder_pan_joint";
  EXPECT_TRUE(jc.configure(jcm));
  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc);
  EXPECT_TRUE(seeded_sampler1.configure(js));

  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  EXPECT_TRUE(seeded_sampler1.sample(ks, ks, 1));
  const double* joint_positions = ks.getVariablePositions();
  const std::vector<double> joint_positions_v(joint_positions, joint_positions + ks.getVariableCount());

  constraint_samplers::JointConstraintSampler seeded_sampler2(ps_, "right_arm");
  EXPECT_TRUE(seeded_sampler2.configure(js));
  ks.setToDefaultValues();
  EXPECT_TRUE(seeded_sampler2.sample(ks, ks, 1));
  const double* joint_positions2 = ks.getVariablePositions();
  const std::vector<double> joint_positions_v2(joint_positions2, joint_positions2 + ks.getVariableCount());
  using namespace testing;
  EXPECT_THAT(joint_positions_v, ContainerEq(joint_positions_v2));

  ros::param::del("~joint_constraint_sampler_random_seed");
  constraint_samplers::JointConstraintSampler seeded_sampler3(ps_, "right_arm");
  EXPECT_TRUE(seeded_sampler3.configure(js));
  ks.setToDefaultValues();
  EXPECT_TRUE(seeded_sampler3.sample(ks, ks, 5));
  const double* joint_positions3 = ks.getVariablePositions();
  const std::vector<double> joint_positions_v3(joint_positions3, joint_positions3 + ks.getVariableCount());
  EXPECT_THAT(joint_positions_v, Not(ContainerEq(joint_positions_v3)));
  EXPECT_THAT(joint_positions_v2, Not(ContainerEq(joint_positions_v3)));
}

TEST_F(LoadPlanningModelsPr2, IKConstraintsSamplerSeeded)
{
  ros::param::set("~ik_constraint_sampler_random_seed", 12345);
  kinematic_constraints::PositionConstraint pc(robot_model_);
  moveit_msgs::PositionConstraint pcm;

  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(1);
  pcm.constraint_region.primitives[0].dimensions[0] = 0.001;

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
  moveit::core::Transforms& tf = ps_->getTransformsNonConst();
  EXPECT_TRUE(pc.configure(pcm, tf));

  constraint_samplers::IKConstraintSampler seeded_sampler1(ps_, "left_arm");
  EXPECT_TRUE(seeded_sampler1.configure(constraint_samplers::IKSamplingPose(pc)));

  moveit::core::RobotState ks(robot_model_);
  ks.setToDefaultValues();
  ks.update();

  EXPECT_TRUE(seeded_sampler1.sample(ks, ks, 1));
  ks.update();
  bool found = false;
  const Eigen::Isometry3d root_to_left_tool1 = ks.getFrameTransform("l_gripper_tool_frame", &found);
  EXPECT_TRUE(found);

  constraint_samplers::IKConstraintSampler seeded_sampler2(ps_, "left_arm");
  EXPECT_TRUE(seeded_sampler2.configure(constraint_samplers::IKSamplingPose(pc)));
  ks.setToDefaultValues();
  ks.update();
  EXPECT_TRUE(seeded_sampler2.sample(ks, ks, 1));
  ks.update();
  found = false;
  const Eigen::Isometry3d root_to_left_tool2 = ks.getFrameTransform("l_gripper_tool_frame", &found);
  EXPECT_TRUE(found);

  ros::param::del("~ik_constraint_sampler_random_seed");
  constraint_samplers::IKConstraintSampler seeded_sampler3(ps_, "left_arm");
  EXPECT_TRUE(seeded_sampler3.configure(constraint_samplers::IKSamplingPose(pc)));
  ks.setToDefaultValues();
  ks.update();
  EXPECT_TRUE(seeded_sampler3.sample(ks, ks, 5));
  ks.update();
  found = false;
  const Eigen::Isometry3d root_to_left_tool3 = ks.getFrameTransform("l_gripper_tool_frame", &found);
  EXPECT_TRUE(found);

  EXPECT_TRUE((root_to_left_tool1 * root_to_left_tool2.inverse()).matrix().isIdentity(1e-7));
  EXPECT_FALSE((root_to_left_tool1 * root_to_left_tool3.inverse()).matrix().isIdentity(1e-7));
  EXPECT_FALSE((root_to_left_tool2 * root_to_left_tool3.inverse()).matrix().isIdentity(1e-7));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_constraint_samplers");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
