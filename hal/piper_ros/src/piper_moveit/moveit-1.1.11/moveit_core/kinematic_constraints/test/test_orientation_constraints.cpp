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

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <gtest/gtest.h>

#include <urdf_parser/urdf_parser.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <boost/math/constants/constants.hpp>

class SphericalRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    moveit::core::RobotModelBuilder builder("robot", "base_link");
    geometry_msgs::Pose origin;
    origin.orientation.w = 1;
    builder.addChain("base_link->roll", "revolute", { origin }, urdf::Vector3(1, 0, 0));
    builder.addChain("roll->pitch", "revolute", { origin }, urdf::Vector3(0, 1, 0));
    builder.addChain("pitch->yaw", "revolute", { origin }, urdf::Vector3(0, 0, 1));
    ASSERT_TRUE(builder.isValid());
    robot_model_ = builder.build();
  }

  std::map<std::string, double> getJointValues(const double roll, const double pitch, const double yaw)
  {
    std::map<std::string, double> jvals;
    jvals["base_link-roll-joint"] = roll;
    jvals["roll-pitch-joint"] = pitch;
    jvals["pitch-yaw-joint"] = yaw;
    return jvals;
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
};

class FloatingJointRobot : public testing::Test
{
protected:
  /** A robot model with a single floating joint will be created. **/
  moveit::core::RobotModelPtr robot_model_;

  /** Test data
   *
   * The test data represents valid orientations for an absolute tolerance of 0.5 around x, y and z.
   *
   * The first matrix contains valid orientations for xyx_euler_angles error, but invalid for rotation_vector.
   * The second matrix is the other way around.
   *
   * The rows contain a quaternion in the order x, y, z and w.
   * */
  Eigen::Matrix<double, 8, 4> valid_euler_data_;
  Eigen::Matrix<double, 8, 4> valid_rotvec_data_;

  void SetUp() override
  {
    // create robot
    moveit::core::RobotModelBuilder robot("floating_robot", "base_link");
    robot.addChain("base_link->ee", "floating");
    robot.addGroupChain("base_link", "ee", "group1");

    ASSERT_TRUE(robot.isValid());
    robot_model_ = robot.build();

    // hardcoded test data created with an external python script
    valid_euler_data_ << -0.1712092272140422, -0.2853625129991958, -0.1712092272140422, 0.9273311367620117,
        -0.28536251299919585, -0.17120922721404216, 0.28536251299919585, 0.8987902273981057, 0.28536251299919585,
        -0.17120922721404216, -0.28536251299919585, 0.8987902273981057, 0.1712092272140422, -0.2853625129991958,
        0.1712092272140422, 0.9273311367620117, -0.28536251299919585, 0.17120922721404216, -0.28536251299919585,
        0.8987902273981057, -0.1712092272140422, 0.2853625129991958, 0.1712092272140422, 0.9273311367620117,
        0.1712092272140422, 0.2853625129991958, -0.1712092272140422, 0.9273311367620117, 0.28536251299919585,
        0.17120922721404216, 0.28536251299919585, 0.8987902273981057;
    valid_rotvec_data_ << -0.23771285949073287, -0.23771285949073287, -0.23771285949073287, 0.911305541132181,
        -0.2401272988734743, -0.2401272988734743, 0.0, 0.9405731022474852, -0.23771285949073287, -0.23771285949073287,
        0.23771285949073287, 0.911305541132181, 0.0, -0.24012729887347428, -0.24012729887347428, 0.9405731022474851,
        0.0, -0.24012729887347428, 0.24012729887347428, 0.9405731022474851, 0.23771285949073287, -0.23771285949073287,
        -0.23771285949073287, 0.911305541132181, 0.2401272988734743, -0.2401272988734743, 0.0, 0.9405731022474852,
        0.23771285949073287, -0.23771285949073287, 0.23771285949073287, 0.911305541132181;
  }

  void TearDown() override
  {
  }
};

/** Helper function to create a quaternion from Euler angles. **/
inline Eigen::Quaterniond xyz_intrinsix_to_quat(double rx, double ry, double rz)
{
  return Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
}

/** Helper function to create a quaternion from a rotation vector. **/
inline Eigen::Quaterniond rotation_vector_to_quat(double rx, double ry, double rz)
{
  Eigen::Vector3d v{ rx, ry, rz };
  Eigen::Matrix3d m{ Eigen::AngleAxisd(v.norm(), v.normalized()) };
  return Eigen::Quaterniond{ m };
}

/** Helper function to set the orientation of the robot end-effector for the FloatingJointRobot. **/
void setRobotEndEffectorOrientation(moveit::core::RobotState& robot_state, const Eigen::Quaterniond& quat)
{
  Eigen::VectorXd joint_values(7);
  joint_values << 0.0, 0.0, 0.0, quat.x(), quat.y(), quat.z(), quat.w();
  robot_state.setJointGroupPositions("group1", joint_values);
  robot_state.update();
}

TEST_F(SphericalRobot, Test1)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.8;
  ocm.absolute_y_axis_tolerance = 0.8;
  ocm.absolute_z_axis_tolerance = 3.15;
  ocm.weight = 1.0;

  moveit::core::RobotState robot_state(robot_model_);
  // This's identical to a -1.57rad rotation around Z-axis
  robot_state.setVariablePositions(getJointValues(3.140208, 3.141588, 1.570821));
  robot_state.update();

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_TRUE(oc.decide(robot_state).satisfied);
}

TEST_F(SphericalRobot, Test2)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 0.2;
  ocm.weight = 1.0;

  moveit::core::RobotState robot_state(robot_model_);
  // Singularity: roll + yaw = theta
  // These violate either absolute_x_axis_tolerance or absolute_z_axis_tolerance
  robot_state.setVariablePositions(getJointValues(0.15, boost::math::constants::half_pi<double>(), 0.15));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.21, boost::math::constants::half_pi<double>(), 0.0));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.0, boost::math::constants::half_pi<double>(), 0.21));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  // Singularity: roll - yaw = theta
  // This's identical to -pi/2 pitch rotation
  robot_state.setVariablePositions(getJointValues(0.15, -boost::math::constants::half_pi<double>(), 0.15));
  robot_state.update();

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_TRUE(oc.decide(robot_state).satisfied);
}

TEST_F(SphericalRobot, Test3)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 0.3;
  ocm.weight = 1.0;

  moveit::core::RobotState robot_state(robot_model_);
  // Singularity: roll + yaw = theta

  // These tests violate absolute_x_axis_tolerance
  robot_state.setVariablePositions(getJointValues(0.21, boost::math::constants::half_pi<double>(), 0.0));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.0, boost::math::constants::half_pi<double>(), 0.21));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  ocm.absolute_x_axis_tolerance = 0.3;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 0.2;

  // These tests violate absolute_z_axis_tolerance
  robot_state.setVariablePositions(getJointValues(0.21, boost::math::constants::half_pi<double>(), 0.0));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.0, boost::math::constants::half_pi<double>(), 0.21));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);
}

TEST_F(SphericalRobot, Test4)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 0.3;
  ocm.weight = 1.0;

  moveit::core::RobotState robot_state(robot_model_);
  // Singularity: roll + yaw = theta

  // These tests violate absolute_x_axis_tolerance
  robot_state.setVariablePositions(getJointValues(0.21, -boost::math::constants::half_pi<double>(), 0.0));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.0, -boost::math::constants::half_pi<double>(), 0.21));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  ocm.absolute_x_axis_tolerance = 0.3;
  ocm.absolute_y_axis_tolerance = 2.0;
  ocm.absolute_z_axis_tolerance = 0.2;

  // These tests violate absolute_z_axis_tolerance
  robot_state.setVariablePositions(getJointValues(0.21, -boost::math::constants::half_pi<double>(), 0.0));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.0, -boost::math::constants::half_pi<double>(), 0.21));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);

  robot_state.setVariablePositions(getJointValues(0.5, -boost::math::constants::half_pi<double>(), 0.21));
  robot_state.update();
  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_FALSE(oc.decide(robot_state).satisfied);
}

// Both the current and the desired orientations are in singularities
TEST_F(SphericalRobot, Test5)
{
  kinematic_constraints::OrientationConstraint oc(robot_model_);

  moveit::core::Transforms tf(robot_model_->getModelFrame());
  moveit_msgs::OrientationConstraint ocm;

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setVariablePositions(getJointValues(0.0, boost::math::constants::half_pi<double>(), 0.0));
  robot_state.update();

  ocm.link_name = "yaw";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation = tf2::toMsg(Eigen::Quaterniond(robot_state.getGlobalLinkTransform(ocm.link_name).linear()));
  ocm.absolute_x_axis_tolerance = 0.0;
  ocm.absolute_y_axis_tolerance = 0.0;
  ocm.absolute_z_axis_tolerance = 1.0;
  ocm.weight = 1.0;

  robot_state.setVariablePositions(getJointValues(0.2, boost::math::constants::half_pi<double>(), 0.3));
  robot_state.update();

  EXPECT_TRUE(oc.configure(ocm, tf));
  EXPECT_TRUE(oc.decide(robot_state, true).satisfied);
}

TEST_F(FloatingJointRobot, TestDefaultParameterization)
{
  // Simple test that checks whether the configuration defaults to the expected parameterization
  // if we put an invalid type in the message.
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "ee";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 0.5;
  ocm.weight = 1.0;

  // set to non-existing parameterization on purpose
  ocm.parameterization = 99;

  // configuring the constraints should still work
  kinematic_constraints::OrientationConstraint oc(robot_model_);
  EXPECT_TRUE(oc.configure(ocm, tf));

  // check if the expected default parameterization was set
  EXPECT_EQ(oc.getParameterizationType(), moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES);
}

TEST_F(FloatingJointRobot, OrientationConstraintsParameterization)
{
  // load and initialize robot model
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.update();

  // center the orientation constraints around the current orientation of the link
  geometry_msgs::Pose p = tf2::toMsg(robot_state.getGlobalLinkTransform("ee"));

  // we also need the name of the base link
  moveit::core::Transforms tf(robot_model_->getModelFrame());

  // create message to configure orientation constraints
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "ee";
  ocm.header.frame_id = robot_model_->getModelFrame();
  ocm.orientation = p.orientation;
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 0.5;
  ocm.weight = 1.0;

  // create orientation constraints with the xyz_euler_angle parameterization
  kinematic_constraints::OrientationConstraint oc_euler(robot_model_);
  ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
  EXPECT_TRUE(oc_euler.configure(ocm, tf));

  // create orientation constraints with the rotation_vector parameterization
  kinematic_constraints::OrientationConstraint oc_rotvec(robot_model_);
  ocm.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
  EXPECT_TRUE(oc_rotvec.configure(ocm, tf));

  // Constraints should be satisfied for current state because we created them around the current orientation
  EXPECT_TRUE(oc_euler.decide(robot_state, true).satisfied);
  EXPECT_TRUE(oc_rotvec.decide(robot_state, true).satisfied);

  // some trivial test cases
  setRobotEndEffectorOrientation(robot_state, xyz_intrinsix_to_quat(0.1, 0.2, -0.3));
  EXPECT_TRUE(oc_euler.decide(robot_state).satisfied);

  setRobotEndEffectorOrientation(robot_state, xyz_intrinsix_to_quat(0.1, 0.2, -0.6));
  EXPECT_FALSE(oc_euler.decide(robot_state).satisfied);

  setRobotEndEffectorOrientation(robot_state, rotation_vector_to_quat(0.1, 0.2, -0.3));
  EXPECT_TRUE(oc_rotvec.decide(robot_state).satisfied);

  setRobotEndEffectorOrientation(robot_state, rotation_vector_to_quat(0.1, 0.2, -0.6));
  EXPECT_FALSE(oc_rotvec.decide(robot_state).satisfied);

  // more extensive testing using the test data hardcoded at the top of this file
  Eigen::VectorXd joint_values(7);
  joint_values << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  for (Eigen::Index i_row{ 0 }; i_row < valid_euler_data_.rows(); ++i_row)
  {
    joint_values[3] = valid_euler_data_(i_row, 0);
    joint_values[4] = valid_euler_data_(i_row, 1);
    joint_values[5] = valid_euler_data_(i_row, 2);
    joint_values[6] = valid_euler_data_(i_row, 3);
    robot_state.setJointGroupPositions("group1", joint_values);
    robot_state.update();
    EXPECT_TRUE(oc_euler.decide(robot_state).satisfied);
    EXPECT_FALSE(oc_rotvec.decide(robot_state).satisfied);

    joint_values[3] = valid_rotvec_data_(i_row, 0);
    joint_values[4] = valid_rotvec_data_(i_row, 1);
    joint_values[5] = valid_rotvec_data_(i_row, 2);
    joint_values[6] = valid_rotvec_data_(i_row, 3);
    robot_state.setJointGroupPositions("group1", joint_values);
    robot_state.update();
    EXPECT_FALSE(oc_euler.decide(robot_state).satisfied);
    EXPECT_TRUE(oc_rotvec.decide(robot_state).satisfied);
  }

  // and now some simple test cases where whe change the nominal orientation of the constraints,
  // instead of changing the orientation of the robot
  robot_state.setToDefaultValues();
  robot_state.update();

  ocm.parameterization = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
  ocm.orientation = tf2::toMsg(xyz_intrinsix_to_quat(0.1, 0.2, -0.3));
  EXPECT_TRUE(oc_euler.configure(ocm, tf));
  EXPECT_TRUE(oc_euler.decide(robot_state).satisfied);

  ocm.orientation = tf2::toMsg(xyz_intrinsix_to_quat(0.1, 0.2, -0.6));
  EXPECT_TRUE(oc_euler.configure(ocm, tf));
  EXPECT_FALSE(oc_euler.decide(robot_state).satisfied);

  ocm.parameterization = moveit_msgs::OrientationConstraint::ROTATION_VECTOR;
  ocm.orientation = tf2::toMsg(rotation_vector_to_quat(0.1, 0.2, -0.3));
  EXPECT_TRUE(oc_rotvec.configure(ocm, tf));
  EXPECT_TRUE(oc_rotvec.decide(robot_state).satisfied);

  ocm.orientation = tf2::toMsg(rotation_vector_to_quat(0.1, 0.2, -0.6));
  EXPECT_TRUE(oc_rotvec.configure(ocm, tf));
  EXPECT_FALSE(oc_rotvec.decide(robot_state).satisfied);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
