/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <urdf_parser/urdf_parser.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <limits>
#include <ctype.h>

namespace
{
constexpr double EPSILON{ 1.e-9 };
}

#if 0  // unused function
static bool sameStringIgnoringWS(const std::string& s1, const std::string& s2)
{
  unsigned int i1 = 0;
  unsigned int i2 = 0;
  while (i1 < s1.size() && isspace(s1[i1]))
    i1++;
  while (i2 < s2.size() && isspace(s2[i2]))
    i2++;
  while (i1 < s1.size() && i2 < s2.size())
  {
    if (i1 < s1.size() && i2 < s2.size())
    {
      if (s1[i1] != s2[i2])
        return false;
      i1++;
      i2++;
    }
    while (i1 < s1.size() && isspace(s1[i1]))
      i1++;
    while (i2 < s2.size() && isspace(s2[i2]))
      i2++;
  }
  return i1 == s1.size() && i2 == s2.size();
}
#endif

static void expect_near(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y,
                        double eps = std::numeric_limits<double>::epsilon())
{
  ASSERT_EQ(x.rows(), y.rows());
  ASSERT_EQ(x.cols(), y.cols());
  for (int r = 0; r < x.rows(); ++r)
    for (int c = 0; c < x.cols(); ++c)
      EXPECT_NEAR(x(r, c), y(r, c), eps) << "(r,c) = (" << r << "," << c << ")";
}

// clang-format off
#define EXPECT_NEAR_TRACED(...) {                 \
	SCOPED_TRACE("expect_near(" #__VA_ARGS__ ")"); \
	expect_near(__VA_ARGS__);                      \
}
// clang-format on

TEST(Loading, SimpleRobot)
{
  moveit::core::RobotModelBuilder builder("myrobot", "base_link");
  builder.addVirtualJoint("odom_combined", "base_link", "floating", "base_joint");
  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotModelPtr model = builder.build();
  moveit::core::RobotState state(model);

  state.setToDefaultValues();

  // make sure that this copy constructor works
  moveit::core::RobotState new_state(state);

  EXPECT_EQ(new_state.getVariablePosition("base_joint/rot_w"), 1.0);

  EXPECT_EQ(std::string("myrobot"), model->getName());
  EXPECT_EQ((unsigned int)7, new_state.getVariableCount());

  const std::vector<moveit::core::LinkModel*>& links = model->getLinkModels();
  EXPECT_EQ((unsigned int)1, links.size());

  const std::vector<moveit::core::JointModel*>& joints = model->getJointModels();
  EXPECT_EQ((unsigned int)1, joints.size());

  const std::vector<std::string>& pgroups = model->getJointModelGroupNames();
  EXPECT_EQ((unsigned int)0, pgroups.size());
}

TEST(LoadingAndFK, SimpleRobot)
{
  moveit::core::RobotModelBuilder builder("myrobot", "base_link");
  geometry_msgs::Pose pose;
  tf2::toMsg(tf2::Vector3(-0.1, 0, 0), pose.position);
  tf2::Quaternion q;
  q.setRPY(0, 0, -1);
  pose.orientation = tf2::toMsg(q);
  builder.addCollisionBox("base_link", { 1, 2, 1 }, pose);
  tf2::toMsg(tf2::Vector3(0, 0, 0), pose.position);
  q.setRPY(0, 0, 0);
  pose.orientation = tf2::toMsg(q);
  builder.addVisualBox("base_link", { 1, 2, 1 }, pose);
  tf2::toMsg(tf2::Vector3(0, 0.099, 0), pose.position);
  q.setRPY(0, 0, 0);
  pose.orientation = tf2::toMsg(q);
  builder.addInertial("base_link", 2.81, pose, 0.1, -0.2, 0.5, -0.09, 1, 0.101);
  builder.addVirtualJoint("odom_combined", "base_link", "planar", "base_joint");
  builder.addGroup({}, { "base_joint" }, "base");

  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotModelPtr model = builder.build();
  moveit::core::RobotState state(model);

  EXPECT_EQ((unsigned int)3, state.getVariableCount());

  state.setToDefaultValues();

  EXPECT_EQ((unsigned int)1, (unsigned int)model->getJointModelCount());
  EXPECT_EQ((unsigned int)3, (unsigned int)model->getJointModels()[0]->getLocalVariableNames().size());

  std::map<std::string, double> joint_values;
  joint_values["base_joint/x"] = 10.0;
  joint_values["base_joint/y"] = 8.0;

  // testing incomplete state
  std::vector<std::string> missing_states;
  state.setVariablePositions(joint_values, missing_states);
  ASSERT_EQ(missing_states.size(), 1u);
  EXPECT_EQ(missing_states[0], std::string("base_joint/theta"));
  joint_values["base_joint/theta"] = 0.1;

  state.setVariablePositions(joint_values, missing_states);
  ASSERT_EQ(missing_states.size(), 0u);

  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("base_link").translation(), Eigen::Vector3d(10, 8, 0));

  state.setVariableAcceleration("base_joint/x", 0.0);

  const auto new_state = std::make_unique<moveit::core::RobotState>(state);  // making sure that values get copied
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("base_link").translation(), Eigen::Vector3d(10, 8, 0));

  std::vector<double> jv(state.getVariableCount(), 0.0);
  jv[state.getRobotModel()->getVariableIndex("base_joint/x")] = 5.0;
  jv[state.getRobotModel()->getVariableIndex("base_joint/y")] = 4.0;
  jv[state.getRobotModel()->getVariableIndex("base_joint/theta")] = 0.0;

  state.setVariablePositions(jv);
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("base_link").translation(), Eigen::Vector3d(5, 4, 0));
}

class OneRobot : public testing::Test
{
protected:
  void SetUp() override
  {
    static const std::string MODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"one_robot\">"
        "<link name=\"base_link\">"
        "  <inertial>"
        "    <mass value=\"2.81\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision name=\"my_collision\">"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<joint name=\"joint_a\" type=\"continuous\">"
        "   <axis xyz=\"0 0 1\"/>"
        "   <parent link=\"base_link\"/>"
        "   <child link=\"link_a\"/>"
        "   <origin rpy=\" 0.0 0 0 \" xyz=\"0.0 0 0 \"/>"
        "</joint>"
        "<link name=\"link_a\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<joint name=\"joint_b\" type=\"fixed\">"
        "  <parent link=\"link_a\"/>"
        "  <child link=\"link_b\"/>"
        "  <origin rpy=\" 0.0 -0.42 0 \" xyz=\"0.0 0.5 0 \"/>"
        "</joint>"
        "<link name=\"link_b\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0.0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "  <joint name=\"joint_c\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.09\" velocity=\"0.2\"/>"
        "    <safety_controller k_position=\"20.0\" k_velocity=\"500.0\" soft_lower_limit=\"0.0\" "
        "soft_upper_limit=\"0.089\"/>"
        "    <parent link=\"link_b\"/>"
        "    <child link=\"link_c\"/>"
        "    <origin rpy=\" 0.0 0.42 0.0 \" xyz=\"0.0 -0.1 0 \"/>"
        "  </joint>"
        "<link name=\"link_c\">"
        "  <inertial>"
        "    <mass value=\"1.0\"/>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 .0\"/>"
        "    <inertia ixx=\"0.1\" ixy=\"-0.2\" ixz=\"0.5\" iyy=\"-.09\" iyz=\"1\" izz=\"0.101\"/>"
        "  </inertial>"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 0 0\" xyz=\"0.0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "  <joint name=\"mim_f\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
        "    <parent link=\"link_c\"/>"
        "    <child link=\"link_d\"/>"
        "    <origin rpy=\" 0.0 0.0 0.0 \" xyz=\"0.1 0.1 0 \"/>"
        "    <mimic joint=\"joint_f\" multiplier=\"1.5\" offset=\"0.1\"/>"
        "  </joint>"
        "  <joint name=\"joint_f\" type=\"prismatic\">"
        "    <axis xyz=\"1 0 0\"/>"
        "    <limit effort=\"100.0\" lower=\"0.0\" upper=\"0.19\" velocity=\"0.2\"/>"
        "    <parent link=\"link_d\"/>"
        "    <child link=\"link_e\"/>"
        "    <origin rpy=\" 0.0 0.0 0.0 \" xyz=\"0.1 0.1 0 \"/>"
        "  </joint>"
        "<link name=\"link_d\">"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "<link name=\"link_e\">"
        "  <collision>"
        "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </collision>"
        "  <visual>"
        "    <origin rpy=\"0 1 0\" xyz=\"0 0.1 0\"/>"
        "    <geometry>"
        "      <box size=\"1 2 1\" />"
        "    </geometry>"
        "  </visual>"
        "</link>"
        "</robot>";

    static const std::string SMODEL2 =
        "<?xml version=\"1.0\" ?>"
        "<robot name=\"one_robot\">"
        "<virtual_joint name=\"base_joint\" child_link=\"base_link\" parent_frame=\"odom_combined\" type=\"planar\"/>"
        "<group name=\"base_from_joints\">"
        "<joint name=\"base_joint\"/>"
        "<joint name=\"joint_a\"/>"
        "<joint name=\"joint_c\"/>"
        "</group>"
        "<group name=\"mim_joints\">"
        "<joint name=\"joint_f\"/>"
        "<joint name=\"mim_f\"/>"
        "</group>"
        "<group name=\"base_with_subgroups\">"
        "<group name=\"base_from_base_to_tip\"/>"
        "<joint name=\"joint_c\"/>"
        "</group>"
        "<group name=\"base_from_base_to_tip\">"
        "<chain base_link=\"base_link\" tip_link=\"link_b\"/>"
        "<joint name=\"base_joint\"/>"
        "</group>"
        "<group name=\"base_from_base_to_e\">"
        "<chain base_link=\"base_link\" tip_link=\"link_e\"/>"
        "<joint name=\"base_joint\"/>"
        "</group>"
        "<group name=\"base_with_bad_subgroups\">"
        "<group name=\"error\"/>"
        "</group>"
        "</robot>";

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(MODEL2);
    srdf::ModelSharedPtr srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, SMODEL2);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  }

  void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelConstPtr robot_model_;
};

TEST_F(OneRobot, FK)
{
  moveit::core::RobotModelConstPtr model = robot_model_;

  // testing that the two planning groups are the same
  const moveit::core::JointModelGroup* g_one = model->getJointModelGroup("base_from_joints");
  const moveit::core::JointModelGroup* g_two = model->getJointModelGroup("base_from_base_to_tip");
  const moveit::core::JointModelGroup* g_three = model->getJointModelGroup("base_with_subgroups");
  const moveit::core::JointModelGroup* g_four = model->getJointModelGroup("base_with_bad_subgroups");
  const moveit::core::JointModelGroup* g_mim = model->getJointModelGroup("mim_joints");

  ASSERT_TRUE(g_one != nullptr);
  ASSERT_TRUE(g_two != nullptr);
  ASSERT_TRUE(g_three != nullptr);
  ASSERT_TRUE(g_four == nullptr);

  // joint_b is a fixed joint, so no one should have it
  ASSERT_EQ(g_one->getJointModelNames().size(), 3u);
  ASSERT_EQ(g_two->getJointModelNames().size(), 3u);
  ASSERT_EQ(g_three->getJointModelNames().size(), 4u);
  ASSERT_EQ(g_mim->getJointModelNames().size(), 2u);

  // only the links in between the joints, and the children of the leafs
  ASSERT_EQ(g_one->getLinkModelNames().size(), 3u);
  // g_two only has three links
  ASSERT_EQ(g_two->getLinkModelNames().size(), 3u);
  ASSERT_EQ(g_three->getLinkModelNames().size(), 4u);

  std::vector<std::string> jmn = g_one->getJointModelNames();
  std::sort(jmn.begin(), jmn.end());
  EXPECT_EQ(jmn[0], "base_joint");
  EXPECT_EQ(jmn[1], "joint_a");
  EXPECT_EQ(jmn[2], "joint_c");
  jmn = g_two->getJointModelNames();
  std::sort(jmn.begin(), jmn.end());
  EXPECT_EQ(jmn[0], "base_joint");
  EXPECT_EQ(jmn[1], "joint_a");
  EXPECT_EQ(jmn[2], "joint_b");
  jmn = g_three->getJointModelNames();
  std::sort(jmn.begin(), jmn.end());
  EXPECT_EQ(jmn[0], "base_joint");
  EXPECT_EQ(jmn[1], "joint_a");
  EXPECT_EQ(jmn[2], "joint_b");
  EXPECT_EQ(jmn[3], "joint_c");

  // but they should have the same links to be updated
  ASSERT_EQ(g_one->getUpdatedLinkModels().size(), 6u);
  ASSERT_EQ(g_two->getUpdatedLinkModels().size(), 6u);
  ASSERT_EQ(g_three->getUpdatedLinkModels().size(), 6u);

  EXPECT_EQ(g_one->getUpdatedLinkModels()[0]->getName(), "base_link");
  EXPECT_EQ(g_one->getUpdatedLinkModels()[1]->getName(), "link_a");
  EXPECT_EQ(g_one->getUpdatedLinkModels()[2]->getName(), "link_b");
  EXPECT_EQ(g_one->getUpdatedLinkModels()[3]->getName(), "link_c");

  EXPECT_EQ(g_two->getUpdatedLinkModels()[0]->getName(), "base_link");
  EXPECT_EQ(g_two->getUpdatedLinkModels()[1]->getName(), "link_a");
  EXPECT_EQ(g_two->getUpdatedLinkModels()[2]->getName(), "link_b");
  EXPECT_EQ(g_two->getUpdatedLinkModels()[3]->getName(), "link_c");

  EXPECT_EQ(g_three->getUpdatedLinkModels()[0]->getName(), "base_link");
  EXPECT_EQ(g_three->getUpdatedLinkModels()[1]->getName(), "link_a");
  EXPECT_EQ(g_three->getUpdatedLinkModels()[2]->getName(), "link_b");
  EXPECT_EQ(g_three->getUpdatedLinkModels()[3]->getName(), "link_c");

  // bracketing so the state gets destroyed before we bring down the model

  moveit::core::RobotState state(model);

  EXPECT_EQ((unsigned int)7, state.getVariableCount());

  state.setToDefaultValues();

  std::map<std::string, double> joint_values;
  joint_values["base_joint/x"] = 1.0;
  joint_values["base_joint/y"] = 1.0;
  joint_values["base_joint/theta"] = 0.5;
  joint_values["joint_a"] = -0.5;
  joint_values["joint_c"] = 0.08;
  state.setVariablePositions(joint_values);

  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("base_link").translation(), Eigen::Vector3d(1, 1, 0));
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("base_link").linear()).x(), 1e-5);
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("base_link").linear()).y(), 1e-5);
  EXPECT_NEAR(0.247404, Eigen::Quaterniond(state.getGlobalLinkTransform("base_link").linear()).z(), 1e-5);
  EXPECT_NEAR(0.968912, Eigen::Quaterniond(state.getGlobalLinkTransform("base_link").linear()).w(), 1e-5);

  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_a").translation(), Eigen::Vector3d(1, 1, 0));
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_a").linear()).x(), 1e-5);
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_a").linear()).y(), 1e-5);
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_a").linear()).z(), 1e-5);
  EXPECT_NEAR(1.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_a").linear()).w(), 1e-5);

  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_b").translation(), Eigen::Vector3d(1, 1.5, 0));
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_b").linear()).x(), 1e-5);
  EXPECT_NEAR(-0.2084598, Eigen::Quaterniond(state.getGlobalLinkTransform("link_b").linear()).y(), 1e-5);
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_b").linear()).z(), 1e-5);
  EXPECT_NEAR(0.97803091, Eigen::Quaterniond(state.getGlobalLinkTransform("link_b").linear()).w(), 1e-5);

  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_c").translation(), Eigen::Vector3d(1.08, 1.4, 0));
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_c").linear()).x(), 1e-5);
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_c").linear()).y(), 1e-5);
  EXPECT_NEAR(0.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_c").linear()).z(), 1e-5);
  EXPECT_NEAR(1.0, Eigen::Quaterniond(state.getGlobalLinkTransform("link_c").linear()).w(), 1e-5);

  EXPECT_TRUE(state.satisfiesBounds());

  std::map<std::string, double> upd_a;
  upd_a["joint_a"] = 0.2;
  state.setVariablePositions(upd_a);
  EXPECT_TRUE(state.satisfiesBounds(model->getJointModel("joint_a")));
  EXPECT_NEAR(state.getVariablePosition("joint_a"), 0.2, 1e-3);
  state.enforceBounds();
  EXPECT_NEAR(state.getVariablePosition("joint_a"), 0.2, 1e-3);

  upd_a["joint_a"] = 3.2;
  state.setVariablePositions(upd_a);
  EXPECT_TRUE(state.satisfiesBounds(model->getJointModel("joint_a")));
  EXPECT_NEAR(state.getVariablePosition("joint_a"), 3.2, 1e-3);
  state.enforceBounds();
  EXPECT_NEAR(state.getVariablePosition("joint_a"), -3.083185, 1e-3);
  EXPECT_TRUE(state.satisfiesBounds(model->getJointModel("joint_a")));

  // mimic joints
  state.setToDefaultValues();
  EXPECT_TRUE(state.dirtyLinkTransforms());
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_c").translation(), Eigen::Vector3d(0.0, 0.4, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_d").translation(), Eigen::Vector3d(0.2, 0.5, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_e").translation(), Eigen::Vector3d(0.3, 0.6, 0));

  // setVariablePosition should update corresponding mimic joints too
  state.setVariablePosition("joint_f", 1.0);
  EXPECT_EQ(state.getVariablePosition("joint_f"), 1.0);
  EXPECT_EQ(state.getVariablePosition("mim_f"), 1.6);
  EXPECT_TRUE(state.dirtyLinkTransforms());
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_c").translation(), Eigen::Vector3d(0.0, 0.4, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_d").translation(), Eigen::Vector3d(1.7, 0.5, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_e").translation(), Eigen::Vector3d(2.8, 0.6, 0));

  ASSERT_EQ(g_mim->getVariableCount(), 2u);
  double gstate[2];
  state.copyJointGroupPositions(g_mim, gstate);

  // setToRandomPositions() uses a different mechanism to update mimic joints
  state.setToRandomPositions(g_mim);
  double joint_f = state.getVariablePosition("joint_f");
  double mim_f = state.getVariablePosition("mim_f");
  EXPECT_NEAR(mim_f, 1.5 * joint_f + 0.1, 1e-8);
  EXPECT_TRUE(state.dirtyLinkTransforms());
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_c").translation(), Eigen::Vector3d(0.0, 0.4, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_d").translation(), Eigen::Vector3d(0.1 + mim_f, 0.5, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_e").translation(),
                     Eigen::Vector3d(0.1 + mim_f + joint_f + 0.1, 0.6, 0));

  // setJointGroupPositions() uses a different mechanism to update mimic joints
  state.setJointGroupPositions(g_mim, gstate);
  EXPECT_EQ(state.getVariablePosition("joint_f"), 1.0);
  EXPECT_EQ(state.getVariablePosition("mim_f"), 1.6);
  EXPECT_TRUE(state.dirtyLinkTransforms());
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_c").translation(), Eigen::Vector3d(0.0, 0.4, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_d").translation(), Eigen::Vector3d(1.7, 0.5, 0));
  EXPECT_NEAR_TRACED(state.getGlobalLinkTransform("link_e").translation(), Eigen::Vector3d(2.8, 0.6, 0));
}

TEST_F(OneRobot, testPrintCurrentPositionWithJointLimits)
{
  moveit::core::RobotState state(robot_model_);
  const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("base_from_base_to_e");
  ASSERT_TRUE(joint_model_group);

  state.setToDefaultValues();

  std::cout << "\nVisual inspection should show NO joints out of bounds:" << std::endl;
  state.printStatePositionsWithJointLimits(joint_model_group);

  std::cout << "\nVisual inspection should show ONE joint out of bounds:" << std::endl;
  std::vector<double> single_joint(1);
  single_joint[0] = -1.0;
  state.setJointPositions("joint_c", single_joint);
  state.printStatePositionsWithJointLimits(joint_model_group);

  std::cout << "\nVisual inspection should show TWO joint out of bounds:" << std::endl;
  single_joint[0] = 1.0;
  state.setJointPositions("joint_f", single_joint);
  state.printStatePositionsWithJointLimits(joint_model_group);

  std::cout << "\nVisual inspection should show ONE joint out of bounds:" << std::endl;
  single_joint[0] = 0.19;
  state.setJointPositions("joint_f", single_joint);
  state.printStatePositionsWithJointLimits(joint_model_group);
}

TEST_F(OneRobot, testInterpolation)
{
  moveit::core::RobotState state_a(robot_model_);

  // Interpolate with itself
  state_a.setToDefaultValues();
  moveit::core::RobotState state_b(state_a);
  moveit::core::RobotState interpolated_state(state_a);
  for (size_t i = 0; i <= 10; ++i)
  {
    state_a.interpolate(state_b, static_cast<double>(i) / 10., interpolated_state,
                        robot_model_->getJointModelGroup("base_from_base_to_e"));
    EXPECT_NEAR(state_a.distance(state_b), 0, EPSILON)
        << "Interpolation between identical states yielded a different state.";

    for (const auto& link_name : robot_model_->getLinkModelNames())
    {
      EXPECT_FALSE(interpolated_state.getCollisionBodyTransform(link_name, 0).matrix().hasNaN())
          << "Interpolation between identical states yielded NaN value.";
    }
  }

  // Some simple interpolation
  std::map<std::string, double> joint_values;
  joint_values["base_joint/x"] = 1.0;
  joint_values["base_joint/y"] = 1.0;
  state_a.setVariablePositions(joint_values);
  joint_values["base_joint/x"] = 0.0;
  joint_values["base_joint/y"] = 2.0;
  state_b.setVariablePositions(joint_values);
  EXPECT_NEAR(3 * std::sqrt(2), state_a.distance(state_b), EPSILON) << "Simple interpolation of base_joint failed.";

  state_a.interpolate(state_b, 0.5, interpolated_state, robot_model_->getJointModelGroup("base_from_base_to_e"));
  EXPECT_NEAR(0., state_a.distance(interpolated_state) - state_b.distance(interpolated_state), EPSILON)
      << "Simple interpolation of base_joint failed.";
  EXPECT_NEAR(0.5, interpolated_state.getVariablePosition("base_joint/x"), EPSILON)
      << "Simple interpolation of base_joint failed.";
  EXPECT_NEAR(1.5, interpolated_state.getVariablePosition("base_joint/y"), EPSILON)
      << "Simple interpolation of base_joint failed.";
  state_a.interpolate(state_b, 0.1, interpolated_state, robot_model_->getJointModelGroup("base_from_base_to_e"));
  EXPECT_NEAR(0.9, interpolated_state.getVariablePosition("base_joint/x"), EPSILON)
      << "Simple interpolation of base_joint failed.";
  EXPECT_NEAR(1.1, interpolated_state.getVariablePosition("base_joint/y"), EPSILON)
      << "Simple interpolation of base_joint failed.";

  // Interpolate all the joints
  joint_values["base_joint/x"] = 0.0;
  joint_values["base_joint/y"] = 20.0;
  joint_values["base_joint/theta"] = 3 * M_PI / 4;
  joint_values["joint_a"] = -4 * M_PI / 5;
  joint_values["joint_c"] = 0.0;
  joint_values["joint_f"] = 1.0;
  state_a.setVariablePositions(joint_values);

  joint_values["base_joint/x"] = 10.0;
  joint_values["base_joint/y"] = 0.0;
  joint_values["base_joint/theta"] = -3 * M_PI / 4;
  joint_values["joint_a"] = 4 * M_PI / 5;
  joint_values["joint_c"] = 0.07;
  joint_values["joint_f"] = 0.0;
  state_b.setVariablePositions(joint_values);

  for (size_t i = 0; i <= 5; ++i)
  {
    double t = static_cast<double>(i) / 5.;
    state_a.interpolate(state_b, t, interpolated_state, robot_model_->getJointModelGroup("base_from_base_to_e"));
    EXPECT_NEAR(10.0 * t, interpolated_state.getVariablePosition("base_joint/x"), EPSILON)
        << "Base joint interpolation failed.";
    EXPECT_NEAR(20.0 * (1 - t), interpolated_state.getVariablePosition("base_joint/y"), EPSILON)
        << "Base joint interpolation failed.";
    if (t < 0.5)
    {
      EXPECT_NEAR(3 * M_PI / 4 + (M_PI / 2) * t, interpolated_state.getVariablePosition("base_joint/theta"), EPSILON)
          << "Base joint theta interpolation failed.";
      EXPECT_NEAR(-4 * M_PI / 5 - (2 * M_PI / 5) * t, interpolated_state.getVariablePosition("joint_a"), EPSILON)
          << "Continuous joint interpolation failed.";
    }
    else
    {
      EXPECT_NEAR(-3 * M_PI / 4 - (M_PI / 2) * (1 - t), interpolated_state.getVariablePosition("base_joint/theta"),
                  EPSILON)
          << "Base joint theta interpolation failed.";
      EXPECT_NEAR(4 * M_PI / 5 + (2 * M_PI / 5) * (1 - t), interpolated_state.getVariablePosition("joint_a"), EPSILON)
          << "Continuous joint interpolation failed.";
    }
    EXPECT_NEAR(0.07 * t, interpolated_state.getVariablePosition("joint_c"), EPSILON)
        << "Interpolation of joint_c failed.";
    EXPECT_NEAR(1 - t, interpolated_state.getVariablePosition("joint_f"), EPSILON)
        << "Interpolation of joint_f failed.";
    EXPECT_NEAR(1.5 * (1 - t) + 0.1, interpolated_state.getVariablePosition("mim_f"), EPSILON)
        << "Interpolation of mimic joint mim_f failed.";
  }

  bool nan_exception = false;
  try
  {
    const double infty = std::numeric_limits<double>::infinity();
    state_a.interpolate(state_b, infty, interpolated_state, robot_model_->getJointModelGroup("base_from_base_to_e"));
  }
  catch (std::exception& e)
  {
    std::cout << "Caught expected exception: " << e.what() << std::endl;
    nan_exception = true;
  }
  EXPECT_TRUE(nan_exception) << "NaN interpolation parameter did not create expected exception.";
}

TEST_F(OneRobot, rigidlyConnectedParent)
{
  // link_e is its own rigidly-connected parent
  const moveit::core::LinkModel* link_e{ robot_model_->getLinkModel("link_e") };
  EXPECT_EQ(robot_model_->getRigidlyConnectedParentLinkModel(link_e), link_e);

  // link_b is rigidly connected to its parent link_a
  const moveit::core::LinkModel* link_a{ robot_model_->getLinkModel("link_a") };
  const moveit::core::LinkModel* link_b{ robot_model_->getLinkModel("link_b") };
  EXPECT_EQ(robot_model_->getRigidlyConnectedParentLinkModel(link_b), link_a);

  moveit::core::RobotState state(robot_model_);

  EXPECT_EQ(state.getRigidlyConnectedParentLinkModel("link_b"), link_a);

  // attach "object" with "subframe" to link_b
  state.attachBody(std::make_unique<moveit::core::AttachedBody>(
      link_b, "object", Eigen::Isometry3d::Identity(), std::vector<shapes::ShapeConstPtr>{},
      EigenSTL::vector_Isometry3d{}, std::set<std::string>{}, trajectory_msgs::JointTrajectory{},
      moveit::core::FixedTransformsMap{ { "subframe", Eigen::Isometry3d::Identity() } }));

  // RobotState's version should resolve these too
  EXPECT_EQ(link_a, state.getRigidlyConnectedParentLinkModel("object"));
  EXPECT_EQ(link_a, state.getRigidlyConnectedParentLinkModel("object/subframe"));

  // test failure cases
  EXPECT_EQ(nullptr, state.getRigidlyConnectedParentLinkModel("no_object"));
  EXPECT_EQ(nullptr, state.getRigidlyConnectedParentLinkModel("object/no_subframe"));
  EXPECT_EQ(nullptr, state.getRigidlyConnectedParentLinkModel(""));
  EXPECT_EQ(nullptr, state.getRigidlyConnectedParentLinkModel("object/"));
  EXPECT_EQ(nullptr, state.getRigidlyConnectedParentLinkModel("/"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
