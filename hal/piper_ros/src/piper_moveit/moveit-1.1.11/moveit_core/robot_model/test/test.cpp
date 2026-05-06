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
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <moveit/profiler/profiler.h>
#include <moveit/utils/robot_model_test_utils.h>

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
  moveit::core::RobotModelConstPtr robot_model_;
};

TEST_F(LoadPlanningModelsPr2, InitOK)
{
  ASSERT_EQ(robot_model_->getURDF()->getName(), "pr2");
  ASSERT_EQ(robot_model_->getSRDF()->getName(), "pr2");
}

TEST_F(LoadPlanningModelsPr2, Model)
{
  // robot_model_->printModelInfo(std::cout);

  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getJointModels();
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    ASSERT_EQ(joints[i]->getJointIndex(), static_cast<int>(i));
    ASSERT_EQ(robot_model_->getJointModel(joints[i]->getName()), joints[i]);
  }
  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModels();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    ASSERT_EQ(links[i]->getLinkIndex(), static_cast<int>(i));
  }
  moveit::tools::Profiler::Status();
}

TEST(SiblingAssociateLinks, SimpleYRobot)
{
  /* base_link - a - b - c
                  \
                   - d ~ e          */
  moveit::core::RobotModelBuilder builder("one_robot", "base_link");
  builder.addChain("base_link->a", "continuous");
  builder.addChain("a->b->c", "fixed");
  builder.addChain("a->d", "fixed");
  builder.addChain("d->e", "continuous");
  builder.addVirtualJoint("odom", "base_link", "planar", "base_joint");
  builder.addGroup({}, { "base_joint" }, "base_joint");
  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotModelConstPtr robot_model = builder.build();

  const std::string a = "a", b = "b", c = "c", d = "d";
  auto connected = { a, b, c, d };  // these are rigidly connected with each other
  moveit::core::LinkTransformMap map;

  for (const std::string& root : connected)
  {
    SCOPED_TRACE("root: " + root);
    std::set<std::string> expected_set(connected);
    expected_set.erase(root);
    std::set<std::string> actual_set;
    for (const auto& item : robot_model->getLinkModel(root)->getAssociatedFixedTransforms())
      actual_set.insert(item.first->getName());

    std::ostringstream expected, actual;
    std::copy(expected_set.begin(), expected_set.end(), std::ostream_iterator<std::string>(expected, " "));
    std::copy(actual_set.begin(), actual_set.end(), std::ostream_iterator<std::string>(actual, " "));

    EXPECT_EQ(expected.str(), actual.str());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
