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

#include <gtest/gtest.h>

// Boost includes
#include <boost/scoped_ptr.hpp>

#include <pluginlib/class_loader.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_industrial_motion_planner/planning_context_loader.h"

#include "test_utils.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

class PlanningContextLoadersTest : public ::testing::TestWithParam<std::vector<std::string>>
{
protected:
  /**
   * @brief To initialize the planning context loader
   * The planning context loader is loaded using the pluginlib.
   * Checks if planning context loader was loaded properly are performed.
   * Exceptions will cause test failure.
   */
  void SetUp() override
  {
    ASSERT_FALSE(robot_model_ == nullptr) << "There is no robot model!";

    // Load the plugin
    try
    {
      planning_context_loader_class_loader_ =
          std::make_unique<pluginlib::ClassLoader<pilz_industrial_motion_planner::PlanningContextLoader>>(
              "pilz_industrial_motion_planner", "pilz_industrial_motion_planner::PlanningContextLoader");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning context loader " << ex.what());
      FAIL();
    }

    // Create planning context loader ptp
    try
    {
      planning_context_loader_.reset(planning_context_loader_class_loader_->createUnmanagedInstance(GetParam().front()));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      FAIL() << ex.what();
    }
    return;
  }

  void TearDown() override
  {
    if (planning_context_loader_class_loader_)
    {
      planning_context_loader_class_loader_->unloadLibraryForClass(GetParam().front());
    }
  }

protected:
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(GetParam().back()).getModel() };

  // Load the plugin
  std::unique_ptr<pluginlib::ClassLoader<pilz_industrial_motion_planner::PlanningContextLoader>>
      planning_context_loader_class_loader_;

  pilz_industrial_motion_planner::PlanningContextLoaderPtr planning_context_loader_;
};

// Instantiate the test cases for all loaders, extend here if you added a new
// ContextLoader you want to test
INSTANTIATE_TEST_SUITE_P(
    InstantiationName, PlanningContextLoadersTest,
    ::testing::Values(std::vector<std::string>{ "pilz_industrial_motion_planner::PlanningContextLoaderPTP", "PTP",
                                                PARAM_MODEL_NO_GRIPPER_NAME },  // Test for PTP
                      std::vector<std::string>{ "pilz_industrial_motion_planner::PlanningContextLoaderPTP", "PTP",
                                                PARAM_MODEL_WITH_GRIPPER_NAME },  // Test for PTP
                      std::vector<std::string>{ "pilz_industrial_motion_planner::PlanningContextLoaderLIN", "LIN",
                                                PARAM_MODEL_NO_GRIPPER_NAME },  // Test for LIN
                      std::vector<std::string>{ "pilz_industrial_motion_planner::PlanningContextLoaderLIN", "LIN",
                                                PARAM_MODEL_WITH_GRIPPER_NAME },  // Test for LIN
                      std::vector<std::string>{ "pilz_industrial_motion_planner::PlanningContextLoaderCIRC", "CIRC",
                                                PARAM_MODEL_NO_GRIPPER_NAME },  // Test for CIRC
                      std::vector<std::string>{ "pilz_industrial_motion_planner::PlanningContextLoaderCIRC", "CIRC",
                                                PARAM_MODEL_WITH_GRIPPER_NAME }  // Test for CIRC
                      ));

/**
 * @brief Test getAlgorithm returns PTP
 */
TEST_P(PlanningContextLoadersTest, GetAlgorithm)
{
  std::string alg = planning_context_loader_->getAlgorithm();
  EXPECT_EQ(alg, GetParam().at(1));
}

/**
 * @brief Check that load Context returns initialized shared pointer
 */
TEST_P(PlanningContextLoadersTest, LoadContext)
{
  planning_interface::PlanningContextPtr planning_context;
  const std::string& group_name = "manipulator";

  // Without limits should return false
  bool res = planning_context_loader_->loadContext(planning_context, "test", group_name);
  EXPECT_EQ(false, res) << "Context returned even when no limits where set";

  // After setting the limits this should work
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
      testutils::createFakeLimits(robot_model_->getVariableNames());
  pilz_industrial_motion_planner::LimitsContainer limits;
  limits.setJointLimits(joint_limits);
  pilz_industrial_motion_planner::CartesianLimit cart_limits;
  cart_limits.setMaxRotationalVelocity(1 * M_PI);
  cart_limits.setMaxTranslationalAcceleration(2);
  cart_limits.setMaxTranslationalDeceleration(2);
  cart_limits.setMaxTranslationalVelocity(1);
  limits.setCartesianLimits(cart_limits);

  planning_context_loader_->setLimits(limits);
  planning_context_loader_->setModel(robot_model_);

  try
  {
    res = planning_context_loader_->loadContext(planning_context, "test", group_name);
  }
  catch (std::exception& ex)
  {
    FAIL() << "Exception!" << ex.what() << " " << typeid(ex).name();
  }

  EXPECT_EQ(true, res) << "Context could not be loaded!";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_planning_context_loaders");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
