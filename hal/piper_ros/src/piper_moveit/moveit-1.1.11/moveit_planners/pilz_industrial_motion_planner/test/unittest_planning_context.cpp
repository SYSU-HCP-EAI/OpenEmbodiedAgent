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

#include <boost/core/demangle.hpp>
#include <gtest/gtest.h>

#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit/planning_interface/planning_interface.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_industrial_motion_planner/joint_limits_container.h"
#include "pilz_industrial_motion_planner/planning_context_circ.h"
#include "pilz_industrial_motion_planner/planning_context_lin.h"
#include "pilz_industrial_motion_planner/planning_context_ptp.h"

#include "test_utils.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

// parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");

/**
 * A value type container to combine type and value
 * In the tests types are trajectory generators.
 * value = 0 refers to robot model without gripper
 * value = 1 refers to robot model with gripper
 */
template <typename T, int N>
class ValueTypeContainer
{
public:
  typedef T Type_;
  static const int VALUE = N;
};
template <typename T, int N>
const int ValueTypeContainer<T, N>::VALUE;

typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextPTP, 0> PTP_NO_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextPTP, 1> PTP_WITH_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextLIN, 0> LIN_NO_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextLIN, 1> LIN_WITH_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextCIRC, 0> CIRC_NO_GRIPPER;
typedef ValueTypeContainer<pilz_industrial_motion_planner::PlanningContextCIRC, 1> CIRC_WITH_GRIPPER;

typedef ::testing::Types<PTP_NO_GRIPPER, PTP_WITH_GRIPPER, LIN_NO_GRIPPER, LIN_WITH_GRIPPER, CIRC_NO_GRIPPER,
                         CIRC_WITH_GRIPPER>
    PlanningContextTestTypes;

/**
 * type parameterized test fixture
 */
template <typename T>
class PlanningContextTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_FALSE(robot_model_ == nullptr) << "There is no robot model!";

    // get parameters
    ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
    ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));

    pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
        testutils::createFakeLimits(robot_model_->getVariableNames());
    pilz_industrial_motion_planner::CartesianLimit cartesian_limit;
    cartesian_limit.setMaxRotationalVelocity(1.0 * M_PI);
    cartesian_limit.setMaxTranslationalAcceleration(1.0 * M_PI);
    cartesian_limit.setMaxTranslationalDeceleration(1.0 * M_PI);
    cartesian_limit.setMaxTranslationalVelocity(1.0 * M_PI);

    pilz_industrial_motion_planner::LimitsContainer limits;
    limits.setJointLimits(joint_limits);
    limits.setCartesianLimits(cartesian_limit);

    planning_context_ = std::unique_ptr<typename T::Type_>(
        new typename T::Type_("TestPlanningContext", planning_group_, robot_model_, limits));

    // Define and set the current scene
    planning_scene::PlanningScenePtr scene(new planning_scene::PlanningScene(robot_model_));
    robot_state::RobotState current_state(robot_model_);
    current_state.setToDefaultValues();
    current_state.setJointGroupPositions(planning_group_, std::vector<double>{ 0, 1.57, 1.57, 0, 0.2, 0 });
    scene->setCurrentState(current_state);
    planning_context_->setPlanningScene(scene);  // TODO Check what happens if this is missing
  }

  /**
   * @brief Generate a valid fully defined request
   */
  planning_interface::MotionPlanRequest getValidRequest(const std::string& context_name) const
  {
    planning_interface::MotionPlanRequest req;

    req.planner_id =
        std::string(context_name).erase(0, std::string("pilz_industrial_motion_planner::PlanningContext").length());
    req.group_name = this->planning_group_;
    req.max_velocity_scaling_factor = 0.01;
    req.max_acceleration_scaling_factor = 0.01;

    // start state
    robot_state::RobotState rstate(this->robot_model_);
    rstate.setToDefaultValues();
    // state state in joint space, used as initial positions, since IK does not
    // work at zero positions
    rstate.setJointGroupPositions(this->planning_group_,
                                  std::vector<double>{ 4.430233957464225e-12, 0.007881892504574495, -1.8157263253868452,
                                                       1.1801525390026025e-11, 1.8236082178909834,
                                                       8.591793942969161e-12 });
    Eigen::Isometry3d start_pose(Eigen::Isometry3d::Identity());
    start_pose.translation() = Eigen::Vector3d(0.3, 0, 0.65);
    rstate.setFromIK(this->robot_model_->getJointModelGroup(this->planning_group_), start_pose);
    moveit::core::robotStateToRobotStateMsg(rstate, req.start_state, false);

    // goal constraint
    Eigen::Isometry3d goal_pose(Eigen::Isometry3d::Identity());
    goal_pose.translation() = Eigen::Vector3d(0, 0.3, 0.65);
    Eigen::Matrix3d goal_rotation;
    goal_rotation = Eigen::AngleAxisd(0 * M_PI, Eigen::Vector3d::UnitZ());
    goal_pose.linear() = goal_rotation;
    rstate.setFromIK(this->robot_model_->getJointModelGroup(this->planning_group_), goal_pose);
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
        rstate, this->robot_model_->getJointModelGroup(this->planning_group_)));

    // path constraint
    req.path_constraints.name = "center";
    moveit_msgs::PositionConstraint center_point;
    center_point.link_name = this->target_link_;
    geometry_msgs::Pose center_position;
    center_position.position.x = 0.0;
    center_position.position.y = 0.0;
    center_position.position.z = 0.65;
    center_point.constraint_region.primitive_poses.push_back(center_position);
    req.path_constraints.position_constraints.push_back(center_point);

    return req;
  }

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{
    robot_model_loader::RobotModelLoader(!T::VALUE ? PARAM_MODEL_NO_GRIPPER_NAME : PARAM_MODEL_WITH_GRIPPER_NAME)
        .getModel()
  };

  std::unique_ptr<planning_interface::PlanningContext> planning_context_;

  std::string planning_group_, target_link_;
};

// Define the types we need to test
TYPED_TEST_SUITE(PlanningContextTest, PlanningContextTestTypes);

/**
 * @brief No request is set. Check the output of solve. Using robot model
 * without gripper.
 */
TYPED_TEST(PlanningContextTest, NoRequest)
{
  planning_interface::MotionPlanResponse res;
  bool result = this->planning_context_->solve(res);

  EXPECT_FALSE(result) << testutils::demangle(typeid(TypeParam).name());
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, res.error_code_.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Solve a valid request.
 */
TYPED_TEST(PlanningContextTest, SolveValidRequest)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req = this->getValidRequest(testutils::demangle(typeid(TypeParam).name()));

  this->planning_context_->setMotionPlanRequest(req);

  // TODO Formulate valid request
  bool result = this->planning_context_->solve(res);

  EXPECT_TRUE(result) << testutils::demangle(typeid(TypeParam).name());
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, res.error_code_.val)
      << testutils::demangle(typeid(TypeParam).name());

  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool result_detailed = this->planning_context_->solve(res_detailed);

  EXPECT_TRUE(result_detailed) << testutils::demangle(typeid(TypeParam).name());
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, res.error_code_.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Solve a valid request. Expect a detailed response.
 */
TYPED_TEST(PlanningContextTest, SolveValidRequestDetailedResponse)
{
  planning_interface::MotionPlanDetailedResponse res;  //<-- Detailed!
  planning_interface::MotionPlanRequest req = this->getValidRequest(testutils::demangle(typeid(TypeParam).name()));

  this->planning_context_->setMotionPlanRequest(req);
  bool result = this->planning_context_->solve(res);

  EXPECT_TRUE(result) << testutils::demangle(typeid(TypeParam).name());
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, res.error_code_.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Call solve on a terminated context.
 */
TYPED_TEST(PlanningContextTest, SolveOnTerminated)
{
  planning_interface::MotionPlanResponse res;
  planning_interface::MotionPlanRequest req = this->getValidRequest(testutils::demangle(typeid(TypeParam).name()));

  this->planning_context_->setMotionPlanRequest(req);

  bool result_termination = this->planning_context_->terminate();
  EXPECT_TRUE(result_termination) << testutils::demangle(typeid(TypeParam).name());

  bool result = this->planning_context_->solve(res);
  EXPECT_FALSE(result) << testutils::demangle(typeid(TypeParam).name());

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::PLANNING_FAILED, res.error_code_.val)
      << testutils::demangle(typeid(TypeParam).name());
}

/**
 * @brief Check if clear can be called. So far only stability is expected.
 */
TYPED_TEST(PlanningContextTest, Clear)
{
  EXPECT_NO_THROW(this->planning_context_->clear()) << testutils::demangle(typeid(TypeParam).name());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_planning_context");
  // ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
