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

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pilz_industrial_motion_planner_testutils/command_types_typedef.h>
#include <pilz_industrial_motion_planner_testutils/sequence.h>
#include <pilz_industrial_motion_planner_testutils/xml_testdata_loader.h>

#include "pilz_industrial_motion_planner/joint_limits_aggregator.h"
#include "pilz_industrial_motion_planner/trajectory_blend_request.h"
#include "pilz_industrial_motion_planner/trajectory_blend_response.h"
#include "pilz_industrial_motion_planner/trajectory_blender_transition_window.h"
#include "pilz_industrial_motion_planner/trajectory_generator_lin.h"
#include "test_utils.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

// parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string CARTESIAN_VELOCITY_TOLERANCE("cartesian_velocity_tolerance");
const std::string CARTESIAN_ANGULAR_VELOCITY_TOLERANCE("cartesian_angular_velocity_tolerance");
const std::string JOINT_VELOCITY_TOLERANCE("joint_velocity_tolerance");
const std::string JOINT_ACCELERATION_TOLERANCE("joint_acceleration_tolerance");
const std::string OTHER_TOLERANCE("other_tolerance");
const std::string SAMPLING_TIME("sampling_time");
const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using namespace pilz_industrial_motion_planner;
using namespace pilz_industrial_motion_planner_testutils;

class TrajectoryBlenderTransitionWindowTest : public testing::TestWithParam<std::string>
{
protected:
  /**
   * @brief Create test scenario for trajectory blender
   *
   */
  void SetUp() override;

  /**
   * @brief Generate lin trajectories for blend sequences
   */
  std::vector<planning_interface::MotionPlanResponse> generateLinTrajs(const Sequence& seq, size_t num_cmds);

protected:
  // ros stuff
  ros::NodeHandle ph_{ "~" };
  robot_model::RobotModelConstPtr robot_model_{ robot_model_loader::RobotModelLoader(GetParam()).getModel() };
  planning_scene::PlanningSceneConstPtr planning_scene_{ new planning_scene::PlanningScene(robot_model_) };

  std::unique_ptr<TrajectoryGenerator> lin_generator_;
  std::unique_ptr<TrajectoryBlenderTransitionWindow> blender_;

  // test parameters from parameter server
  std::string planning_group_, target_link_;
  double cartesian_velocity_tolerance_, cartesian_angular_velocity_tolerance_, joint_velocity_tolerance_,
      joint_acceleration_tolerance_, sampling_time_;
  LimitsContainer planner_limits_;

  std::string test_data_file_name_;
  XmlTestDataLoaderUPtr data_loader_;
};

void TrajectoryBlenderTransitionWindowTest::SetUp()
{
  // get parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph_.getParam(CARTESIAN_VELOCITY_TOLERANCE, cartesian_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(CARTESIAN_ANGULAR_VELOCITY_TOLERANCE, cartesian_angular_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_VELOCITY_TOLERANCE, joint_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_ACCELERATION_TOLERANCE, joint_acceleration_tolerance_));
  ASSERT_TRUE(ph_.getParam(SAMPLING_TIME, sampling_time_));
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  // load the test data provider
  data_loader_ = std::make_unique<XmlTestdataLoader>(test_data_file_name_, robot_model_);
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // create the limits container
  pilz_industrial_motion_planner::JointLimitsContainer joint_limits =
      pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(ph_,
                                                                                 robot_model_->getActiveJointModels());
  CartesianLimit cart_limits;
  cart_limits.setMaxRotationalVelocity(1 * M_PI);
  cart_limits.setMaxTranslationalAcceleration(2);
  cart_limits.setMaxTranslationalDeceleration(2);
  cart_limits.setMaxTranslationalVelocity(1);
  planner_limits_.setJointLimits(joint_limits);
  planner_limits_.setCartesianLimits(cart_limits);

  // initialize trajectory generators and blender
  lin_generator_ = std::make_unique<TrajectoryGeneratorLIN>(robot_model_, planner_limits_, planning_group_);
  ASSERT_NE(nullptr, lin_generator_) << "failed to create LIN trajectory generator";
  blender_ = std::make_unique<TrajectoryBlenderTransitionWindow>(planner_limits_);
  ASSERT_NE(nullptr, blender_) << "failed to create trajectory blender";
}

std::vector<planning_interface::MotionPlanResponse>
TrajectoryBlenderTransitionWindowTest::generateLinTrajs(const Sequence& seq, size_t num_cmds)
{
  std::vector<planning_interface::MotionPlanResponse> responses(num_cmds);

  for (size_t index = 0; index < num_cmds; ++index)
  {
    planning_interface::MotionPlanRequest req{ seq.getCmd<LinCart>(index).toRequest() };
    // Set start state of request to end state of previous trajectory (except
    // for first)
    if (index > 0)
    {
      moveit::core::robotStateToRobotStateMsg(responses[index - 1].trajectory_->getLastWayPoint(), req.start_state);
    }
    // generate trajectory
    planning_interface::MotionPlanResponse resp;
    if (!lin_generator_->generate(planning_scene_, req, resp, sampling_time_))
    {
      std::runtime_error("Failed to generate trajectory.");
    }
    responses.at(index) = resp;
  }
  return responses;
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_SUITE_P(InstantiationName, TrajectoryBlenderTransitionWindowTest,
                         ::testing::Values(PARAM_MODEL_NO_GRIPPER_NAME, PARAM_MODEL_WITH_GRIPPER_NAME));

/**
 * @brief  Tests the blending of two trajectories with an invalid group name.
 *
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with invalid group name.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testInvalidGroupName)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = "invalid_group_name";
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  blend_req.blend_radius = seq.getBlendRadius(0);
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with an invalid target link.
 *
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with invalid target link.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testInvalidTargetLink)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = "invalid_target_link";
  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  blend_req.blend_radius = seq.getBlendRadius(0);
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with a negative blending
 * radius.
 *
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with negative blending radius.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testNegativeRadius)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  blend_req.blend_radius = -0.1;
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with zero blending radius.
 *
 * Test Sequence:
 *    1. Generate two linear trajectories.
 *    2. Try to generate blending trajectory with zero blending radius.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testZeroRadius)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  blend_req.blend_radius = 0.;
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with differenent sampling
 * times.
 *
 * Test Sequence:
 *    1. Generate two linear trajectories with different sampling times.
 *    2. Try to generate blending trajectory.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testDifferentSamplingTimes)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  // perform lin trajectory generation and modify sampling time
  std::size_t num_cmds{ 2 };
  std::vector<planning_interface::MotionPlanResponse> responses(num_cmds);

  for (size_t index = 0; index < num_cmds; ++index)
  {
    planning_interface::MotionPlanRequest req{ seq.getCmd<LinCart>(index).toRequest() };
    // Set start state of request to end state of previous trajectory (except
    // for first)
    if (index > 0)
    {
      moveit::core::robotStateToRobotStateMsg(responses[index - 1].trajectory_->getLastWayPoint(), req.start_state);
      sampling_time_ *= 2;
    }
    // generate trajectory
    planning_interface::MotionPlanResponse resp;
    if (!lin_generator_->generate(planning_scene_, req, resp, sampling_time_))
    {
      std::runtime_error("Failed to generate trajectory.");
    }
    responses.at(index) = resp;
  }

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = responses[0].trajectory_;
  blend_req.second_trajectory = responses[1].trajectory_;
  blend_req.blend_radius = seq.getBlendRadius(0);
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories with one trajectory
 * having non-uniform sampling time (apart from the last sample,
 * which is ignored).
 *
 * Test Sequence:
 *    1. Generate two linear trajectories and corrupt uniformity of sampling
 * time.
 *    2. Try to generate blending trajectory.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testNonUniformSamplingTime)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  // Modify first time interval
  EXPECT_GT(res[0].trajectory_->getWayPointCount(), 2u);
  res[0].trajectory_->setWayPointDurationFromPrevious(1, 2 * sampling_time_);

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;
  blend_req.blend_radius = seq.getBlendRadius(0);
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two trajectories which do not intersect.
 *
 * Test Sequence:
 *    1. Generate two trajectories from valid test data set.
 *    2. Replace the second trajectory by the first one.
 *    2. Try to generate blending trajectory.
 *
 * Expected Results:
 *    1. Two trajectories generated.
 *    2. Two trajectories that do not intersect.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testNotIntersectingTrajectories)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 1) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res.at(0).trajectory_;
  // replace the second trajectory to make the two trajectories timely not
  // intersect
  blend_req.second_trajectory = res.at(0).trajectory_;
  blend_req.blend_radius = seq.getBlendRadius(0);
  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two cartesian trajectories with the
 * shared point (last point of first, first point of second trajectory)
 * having a non-zero velocity
 *
 * Test Sequence:
 *    1. Generate two trajectories from the test data set.
 *    2. Generate blending trajectory modify the shared point to have velocity.
 * Expected Results:
 *    1. Two trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testNonStationaryPoint)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = seq.getBlendRadius(0);

  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  // Modify last waypoint of first trajectory and first point of second
  // trajectory
  blend_req.first_trajectory->getLastWayPointPtr()->setVariableVelocity(0, 1.0);
  blend_req.second_trajectory->getFirstWayPointPtr()->setVariableVelocity(0, 1.0);

  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief Tests the blending of two cartesian trajectories where the first
 * trajectory is completely within the sphere defined by the blend radius
 *
 * Test Sequence:
 *    1. Generate two trajectories from the test data set.
 *    2. Generate blending trajectory with a blend_radius larger
 *        than the smaller trajectory.
 *
 * Expected Results:
 *    1. Two trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testTraj1InsideBlendRadius)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  double lin1_distance;
  lin1_distance = (res[0].trajectory_->getFirstWayPoint().getFrameTransform(target_link_).translation() -
                   res[0].trajectory_->getLastWayPoint().getFrameTransform(target_link_).translation())
                      .norm();

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = 1.1 * lin1_distance;

  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief Tests the blending of two cartesian trajectories where the second
 * trajectory is completely within the sphere defined by the blend radius
 *
 * Test Sequence:
 *    1. Generate two trajectories from the test data set.
 *    2. Generate blending trajectory with a blend_radius larger
 *        than the smaller trajectory.
 *
 * Expected Results:
 *    1. Two trajectories generated.
 *    2. Blending trajectory cannot be generated.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testTraj2InsideBlendRadius)
{
  Sequence seq{ data_loader_->getSequence("NoIntersectionTraj2") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = seq.getBlendRadius(0);

  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  EXPECT_FALSE(blender_->blend(planning_scene_, blend_req, blend_res));
}

/**
 * @brief  Tests the blending of two cartesian linear trajectories using robot
 * model
 *
 * Test Sequence:
 *    1. Generate two linear trajectories from the test data set.
 *    2. Generate blending trajectory.
 *    3. Check blending trajectory:
 *      - for position, velocity, and acceleration bounds,
 *      - for continuity in joint space,
 *      - for continuity in cartesian space.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory generated.
 *    3. No bound is violated, the trajectories are continuous
 *        in joint and cartesian space.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testLinLinBlending)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = seq.getBlendRadius(0);

  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;

  EXPECT_TRUE(blender_->blend(planning_scene_, blend_req, blend_res));

  EXPECT_TRUE(testutils::checkBlendResult(blend_req, blend_res, planner_limits_, joint_velocity_tolerance_,
                                          joint_acceleration_tolerance_, cartesian_velocity_tolerance_,
                                          cartesian_angular_velocity_tolerance_));
}

/**
 * @brief  Tests the blending of two cartesian linear trajectories which have
 * an overlap in the blending sphere using robot model. To be precise,
 * the trajectories exactly lie on top of each other.
 *
 * Test Sequence:
 *    1. Generate two linear trajectories from the test data set. Set goal of
 * second traj to start of first traj.
 *    2. Generate blending trajectory.
 *    3. Check blending trajectory:
 *      - for position, velocity, and acceleration bounds,
 *      - for continuity in joint space,
 *      - for continuity in cartesian space.
 *
 * Expected Results:
 *    1. Two linear trajectories generated.
 *    2. Blending trajectory generated.
 *    3. No bound is violated, the trajectories are continuous
 *        in joint and cartesian space.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testOverlappingBlendTrajectories)
{
  Sequence seq{ data_loader_->getSequence("SimpleSequence") };
  // Set goal of second traj to start of first traj.
  seq.getCmd<LinCart>(1).setGoalConfiguration(seq.getCmd<LinCart>(0).getStartConfiguration());

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_req;
  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.first_trajectory = res.at(0).trajectory_;
  blend_req.second_trajectory = res.at(1).trajectory_;
  blend_req.blend_radius = seq.getBlendRadius(0);
  EXPECT_TRUE(blender_->blend(planning_scene_, blend_req, blend_res));

  EXPECT_TRUE(testutils::checkBlendResult(blend_req, blend_res, planner_limits_, joint_velocity_tolerance_,
                                          joint_acceleration_tolerance_, cartesian_velocity_tolerance_,
                                          cartesian_angular_velocity_tolerance_));
}

/**
 * @brief Tests the blending of two cartesian trajectories which differ
 * from a straight line.
 *
 * Test Sequence:
 *    1. Generate two trajectories from the test data set.
 *    2. Add scaled sine function to cartesian trajectories, such that
 *        start and end state remain unchanged; generate resulting
 *        joint trajectories using a time scaling in order to preserve
 *        joint velocity limits.
 *    3. Generate blending trajectory.
 *    4. Check blending trajectory:
 *      - for position, velocity, and acceleration bounds,
 *      - for continuity in joint space,
 *      - for continuity in cartesian space.
 *
 * Expected Results:
 *    1. Two trajectories generated.
 *    2. Modified joint trajectories generated.
 *    3. Blending trajectory generated.
 *    4. No bound is violated, the trajectories are continuous
 *        in joint and cartesian space.
 */
TEST_P(TrajectoryBlenderTransitionWindowTest, testNonLinearBlending)
{
  const double sine_scaling_factor{ 0.01 };
  const double time_scaling_factor{ 10 };

  Sequence seq{ data_loader_->getSequence("SimpleSequence") };

  std::vector<planning_interface::MotionPlanResponse> res{ generateLinTrajs(seq, 2) };

  // prepare looping over trajectories
  std::vector<robot_trajectory::RobotTrajectoryPtr> sine_trajs(2);

  for (size_t traj_index = 0; traj_index < 2; ++traj_index)
  {
    auto lin_traj{ res.at(traj_index).trajectory_ };

    CartesianTrajectory cart_traj;
    trajectory_msgs::JointTrajectory joint_traj;
    const double duration{ lin_traj->getWayPointDurationFromStart(lin_traj->getWayPointCount()) };
    // time from start zero does not work
    const double time_from_start_offset{ time_scaling_factor * lin_traj->getWayPointDurations().back() };

    // generate modified cartesian trajectory
    for (size_t i = 0; i < lin_traj->getWayPointCount(); ++i)
    {
      // transform time to interval [0, 4*pi]
      const double sine_arg{ 4 * M_PI * lin_traj->getWayPointDurationFromStart(i) / duration };

      // get pose
      CartesianTrajectoryPoint waypoint;
      const Eigen::Isometry3d eigen_pose{ lin_traj->getWayPointPtr(i)->getFrameTransform(target_link_) };
      geometry_msgs::Pose waypoint_pose = tf2::toMsg(eigen_pose);

      // add scaled sine function
      waypoint_pose.position.x += sine_scaling_factor * sin(sine_arg);
      waypoint_pose.position.y += sine_scaling_factor * sin(sine_arg);
      waypoint_pose.position.z += sine_scaling_factor * sin(sine_arg);

      // add to trajectory
      waypoint.pose = waypoint_pose;
      waypoint.time_from_start =
          ros::Duration(time_from_start_offset + time_scaling_factor * lin_traj->getWayPointDurationFromStart(i));
      cart_traj.points.push_back(waypoint);
    }

    // prepare ik
    std::map<std::string, double> initial_joint_position, initial_joint_velocity;
    for (const std::string& joint_name :
         lin_traj->getFirstWayPointPtr()->getJointModelGroup(planning_group_)->getActiveJointModelNames())
    {
      if (traj_index == 0)
      {
        initial_joint_position[joint_name] = lin_traj->getFirstWayPoint().getVariablePosition(joint_name);
        initial_joint_velocity[joint_name] = lin_traj->getFirstWayPoint().getVariableVelocity(joint_name);
      }
      else
      {
        initial_joint_position[joint_name] =
            sine_trajs[traj_index - 1]->getLastWayPoint().getVariablePosition(joint_name);
        initial_joint_velocity[joint_name] =
            sine_trajs[traj_index - 1]->getLastWayPoint().getVariableVelocity(joint_name);
      }
    }

    moveit_msgs::MoveItErrorCodes error_code;
    if (!generateJointTrajectory(planning_scene_, planner_limits_.getJointLimitContainer(), cart_traj, planning_group_,
                                 target_link_, initial_joint_position, initial_joint_velocity, joint_traj, error_code,
                                 true))
    {
      std::runtime_error("Failed to generate trajectory.");
    }

    joint_traj.points.back().velocities.assign(joint_traj.points.back().velocities.size(), 0.0);
    joint_traj.points.back().accelerations.assign(joint_traj.points.back().accelerations.size(), 0.0);

    // convert trajectory_msgs::JointTrajectory to
    // robot_trajectory::RobotTrajectory
    sine_trajs[traj_index] = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_group_);
    sine_trajs.at(traj_index)->setRobotTrajectoryMsg(lin_traj->getFirstWayPoint(), joint_traj);
  }

  TrajectoryBlendRequest blend_req;
  TrajectoryBlendResponse blend_res;

  blend_req.group_name = planning_group_;
  blend_req.link_name = target_link_;
  blend_req.blend_radius = seq.getBlendRadius(0);

  blend_req.first_trajectory = sine_trajs.at(0);
  blend_req.second_trajectory = sine_trajs.at(1);

  EXPECT_TRUE(blender_->blend(planning_scene_, blend_req, blend_res));

  EXPECT_TRUE(testutils::checkBlendResult(blend_req, blend_res, planner_limits_, joint_velocity_tolerance_,
                                          joint_acceleration_tolerance_, cartesian_velocity_tolerance_,
                                          cartesian_angular_velocity_tolerance_));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_trajectory_blender_transition_window");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
