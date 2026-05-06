/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Tyler Weaver
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
 *   * Neither the name of PickNik Robotics nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Tyler Weaver, Boston Cleek */

/* These integration tests are based on the tutorials for using move_group:
 * https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
 */

// C++
#include <string>
#include <vector>
#include <map>
#include <future>

// ROS
#include <ros/ros.h>

// The Testing Framework and Utils
#include <gtest/gtest.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// 10um acuracy tested for position and orientation
constexpr double EPSILON = 1e-5;

static const std::string PLANNING_GROUP = "panda_arm";
constexpr double PLANNING_TIME_S = 30.0;
constexpr double MAX_VELOCITY_SCALE = 1.0;
constexpr double MAX_ACCELERATION_SCALE = 1.0;
constexpr double GOAL_TOLERANCE = 1e-6;

class MoveGroupTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    nh_ = ros::NodeHandle("/move_group_interface_cpp_test");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

    // set velocity and acceleration scaling factors (full speed)
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);

    // allow more time for planning
    move_group_->setPlanningTime(PLANNING_TIME_S);

    // set the tolerance for the goals to be smaller than epsilon
    move_group_->setGoalTolerance(GOAL_TOLERANCE);

    /* the tf buffer is not strictly needed,
       but it's a simple way to add the codepaths to the tests */
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description",
                                                                          moveit::planning_interface::getSharedTF());
    psm_->startSceneMonitor("/move_group/monitored_planning_scene");
    psm_->requestPlanningSceneState();

    // give move_group_, planning_scene_interface_ and psm_ time to connect their topics
    ros::Duration(0.5).sleep();
  }

  // run updater() and ensure at least one geometry update was processed by the `move_group` node after doing so
  void synchronizeGeometryUpdate(const std::function<void()>& updater)
  {
    SCOPED_TRACE("synchronizeGeometryUpdate");
    std::promise<void> promise;
    std::future<void> future = promise.get_future();
    psm_->addUpdateCallback([this, &promise](planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType t) {
      if (t & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
        promise.set_value();
      psm_->clearUpdateCallbacks();
    });
    updater();
    // the updater must have triggered a geometry update, otherwise we can't be sure about the state of the scene anymore
    ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  }

  void planAndMoveToPose(const geometry_msgs::Pose& pose)
  {
    SCOPED_TRACE("planAndMoveToPose");
    ASSERT_TRUE(move_group_->setJointValueTarget(pose));
    planAndMove();
  }

  void planAndMove()
  {
    SCOPED_TRACE("planAndMove");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ASSERT_EQ(move_group_->plan(my_plan), moveit::core::MoveItErrorCode::SUCCESS);
    ASSERT_EQ(move_group_->move(), moveit::core::MoveItErrorCode::SUCCESS);
  }

  void testEigenPose(const Eigen::Isometry3d& expected, const Eigen::Isometry3d& actual)
  {
    SCOPED_TRACE("testEigenPose");
    std::stringstream ss;
    ss << "expected: \n" << expected.matrix() << "\nactual: \n" << actual.matrix();
    EXPECT_TRUE(actual.isApprox(expected, EPSILON)) << ss.str();
  }

  void testPose(const Eigen::Isometry3d& expected_pose)
  {
    SCOPED_TRACE("testPose(const Eigen::Isometry3d&)");
    // get the pose of the end effector link after the movement
    geometry_msgs::PoseStamped actual_pose_stamped = move_group_->getCurrentPose();
    Eigen::Isometry3d actual_pose;
    tf2::fromMsg(actual_pose_stamped.pose, actual_pose);

    // compare to planned pose
    testEigenPose(expected_pose, actual_pose);
  }

  void testPose(const geometry_msgs::Pose& expected_pose_msg)
  {
    SCOPED_TRACE("testPose(const geometry_msgs::Pose&)");
    Eigen::Isometry3d expected_pose;
    tf2::fromMsg(expected_pose_msg, expected_pose);
    testPose(expected_pose);
  }

  void testJointPositions(const std::vector<double>& expected)
  {
    SCOPED_TRACE("testJointPositions");
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> actual;
    move_group_->getCurrentState()->copyJointGroupPositions(joint_model_group, actual);
    ASSERT_EQ(expected.size(), actual.size());
    for (size_t i = 0; i < actual.size(); ++i)
    {
      double delta = std::abs(expected[i] - actual[i]);
      EXPECT_LT(delta, EPSILON) << "joint index: " << i << ", plan: " << expected[i] << ", result: " << actual[i];
    }
  }

protected:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

TEST_F(MoveGroupTestFixture, PathConstraintCollisionTest)
{
  SCOPED_TRACE("PathConstraintCollisionTest");

  ////////////////////////////////////////////////////////////////////
  // set a custom start state
  // this simplifies planning for the orientation constraint bellow
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.3;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.6;
  planAndMoveToPose(start_pose);

  ////////////////////////////////////////////////////////////////////
  // Test setting target pose with eigen and with geometry_msgs
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.3;
  target_pose.position.y = -0.3;
  target_pose.position.z = 0.6;

  // convert to eigen
  Eigen::Isometry3d eigen_target_pose;
  tf2::fromMsg(target_pose, eigen_target_pose);

  // set with eigen, get ros message representation
  move_group_->setPoseTarget(eigen_target_pose);
  geometry_msgs::PoseStamped set_target_pose = move_group_->getPoseTarget();
  Eigen::Isometry3d eigen_set_target_pose;
  tf2::fromMsg(set_target_pose.pose, eigen_set_target_pose);

  // expect that they are identical
  testEigenPose(eigen_target_pose, eigen_set_target_pose);

  ////////////////////////////////////////////////////////////////////
  // create an orientation constraint
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = move_group_->getEndEffectorLink();
  ocm.header.frame_id = move_group_->getPlanningFrame();
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_->setPathConstraints(test_constraints);

  ////////////////////////////////////////////////////////////////////
  // plan and move
  planAndMove();

  // get the pose after the movement
  testPose(eigen_target_pose);

  // clear path constraints
  move_group_->clearPathConstraints();
}

TEST_F(MoveGroupTestFixture, ModifyPlanningSceneAsyncInterfaces)
{
  ////////////////////////////////////////////////////////////////////
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 1.0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  synchronizeGeometryUpdate([&]() { planning_scene_interface_.addCollisionObjects(collision_objects); });

  // attach and detach collision object
  synchronizeGeometryUpdate([&]() { EXPECT_TRUE(move_group_->attachObject(collision_object.id)); });
  EXPECT_EQ(planning_scene_interface_.getAttachedObjects().size(), std::size_t(1));
  synchronizeGeometryUpdate([&]() { EXPECT_TRUE(move_group_->detachObject(collision_object.id)); });
  EXPECT_EQ(planning_scene_interface_.getAttachedObjects().size(), std::size_t(0));

  // remove object from world
  const std::vector<std::string> object_ids = { collision_object.id };
  EXPECT_EQ(planning_scene_interface_.getObjects().size(), std::size_t(1));
  synchronizeGeometryUpdate([&]() { planning_scene_interface_.removeCollisionObjects(object_ids); });
  EXPECT_EQ(planning_scene_interface_.getObjects().size(), std::size_t(0));
}

TEST_F(MoveGroupTestFixture, CartPathTest)
{
  SCOPED_TRACE("CartPathTest");

  // Plan from current pose
  const geometry_msgs::PoseStamped start_pose = move_group_->getCurrentPose();

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose.pose);

  geometry_msgs::Pose target_waypoint = start_pose.pose;
  target_waypoint.position.z -= 0.2;
  waypoints.push_back(target_waypoint);  // down

  target_waypoint.position.y -= 0.2;
  waypoints.push_back(target_waypoint);  // right

  target_waypoint.position.z += 0.2;
  target_waypoint.position.y += 0.2;
  target_waypoint.position.x -= 0.2;
  waypoints.push_back(target_waypoint);  // up and left

  moveit_msgs::RobotTrajectory trajectory;
  const auto jump_threshold = 0.0;
  const auto eef_step = 0.01;

  // test below is meaningless if Cartesian planning did not succeed
  ASSERT_GE(EPSILON + move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory), 1.0);

  // Execute trajectory
  EXPECT_EQ(move_group_->execute(trajectory), moveit::core::MoveItErrorCode::SUCCESS);

  // get the pose after the movement
  testPose(target_waypoint);
}

TEST_F(MoveGroupTestFixture, JointSpaceGoalTest)
{
  SCOPED_TRACE("JointSpaceGoalTest");

  // Next get the current set of joint values for the group.
  std::vector<double> plan_joint_positions;
  move_group_->getCurrentState()->copyJointGroupPositions(
      move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP), plan_joint_positions);

  // Now, let's modify the joint positions.  (radians)
  ASSERT_EQ(plan_joint_positions.size(), std::size_t(7));
  plan_joint_positions = { 1.2, -1.0, -0.1, -2.4, 0.0, 1.5, 0.6 };
  move_group_->setJointValueTarget(plan_joint_positions);

  // plan and move
  planAndMove();

  // test that we moved to the expected joint positions
  testJointPositions(plan_joint_positions);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_cpp_test");
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
