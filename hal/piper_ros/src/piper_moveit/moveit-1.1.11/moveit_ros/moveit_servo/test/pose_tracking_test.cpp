/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Andy Zelenak
   Desc:   Test of tracking toward a pose
*/

// C++
#include <string>
#include <thread>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Servo
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/make_shared_from_pool.h>

static const std::string LOGNAME = "servo_cpp_interface_test";
static constexpr double TRANSLATION_TOLERANCE = 0.01;  // meters
static constexpr double ROTATION_TOLERANCE = 0.1;      // quaternion
static constexpr double ROS_PUB_SUB_DELAY = 4;         // allow for subscribers to initialize

namespace moveit_servo
{
class PoseTrackingFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    // Wait for several key topics / parameters
    ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    while (!nh_.hasParam("/robot_description") && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Load the planning scene monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);

    tracker_ = std::make_shared<moveit_servo::PoseTracking>(nh_, planning_scene_monitor_);

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_pose", 1 /* queue */, true /* latch */);

    // Tolerance for pose seeking
    translation_tolerance_ << TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE;
  }
  void TearDown() override
  {
  }

protected:
  ros::NodeHandle nh_{ "~" };
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  Eigen::Vector3d translation_tolerance_;
  moveit_servo::PoseTrackingPtr tracker_;
  ros::Publisher target_pose_pub_;
};  // class PoseTrackingFixture

// Check for commands going out to ros_control
TEST_F(PoseTrackingFixture, OutgoingMsgTest)
{
  // halt Servoing when first msg to ros_control is received
  // and test some conditions
  trajectory_msgs::JointTrajectory last_received_msg;
  boost::function<void(const trajectory_msgs::JointTrajectoryConstPtr&)> traj_callback =
      [&/* this */](const trajectory_msgs::JointTrajectoryConstPtr& msg) {
        EXPECT_EQ(msg->header.frame_id, "panda_link0");
        // Check for an expected joint position command
        // As of now, the robot doesn't actually move because there are no controllers enabled.
        double angle_tolerance = 0.08;  // rad
        EXPECT_NEAR(msg->points[0].positions[0], 0, angle_tolerance);
        EXPECT_NEAR(msg->points[0].positions[1], -0.785, angle_tolerance);
        EXPECT_NEAR(msg->points[0].positions[2], 0, angle_tolerance);
        EXPECT_NEAR(msg->points[0].positions[3], -2.360, angle_tolerance);
        EXPECT_NEAR(msg->points[0].positions[4], 0, angle_tolerance);
        EXPECT_NEAR(msg->points[0].positions[5], 1.571, angle_tolerance);
        EXPECT_NEAR(msg->points[0].positions[6], 0.785, angle_tolerance);

        this->tracker_->stopMotion();
        return;
      };
  auto traj_sub = nh_.subscribe("servo_server/command", 1, traj_callback);

  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "panda_link4";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.2;
  target_pose.pose.position.y = 0.2;
  target_pose.pose.position.z = 0.2;
  target_pose.pose.orientation.w = 1;

  // Republish the target pose in a new thread, as if the target is moving
  std::thread target_pub_thread([&] {
    size_t msg_count = 0;
    while (++msg_count < 100)
    {
      target_pose_pub_.publish(target_pose);
      ros::Duration(0.01).sleep();
    }
  });

  ros::Duration(ROS_PUB_SUB_DELAY).sleep();

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  tracker_->resetTargetPose();

  tracker_->moveToPose(translation_tolerance_, ROTATION_TOLERANCE, 1 /* target pose timeout */);

  target_pub_thread.join();
}

}  // namespace moveit_servo

int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  testing::InitGoogleTest(&argc, argv);

  ros::AsyncSpinner spinner(8);
  spinner.start();

  int result = RUN_ALL_TESTS();
  return result;
}
