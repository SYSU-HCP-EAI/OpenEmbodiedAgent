/*******************************************************************************
 *      Title     : spacenav_to_twist.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "geometry_msgs/TwistStamped.h"
#include "control_msgs/JointJog.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

namespace moveit_servo
{
static const int NUM_SPINNERS = 1;
static const int QUEUE_LENGTH = 1;

class SpaceNavToTwist
{
public:
  SpaceNavToTwist() : spinner_(NUM_SPINNERS)
  {
    joy_sub_ = n_.subscribe("spacenav/joy", QUEUE_LENGTH, &SpaceNavToTwist::joyCallback, this);
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("servo_server/delta_twist_cmds", QUEUE_LENGTH);
    joint_delta_pub_ = n_.advertise<control_msgs::JointJog>("servo_server/delta_joint_cmds", QUEUE_LENGTH);

    spinner_.start();
    ros::waitForShutdown();
  };

private:
  // Convert incoming joy commands to TwistStamped commands for servoing.
  // The TwistStamped component goes to servoing, while buttons 0 & 1 control
  // joints directly.
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Cartesian servoing with the axes
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.twist.linear.x = msg->axes[0];
    twist.twist.linear.y = msg->axes[1];
    twist.twist.linear.z = msg->axes[2];

    twist.twist.angular.x = msg->axes[3];
    twist.twist.angular.y = msg->axes[4];
    twist.twist.angular.z = msg->axes[5];

    // Joint servoing with the buttons
    control_msgs::JointJog joint_deltas;
    // This example is for a UR5.
    joint_deltas.joint_names.push_back("shoulder_pan_joint");

    // Button 0: positive on the wrist joint
    // Button 1: negative on the wrist joint
    joint_deltas.velocities.push_back(msg->buttons[0] - msg->buttons[1]);
    joint_deltas.header.stamp = ros::Time::now();

    twist_pub_.publish(twist);
    joint_delta_pub_.publish(joint_deltas);
  }

  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;
  ros::Publisher twist_pub_, joint_delta_pub_;
  ros::AsyncSpinner spinner_;
};
}  // namespace moveit_servo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spacenav_to_twist");

  moveit_servo::SpaceNavToTwist to_twist;

  return 0;
}
