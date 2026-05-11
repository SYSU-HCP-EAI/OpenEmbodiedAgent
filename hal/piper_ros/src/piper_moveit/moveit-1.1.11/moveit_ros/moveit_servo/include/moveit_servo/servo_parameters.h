/*******************************************************************************
 *      Title     : servo_parameters.h
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

#pragma once

namespace moveit_servo
{
// Size of queues used in ros pub/sub/service
constexpr size_t ROS_QUEUE_SIZE = 2;

// ROS params to be read. See the yaml file in /config for a description of each.
struct ServoParameters
{
  std::string move_group_name;
  std::string joint_topic;
  std::string cartesian_command_in_topic;
  std::string robot_link_command_frame;
  std::string command_out_topic;
  std::string planning_frame;
  std::string ee_frame_name;
  std::string status_topic;
  std::string joint_command_in_topic;
  std::string command_in_type;
  std::string command_out_type;
  double linear_scale;
  double rotational_scale;
  double joint_scale;
  double lower_singularity_threshold;
  double hard_stop_singularity_threshold;
  double low_pass_filter_coeff;
  double publish_period;
  double incoming_command_timeout;
  double joint_limit_margin;
  int num_outgoing_halt_msgs_to_publish;
  bool use_gazebo;
  bool publish_joint_positions;
  bool publish_joint_velocities;
  bool publish_joint_accelerations;
  bool low_latency_mode;
  // Collision checking
  bool check_collisions;
  std::string collision_check_type;
  double collision_check_rate;
  double scene_collision_proximity_threshold;
  double self_collision_proximity_threshold;
  double collision_distance_safety_factor;
  double min_allowable_collision_distance;
};

}  // namespace moveit_servo
