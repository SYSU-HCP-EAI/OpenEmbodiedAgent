/*******************************************************************************
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

/*      Title     : servo_calcs.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <cassert>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo_calcs.h>

static const std::string LOGNAME = "servo_calcs";
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops

namespace moveit_servo
{
namespace
{
// Helper function for detecting zeroed message
bool isNonZero(const geometry_msgs::TwistStamped& msg)
{
  return msg.twist.linear.x != 0.0 || msg.twist.linear.y != 0.0 || msg.twist.linear.z != 0.0 ||
         msg.twist.angular.x != 0.0 || msg.twist.angular.y != 0.0 || msg.twist.angular.z != 0.0;
}

// Helper function for detecting zeroed message
bool isNonZero(const control_msgs::JointJog& msg)
{
  bool all_zeros = true;
  for (double delta : msg.velocities)
  {
    all_zeros &= (delta == 0.0);
  };
  return !all_zeros;
}

// Helper function for converting Eigen::Isometry3d to geometry_msgs/TransformStamped
geometry_msgs::TransformStamped convertIsometryToTransform(const Eigen::Isometry3d& eigen_tf,
                                                           const std::string& parent_frame,
                                                           const std::string& child_frame)
{
  geometry_msgs::TransformStamped output = tf2::eigenToTransform(eigen_tf);
  output.header.frame_id = parent_frame;
  output.child_frame_id = child_frame;

  return output;
}
}  // namespace

// Constructor for the class that handles servoing calculations
ServoCalcs::ServoCalcs(ros::NodeHandle& nh, ServoParameters& parameters,
                       const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , stop_requested_(true)
  , paused_(false)
{
  // MoveIt Setup
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = current_state_->getJointModelGroup(parameters_.move_group_name);
  prev_joint_velocity_ = Eigen::ArrayXd::Zero(joint_model_group_->getActiveJointModels().size());

  // Subscribe to command topics
  twist_stamped_sub_ =
      nh_.subscribe(parameters_.cartesian_command_in_topic, ROS_QUEUE_SIZE, &ServoCalcs::twistStampedCB, this);
  joint_cmd_sub_ = nh_.subscribe(parameters_.joint_command_in_topic, ROS_QUEUE_SIZE, &ServoCalcs::jointCmdCB, this);

  // ROS Server for allowing drift in some dimensions
  drift_dimensions_server_ = nh_.advertiseService(ros::names::append(nh_.getNamespace(), "change_drift_dimensions"),
                                                  &ServoCalcs::changeDriftDimensions, this);

  // ROS Server for changing the control dimensions
  control_dimensions_server_ = nh_.advertiseService(ros::names::append(nh_.getNamespace(), "change_control_dimensions"),
                                                    &ServoCalcs::changeControlDimensions, this);

  // ROS Server to reset the status, e.g. so the arm can move again after a collision
  reset_servo_status_ = nh_.advertiseService(ros::names::append(nh_.getNamespace(), "reset_servo_status"),
                                             &ServoCalcs::resetServoStatus, this);

  // Publish and Subscribe to internal namespace topics
  ros::NodeHandle internal_nh(nh_, "internal");
  collision_velocity_scale_sub_ =
      internal_nh.subscribe("collision_velocity_scale", ROS_QUEUE_SIZE, &ServoCalcs::collisionVelocityScaleCB, this);
  worst_case_stop_time_pub_ = internal_nh.advertise<std_msgs::Float64>("worst_case_stop_time", ROS_QUEUE_SIZE);

  // Publish freshly-calculated joints to the robot.
  // Put the outgoing msg in the right format (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
  if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    outgoing_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(parameters_.command_out_topic, ROS_QUEUE_SIZE);
  else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    outgoing_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(parameters_.command_out_topic, ROS_QUEUE_SIZE);

  // Publish status
  status_pub_ = nh_.advertise<std_msgs::Int8>(parameters_.status_topic, ROS_QUEUE_SIZE);

  internal_joint_state_.name = joint_model_group_->getActiveJointModelNames();
  num_joints_ = internal_joint_state_.name.size();
  internal_joint_state_.position.resize(num_joints_);
  internal_joint_state_.velocity.resize(num_joints_);

  // A map for the indices of incoming joint commands
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    joint_state_name_map_[internal_joint_state_.name[i]] = i;
  }

  // Low-pass filters for the joint positions
  for (size_t i = 0; i < num_joints_; ++i)
  {
    position_filters_.emplace_back(parameters_.low_pass_filter_coeff);
  }

  // A matrix of all zeros is used to check whether matrices have been initialized
  Eigen::Matrix3d empty_matrix;
  empty_matrix.setZero();
  tf_moveit_to_ee_frame_ = empty_matrix;
  tf_moveit_to_robot_cmd_frame_ = empty_matrix;
}

ServoCalcs::~ServoCalcs()
{
  stop();
}

void ServoCalcs::start()
{
  // Stop the thread if we are currently running
  stop();

  // We will update last_sent_command_ every time we start servo
  auto initial_joint_trajectory = moveit::util::make_shared_from_pool<trajectory_msgs::JointTrajectory>();

  // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
  initial_joint_trajectory->header.stamp = ros::Time(0);
  initial_joint_trajectory->header.frame_id = parameters_.planning_frame;
  initial_joint_trajectory->joint_names = internal_joint_state_.name;
  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(parameters_.publish_period);

  if (parameters_.publish_joint_positions)
    planning_scene_monitor_->getStateMonitor()->getCurrentState()->copyJointGroupPositions(joint_model_group_,
                                                                                           point.positions);
  if (parameters_.publish_joint_velocities)
  {
    std::vector<double> velocity(num_joints_);
    point.velocities = velocity;
  }
  if (parameters_.publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    point.accelerations.resize(num_joints_);
  }
  initial_joint_trajectory->points.push_back(point);
  last_sent_command_ = initial_joint_trajectory;

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_.ee_frame_name);
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(parameters_.robot_link_command_frame);

  stop_requested_ = false;
  thread_ = std::thread([this] { mainCalcLoop(); });
  new_input_cmd_ = false;
}

void ServoCalcs::stop()
{
  // Request stop
  stop_requested_ = true;

  // Notify condition variable in case the thread is blocked on it
  {
    // scope so the mutex is unlocked after so the thread can continue
    // and therefore be joinable
    const std::lock_guard<std::mutex> lock(input_mutex_);
    new_input_cmd_ = false;
    input_cv_.notify_all();
  }

  // Join the thread
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void ServoCalcs::mainCalcLoop()
{
  ros::Rate rate(1.0 / parameters_.publish_period);

  while (ros::ok() && !stop_requested_)
  {
    // lock the input state mutex
    std::unique_lock<std::mutex> input_lock(input_mutex_);

    // low latency mode -- begin calculations as soon as a new command is received.
    if (parameters_.low_latency_mode)
    {
      input_cv_.wait(input_lock, [this] { return (new_input_cmd_ || stop_requested_); });
    }

    // reset new_input_cmd_ flag
    new_input_cmd_ = false;

    // run servo calcs
    const auto start_time = ros::Time::now();
    calculateSingleIteration();
    const auto run_duration = ros::Time::now() - start_time;

    // Log warning when the run duration was longer than the period
    if (run_duration.toSec() > parameters_.publish_period)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                     "run_duration: " << run_duration.toSec() << " (" << parameters_.publish_period
                                                      << ")");
    }

    // normal mode, unlock input mutex and wait for the period of the loop
    if (!parameters_.low_latency_mode)
    {
      input_lock.unlock();
      rate.sleep();
    }
  }
}

void ServoCalcs::calculateSingleIteration()
{
  // Publish status each loop iteration
  auto status_msg = moveit::util::make_shared_from_pool<std_msgs::Int8>();
  status_msg->data = static_cast<int8_t>(status_);
  status_pub_.publish(status_msg);

  // Always update the joints and end-effector transform for 2 reasons:
  // 1) in case the getCommandFrameTransform() method is being used
  // 2) so the low-pass filters are up to date and don't cause a jump
  updateJoints();

  // Update from latest state
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();

  if (latest_twist_stamped_)
    twist_stamped_cmd_ = *latest_twist_stamped_;
  if (latest_joint_cmd_)
    joint_servo_cmd_ = *latest_joint_cmd_;

  // Check for stale cmds
  twist_command_is_stale_ =
      ((ros::Time::now() - latest_twist_command_stamp_) >= ros::Duration(parameters_.incoming_command_timeout));
  joint_command_is_stale_ =
      ((ros::Time::now() - latest_joint_command_stamp_) >= ros::Duration(parameters_.incoming_command_timeout));

  have_nonzero_twist_stamped_ = latest_nonzero_twist_stamped_;
  have_nonzero_joint_command_ = latest_nonzero_joint_cmd_;

  // Get the transform from MoveIt planning frame to servoing command frame
  // Calculate this transform to ensure it is available via C++ API
  // We solve (planning_frame -> base -> robot_link_command_frame)
  // by computing (base->planning_frame)^-1 * (base->robot_link_command_frame)
  tf_moveit_to_robot_cmd_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                                  current_state_->getGlobalLinkTransform(parameters_.robot_link_command_frame);

  // Calculate the transform from MoveIt planning frame to End Effector frame
  // Calculate this transform to ensure it is available via C++ API
  tf_moveit_to_ee_frame_ = current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
                           current_state_->getGlobalLinkTransform(parameters_.ee_frame_name);

  have_nonzero_command_ = have_nonzero_twist_stamped_ || have_nonzero_joint_command_;

  // Don't end this function without updating the filters
  updated_filters_ = false;

  // If paused or while waiting for initial servo commands, just keep the low-pass filters up to date with current
  // joints so a jump doesn't occur when restarting
  if (wait_for_servo_commands_ || paused_)
  {
    resetLowPassFilters(original_joint_state_);

    // Check if there are any new commands with valid timestamp
    wait_for_servo_commands_ =
        twist_stamped_cmd_.header.stamp == ros::Time(0.) && joint_servo_cmd_.header.stamp == ros::Time(0.);

    // Early exit
    return;
  }

  // If not waiting for initial command, and not paused.
  // Do servoing calculations only if the robot should move, for efficiency
  // Create new outgoing joint trajectory command message
  auto joint_trajectory = moveit::util::make_shared_from_pool<trajectory_msgs::JointTrajectory>();

  // Prioritize cartesian servoing above joint servoing
  // Only run commands if not stale and nonzero
  if (have_nonzero_twist_stamped_ && !twist_command_is_stale_)
  {
    if (!cartesianServoCalcs(twist_stamped_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else if (have_nonzero_joint_command_ && !joint_command_is_stale_)
  {
    if (!jointServoCalcs(joint_servo_cmd_, *joint_trajectory))
    {
      resetLowPassFilters(original_joint_state_);
      return;
    }
  }
  else
  {
    // Joint trajectory is not populated with anything, so set it to the last positions and 0 velocity
    *joint_trajectory = *last_sent_command_;
    for (auto& point : joint_trajectory->points)
    {
      point.velocities.assign(point.velocities.size(), 0);
    }
  }

  // Print a warning to the user if both are stale
  if (twist_command_is_stale_ && joint_command_is_stale_)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(10, LOGNAME,
                                   "Stale command. "
                                   "Try a larger 'incoming_command_timeout' parameter?");
  }

  // If we should halt
  if (!have_nonzero_command_)
  {
    suddenHalt(*joint_trajectory);
    have_nonzero_twist_stamped_ = false;
    have_nonzero_joint_command_ = false;
  }

  // Skip the servoing publication if all inputs have been zero for several cycles in a row.
  // num_outgoing_halt_msgs_to_publish == 0 signifies that we should keep republishing forever.
  if (!have_nonzero_command_ && (parameters_.num_outgoing_halt_msgs_to_publish != 0) &&
      (zero_velocity_count_ > parameters_.num_outgoing_halt_msgs_to_publish))
  {
    ok_to_publish_ = false;
    ROS_DEBUG_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "All-zero command. Doing nothing.");
  }
  else
  {
    ok_to_publish_ = true;
  }

  // Store last zero-velocity message flag to prevent superfluous warnings.
  // Cartesian and joint commands must both be zero.
  if (!have_nonzero_command_)
  {
    // Avoid overflow
    if (zero_velocity_count_ < std::numeric_limits<int>::max())
      ++zero_velocity_count_;
  }
  else
  {
    zero_velocity_count_ = 0;
  }

  if (ok_to_publish_ && !paused_)
  {
    // Put the outgoing msg in the right format
    // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
    if (parameters_.command_out_type == "trajectory_msgs/JointTrajectory")
    {
      // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
      // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
      joint_trajectory->header.stamp = ros::Time(0);
      outgoing_cmd_pub_.publish(joint_trajectory);
    }
    else if (parameters_.command_out_type == "std_msgs/Float64MultiArray")
    {
      auto joints = moveit::util::make_shared_from_pool<std_msgs::Float64MultiArray>();
      if (parameters_.publish_joint_positions && !joint_trajectory->points.empty())
        joints->data = joint_trajectory->points[0].positions;
      else if (parameters_.publish_joint_velocities && !joint_trajectory->points.empty())
        joints->data = joint_trajectory->points[0].velocities;
      outgoing_cmd_pub_.publish(joints);
    }

    last_sent_command_ = joint_trajectory;
  }

  // Update the filters if we haven't yet
  if (!updated_filters_)
    resetLowPassFilters(original_joint_state_);
}
// Perform the servoing calculations
bool ServoCalcs::cartesianServoCalcs(geometry_msgs::TwistStamped& cmd,
                                     trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // Check for nan's in the incoming command
  if (std::isnan(cmd.twist.linear.x) || std::isnan(cmd.twist.linear.y) || std::isnan(cmd.twist.linear.z) ||
      std::isnan(cmd.twist.angular.x) || std::isnan(cmd.twist.angular.y) || std::isnan(cmd.twist.angular.z))
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                   "nan in incoming command. Skipping this datapoint.");
    return false;
  }

  // If incoming commands should be in the range [-1:1], check for |delta|>1
  if (parameters_.command_in_type == "unitless")
  {
    if ((fabs(cmd.twist.linear.x) > 1) || (fabs(cmd.twist.linear.y) > 1) || (fabs(cmd.twist.linear.z) > 1) ||
        (fabs(cmd.twist.angular.x) > 1) || (fabs(cmd.twist.angular.y) > 1) || (fabs(cmd.twist.angular.z) > 1))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                     "Component of incoming command is >1. Skipping this datapoint.");
      return false;
    }
  }

  // Set uncontrolled dimensions to 0 in command frame
  if (!control_dimensions_[0])
    cmd.twist.linear.x = 0;
  if (!control_dimensions_[1])
    cmd.twist.linear.y = 0;
  if (!control_dimensions_[2])
    cmd.twist.linear.z = 0;
  if (!control_dimensions_[3])
    cmd.twist.angular.x = 0;
  if (!control_dimensions_[4])
    cmd.twist.angular.y = 0;
  if (!control_dimensions_[5])
    cmd.twist.angular.z = 0;

  // Transform the command to the MoveGroup planning frame
  if (cmd.header.frame_id != parameters_.planning_frame)
  {
    Eigen::Vector3d translation_vector(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);
    Eigen::Vector3d angular_vector(cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z);

    // If the incoming frame is empty or is the command frame, we use the previously calculated tf
    if (cmd.header.frame_id.empty() || cmd.header.frame_id == parameters_.robot_link_command_frame)
    {
      translation_vector = tf_moveit_to_robot_cmd_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_robot_cmd_frame_.linear() * angular_vector;
    }
    else if (cmd.header.frame_id == parameters_.ee_frame_name)
    {
      // If the frame is the EE frame, we already have that transform as well
      translation_vector = tf_moveit_to_ee_frame_.linear() * translation_vector;
      angular_vector = tf_moveit_to_ee_frame_.linear() * angular_vector;
    }
    else
    {
      // We solve (planning_frame -> base -> cmd.header.frame_id)
      // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
      const auto tf_moveit_to_incoming_cmd_frame =
          current_state_->getGlobalLinkTransform(parameters_.planning_frame).inverse() *
          current_state_->getGlobalLinkTransform(cmd.header.frame_id);

      translation_vector = tf_moveit_to_incoming_cmd_frame.linear() * translation_vector;
      angular_vector = tf_moveit_to_incoming_cmd_frame.linear() * angular_vector;
    }

    // Put these components back into a TwistStamped
    cmd.header.frame_id = parameters_.planning_frame;
    cmd.twist.linear.x = translation_vector(0);
    cmd.twist.linear.y = translation_vector(1);
    cmd.twist.linear.z = translation_vector(2);
    cmd.twist.angular.x = angular_vector(0);
    cmd.twist.angular.y = angular_vector(1);
    cmd.twist.angular.z = angular_vector(2);
  }

  Eigen::VectorXd delta_x = scaleCartesianCommand(cmd);

  // Convert from cartesian commands to joint commands
  Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);

  // May allow some dimensions to drift, based on drift_dimensions
  // i.e. take advantage of task redundancy.
  // Remove the Jacobian rows corresponding to True in the vector drift_dimensions
  // Work backwards through the 6-vector so indices don't get out of order
  for (auto dimension = jacobian.rows() - 1; dimension >= 0; --dimension)
  {
    if (drift_dimensions_[dimension] && jacobian.rows() > 1)
    {
      removeDimension(jacobian, delta_x, dimension);
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd =
      Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
  Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

  delta_theta_ = pseudo_inverse * delta_x;

  enforceVelLimits(delta_theta_);

  // If close to a collision or a singularity, decelerate
  applyVelocityScaling(delta_theta_, velocityScalingFactorForSingularity(delta_x, svd, pseudo_inverse));

  prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;

  return convertDeltasToOutgoingCmd(joint_trajectory);
}

bool ServoCalcs::jointServoCalcs(const control_msgs::JointJog& cmd, trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // Check for nan's
  for (double velocity : cmd.velocities)
  {
    if (std::isnan(velocity))
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                     "nan in incoming command. Skipping this datapoint.");
      return false;
    }
  }

  // Apply user-defined scaling
  delta_theta_ = scaleJointCommand(cmd);

  enforceVelLimits(delta_theta_);

  // If close to a collision, decelerate
  applyVelocityScaling(delta_theta_, 1.0 /* scaling for singularities -- ignore for joint motions */);

  prev_joint_velocity_ = delta_theta_ / parameters_.publish_period;

  return convertDeltasToOutgoingCmd(joint_trajectory);
}

bool ServoCalcs::convertDeltasToOutgoingCmd(trajectory_msgs::JointTrajectory& joint_trajectory)
{
  internal_joint_state_ = original_joint_state_;
  if (!addJointIncrements(internal_joint_state_, delta_theta_))
    return false;

  lowPassFilterPositions(internal_joint_state_);

  // Calculate joint velocities here so that positions are filtered and SRDF bounds still get checked
  calculateJointVelocities(internal_joint_state_, delta_theta_);

  composeJointTrajMessage(internal_joint_state_, joint_trajectory);

  if (!enforcePositionLimits(internal_joint_state_))
  {
    suddenHalt(joint_trajectory);
    status_ = StatusCode::JOINT_BOUND;
  }

  // done with calculations
  if (parameters_.use_gazebo)
  {
    insertRedundantPointsIntoTrajectory(joint_trajectory, gazebo_redundant_message_count_);
  }

  return true;
}

// Spam several redundant points into the trajectory. The first few may be skipped if the
// time stamp is in the past when it reaches the client. Needed for gazebo simulation.
void ServoCalcs::insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory, int count) const
{
  joint_trajectory.points.resize(count);
  auto point = joint_trajectory.points[0];
  // Start from 2nd point (i = 1) because we already have the first point.
  // The timestamps are shifted up one period since first point is at 1 * publish_period, not 0.
  for (int i = 1; i < count; ++i)
  {
    point.time_from_start = ros::Duration((i + 1) * parameters_.publish_period);
    joint_trajectory.points[i] = point;
  }
}

void ServoCalcs::lowPassFilterPositions(sensor_msgs::JointState& joint_state)
{
  for (size_t i = 0; i < position_filters_.size(); ++i)
  {
    joint_state.position[i] = position_filters_[i].filter(joint_state.position[i]);
  }

  updated_filters_ = true;
}

void ServoCalcs::resetLowPassFilters(const sensor_msgs::JointState& joint_state)
{
  for (std::size_t i = 0; i < position_filters_.size(); ++i)
  {
    position_filters_[i].reset(joint_state.position[i]);
  }

  updated_filters_ = true;
}

void ServoCalcs::calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta)
{
  for (int i = 0; i < delta_theta.size(); ++i)
  {
    joint_state.velocity[i] = delta_theta[i] / parameters_.publish_period;
  }
}

void ServoCalcs::composeJointTrajMessage(const sensor_msgs::JointState& joint_state,
                                         trajectory_msgs::JointTrajectory& joint_trajectory) const
{
  // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement
  joint_trajectory.header.stamp = ros::Time(0);
  joint_trajectory.header.frame_id = parameters_.planning_frame;
  joint_trajectory.joint_names = joint_state.name;

  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(parameters_.publish_period);
  if (parameters_.publish_joint_positions)
    point.positions = joint_state.position;
  if (parameters_.publish_joint_velocities)
    point.velocities = joint_state.velocity;
  if (parameters_.publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    std::vector<double> acceleration(num_joints_);
    point.accelerations = acceleration;
  }
  joint_trajectory.points.push_back(point);
}

// Apply velocity scaling for proximity of collisions and singularities.
void ServoCalcs::applyVelocityScaling(Eigen::ArrayXd& delta_theta, double singularity_scale)
{
  double collision_scale = collision_velocity_scale_;

  if (collision_scale > 0 && collision_scale < 1)
  {
    status_ = StatusCode::DECELERATE_FOR_COLLISION;
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status_));
  }
  else if (collision_scale == 0)
  {
    status_ = StatusCode::HALT_FOR_COLLISION;
  }

  delta_theta = collision_scale * singularity_scale * delta_theta;

  if (status_ == StatusCode::HALT_FOR_COLLISION)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(3, LOGNAME, "Halting for collision!");
    delta_theta_.setZero();
  }
}

// Possibly calculate a velocity scaling factor, due to proximity of singularity and direction of motion
double ServoCalcs::velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                                       const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                                       const Eigen::MatrixXd& pseudo_inverse)
{
  double velocity_scale = 1;
  std::size_t num_dimensions = commanded_velocity.size();

  // Find the direction away from nearest singularity.
  // The last column of U from the SVD of the Jacobian points directly toward or away from the singularity.
  // The sign can flip at any time, so we have to do some extra checking.
  // Look ahead to see if the Jacobian's condition will decrease.
  Eigen::VectorXd vector_toward_singularity = svd.matrixU().col(num_dimensions - 1);

  double ini_condition = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

  // This singular vector tends to flip direction unpredictably. See R. Bro,
  // "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  // Look ahead to see if the Jacobian's condition will decrease in this
  // direction. Start with a scaled version of the singular vector
  Eigen::VectorXd delta_x(num_dimensions);
  double scale = 100;
  delta_x = vector_toward_singularity / scale;

  // Calculate a small change in joints
  Eigen::VectorXd new_theta;
  current_state_->copyJointGroupPositions(joint_model_group_, new_theta);
  new_theta += pseudo_inverse * delta_x;
  current_state_->setJointGroupPositions(joint_model_group_, new_theta);
  Eigen::MatrixXd new_jacobian = current_state_->getJacobian(joint_model_group_);

  Eigen::JacobiSVD<Eigen::MatrixXd> new_svd(new_jacobian);
  double new_condition = new_svd.singularValues()(0) / new_svd.singularValues()(new_svd.singularValues().size() - 1);
  // If new_condition < ini_condition, the singular vector does point towards a
  // singularity. Otherwise, flip its direction.
  if (ini_condition >= new_condition)
  {
    vector_toward_singularity *= -1;
  }

  // If this dot product is positive, we're moving toward singularity ==> decelerate
  double dot = vector_toward_singularity.dot(commanded_velocity);
  if (dot > 0)
  {
    // Ramp velocity down linearly when the Jacobian condition is between lower_singularity_threshold and
    // hard_stop_singularity_threshold, and we're moving towards the singularity
    if ((ini_condition > parameters_.lower_singularity_threshold) &&
        (ini_condition < parameters_.hard_stop_singularity_threshold))
    {
      velocity_scale = 1. - (ini_condition - parameters_.lower_singularity_threshold) /
                                (parameters_.hard_stop_singularity_threshold - parameters_.lower_singularity_threshold);
      status_ = StatusCode::DECELERATE_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status_));
    }

    // Very close to singularity, so halt.
    else if (ini_condition > parameters_.hard_stop_singularity_threshold)
    {
      velocity_scale = 0;
      status_ = StatusCode::HALT_FOR_SINGULARITY;
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, SERVO_STATUS_CODE_MAP.at(status_));
    }
  }

  return velocity_scale;
}

void ServoCalcs::enforceVelLimits(Eigen::ArrayXd& delta_theta)
{
  // Convert to joint angle velocities for checking and applying joint specific velocity limits.
  Eigen::ArrayXd velocity = delta_theta / parameters_.publish_period;

  std::size_t joint_delta_index{ 0 };
  double velocity_scaling_factor{ 1.0 };
  for (const moveit::core::JointModel* joint : joint_model_group_->getActiveJointModels())
  {
    const auto& bounds = joint->getVariableBounds(joint->getName());
    if (bounds.velocity_bounded_ && velocity(joint_delta_index) != 0.0)
    {
      const double unbounded_velocity = velocity(joint_delta_index);
      // Clamp each joint velocity to a joint specific [min_velocity, max_velocity] range.
      const auto bounded_velocity = std::min(std::max(unbounded_velocity, bounds.min_velocity_), bounds.max_velocity_);
      velocity_scaling_factor = std::min(velocity_scaling_factor, bounded_velocity / unbounded_velocity);
    }
    ++joint_delta_index;
  }

  // Convert back to joint angle increments.
  delta_theta = velocity_scaling_factor * velocity * parameters_.publish_period;
}

bool ServoCalcs::enforcePositionLimits(sensor_msgs::JointState& joint_state)
{
  bool halting = false;

  for (auto joint : joint_model_group_->getActiveJointModels())
  {
    // Halt if we're past a joint margin and joint velocity is moving even farther past
    double joint_angle = 0;
    for (std::size_t c = 0; c < original_joint_state_.name.size(); ++c)
    {
      if (original_joint_state_.name[c] == joint->getName())
      {
        joint_angle = original_joint_state_.position.at(c);
        break;
      }
    }
    if (!current_state_->satisfiesPositionBounds(joint, -parameters_.joint_limit_margin))
    {
      const std::vector<moveit_msgs::JointLimits>& limits = joint->getVariableBoundsMsg();

      // Joint limits are not defined for some joints. Skip them.
      if (!limits.empty())
      {
        // Check if pending velocity command is moving in the right direction
        auto joint_itr = std::find(joint_state.name.begin(), joint_state.name.end(), joint->getName());
        auto joint_idx = std::distance(joint_state.name.begin(), joint_itr);

        if ((joint_state.velocity.at(joint_idx) < 0 &&
             (joint_angle < (limits[0].min_position + parameters_.joint_limit_margin))) ||
            (joint_state.velocity.at(joint_idx) > 0 &&
             (joint_angle > (limits[0].max_position - parameters_.joint_limit_margin))))
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                         ros::this_node::getName() << " " << joint->getName()
                                                                   << " close to a "
                                                                      " position limit. Halting.");
          halting = true;
        }
      }
    }
  }
  return !halting;
}

// Suddenly halt for a joint limit or other critical issue.
// Is handled differently for position vs. velocity control.
void ServoCalcs::suddenHalt(trajectory_msgs::JointTrajectory& joint_trajectory)
{
  // Prepare the joint trajectory message to stop the robot
  joint_trajectory.points.clear();
  joint_trajectory.points.emplace_back();
  trajectory_msgs::JointTrajectoryPoint& point = joint_trajectory.points.front();

  // When sending out trajectory_msgs/JointTrajectory type messages, the "trajectory" is just a single point.
  // That point cannot have the same timestamp as the start of trajectory execution since that would mean the
  // arm has to reach the first trajectory point the moment execution begins. To prevent errors about points
  // being 0 seconds in the past, the smallest supported timestep is added as time from start to the trajectory point.
  point.time_from_start.fromNSec(1);

  if (parameters_.publish_joint_positions)
    point.positions.resize(num_joints_);
  if (parameters_.publish_joint_velocities)
    point.velocities.resize(num_joints_);

  // Assert the following loop is safe to execute
  assert(original_joint_state_.position.size() >= num_joints_);

  // Set the positions and velocities vectors
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    // For position-controlled robots, can reset the joints to a known, good state
    if (parameters_.publish_joint_positions)
      point.positions[i] = original_joint_state_.position[i];

    // For velocity-controlled robots, stop
    if (parameters_.publish_joint_velocities)
      point.velocities[i] = 0;
  }
}

// Parse the incoming joint msg for the joints of our MoveGroup
void ServoCalcs::updateJoints()
{
  // Get the latest joint group positions
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->copyJointGroupPositions(joint_model_group_, internal_joint_state_.position);
  current_state_->copyJointGroupVelocities(joint_model_group_, internal_joint_state_.velocity);

  // Cache the original joints in case they need to be reset
  original_joint_state_ = internal_joint_state_;

  // Calculate worst case joint stop time, for collision checking
  std::string joint_name = "";
  moveit::core::JointModel::Bounds kinematic_bounds;
  double accel_limit = 0;
  double joint_velocity = 0;
  double worst_case_stop_time = 0;
  for (size_t jt_state_idx = 0; jt_state_idx < internal_joint_state_.velocity.size(); ++jt_state_idx)
  {
    joint_name = internal_joint_state_.name[jt_state_idx];

    // Get acceleration limit for this joint
    for (auto joint_model : joint_model_group_->getActiveJointModels())
    {
      if (joint_model->getName() == joint_name)
      {
        kinematic_bounds = joint_model->getVariableBounds();
        // Some joints do not have acceleration limits
        if (kinematic_bounds[0].acceleration_bounded_)
        {
          // Be conservative when calculating overall acceleration limit from min and max limits
          accel_limit =
              std::min(fabs(kinematic_bounds[0].min_acceleration_), fabs(kinematic_bounds[0].max_acceleration_));
        }
        else
        {
          ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                         "An acceleration limit is not defined for this joint; minimum stop distance "
                                         "should not be used for collision checking");
        }
        break;
      }
    }

    // Get the current joint velocity
    joint_velocity = internal_joint_state_.velocity[jt_state_idx];

    // Calculate worst case stop time
    worst_case_stop_time = std::max(worst_case_stop_time, fabs(joint_velocity / accel_limit));
  }

  // publish message
  {
    auto msg = moveit::util::make_shared_from_pool<std_msgs::Float64>();
    msg->data = worst_case_stop_time;
    worst_case_stop_time_pub_.publish(msg);
  }
}

// Scale the incoming servo command
Eigen::VectorXd ServoCalcs::scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const
{
  Eigen::VectorXd result(6);

  // Apply user-defined scaling if inputs are unitless [-1:1]
  if (parameters_.command_in_type == "unitless")
  {
    result[0] = parameters_.linear_scale * parameters_.publish_period * command.twist.linear.x;
    result[1] = parameters_.linear_scale * parameters_.publish_period * command.twist.linear.y;
    result[2] = parameters_.linear_scale * parameters_.publish_period * command.twist.linear.z;
    result[3] = parameters_.rotational_scale * parameters_.publish_period * command.twist.angular.x;
    result[4] = parameters_.rotational_scale * parameters_.publish_period * command.twist.angular.y;
    result[5] = parameters_.rotational_scale * parameters_.publish_period * command.twist.angular.z;
  }
  // Otherwise, commands are in m/s and rad/s
  else if (parameters_.command_in_type == "speed_units")
  {
    result[0] = command.twist.linear.x * parameters_.publish_period;
    result[1] = command.twist.linear.y * parameters_.publish_period;
    result[2] = command.twist.linear.z * parameters_.publish_period;
    result[3] = command.twist.angular.x * parameters_.publish_period;
    result[4] = command.twist.angular.y * parameters_.publish_period;
    result[5] = command.twist.angular.z * parameters_.publish_period;
  }
  else
    ROS_ERROR_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "Unexpected command_in_type");

  return result;
}

Eigen::VectorXd ServoCalcs::scaleJointCommand(const control_msgs::JointJog& command) const
{
  Eigen::VectorXd result(num_joints_);
  result.setZero();

  std::size_t c;
  for (std::size_t m = 0; m < command.joint_names.size(); ++m)
  {
    try
    {
      c = joint_state_name_map_.at(command.joint_names[m]);
    }
    catch (const std::out_of_range& e)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "Ignoring joint " << command.joint_names[m]);
      continue;
    }
    // Apply user-defined scaling if inputs are unitless [-1:1]
    if (parameters_.command_in_type == "unitless")
      result[c] = command.velocities[m] * parameters_.joint_scale * parameters_.publish_period;
    // Otherwise, commands are in m/s and rad/s
    else if (parameters_.command_in_type == "speed_units")
      result[c] = command.velocities[m] * parameters_.publish_period;
    else
      ROS_ERROR_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "Unexpected command_in_type, check yaml file.");
  }

  return result;
}

// Add the deltas to each joint
bool ServoCalcs::addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const
{
  for (std::size_t i = 0, size = static_cast<std::size_t>(increments.size()); i < size; ++i)
  {
    try
    {
      output.position[i] += increments[i];
    }
    catch (const std::out_of_range& e)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                      ros::this_node::getName() << " Lengths of output and "
                                                                   "increments do not match.");
      return false;
    }
  }

  return true;
}

void ServoCalcs::removeDimension(Eigen::MatrixXd& jacobian, Eigen::VectorXd& delta_x, unsigned int row_to_remove)
{
  unsigned int num_rows = jacobian.rows() - 1;
  unsigned int num_cols = jacobian.cols();

  if (row_to_remove < num_rows)
  {
    jacobian.block(row_to_remove, 0, num_rows - row_to_remove, num_cols) =
        jacobian.block(row_to_remove + 1, 0, num_rows - row_to_remove, num_cols);
    delta_x.segment(row_to_remove, num_rows - row_to_remove) =
        delta_x.segment(row_to_remove + 1, num_rows - row_to_remove);
  }
  jacobian.conservativeResize(num_rows, num_cols);
  delta_x.conservativeResize(num_rows);
}

bool ServoCalcs::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  transform = tf_moveit_to_robot_cmd_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getCommandFrameTransform(geometry_msgs::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_robot_cmd_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform = convertIsometryToTransform(tf_moveit_to_robot_cmd_frame_, parameters_.planning_frame,
                                         parameters_.robot_link_command_frame);
  return true;
}

bool ServoCalcs::getEEFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  transform = tf_moveit_to_ee_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

bool ServoCalcs::getEEFrameTransform(geometry_msgs::TransformStamped& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  // All zeros means the transform wasn't initialized, so return false
  if (tf_moveit_to_ee_frame_.matrix().isZero(0))
  {
    return false;
  }

  transform = convertIsometryToTransform(tf_moveit_to_ee_frame_, parameters_.planning_frame, parameters_.ee_frame_name);
  return true;
}

void ServoCalcs::twistStampedCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  latest_twist_stamped_ = msg;
  latest_nonzero_twist_stamped_ = isNonZero(*latest_twist_stamped_);

  if (msg->header.stamp != ros::Time(0.))
    latest_twist_command_stamp_ = msg->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::jointCmdCB(const control_msgs::JointJogConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  latest_joint_cmd_ = msg;
  latest_nonzero_joint_cmd_ = isNonZero(*latest_joint_cmd_);

  if (msg->header.stamp != ros::Time(0.))
    latest_joint_command_stamp_ = msg->header.stamp;

  // notify that we have a new input
  new_input_cmd_ = true;
  input_cv_.notify_all();
}

void ServoCalcs::collisionVelocityScaleCB(const std_msgs::Float64ConstPtr& msg)
{
  collision_velocity_scale_ = msg->data;
}

bool ServoCalcs::changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                                       moveit_msgs::ChangeDriftDimensions::Response& res)
{
  drift_dimensions_[0] = req.drift_x_translation;
  drift_dimensions_[1] = req.drift_y_translation;
  drift_dimensions_[2] = req.drift_z_translation;
  drift_dimensions_[3] = req.drift_x_rotation;
  drift_dimensions_[4] = req.drift_y_rotation;
  drift_dimensions_[5] = req.drift_z_rotation;

  res.success = true;
  return true;
}

bool ServoCalcs::changeControlDimensions(moveit_msgs::ChangeControlDimensions::Request& req,
                                         moveit_msgs::ChangeControlDimensions::Response& res)
{
  control_dimensions_[0] = req.control_x_translation;
  control_dimensions_[1] = req.control_y_translation;
  control_dimensions_[2] = req.control_z_translation;
  control_dimensions_[3] = req.control_x_rotation;
  control_dimensions_[4] = req.control_y_rotation;
  control_dimensions_[5] = req.control_z_rotation;

  res.success = true;
  return true;
}

bool ServoCalcs::resetServoStatus(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
{
  status_ = StatusCode::NO_WARNING;
  return true;
}

void ServoCalcs::setPaused(bool paused)
{
  paused_ = paused;
}

void ServoCalcs::changeRobotLinkCommandFrame(const std::string& new_command_frame)
{
  parameters_.robot_link_command_frame = new_command_frame;
}

}  // namespace moveit_servo
