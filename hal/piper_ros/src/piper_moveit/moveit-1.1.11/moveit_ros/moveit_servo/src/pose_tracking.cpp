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

#include "moveit_servo/pose_tracking.h"

namespace
{
constexpr char LOGNAME[] = "pose_tracking";
constexpr double DEFAULT_LOOP_RATE = 100;  // Hz
constexpr double ROS_STARTUP_WAIT = 10;    // sec
}  // namespace

namespace moveit_servo
{
PoseTracking::PoseTracking(const ros::NodeHandle& nh,
                           const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh)
  , planning_scene_monitor_(planning_scene_monitor)
  , loop_rate_(DEFAULT_LOOP_RATE)
  , transform_listener_(transform_buffer_)
  , stop_requested_(false)
  , angular_error_(boost::none)
{
  readROSParams();

  robot_model_ = planning_scene_monitor_->getRobotModel();
  joint_model_group_ = robot_model_->getJointModelGroup(move_group_name_);

  // Initialize PID controllers
  initializePID(x_pid_config_, cartesian_position_pids_);
  initializePID(y_pid_config_, cartesian_position_pids_);
  initializePID(z_pid_config_, cartesian_position_pids_);
  initializePID(angular_pid_config_, cartesian_orientation_pids_);

  // Use the C++ interface that Servo provides
  servo_ = std::make_unique<moveit_servo::Servo>(nh_, planning_scene_monitor_);
  servo_->start();

  // Connect to Servo ROS interfaces
  target_pose_sub_ =
      nh_.subscribe<geometry_msgs::PoseStamped>("target_pose", 1, &PoseTracking::targetPoseCallback, this);

  // Publish outgoing twist commands to the Servo object
  twist_stamped_pub_ =
      nh_.advertise<geometry_msgs::TwistStamped>(servo_->getParameters().cartesian_command_in_topic, 1);
}

PoseTrackingStatusCode PoseTracking::moveToPose(const Eigen::Vector3d& positional_tolerance,
                                                const double angular_tolerance, const double target_pose_timeout)
{
  // Reset stop requested flag before starting motions
  stop_requested_ = false;
  // Wait a bit for a target pose message to arrive.
  // The target pose may get updated by new messages as the robot moves (in a callback function).
  const ros::Time start_time = ros::Time::now();
  while ((!haveRecentTargetPose(target_pose_timeout) || !haveRecentEndEffectorPose(target_pose_timeout)) &&
         ((ros::Time::now() - start_time).toSec() < target_pose_timeout))
  {
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = ros::Time::now();
    }
    ros::Duration(0.001).sleep();
  }

  if (!haveRecentTargetPose(target_pose_timeout))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "The target pose was not updated recently. Aborting.");
    return PoseTrackingStatusCode::NO_RECENT_TARGET_POSE;
  }

  // Continue sending PID controller output to Servo until one of the following conditions is met:
  // - Goal tolerance is satisfied
  // - Target pose becomes outdated
  // - Command frame transform becomes outdated
  // - Another thread requested a stop
  while (ros::ok() && !satisfiesPoseTolerance(positional_tolerance, angular_tolerance))
  {
    // Attempt to update robot pose
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = ros::Time::now();
    }

    // Check that end-effector pose (command frame transform) is recent enough.
    if (!haveRecentEndEffectorPose(target_pose_timeout))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "The end effector pose was not updated in time. Aborting.");
      doPostMotionReset();
      return PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE;
    }

    if (stop_requested_)
    {
      ROS_INFO_STREAM_NAMED(LOGNAME, "Halting servo motion, a stop was requested.");
      doPostMotionReset();
      return PoseTrackingStatusCode::STOP_REQUESTED;
    }

    // Compute servo command from PID controller output and send it to the Servo object, for execution
    twist_stamped_pub_.publish(calculateTwistCommand());

    if (!loop_rate_.sleep())
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(1, LOGNAME, "Target control rate was missed");
    }
  }

  doPostMotionReset();
  return PoseTrackingStatusCode::SUCCESS;
}

void PoseTracking::readROSParams()
{
  // Optional parameter sub-namespace specified in the launch file. All other parameters will be read from this namespace.
  std::string parameter_ns;
  ros::param::get("~parameter_ns", parameter_ns);

  // If parameters have been loaded into sub-namespace within the node namespace, append the parameter namespace
  // to load the parameters correctly.
  ros::NodeHandle nh = parameter_ns.empty() ? nh_ : ros::NodeHandle(nh_, parameter_ns);

  // Wait for ROS parameters to load
  ros::Time begin = ros::Time::now();
  while (ros::ok() && !nh.hasParam("planning_frame") && ((ros::Time::now() - begin).toSec() < ROS_STARTUP_WAIT))
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Waiting for parameter: "
                                       << "planning_frame");
    ros::Duration(0.1).sleep();
  }

  std::size_t error = 0;

  error += !rosparam_shortcuts::get(LOGNAME, nh, "planning_frame", planning_frame_);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "move_group_name", move_group_name_);
  if (!planning_scene_monitor_->getRobotModel()->hasJointModelGroup(move_group_name_))
  {
    ++error;
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unable to find the specified joint model group: " << move_group_name_);
  }

  double publish_period;
  error += !rosparam_shortcuts::get(LOGNAME, nh, "publish_period", publish_period);
  loop_rate_ = ros::Rate(1 / publish_period);

  x_pid_config_.dt = publish_period;
  y_pid_config_.dt = publish_period;
  z_pid_config_.dt = publish_period;
  angular_pid_config_.dt = publish_period;

  double windup_limit;
  error += !rosparam_shortcuts::get(LOGNAME, nh, "windup_limit", windup_limit);
  x_pid_config_.windup_limit = windup_limit;
  y_pid_config_.windup_limit = windup_limit;
  z_pid_config_.windup_limit = windup_limit;
  angular_pid_config_.windup_limit = windup_limit;

  error += !rosparam_shortcuts::get(LOGNAME, nh, "x_proportional_gain", x_pid_config_.k_p);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "y_proportional_gain", y_pid_config_.k_p);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "z_proportional_gain", z_pid_config_.k_p);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "x_integral_gain", x_pid_config_.k_i);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "y_integral_gain", y_pid_config_.k_i);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "z_integral_gain", z_pid_config_.k_i);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "x_derivative_gain", x_pid_config_.k_d);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "y_derivative_gain", y_pid_config_.k_d);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "z_derivative_gain", z_pid_config_.k_d);

  error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_proportional_gain", angular_pid_config_.k_p);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_integral_gain", angular_pid_config_.k_i);
  error += !rosparam_shortcuts::get(LOGNAME, nh, "angular_derivative_gain", angular_pid_config_.k_d);

  rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);
}

void PoseTracking::initializePID(const PIDConfig& pid_config, std::vector<control_toolbox::Pid>& pid_vector)
{
  bool use_anti_windup = true;
  pid_vector.push_back(control_toolbox::Pid(pid_config.k_p, pid_config.k_i, pid_config.k_d, pid_config.windup_limit,
                                            -pid_config.windup_limit, use_anti_windup));
}

bool PoseTracking::haveRecentTargetPose(const double timespan)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  return ((ros::Time::now() - target_pose_.header.stamp).toSec() < timespan);
}

bool PoseTracking::haveRecentEndEffectorPose(const double timespan)
{
  return ((ros::Time::now() - command_frame_transform_stamp_).toSec() < timespan);
}

bool PoseTracking::satisfiesPoseTolerance(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  double x_error = target_pose_.pose.position.x - command_frame_transform_.translation()(0);
  double y_error = target_pose_.pose.position.y - command_frame_transform_.translation()(1);
  double z_error = target_pose_.pose.position.z - command_frame_transform_.translation()(2);

  // If uninitialized, likely haven't received the target pose yet.
  if (!angular_error_)
    return false;

  return ((std::abs(x_error) < positional_tolerance(0)) && (std::abs(y_error) < positional_tolerance(1)) &&
          (std::abs(z_error) < positional_tolerance(2)) && (std::abs(*angular_error_) < angular_tolerance));
}

void PoseTracking::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  target_pose_ = *msg;

  // If the target pose is not defined in planning frame, transform the target pose.
  if (target_pose_.header.frame_id != planning_frame_)
  {
    try
    {
      geometry_msgs::TransformStamped target_to_planning_frame = transform_buffer_.lookupTransform(
          planning_frame_, target_pose_.header.frame_id, ros::Time(0), ros::Duration(0.1));
      tf2::doTransform(target_pose_, target_pose_, target_to_planning_frame);

      // Prevent doTransform from copying a stamp of 0, which will cause the haveRecentTargetPose check to fail servo motions
      target_pose_.header.stamp = ros::Time::now();
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, ex.what());
      return;
    }
  }
}

geometry_msgs::TwistStampedConstPtr PoseTracking::calculateTwistCommand()
{
  // use the shared pool to create a message more efficiently
  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();

  // Get twist components from PID controllers
  geometry_msgs::Twist& twist = msg->twist;
  Eigen::Quaterniond q_desired;

  // Scope mutex locking only to operations which require access to target pose.
  {
    std::lock_guard<std::mutex> lock(target_pose_mtx_);
    msg->header.frame_id = target_pose_.header.frame_id;

    // Position
    twist.linear.x = cartesian_position_pids_[0].computeCommand(
        target_pose_.pose.position.x - command_frame_transform_.translation()(0), loop_rate_.expectedCycleTime());
    twist.linear.y = cartesian_position_pids_[1].computeCommand(
        target_pose_.pose.position.y - command_frame_transform_.translation()(1), loop_rate_.expectedCycleTime());
    twist.linear.z = cartesian_position_pids_[2].computeCommand(
        target_pose_.pose.position.z - command_frame_transform_.translation()(2), loop_rate_.expectedCycleTime());

    // Orientation algorithm:
    // - Find the orientation error as a quaternion: q_error = q_desired * q_current ^ -1
    // - Use the angle-axis PID controller to calculate an angular rate
    // - Convert to angular velocity for the TwistStamped message
    q_desired = Eigen::Quaterniond(target_pose_.pose.orientation.w, target_pose_.pose.orientation.x,
                                   target_pose_.pose.orientation.y, target_pose_.pose.orientation.z);
  }

  Eigen::Quaterniond q_current(command_frame_transform_.rotation());
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();

  // Convert axis-angle to angular velocity
  Eigen::AngleAxisd axis_angle(q_error);
  // Cache the angular error, for rotation tolerance checking
  angular_error_ = axis_angle.angle();
  double ang_vel_magnitude =
      cartesian_orientation_pids_[0].computeCommand(*angular_error_, loop_rate_.expectedCycleTime());
  twist.angular.x = ang_vel_magnitude * axis_angle.axis()[0];
  twist.angular.y = ang_vel_magnitude * axis_angle.axis()[1];
  twist.angular.z = ang_vel_magnitude * axis_angle.axis()[2];

  msg->header.stamp = ros::Time::now();

  return msg;
}

void PoseTracking::stopMotion()
{
  stop_requested_ = true;

  // Send a 0 command to Servo to halt arm motion
  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  {
    std::lock_guard<std::mutex> lock(target_pose_mtx_);
    msg->header.frame_id = target_pose_.header.frame_id;
  }
  msg->header.stamp = ros::Time::now();
  twist_stamped_pub_.publish(msg);
}

void PoseTracking::doPostMotionReset()
{
  stopMotion();
  stop_requested_ = false;
  angular_error_ = boost::none;

  // Reset error integrals and previous errors of PID controllers
  cartesian_position_pids_[0].reset();
  cartesian_position_pids_[1].reset();
  cartesian_position_pids_[2].reset();
  cartesian_orientation_pids_[0].reset();
}

void PoseTracking::updatePIDConfig(const double x_proportional_gain, const double x_integral_gain,
                                   const double x_derivative_gain, const double y_proportional_gain,
                                   const double y_integral_gain, const double y_derivative_gain,
                                   const double z_proportional_gain, const double z_integral_gain,
                                   const double z_derivative_gain, const double angular_proportional_gain,
                                   const double angular_integral_gain, const double angular_derivative_gain)
{
  stopMotion();

  x_pid_config_.k_p = x_proportional_gain;
  x_pid_config_.k_i = x_integral_gain;
  x_pid_config_.k_d = x_derivative_gain;
  y_pid_config_.k_p = y_proportional_gain;
  y_pid_config_.k_i = y_integral_gain;
  y_pid_config_.k_d = y_derivative_gain;
  z_pid_config_.k_p = z_proportional_gain;
  z_pid_config_.k_i = z_integral_gain;
  z_pid_config_.k_d = z_derivative_gain;

  angular_pid_config_.k_p = angular_proportional_gain;
  angular_pid_config_.k_i = angular_integral_gain;
  angular_pid_config_.k_d = angular_derivative_gain;

  cartesian_position_pids_.clear();
  cartesian_orientation_pids_.clear();
  initializePID(x_pid_config_, cartesian_position_pids_);
  initializePID(y_pid_config_, cartesian_position_pids_);
  initializePID(z_pid_config_, cartesian_position_pids_);
  initializePID(angular_pid_config_, cartesian_orientation_pids_);

  doPostMotionReset();
}

void PoseTracking::getPIDErrors(double& x_error, double& y_error, double& z_error, double& orientation_error)
{
  double dummy1, dummy2;
  cartesian_position_pids_.at(0).getCurrentPIDErrors(&x_error, &dummy1, &dummy2);
  cartesian_position_pids_.at(1).getCurrentPIDErrors(&y_error, &dummy1, &dummy2);
  cartesian_position_pids_.at(2).getCurrentPIDErrors(&z_error, &dummy1, &dummy2);
  cartesian_orientation_pids_.at(0).getCurrentPIDErrors(&orientation_error, &dummy1, &dummy2);
}

void PoseTracking::resetTargetPose()
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  target_pose_ = geometry_msgs::PoseStamped();
  target_pose_.header.stamp = ros::Time(0);
}

bool PoseTracking::getCommandFrameTransform(geometry_msgs::TransformStamped& transform)
{
  return servo_->getCommandFrameTransform(transform);
}
}  // namespace moveit_servo
