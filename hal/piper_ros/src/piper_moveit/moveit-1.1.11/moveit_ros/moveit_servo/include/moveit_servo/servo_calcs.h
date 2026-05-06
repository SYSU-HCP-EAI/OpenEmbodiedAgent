/*******************************************************************************
 *      Title     : servo_calcs.h
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

// C++
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>

// ROS
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ChangeDriftDimensions.h>
#include <moveit_msgs/ChangeControlDimensions.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>

// moveit_servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/low_pass_filter.h>

namespace moveit_servo
{
class ServoCalcs
{
public:
  ServoCalcs(ros::NodeHandle& nh, ServoParameters& parameters,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~ServoCalcs();

  /** \brief Start the timer where we do work and publish outputs */
  void start();

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getCommandFrameTransform(Eigen::Isometry3d& transform);
  bool getCommandFrameTransform(geometry_msgs::TransformStamped& transform);

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getEEFrameTransform(Eigen::Isometry3d& transform);
  bool getEEFrameTransform(geometry_msgs::TransformStamped& transform);

  /** \brief Pause or unpause processing servo commands while keeping the timers alive */
  void setPaused(bool paused);

  /** \brief Change the controlled link. Often, this is the end effector
   * This must be a link on the robot since MoveIt tracks the transform (not tf)
   */
  void changeRobotLinkCommandFrame(const std::string& new_command_frame);

  // Give test access to private/protected methods
  friend class ServoFixture;

private:
  /** \brief Run the main calculation loop */
  void mainCalcLoop();

  /** \brief Do calculations for a single iteration. Publish one outgoing command */
  void calculateSingleIteration();

  /** \brief Stop the currently running thread */
  void stop();

  /** \brief Do servoing calculations for Cartesian twist commands. */
  bool cartesianServoCalcs(geometry_msgs::TwistStamped& cmd, trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief Do servoing calculations for direct commands to a joint. */
  bool jointServoCalcs(const control_msgs::JointJog& cmd, trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief Parse the incoming joint msg for the joints of our MoveGroup */
  void updateJoints();

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
  Eigen::VectorXd scaleJointCommand(const control_msgs::JointJog& command) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  /** \brief Suddenly halt for a joint limit or other critical issue.
   * Is handled differently for position vs. velocity control.
   */
  void suddenHalt(trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief  Scale the delta theta to match joint velocity/acceleration limits */
  void enforceVelLimits(Eigen::ArrayXd& delta_theta);

  /** \brief Avoid overshooting joint limits */
  bool enforcePositionLimits(sensor_msgs::JointState& joint_state);

  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
   */
  double velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                             const Eigen::MatrixXd& pseudo_inverse);

  /**
   * Slow motion down if close to singularity or collision.
   * @param delta_theta motion command, used in calculating new_joint_tray
   * @param singularity_scale tells how close we are to a singularity
   */
  void applyVelocityScaling(Eigen::ArrayXd& delta_theta, double singularity_scale);

  /** \brief Compose the outgoing JointTrajectory message */
  void composeJointTrajMessage(const sensor_msgs::JointState& joint_state,
                               trajectory_msgs::JointTrajectory& joint_trajectory) const;

  /** \brief Smooth position commands with a lowpass filter */
  void lowPassFilterPositions(sensor_msgs::JointState& joint_state);

  /** \brief Set the filters to the specified values */
  void resetLowPassFilters(const sensor_msgs::JointState& joint_state);

  /** \brief Convert an incremental position command to joint velocity message */
  void calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta);

  /** \brief Convert joint deltas to an outgoing JointTrajectory command.
   * This happens for joint commands and Cartesian commands.
   */
  bool convertDeltasToOutgoingCmd(trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief Gazebo simulations have very strict message timestamp requirements.
   * Satisfy Gazebo by stuffing multiple messages into one.
   */
  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory, int count) const;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
  void removeDimension(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x, unsigned int row_to_remove);

  /* \brief Callback for joint subsription */
  void jointStateCB(const sensor_msgs::JointStateConstPtr& msg);

  /* \brief Command callbacks */
  void twistStampedCB(const geometry_msgs::TwistStampedConstPtr& msg);
  void jointCmdCB(const control_msgs::JointJogConstPtr& msg);
  void collisionVelocityScaleCB(const std_msgs::Float64ConstPtr& msg);

  /**
   * Allow drift in certain dimensions. For example, may allow the wrist to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
  bool changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                             moveit_msgs::ChangeDriftDimensions::Response& res);

  /** \brief Service callback for changing servoing dimensions (e.g. ignore rotation about X) */
  bool changeControlDimensions(moveit_msgs::ChangeControlDimensions::Request& req,
                               moveit_msgs::ChangeControlDimensions::Response& res);

  /** \brief Service callback to reset Servo status, e.g. so the arm can move again after a collision */
  bool resetServoStatus(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  ros::NodeHandle nh_;

  // Parameters from yaml
  ServoParameters& parameters_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count_ = 0;

  // Flag for staying inactive while there are no incoming commands
  bool wait_for_servo_commands_ = true;

  // Flag saying if the filters were updated during the timer callback
  bool updated_filters_ = false;

  // Nonzero status flags
  bool have_nonzero_twist_stamped_ = false;
  bool have_nonzero_joint_command_ = false;
  bool have_nonzero_command_ = false;

  // Incoming command messages
  geometry_msgs::TwistStamped twist_stamped_cmd_;
  control_msgs::JointJog joint_servo_cmd_;

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr current_state_;

  // incoming_joint_state_ is the incoming message. It may contain passive joints or other joints we don't care about.
  // (mutex protected below)
  // internal_joint_state_ is used in servo calculations. It shouldn't be relied on to be accurate.
  // original_joint_state_ is the same as incoming_joint_state_ except it only contains the joints the servo node acts
  // on.
  sensor_msgs::JointState internal_joint_state_, original_joint_state_;
  std::map<std::string, std::size_t> joint_state_name_map_;

  std::vector<LowPassFilter> position_filters_;

  trajectory_msgs::JointTrajectoryConstPtr last_sent_command_;

  // ROS
  ros::Subscriber joint_state_sub_;
  ros::Subscriber twist_stamped_sub_;
  ros::Subscriber joint_cmd_sub_;
  ros::Subscriber collision_velocity_scale_sub_;
  ros::Publisher status_pub_;
  ros::Publisher worst_case_stop_time_pub_;
  ros::Publisher outgoing_cmd_pub_;
  ros::ServiceServer drift_dimensions_server_;
  ros::ServiceServer control_dimensions_server_;
  ros::ServiceServer reset_servo_status_;

  // Main tracking / result publisher loop
  std::thread thread_;
  bool stop_requested_;

  // Status
  StatusCode status_ = StatusCode::NO_WARNING;
  std::atomic<bool> paused_;
  bool twist_command_is_stale_ = false;
  bool joint_command_is_stale_ = false;
  bool ok_to_publish_ = false;
  double collision_velocity_scale_ = 1.0;

  // Use ArrayXd type to enable more coefficient-wise operations
  Eigen::ArrayXd delta_theta_;
  Eigen::ArrayXd prev_joint_velocity_;

  const int gazebo_redundant_message_count_ = 30;

  uint num_joints_;

  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> drift_dimensions_ = { { false, false, false, false, false, false } };

  // The dimesions to control. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> control_dimensions_ = { { true, true, true, true, true, true } };

  // input_mutex_ is used to protect the state below it
  mutable std::mutex input_mutex_;
  Eigen::Isometry3d tf_moveit_to_robot_cmd_frame_;
  Eigen::Isometry3d tf_moveit_to_ee_frame_;
  geometry_msgs::TwistStampedConstPtr latest_twist_stamped_;
  control_msgs::JointJogConstPtr latest_joint_cmd_;
  ros::Time latest_twist_command_stamp_ = ros::Time(0.);
  ros::Time latest_joint_command_stamp_ = ros::Time(0.);
  bool latest_nonzero_twist_stamped_ = false;
  bool latest_nonzero_joint_cmd_ = false;

  // input condition variable used for low latency mode
  std::condition_variable input_cv_;
  bool new_input_cmd_ = false;
};
}  // namespace moveit_servo
