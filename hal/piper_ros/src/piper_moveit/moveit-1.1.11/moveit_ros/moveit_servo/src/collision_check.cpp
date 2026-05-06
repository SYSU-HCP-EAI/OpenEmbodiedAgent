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

/*      Title     : collision_check.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 */

#include <std_msgs/Float64.h>

#include <moveit_servo/collision_check.h>
#include <moveit_servo/make_shared_from_pool.h>

static const char LOGNAME[] = "collision_check";
static const double MIN_RECOMMENDED_COLLISION_RATE = 10;
constexpr double EPSILON = 1e-6;                // For very small numeric comparisons
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops

namespace moveit_servo
{
// Constructor for the class that handles collision checking
CollisionCheck::CollisionCheck(ros::NodeHandle& nh, const moveit_servo::ServoParameters& parameters,
                               const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : nh_(nh)
  , parameters_(parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , self_velocity_scale_coefficient_(-log(0.001) / parameters.self_collision_proximity_threshold)
  , scene_velocity_scale_coefficient_(-log(0.001) / parameters.scene_collision_proximity_threshold)
  , period_(1. / parameters_.collision_check_rate)
{
  // Init collision request
  collision_request_.group_name = parameters_.move_group_name;
  collision_request_.distance = true;  // enable distance-based collision checking
  collision_request_.contacts = true;  // Record the names of collision pairs

  if (parameters_.collision_check_rate < MIN_RECOMMENDED_COLLISION_RATE)
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                   "Collision check rate is low, increase it in yaml file if CPU allows");

  collision_check_type_ =
      (parameters_.collision_check_type == "threshold_distance" ? K_THRESHOLD_DISTANCE : K_STOP_DISTANCE);
  safety_factor_ = parameters_.collision_distance_safety_factor;

  // Internal namespace
  ros::NodeHandle internal_nh(nh_, "internal");
  collision_velocity_scale_pub_ = internal_nh.advertise<std_msgs::Float64>("collision_velocity_scale", ROS_QUEUE_SIZE);
  worst_case_stop_time_sub_ =
      internal_nh.subscribe("worst_case_stop_time", ROS_QUEUE_SIZE, &CollisionCheck::worstCaseStopTimeCB, this);

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  acm_ = getLockedPlanningSceneRO()->getAllowedCollisionMatrix();
}

planning_scene_monitor::LockedPlanningSceneRO CollisionCheck::getLockedPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

void CollisionCheck::start()
{
  timer_ = nh_.createTimer(period_, &CollisionCheck::run, this);
}

void CollisionCheck::run(const ros::TimerEvent& timer_event)
{
  // Log warning when the last loop duration was longer than the period
  if (timer_event.profile.last_duration.toSec() > period_.toSec())
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME,
                                   "last_duration: " << timer_event.profile.last_duration.toSec() << " ("
                                                     << period_.toSec() << ")");
  }

  if (paused_)
  {
    return;
  }

  // Update to the latest current state
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->updateCollisionBodyTransforms();
  collision_detected_ = false;

  // Do a timer-safe distance-based collision detection
  collision_result_.clear();
  getLockedPlanningSceneRO()->getCollisionEnv()->checkRobotCollision(collision_request_, collision_result_,
                                                                     *current_state_);
  scene_collision_distance_ = collision_result_.distance;
  collision_detected_ |= collision_result_.collision;
  collision_result_.print();

  collision_result_.clear();
  // Self-collisions and scene collisions are checked separately so different thresholds can be used
  getLockedPlanningSceneRO()->getCollisionEnvUnpadded()->checkSelfCollision(collision_request_, collision_result_,
                                                                            *current_state_, acm_);
  self_collision_distance_ = collision_result_.distance;
  collision_detected_ |= collision_result_.collision;
  collision_result_.print();

  velocity_scale_ = 1;
  // If we're definitely in collision, stop immediately
  if (collision_detected_)
  {
    velocity_scale_ = 0;
  }
  // If threshold distances were specified
  else if (collision_check_type_ == K_THRESHOLD_DISTANCE)
  {
    // If we are far from a collision, velocity_scale should be 1.
    // If we are very close to a collision, velocity_scale should be ~zero.
    // When scene_collision_proximity_threshold is breached, start decelerating exponentially.
    if (scene_collision_distance_ < parameters_.scene_collision_proximity_threshold)
    {
      // velocity_scale = e ^ k * (collision_distance - threshold)
      // k = - ln(0.001) / collision_proximity_threshold
      // velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
      // velocity_scale should equal 0.001 when collision_distance is at zero.
      velocity_scale_ =
          std::min(velocity_scale_, exp(scene_velocity_scale_coefficient_ *
                                        (scene_collision_distance_ - parameters_.scene_collision_proximity_threshold)));
    }

    if (self_collision_distance_ < parameters_.self_collision_proximity_threshold)
    {
      velocity_scale_ =
          std::min(velocity_scale_, exp(self_velocity_scale_coefficient_ *
                                        (self_collision_distance_ - parameters_.self_collision_proximity_threshold)));
    }
  }
  // Else, we stop based on worst-case stopping distance
  else
  {
    // Retrieve the worst-case time to stop, based on current joint velocities

    // Calculate rate of change of distance to nearest collision
    current_collision_distance_ = std::min(scene_collision_distance_, self_collision_distance_);
    derivative_of_collision_distance_ = (current_collision_distance_ - prev_collision_distance_) / period_.toSec();

    if (current_collision_distance_ < parameters_.min_allowable_collision_distance &&
        derivative_of_collision_distance_ <= 0)
    {
      velocity_scale_ = 0;
    }
    // Only bother doing calculations if we are moving toward the nearest collision
    else if (derivative_of_collision_distance_ < -EPSILON)
    {
      // At the rate the collision distance is decreasing, how long until we collide?
      est_time_to_collision_ = fabs(current_collision_distance_ / derivative_of_collision_distance_);

      // halt if we can't stop fast enough (including the safety factor)
      if (est_time_to_collision_ < (safety_factor_ * worst_case_stop_time_))
      {
        velocity_scale_ = 0;
      }
    }

    // Update for the next iteration
    prev_collision_distance_ = current_collision_distance_;
  }

  // publish message
  {
    auto msg = moveit::util::make_shared_from_pool<std_msgs::Float64>();
    msg->data = velocity_scale_;
    collision_velocity_scale_pub_.publish(msg);
  }
}

void CollisionCheck::worstCaseStopTimeCB(const std_msgs::Float64ConstPtr& msg)
{
  worst_case_stop_time_ = msg->data;
}

void CollisionCheck::setPaused(bool paused)
{
  paused_ = paused;
}

}  // namespace moveit_servo
