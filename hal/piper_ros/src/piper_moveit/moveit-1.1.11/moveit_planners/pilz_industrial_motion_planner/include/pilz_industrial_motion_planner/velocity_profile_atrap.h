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

#pragma once

#include "kdl/velocityprofile.hpp"
#include <iostream>

namespace pilz_industrial_motion_planner
{
/**
 * @brief A PTP Trajectory Generator of Asymmetric Trapezoidal Velocity Profile.
 * Differences to VelocityProfile_Trap:
 *   - Maximal acceleration and deceleration can be different, resulting
 *     an asymmetric trapezoid shaped velocity profile.
 *   - Function to generate full synchronized PTP trajectory is provided.
 *   - Function to generate trapezoid shaped velocity profile with start
 * velocity.
 */
class VelocityProfileATrap : public KDL::VelocityProfile
{
public:
  /**
   * @brief Constructor
   * @param max_vel: maximal velocity (absolute value, always positive)
   * @param max_acc: maximal acceleration (absolute value, always positive)
   * @param max_dec: maximal deceleration (absolute value, always positive)
   */
  VelocityProfileATrap(double max_vel = 0, double max_acc = 0, double max_dec = 0);

  /**
   * @brief compute the fastest profile
   * Algorithm:
   *  - compute the minimal distance which is needed to reach maximal velocity
   *  - if maximal velocity can be reached
   *     - compute the coefficients of the trajectory
   *  - if maximal velocity can not be reached
   *     - compute the new velocity can be reached
   *     - compute the coefficients based on this new velocity
   *
   * @param pos1: start position
   * @param pos2: goal position
   */
  void SetProfile(double pos1, double pos2) override;

  /**
   * @brief Profile scaled by the total duration
   * @param pos1: start position
   * @param pos2: goal position
   * @param duration: trajectory duration (must be longer than fastest case,
   * otherwise will be ignored)
   */
  void SetProfileDuration(double pos1, double pos2, double duration) override;

  /**
   * @brief Profile with given acceleration/constant/deceleration durations.
   * Each duration must obey the maximal velocity/acceleration/deceleration
   * constraints.
   * Otherwise the operation will be ignored.
   * Algorithm:
   * - compute the maximal velocity of given durations
   * - compute the acceleration and deceleration of given duraitons
   * - if limits are fulfilled
   *   - compute the coefficients
   * @param pos1: start position
   * @param pos2: goal position
   * @param acc_duration: time of acceleration phase
   * @param const_duration: time of constant phase
   * @param dec_duration: time of deceleration phase
   * @return ture if the combination of three durations is valid
   */
  bool setProfileAllDurations(double pos1, double pos2, double duration1, double duration2, double duration3);

  /**
   * @brief Profile with start velocity
   * Note: This function is not general and is currently only used for live
   * control (vel1*(pos2-pos1)>0).
   * @param pos1: start position
   * @param pos2: goal position
   * @param vel1: start velocity
   * @return
   */
  bool setProfileStartVelocity(double pos1, double pos2, double vel1);

  /**
   * @brief get the time of first phase
   * @return
   */
  double firstPhaseDuration() const
  {
    return t_a_;
  }
  /**
   * @brief get the time of second phase
   * @return
   */
  double secondPhaseDuration() const
  {
    return t_b_;
  }
  /**
   * @brief get the time of third phase
   * @return
   */
  double thirdPhaseDuration() const
  {
    return t_c_;
  }

  /**
   * @brief Compares two Asymmetric Trapezoidal Velocity Profiles.
   *
   * @return True if equal, false otherwise.
   */
  bool operator==(const VelocityProfileATrap& other) const;

  /**
   * @brief Duration
   * @return total duration of the trajectory
   */
  double Duration() const override;
  /**
   * @brief Get position at given time
   * @param time
   * @return
   */
  double Pos(double time) const override;
  /**
   * @brief Get velocity at given time
   * @param time
   * @return
   */
  double Vel(double time) const override;
  /**
   * @brief Get given acceleration/deceleration at given time
   * @param time
   * @return
   */
  double Acc(double time) const override;
  /**
   * @brief Write basic information
   * @param os
   */
  void Write(std::ostream& os) const override;
  /**
   * @brief returns copy of current VelocityProfile object
   * @return
   */
  KDL::VelocityProfile* Clone() const override;

  friend std::ostream& operator<<(std::ostream& os, const VelocityProfileATrap& p);  // LCOV_EXCL_LINE

  ~VelocityProfileATrap() override;

private:
  /// helper functions
  void setEmptyProfile();

private:
  /// specification of the motion profile :
  const double max_vel_;
  const double max_acc_;
  const double max_dec_;
  double start_pos_;
  double end_pos_;

  /// for initial velocity
  double start_vel_;

  /// three phases of trapezoid
  double a1_, a2_, a3_;  /// coef. from ^0 -> ^2 of first phase
  double b1_, b2_, b3_;  /// of second phase
  double c1_, c2_, c3_;  /// of third phase

  /// time of three phases
  double t_a_;  /// the duration of first phase
  double t_b_;  /// the duration of second phase
  double t_c_;  /// the duration of third phase
};

std::ostream& operator<<(std::ostream& os,
                         const VelocityProfileATrap& p);  // LCOV_EXCL_LINE

}  // namespace pilz_industrial_motion_planner
