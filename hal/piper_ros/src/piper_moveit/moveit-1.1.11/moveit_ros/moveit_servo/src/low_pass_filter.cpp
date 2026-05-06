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

/*      Title     : low_pass_filter.cpp
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Andy Zelenak
 */

#include <moveit_servo/low_pass_filter.h>
#include <cmath>
#include <string>
#include <ros/ros.h>

namespace moveit_servo
{
namespace
{
constexpr char LOGNAME[] = "low_pass_filter";
constexpr double EPSILON = 1e-9;
}  // namespace

LowPassFilter::LowPassFilter(double low_pass_filter_coeff)
  : previous_measurements_{ 0., 0. }
  , previous_filtered_measurement_(0.)
  , scale_term_(1. / (1. + low_pass_filter_coeff))
  , feedback_term_(1. - low_pass_filter_coeff)
{
  // guarantee this doesn't change because the logic below depends on this length implicity
  static_assert(LowPassFilter::FILTER_LENGTH == 2, "moveit_servo::LowPassFilter::FILTER_LENGTH should be 2");

  ROS_ASSERT_MSG(!std::isinf(feedback_term_), "%s: outputs from filter will be inf because feedback term is inf",
                 LOGNAME);
  ROS_ASSERT_MSG(!std::isinf(scale_term_), "%s: outputs from filter will be inf because denominator of scale is 0",
                 LOGNAME);
  ROS_ASSERT_MSG(low_pass_filter_coeff >= 1., "%s: Filter coefficient < 1. makes the lowpass filter unstable", LOGNAME);

  if (std::abs(feedback_term_) < EPSILON)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Filter coefficient value of " << low_pass_filter_coeff
                                                                  << " resulted in feedback term of 0. "
                                                                     " This results in a window averaging Finite "
                                                                     "Impulse Response (FIR) filter with a gain of "
                                                                  << scale_term_ * LowPassFilter::FILTER_LENGTH);
  }
}

void LowPassFilter::reset(double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;

  previous_filtered_measurement_ = data;
}

double LowPassFilter::filter(double new_measurement)
{
  // Push in the new measurement
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement;

  double new_filtered_measurement = scale_term_ * (previous_measurements_[1] + previous_measurements_[0] -
                                                   feedback_term_ * previous_filtered_measurement_);

  // Store the new filtered measurement
  previous_filtered_measurement_ = new_filtered_measurement;

  return new_filtered_measurement;
}
}  // namespace moveit_servo
