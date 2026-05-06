/*******************************************************************************
 *      Title     : low_pass_filter.h
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Andy Zelenak
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

#include <cstddef>

namespace moveit_servo
{
/**
 * Class LowPassFilter - Filter a signal to soften jerks.
 * This is a first-order Butterworth low-pass filter.
 *
 * TODO: Use ROS filters package (http://wiki.ros.org/filters, https://github.com/ros/filters)
 */
class LowPassFilter
{
public:
  // Larger filter_coeff-> more smoothing of servo commands, but more lag.
  // Rough plot, with cutoff frequency on the y-axis:
  // https://www.wolframalpha.com/input/?i=plot+arccot(c)
  explicit LowPassFilter(double low_pass_filter_coeff);
  double filter(double new_measurement);
  void reset(double data);

private:
  static constexpr std::size_t FILTER_LENGTH = 2;
  double previous_measurements_[FILTER_LENGTH];
  double previous_filtered_measurement_;
  // Scale and feedback term are calculated from supplied filter coefficient
  double scale_term_;
  double feedback_term_;
};
}  // namespace moveit_servo
