/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ken Anderson */

#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_parameterization.h>

namespace trajectory_processing
{
/// \brief This class  modifies the timestamps of a trajectory to respect
/// velocity and acceleration constraints.
MOVEIT_CLASS_FORWARD(IterativeParabolicTimeParameterization);
class IterativeParabolicTimeParameterization : public TimeParameterization
{
public:
  IterativeParabolicTimeParameterization(unsigned int max_iterations = 100, double max_time_change_per_it = .01);

  bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor = 1.0,
                         const double max_acceleration_scaling_factor = 1.0) const override;

  static void updateTrajectory(robot_trajectory::RobotTrajectory& rob_trajectory, const std::vector<double>& time_diff);

private:
  unsigned int max_iterations_;    /// @brief maximum number of iterations to find solution
  double max_time_change_per_it_;  /// @brief maximum allowed time change per iteration in seconds

  void applyVelocityConstraints(robot_trajectory::RobotTrajectory& rob_trajectory, std::vector<double>& time_diff,
                                const double max_velocity_scaling_factor) const;

  void applyAccelerationConstraints(robot_trajectory::RobotTrajectory& rob_trajectory, std::vector<double>& time_diff,
                                    const double max_acceleration_scaling_factor) const;

  double findT1(const double d1, const double d2, double t1, const double t2, const double a_max) const;
  double findT2(const double d1, const double d2, const double t1, double t2, const double a_max) const;
};

}  // namespace trajectory_processing
