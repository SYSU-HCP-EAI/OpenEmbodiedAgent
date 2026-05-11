/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Benjamin Scholz
 *  Copyright (c) 2021, Thies Oelerich
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
 *   * Neither the name of the authors nor the names of other
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

/* Authors: Benjamin Scholz, Thies Oelerich */

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/limit_cartesian_speed.h>

// Name of logger
const char* LOGGER_NAME = "trajectory_processing.cartesian_speed";

namespace trajectory_processing
{
bool limitMaxCartesianLinkSpeed(robot_trajectory::RobotTrajectory& trajectory, const double max_speed,
                                const std::string& link_name)
{
  std::vector<const moveit::core::LinkModel*> links;

  if (!link_name.empty())
  {
    const moveit::core::RobotModel& model{ *trajectory.getRobotModel() };
    bool found = false;
    const auto* link = model.getLinkModel(link_name, &found);

    if (!found)
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Unknown link model '" << link_name << "'");
    else
      links.push_back(link);
  }
  // In case the link name is not given but the trajectory belongs to a group,
  // retrieve the end effectors from that joint model group
  else if (trajectory.getGroup())
  {
    trajectory.getGroup()->getEndEffectorTips(links);
    if (links.empty())
    {
      ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "No link(s) specified");
    }
  }

  // Call function for speed setting using the created link model
  for (const auto& link : links)
  {
    if (!limitMaxCartesianLinkSpeed(trajectory, max_speed, link))
      return false;
  }

  return !links.empty();
}

bool limitMaxCartesianLinkSpeed(robot_trajectory::RobotTrajectory& trajectory, const double max_speed,
                                const moveit::core::LinkModel* link_model)
{
  if (max_speed <= 0.0)
  {
    ROS_ERROR_STREAM_NAMED(LOGGER_NAME, "Link speed must be greater than 0.");
    return false;
  }

  size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints == 0)
    return false;

  // do forward kinematics to get Cartesian positions of link for current waypoint
  double euclidean_distance, new_time_diff, old_time_diff;
  std::vector<double> time_diff(num_waypoints - 1, 0.0);

  for (size_t i = 0; i < num_waypoints - 1; i++)
  {
    // get link state for current and next waypoint
    const Eigen::Isometry3d& current_link_state = trajectory.getWayPointPtr(i)->getGlobalLinkTransform(link_model);
    const Eigen::Isometry3d& next_link_state = trajectory.getWayPointPtr(i + 1)->getGlobalLinkTransform(link_model);

    // get Euclidean distance between the two waypoints
    euclidean_distance = (next_link_state.translation() - current_link_state.translation()).norm();

    new_time_diff = (euclidean_distance / max_speed);
    old_time_diff = trajectory.getWayPointDurationFromPrevious(i + 1);

    // slow-down segment if it was too fast before
    time_diff[i] = std::max(new_time_diff, old_time_diff);
  }
  // update time stamps, velocities and accelerations of the trajectory
  IterativeParabolicTimeParameterization::updateTrajectory(trajectory, time_diff);
  return true;
}
}  // namespace trajectory_processing
