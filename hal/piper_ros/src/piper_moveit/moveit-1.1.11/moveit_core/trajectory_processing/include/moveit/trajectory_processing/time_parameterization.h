#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trajectory_processing
{
/**
 * @brief Base class for trajectory parameterization algorithms
 */
MOVEIT_CLASS_FORWARD(TimeParameterization);
class TimeParameterization
{
public:
  virtual ~TimeParameterization() = default;
  virtual bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                 const double max_velocity_scaling_factor = 1.0,
                                 const double max_acceleration_scaling_factor = 1.0) const = 0;
};
}  // namespace trajectory_processing
