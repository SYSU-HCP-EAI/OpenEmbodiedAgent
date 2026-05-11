/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef CARTESIANCONFIGURATION_H
#define CARTESIANCONFIGURATION_H

#include <vector>
#include <sstream>

#include <boost/optional.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include "robotconfiguration.h"
#include "jointconfiguration.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Class to define a robot configuration in space
 * with the help of cartesian coordinates.
 */
class CartesianConfiguration : public RobotConfiguration
{
public:
  CartesianConfiguration();

  CartesianConfiguration(const std::string& group_name, const std::string& link_name, const std::vector<double>& config);

  CartesianConfiguration(const std::string& group_name, const std::string& link_name, const std::vector<double>& config,
                         const moveit::core::RobotModelConstPtr& robot_model);

public:
  moveit_msgs::Constraints toGoalConstraints() const override;
  moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

  void setLinkName(const std::string& link_name);
  const std::string& getLinkName() const;

  void setPose(const geometry_msgs::Pose& pose);
  const geometry_msgs::Pose& getPose() const;
  geometry_msgs::Pose& getPose();

  void setSeed(const JointConfiguration& config);
  const JointConfiguration& getSeed() const;
  //! @brief States if a seed for the cartesian configuration is set.
  bool hasSeed() const;

  void setPoseTolerance(const double tol);
  const boost::optional<double> getPoseTolerance() const;

  void setAngleTolerance(const double tol);
  const boost::optional<double> getAngleTolerance() const;

private:
  static geometry_msgs::Pose toPose(const std::vector<double>& pose);
  static geometry_msgs::PoseStamped toStampedPose(const geometry_msgs::Pose& pose);

private:
  std::string link_name_;
  geometry_msgs::Pose pose_;

  //! @brief The dimensions of the sphere associated with the target region
  //! of the position constraint.
  boost::optional<double> tolerance_pose_{ boost::none };

  //! @brief The value to assign to the absolute tolerances of the
  //! orientation constraint.
  boost::optional<double> tolerance_angle_{ boost::none };

  //! @brief The seed for computing the IK solution of the cartesian configuration.
  boost::optional<JointConfiguration> seed_{ boost::none };
};

std::ostream& operator<<(std::ostream& /*os*/, const CartesianConfiguration& /*obj*/);

inline void CartesianConfiguration::setLinkName(const std::string& link_name)
{
  link_name_ = link_name;
}

inline const std::string& CartesianConfiguration::getLinkName() const
{
  return link_name_;
}

inline void CartesianConfiguration::setPose(const geometry_msgs::Pose& pose)
{
  pose_ = pose;
}

inline const geometry_msgs::Pose& CartesianConfiguration::getPose() const
{
  return pose_;
}

inline geometry_msgs::Pose& CartesianConfiguration::getPose()
{
  return pose_;
}

inline moveit_msgs::Constraints CartesianConfiguration::toGoalConstraints() const
{
  if (!tolerance_pose_ || !tolerance_angle_)
  {
    return kinematic_constraints::constructGoalConstraints(link_name_, toStampedPose(pose_));
  }
  else
  {
    return kinematic_constraints::constructGoalConstraints(link_name_, toStampedPose(pose_), tolerance_pose_.value(),
                                                           tolerance_angle_.value());
  }
}

inline void CartesianConfiguration::setSeed(const JointConfiguration& config)
{
  seed_ = config;
}

inline const JointConfiguration& CartesianConfiguration::getSeed() const
{
  return seed_.value();
}

inline bool CartesianConfiguration::hasSeed() const
{
  return seed_.is_initialized();
}

inline void CartesianConfiguration::setPoseTolerance(const double tol)
{
  tolerance_pose_ = tol;
}

inline const boost::optional<double> CartesianConfiguration::getPoseTolerance() const
{
  return tolerance_pose_;
}

inline void CartesianConfiguration::setAngleTolerance(const double tol)
{
  tolerance_angle_ = tol;
}

inline const boost::optional<double> CartesianConfiguration::getAngleTolerance() const
{
  return tolerance_angle_;
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // CARTESIANCONFIGURATION_H
