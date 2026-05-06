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

#include "pilz_industrial_motion_planner_testutils/cartesianconfiguration.h"

#include <stdexcept>
#include <tf2_eigen/tf2_eigen.h>

namespace pilz_industrial_motion_planner_testutils
{
CartesianConfiguration::CartesianConfiguration() : RobotConfiguration()
{
}

CartesianConfiguration::CartesianConfiguration(const std::string& group_name, const std::string& link_name,
                                               const std::vector<double>& config)
  : RobotConfiguration(group_name), link_name_(link_name), pose_(toPose(config))
{
}

CartesianConfiguration::CartesianConfiguration(const std::string& group_name, const std::string& link_name,
                                               const std::vector<double>& config,
                                               const moveit::core::RobotModelConstPtr& robot_model)
  : RobotConfiguration(group_name, robot_model), link_name_(link_name), pose_(toPose(config))
{
  if (robot_model && (!robot_model_->hasLinkModel(link_name_)))
  {
    std::string msg{ "Link \"" };
    msg.append(link_name).append("\" not known to robot model");
    throw std::invalid_argument(msg);
  }

  if (robot_model && (!robot_state::RobotState(robot_model_).knowsFrameTransform(link_name_)))
  {
    std::string msg{ "Tranform of \"" };
    msg.append(link_name).append("\" is unknown");
    throw std::invalid_argument(msg);
  }
}

geometry_msgs::Pose CartesianConfiguration::toPose(const std::vector<double>& pose)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose.at(0);
  pose_msg.position.y = pose.at(1);
  pose_msg.position.z = pose.at(2);
  pose_msg.orientation.x = pose.at(3);
  pose_msg.orientation.y = pose.at(4);
  pose_msg.orientation.z = pose.at(5);
  pose_msg.orientation.w = pose.at(6);

  return pose_msg;
}

geometry_msgs::PoseStamped CartesianConfiguration::toStampedPose(const geometry_msgs::Pose& pose)
{
  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = pose;
  return pose_stamped_msg;
}

moveit_msgs::RobotState CartesianConfiguration::toMoveitMsgsRobotState() const
{
  if (!robot_model_)
  {
    throw std::runtime_error("No robot model set");
  }

  robot_state::RobotState rstate(robot_model_);
  rstate.setToDefaultValues();
  if (hasSeed())
  {
    rstate.setJointGroupPositions(group_name_, getSeed().getJoints());
  }

  rstate.update();

  // set to Cartesian pose
  Eigen::Isometry3d start_pose;
  tf2::fromMsg(pose_, start_pose);
  if (!rstate.setFromIK(rstate.getRobotModel()->getJointModelGroup(group_name_), start_pose, link_name_))
  {
    std::ostringstream os;
    os << "No solution for ik \n" << start_pose.translation() << "\n" << start_pose.linear();
    throw std::runtime_error(os.str());
  }

  // Conversion to RobotState msg type
  moveit_msgs::RobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(rstate, robot_state_msg, true);
  return robot_state_msg;
}

std::ostream& operator<<(std::ostream& os, const CartesianConfiguration& obj)
{
  os << "Group name: \"" << obj.getGroupName() << "\"";
  os << " | link name: \"" << obj.getLinkName() << "\"";
  os << "\n" << obj.getPose();
  return os;
}

}  // namespace pilz_industrial_motion_planner_testutils
