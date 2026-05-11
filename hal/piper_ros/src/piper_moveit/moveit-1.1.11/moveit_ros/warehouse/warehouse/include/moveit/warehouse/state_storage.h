/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/warehouse/moveit_message_storage.h>
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/RobotState.h>

namespace moveit_warehouse
{
typedef warehouse_ros::MessageWithMetadata<moveit_msgs::RobotState>::ConstPtr RobotStateWithMetadata;
typedef warehouse_ros::MessageCollection<moveit_msgs::RobotState>::Ptr RobotStateCollection;

MOVEIT_CLASS_FORWARD(RobotStateStorage);  // Defines RobotStateStoragePtr, ConstPtr, WeakPtr... etc

class RobotStateStorage : public MoveItMessageStorage
{
public:
  static const std::string DATABASE_NAME;

  static const std::string STATE_NAME;
  static const std::string ROBOT_NAME;

  RobotStateStorage(warehouse_ros::DatabaseConnection::Ptr conn);

  void addRobotState(const moveit_msgs::RobotState& msg, const std::string& name, const std::string& robot = "");
  bool hasRobotState(const std::string& name, const std::string& robot = "") const;
  void getKnownRobotStates(std::vector<std::string>& names, const std::string& robot = "") const;
  void getKnownRobotStates(const std::string& regex, std::vector<std::string>& names,
                           const std::string& robot = "") const;

  /** \brief Get the constraints named \e name. Return false on failure. */
  bool getRobotState(RobotStateWithMetadata& msg_m, const std::string& name, const std::string& robot = "") const;

  void renameRobotState(const std::string& old_name, const std::string& new_name, const std::string& robot = "");

  void removeRobotState(const std::string& name, const std::string& robot = "");

  void reset();

private:
  void createCollections();

  RobotStateCollection state_collection_;
};
}  // namespace moveit_warehouse
