/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Mario Prats, Ioan Sucan */

#pragma once

#include "moveit/warehouse/moveit_message_storage.h"
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/TrajectoryConstraints.h>

namespace moveit_warehouse
{
typedef warehouse_ros::MessageWithMetadata<moveit_msgs::TrajectoryConstraints>::ConstPtr
    TrajectoryConstraintsWithMetadata;
typedef warehouse_ros::MessageCollection<moveit_msgs::TrajectoryConstraints>::Ptr TrajectoryConstraintsCollection;

MOVEIT_CLASS_FORWARD(TrajectoryConstraintsStorage);  // Defines TrajectoryConstraintsStoragePtr, ConstPtr, WeakPtr... etc

class TrajectoryConstraintsStorage : public MoveItMessageStorage
{
public:
  static const std::string DATABASE_NAME;

  static const std::string CONSTRAINTS_ID_NAME;
  static const std::string CONSTRAINTS_GROUP_NAME;
  static const std::string ROBOT_NAME;

  TrajectoryConstraintsStorage(warehouse_ros::DatabaseConnection::Ptr conn);

  void addTrajectoryConstraints(const moveit_msgs::TrajectoryConstraints& msg, const std::string& name,
                                const std::string& robot = "", const std::string& group = "");
  bool hasTrajectoryConstraints(const std::string& name, const std::string& robot = "",
                                const std::string& group = "") const;
  void getKnownTrajectoryConstraints(std::vector<std::string>& names, const std::string& robot = "",
                                     const std::string& group = "") const;
  void getKnownTrajectoryConstraints(const std::string& regex, std::vector<std::string>& names,
                                     const std::string& robot = "", const std::string& group = "") const;

  /** \brief Get the constraints named \e name. Return false on failure. */
  bool getTrajectoryConstraints(TrajectoryConstraintsWithMetadata& msg_m, const std::string& name,
                                const std::string& robot = "", const std::string& group = "") const;

  void renameTrajectoryConstraints(const std::string& old_name, const std::string& new_name,
                                   const std::string& robot = "", const std::string& group = "");

  void removeTrajectoryConstraints(const std::string& name, const std::string& robot = "",
                                   const std::string& group = "");

  void reset();

private:
  void createCollections();

  TrajectoryConstraintsCollection constraints_collection_;
};
}  // namespace moveit_warehouse
