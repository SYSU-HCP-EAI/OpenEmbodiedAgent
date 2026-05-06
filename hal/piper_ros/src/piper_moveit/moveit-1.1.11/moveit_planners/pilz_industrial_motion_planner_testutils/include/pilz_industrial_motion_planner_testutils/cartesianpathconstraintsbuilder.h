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

#ifndef CARTESIANPATHCONSTRAINTSBUILDER_H
#define CARTESIANPATHCONSTRAINTSBUILDER_H

#include <string>

#include <moveit_msgs/Constraints.h>

#include "cartesianconfiguration.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Helper class to build moveit_msgs::Constraints from a
 * given configuration.
 */
class CartesianPathConstraintsBuilder
{
public:
  CartesianPathConstraintsBuilder& setConstraintName(const std::string& constraint_name);
  CartesianPathConstraintsBuilder& setConfiguration(const CartesianConfiguration& configuration);

  moveit_msgs::Constraints toPathConstraints() const;

private:
  std::string constraint_name_;
  CartesianConfiguration configuration_;
};

inline CartesianPathConstraintsBuilder&
CartesianPathConstraintsBuilder::setConstraintName(const std::string& constraint_name)
{
  constraint_name_ = constraint_name;
  return *this;
}

inline CartesianPathConstraintsBuilder&
CartesianPathConstraintsBuilder::setConfiguration(const CartesianConfiguration& configuration)
{
  configuration_ = configuration;
  return *this;
}

inline moveit_msgs::Constraints CartesianPathConstraintsBuilder::toPathConstraints() const
{
  moveit_msgs::PositionConstraint pos_constraint;
  pos_constraint.link_name = configuration_.getLinkName();
  pos_constraint.constraint_region.primitive_poses.push_back(configuration_.getPose());

  moveit_msgs::Constraints path_constraints;
  path_constraints.name = constraint_name_;
  path_constraints.position_constraints.push_back(pos_constraint);
  return path_constraints;
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // CARTESIANPATHCONSTRAINTSBUILDER_H
