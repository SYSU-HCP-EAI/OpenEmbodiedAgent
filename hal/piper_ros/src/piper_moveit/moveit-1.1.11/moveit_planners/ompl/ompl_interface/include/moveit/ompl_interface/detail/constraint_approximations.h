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

#include <moveit/macros/declare_ptr.h>
#include <planning_scene/planning_scene.h>
#include <kinematic_constraints/kinematic_constraint.h>
#include <ompl/base/StateStorage.h>
#include <visualization_msgs/MarkerArray.h>

namespace ompl_interface
{
typedef ompl::base::StateStorageWithMetadata<std::vector<std::size_t> > ConstraintApproximationStateStorage;
typedef std::function<bool(const ompl::base::State*, const ompl::base::State*)> ConstraintStateStorageOrderFn;

struct ConstraintApproximation
{
  ConstraintApproximation(const planning_models::RobotModelConstPtr& kinematic_model, const std::string& group,
                          const std::string& factory, const std::string& serialization, const std::string& filename,
                          const ompl::base::StateStoragePtr& storage);
  ConstraintApproximation(const planning_models::RobotModelConstPtr& kinematic_model, const std::string& group,
                          const std::string& factory, const moveit_msgs::Constraints& msg, const std::string& filename,
                          const ompl::base::StateStoragePtr& storage);

  void visualizeDistribution(const std::string& link_name, unsigned int count,
                             visualization_msgs::MarkerArray& arr) const;

  std::string group_;
  std::string factory_;
  std::string serialization_;
  moveit_msgs::Constraints constraint_msg_;
  planning_models::RobotModelConstPtr robot_model_;
  kinematic_constraints::KinematicConstraintSetPtr kconstraints_set_;
  std::vector<int> space_signature_;

  std::string ompldb_filename_;
  ompl::base::StateStoragePtr state_storage_ptr_;
  ConstraintApproximationStateStorage* state_storage_;
};

MOVEIT_DECLARE_PTR(ConstraintApproximations, std::vector<ConstraintApproximation>)
}  // namespace ompl_interface
