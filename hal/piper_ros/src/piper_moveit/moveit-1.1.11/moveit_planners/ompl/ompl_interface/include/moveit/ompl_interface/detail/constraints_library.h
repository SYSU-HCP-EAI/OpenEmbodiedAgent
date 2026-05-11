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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <ompl/base/StateStorage.h>
#include <boost/serialization/map.hpp>

namespace ompl_interface
{
typedef std::pair<std::vector<std::size_t>, std::map<std::size_t, std::pair<std::size_t, std::size_t> > >
    ConstrainedStateMetadata;
typedef ompl::base::StateStorageWithMetadata<ConstrainedStateMetadata> ConstraintApproximationStateStorage;

MOVEIT_CLASS_FORWARD(ConstraintApproximation);

class ConstraintApproximation
{
public:
  ConstraintApproximation(std::string group, std::string state_space_parameterization, bool explicit_motions,
                          moveit_msgs::Constraints msg, std::string filename, ompl::base::StateStoragePtr storage,
                          std::size_t milestones = 0);

  virtual ~ConstraintApproximation()
  {
  }

  const std::string& getName() const
  {
    return constraint_msg_.name;
  }

  ompl::base::StateSamplerAllocator getStateSamplerAllocator(const moveit_msgs::Constraints& msg) const;

  InterpolationFunction getInterpolationFunction() const;

  const std::vector<int>& getSpaceSignature() const
  {
    return space_signature_;
  }

  const std::string& getGroup() const
  {
    return group_;
  }

  bool hasExplicitMotions() const
  {
    return explicit_motions_;
  }

  std::size_t getMilestoneCount() const
  {
    return milestones_;
  }

  const std::string& getStateSpaceParameterization() const
  {
    return state_space_parameterization_;
  }

  const moveit_msgs::Constraints& getConstraintsMsg() const
  {
    return constraint_msg_;
  }

  const ompl::base::StateStoragePtr& getStateStorage() const
  {
    return state_storage_ptr_;
  }

  const std::string& getFilename() const
  {
    return ompldb_filename_;
  }

protected:
  std::string group_;
  std::string state_space_parameterization_;
  bool explicit_motions_;

  moveit_msgs::Constraints constraint_msg_;

  std::vector<int> space_signature_;

  std::string ompldb_filename_;
  ompl::base::StateStoragePtr state_storage_ptr_;
  ConstraintApproximationStateStorage* state_storage_;
  std::size_t milestones_;
};

struct ConstraintApproximationConstructionOptions
{
  ConstraintApproximationConstructionOptions()
    : samples(0)
    , edges_per_sample(0)
    , max_edge_length(std::numeric_limits<double>::infinity())
    , explicit_motions(false)
    , explicit_points_resolution(0.0)
    , max_explicit_points(0)
  {
  }

  std::string state_space_parameterization;
  unsigned int samples;
  unsigned int edges_per_sample;
  double max_edge_length;
  bool explicit_motions;
  double explicit_points_resolution;
  unsigned int max_explicit_points;
};

struct ConstraintApproximationConstructionResults
{
  ConstraintApproximationPtr approx;
  std::size_t milestones;
  double state_sampling_time;
  double state_connection_time;
  double sampling_success_rate;
};

MOVEIT_CLASS_FORWARD(ConstraintsLibrary);  // Defines ConstraintsLibraryPtr, ConstPtr, WeakPtr... etc

class ConstraintsLibrary
{
public:
  ConstraintsLibrary(ModelBasedPlanningContext* pcontext) : context_(pcontext)
  {
  }

  void loadConstraintApproximations(const std::string& path);

  void saveConstraintApproximations(const std::string& path);

  ConstraintApproximationConstructionResults
  addConstraintApproximation(const moveit_msgs::Constraints& constr_sampling,
                             const moveit_msgs::Constraints& constr_hard, const std::string& group,
                             const planning_scene::PlanningSceneConstPtr& scene,
                             const ConstraintApproximationConstructionOptions& options);

  ConstraintApproximationConstructionResults
  addConstraintApproximation(const moveit_msgs::Constraints& constr, const std::string& group,
                             const planning_scene::PlanningSceneConstPtr& scene,
                             const ConstraintApproximationConstructionOptions& options);

  void printConstraintApproximations(std::ostream& out = std::cout) const;
  void clearConstraintApproximations();

  void registerConstraintApproximation(const ConstraintApproximationPtr& approx)
  {
    constraint_approximations_[approx->getName()] = approx;
  }

  const ConstraintApproximationPtr& getConstraintApproximation(const moveit_msgs::Constraints& msg) const;

private:
  ompl::base::StateStoragePtr
  constructConstraintApproximation(ModelBasedPlanningContext* pcontext, const moveit_msgs::Constraints& constr_sampling,
                                   const moveit_msgs::Constraints& constr_hard,
                                   const ConstraintApproximationConstructionOptions& options,
                                   ConstraintApproximationConstructionResults& result);

  ModelBasedPlanningContext* context_;
  std::map<std::string, ConstraintApproximationPtr> constraint_approximations_;
};
}  // namespace ompl_interface
