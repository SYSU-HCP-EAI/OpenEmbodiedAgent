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

#include <ompl/config.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>

// TODO: remove when ROS Lunar and older are no longer supported
#if OMPL_VERSION_VALUE >= 1004000  // Version greater than 1.4.0
typedef Eigen::Ref<Eigen::VectorXd> OMPLProjection;
#else  // All other versions
typedef ompl::base::EuclideanProjection& OMPLProjection;
#endif

namespace ompl_interface
{
class ModelBasedPlanningContext;

/** @class ProjectionEvaluatorLinkPose
    @brief */
class ProjectionEvaluatorLinkPose : public ompl::base::ProjectionEvaluator
{
public:
  ProjectionEvaluatorLinkPose(const ModelBasedPlanningContext* pc, const std::string& link);

  unsigned int getDimension() const override;
  void defaultCellSizes() override;
  void project(const ompl::base::State* state, OMPLProjection projection) const override;

private:
  const ModelBasedPlanningContext* planning_context_;
  const moveit::core::LinkModel* link_;
  TSStateStorage tss_;
};

/** @class ProjectionEvaluatorJointValue
    @brief */
class ProjectionEvaluatorJointValue : public ompl::base::ProjectionEvaluator
{
public:
  ProjectionEvaluatorJointValue(const ModelBasedPlanningContext* pc, std::vector<unsigned int> variables);

  unsigned int getDimension() const override;
  void defaultCellSizes() override;
  void project(const ompl::base::State* state, OMPLProjection projection) const override;

private:
  std::vector<unsigned int> variables_;
};
}  // namespace ompl_interface
