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

#include <moveit/ompl_interface/detail/projection_evaluators.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

#include <utility>

ompl_interface::ProjectionEvaluatorLinkPose::ProjectionEvaluatorLinkPose(const ModelBasedPlanningContext* pc,
                                                                         const std::string& link)
  : ompl::base::ProjectionEvaluator(pc->getOMPLStateSpace())
  , planning_context_(pc)
  , link_(planning_context_->getJointModelGroup()->getLinkModel(link))
  , tss_(planning_context_->getCompleteInitialRobotState())
{
}

unsigned int ompl_interface::ProjectionEvaluatorLinkPose::getDimension() const
{
  return 3;
}

void ompl_interface::ProjectionEvaluatorLinkPose::defaultCellSizes()
{
  cellSizes_.resize(3);
  cellSizes_[0] = 0.1;
  cellSizes_[1] = 0.1;
  cellSizes_[2] = 0.1;
}

void ompl_interface::ProjectionEvaluatorLinkPose::project(const ompl::base::State* state,
                                                          OMPLProjection projection) const
{
  moveit::core::RobotState* s = tss_.getStateStorage();
  planning_context_->getOMPLStateSpace()->copyToRobotState(*s, state);

  const Eigen::Vector3d& o = s->getGlobalLinkTransform(link_).translation();
  projection(0) = o.x();
  projection(1) = o.y();
  projection(2) = o.z();
}

ompl_interface::ProjectionEvaluatorJointValue::ProjectionEvaluatorJointValue(const ModelBasedPlanningContext* pc,
                                                                             std::vector<unsigned int> variables)
  : ompl::base::ProjectionEvaluator(pc->getOMPLStateSpace()), variables_(std::move(variables))
{
}

unsigned int ompl_interface::ProjectionEvaluatorJointValue::getDimension() const
{
  return variables_.size();
}

void ompl_interface::ProjectionEvaluatorJointValue::defaultCellSizes()
{
  cellSizes_.clear();
  cellSizes_.resize(variables_.size(), 0.1);
}

void ompl_interface::ProjectionEvaluatorJointValue::project(const ompl::base::State* state,
                                                            OMPLProjection projection) const
{
  for (std::size_t i = 0; i < variables_.size(); ++i)
    projection(i) = state->as<ModelBasedStateSpace::StateType>()->values[variables_[i]];
}
