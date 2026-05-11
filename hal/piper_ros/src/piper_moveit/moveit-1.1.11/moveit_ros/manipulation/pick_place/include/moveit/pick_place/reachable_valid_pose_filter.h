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

/* Author: Ioan Sucan, Sachin Chitta */

#pragma once

#include <moveit/pick_place/manipulation_stage.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_scene/planning_scene.h>

namespace pick_place
{
class ReachableAndValidPoseFilter : public ManipulationStage
{
public:
  ReachableAndValidPoseFilter(const planning_scene::PlanningSceneConstPtr& scene,
                              const collision_detection::AllowedCollisionMatrixConstPtr& collision_matrix,
                              const constraint_samplers::ConstraintSamplerManagerPtr& constraints_sampler_manager);

  bool evaluate(const ManipulationPlanPtr& plan) const override;

private:
  bool isEndEffectorFree(const ManipulationPlanPtr& plan, moveit::core::RobotState& token_state) const;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  collision_detection::AllowedCollisionMatrixConstPtr collision_matrix_;
  constraint_samplers::ConstraintSamplerManagerPtr constraints_sampler_manager_;
};
}  // namespace pick_place
