/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include "pilz_industrial_motion_planner/plan_components_builder.h"

#include <cassert>

#include "pilz_industrial_motion_planner/tip_frame_getter.h"

namespace pilz_industrial_motion_planner
{
std::vector<robot_trajectory::RobotTrajectoryPtr> PlanComponentsBuilder::build() const
{
  std::vector<robot_trajectory::RobotTrajectoryPtr> res_vec{ traj_cont_ };
  if (traj_tail_)
  {
    assert(!res_vec.empty());
    appendWithStrictTimeIncrease(*(res_vec.back()), *traj_tail_);
  }
  return res_vec;
}

void PlanComponentsBuilder::appendWithStrictTimeIncrease(robot_trajectory::RobotTrajectory& result,
                                                         const robot_trajectory::RobotTrajectory& source)
{
  if (result.empty() ||
      !pilz_industrial_motion_planner::isRobotStateEqual(result.getLastWayPoint(), source.getFirstWayPoint(),
                                                         result.getGroupName(), ROBOT_STATE_EQUALITY_EPSILON))
  {
    result.append(source, 0.0);
    return;
  }

  for (size_t i = 1; i < source.getWayPointCount(); ++i)
  {
    result.addSuffixWayPoint(source.getWayPoint(i), source.getWayPointDurationFromPrevious(i));
  }
}

void PlanComponentsBuilder::blend(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius)
{
  if (!blender_)
  {
    throw NoBlenderSetException("No blender set");
  }

  assert(other->getGroupName() == traj_tail_->getGroupName());

  pilz_industrial_motion_planner::TrajectoryBlendRequest blend_request;

  blend_request.first_trajectory = traj_tail_;
  blend_request.second_trajectory = other;
  blend_request.blend_radius = blend_radius;
  blend_request.group_name = traj_tail_->getGroupName();
  blend_request.link_name = getSolverTipFrame(model_->getJointModelGroup(blend_request.group_name));

  pilz_industrial_motion_planner::TrajectoryBlendResponse blend_response;
  if (!blender_->blend(planning_scene, blend_request, blend_response))
  {
    throw BlendingFailedException("Blending failed");
  }

  // Append the new trajectory elements
  appendWithStrictTimeIncrease(*(traj_cont_.back()), *blend_response.first_trajectory);
  traj_cont_.back()->append(*blend_response.blend_trajectory, 0.0);
  // Store the last new trajectory element for future processing
  traj_tail_ = blend_response.second_trajectory;  // first for next blending segment
}

void PlanComponentsBuilder::append(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const robot_trajectory::RobotTrajectoryPtr& other, const double blend_radius)
{
  if (!model_)
  {
    throw NoRobotModelSetException("No robot model set");
  }

  if (!traj_tail_)
  {
    traj_tail_ = other;
    // Reserve space in container for new trajectory
    traj_cont_.emplace_back(new robot_trajectory::RobotTrajectory(model_, other->getGroupName()));
    return;
  }

  // Create new trajectory for every group change
  if (other->getGroupName() != traj_tail_->getGroupName())
  {
    appendWithStrictTimeIncrease(*(traj_cont_.back()), *traj_tail_);
    traj_tail_ = other;
    // Create new container element
    traj_cont_.emplace_back(new robot_trajectory::RobotTrajectory(model_, other->getGroupName()));
    return;
  }

  // No blending
  if (blend_radius <= 0.0)
  {
    appendWithStrictTimeIncrease(*(traj_cont_.back()), *traj_tail_);
    traj_tail_ = other;
    return;
  }

  blend(planning_scene, other, blend_radius);
}

}  // namespace pilz_industrial_motion_planner
