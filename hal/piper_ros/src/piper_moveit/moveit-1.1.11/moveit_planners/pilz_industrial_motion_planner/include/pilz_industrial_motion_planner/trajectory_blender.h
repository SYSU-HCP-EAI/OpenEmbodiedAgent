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

#pragma once

#include "pilz_industrial_motion_planner/limits_container.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>

#include "pilz_industrial_motion_planner/trajectory_blend_request.h"
#include "pilz_industrial_motion_planner/trajectory_blend_response.h"

namespace pilz_industrial_motion_planner
{
/**
 * @brief Base class of trajectory blenders
 */
class TrajectoryBlender
{
public:
  TrajectoryBlender(const pilz_industrial_motion_planner::LimitsContainer& planner_limits) : limits_(planner_limits)
  {
  }

  virtual ~TrajectoryBlender()
  {
  }

  /**
   * @brief Blend two robot trajectories with the given blending radius
   * @param planning_scene: planning scene
   * @param req: trajectory blend request
   * @param res: trajectroy blend response
   * @return true if blend succeed
   */
  virtual bool blend(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                     pilz_industrial_motion_planner::TrajectoryBlendResponse& res) = 0;

protected:
  const pilz_industrial_motion_planner::LimitsContainer limits_;
};

typedef std::unique_ptr<TrajectoryBlender> TrajectoryBlenderUniquePtr;

}  // namespace pilz_industrial_motion_planner
