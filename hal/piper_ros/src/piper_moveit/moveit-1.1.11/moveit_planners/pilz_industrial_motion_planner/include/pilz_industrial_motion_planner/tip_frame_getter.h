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

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include "pilz_industrial_motion_planner/trajectory_generation_exceptions.h"

namespace pilz_industrial_motion_planner
{
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoSolverException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(MoreThanOneTipFrameException, moveit_msgs::MoveItErrorCodes::FAILURE);

/**
 * @returns true if the JointModelGroup has a solver, false otherwise.
 *
 * @tparam JointModelGroup aims at moveit::core::JointModelGroup
 * @throws exception in case group is null.
 */
template <class JointModelGroup>
static bool hasSolver(const JointModelGroup* group)
{
  if (group == nullptr)
  {
    throw std::invalid_argument("Group must not be null");
  }
  return group->getSolverInstance() != nullptr;
}

/**
 * @return The name of the tip frame (link) of the specified group
 * returned by the solver.
 *
 * @tparam JointModelGroup aims at moveit::core::JointModelGroup
 * @throws exception in case the group has no solver.
 * @throws exception in case the solver for the group has more than one tip
 * frame.
 */
template <class JointModelGroup>
static const std::string& getSolverTipFrame(const JointModelGroup* group)
{
  if (!hasSolver(group))
  {
    throw NoSolverException("No solver for group " + group->getName());
  }

  const std::vector<std::string>& tip_frames{ group->getSolverInstance()->getTipFrames() };
  if (tip_frames.size() > 1)
  {
    throw MoreThanOneTipFrameException("Solver for group \"" + group->getName() + "\" has more than one tip frame");
  }
  return tip_frames.front();
}

}  // namespace pilz_industrial_motion_planner
