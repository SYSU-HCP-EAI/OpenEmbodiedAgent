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

#ifndef BASECMD_H
#define BASECMD_H

#include <string>

#include "motioncmd.h"

namespace pilz_industrial_motion_planner_testutils
{
template <class StartType, class GoalType>
class BaseCmd : public MotionCmd
{
public:
  BaseCmd() : MotionCmd()
  {
  }

  virtual ~BaseCmd() = default;

public:
  planning_interface::MotionPlanRequest toRequest() const override;

  void setStartConfiguration(StartType start);
  void setGoalConfiguration(GoalType goal);

  StartType& getStartConfiguration();
  const StartType& getStartConfiguration() const;

  GoalType& getGoalConfiguration();
  const GoalType& getGoalConfiguration() const;

private:
  virtual std::string getPlannerId() const = 0;

protected:
  GoalType goal_;
  StartType start_;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class GoalType>
inline void BaseCmd<StartType, GoalType>::setStartConfiguration(StartType start)
{
  start_ = start;
}

template <class StartType, class GoalType>
inline void BaseCmd<StartType, GoalType>::setGoalConfiguration(GoalType goal)
{
  goal_ = goal;
}

template <class StartType, class GoalType>
inline StartType& BaseCmd<StartType, GoalType>::getStartConfiguration()
{
  return start_;
}

template <class StartType, class GoalType>
inline const StartType& BaseCmd<StartType, GoalType>::getStartConfiguration() const
{
  return start_;
}

template <class StartType, class GoalType>
inline GoalType& BaseCmd<StartType, GoalType>::getGoalConfiguration()
{
  return goal_;
}

template <class StartType, class GoalType>
inline const GoalType& BaseCmd<StartType, GoalType>::getGoalConfiguration() const
{
  return goal_;
}

template <class StartType, class GoalType>
planning_interface::MotionPlanRequest BaseCmd<StartType, GoalType>::toRequest() const
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = getPlannerId();
  req.group_name = this->planning_group_;

  req.max_velocity_scaling_factor = this->vel_scale_;
  req.max_acceleration_scaling_factor = this->acc_scale_;

  req.start_state = this->start_.toMoveitMsgsRobotState();
  req.goal_constraints.push_back(this->goal_.toGoalConstraints());

  return req;
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // BASECMD_H
