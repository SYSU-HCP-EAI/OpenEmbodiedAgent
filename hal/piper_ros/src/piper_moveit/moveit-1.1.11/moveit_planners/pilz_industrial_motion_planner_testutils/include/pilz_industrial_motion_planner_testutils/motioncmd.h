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

#ifndef MOTIONCMD_H
#define MOTIONCMD_H

#include <string>
#include <memory>

#include "motionplanrequestconvertible.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Base class for commands storing all general information of a command.
 */
class MotionCmd : public MotionPlanRequestConvertible
{
public:
  MotionCmd() : MotionPlanRequestConvertible()
  {
  }

public:
  void setPlanningGroup(const std::string& planning_group);
  const std::string& getPlanningGroup() const;

  void setVelocityScale(double velocity_scale);
  void setAccelerationScale(double acceleration_scale);

protected:
  std::string planning_group_;
  //! Link to which all cartesian poses refer to.
  std::string target_link_;
  double vel_scale_{ 1.0 };
  double acc_scale_{ 1.0 };
};

inline void MotionCmd::setPlanningGroup(const std::string& planning_group)
{
  planning_group_ = planning_group;
}

inline const std::string& MotionCmd::getPlanningGroup() const
{
  return planning_group_;
}

inline void MotionCmd::setVelocityScale(double velocity_scale)
{
  vel_scale_ = velocity_scale;
}

inline void MotionCmd::setAccelerationScale(double acceleration_scale)
{
  acc_scale_ = acceleration_scale;
}

using MotionCmdUPtr = std::unique_ptr<MotionCmd>;
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // MOTIONCMD_H
