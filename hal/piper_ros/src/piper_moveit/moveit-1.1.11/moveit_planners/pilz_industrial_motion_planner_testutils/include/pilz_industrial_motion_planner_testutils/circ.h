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

#ifndef CIRC_H
#define CIRC_H

#include <memory>

#include "basecmd.h"
#include "circauxiliary.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Data class storing all information regarding a Circ command.
 */
template <class StartType, class AuxiliaryType, class GoalType>
class Circ : public BaseCmd<StartType, GoalType>
{
public:
  Circ() : BaseCmd<StartType, GoalType>()
  {
  }

public:
  void setAuxiliaryConfiguration(AuxiliaryType auxiliary);
  AuxiliaryType& getAuxiliaryConfiguration();
  const AuxiliaryType& getAuxiliaryConfiguration() const;

public:
  planning_interface::MotionPlanRequest toRequest() const override;

private:
  std::string getPlannerId() const override;

private:
  AuxiliaryType auxiliary_;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class AuxiliaryType, class GoalType>
inline void Circ<StartType, AuxiliaryType, GoalType>::setAuxiliaryConfiguration(AuxiliaryType auxiliary)
{
  auxiliary_ = auxiliary;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline std::string Circ<StartType, AuxiliaryType, GoalType>::getPlannerId() const
{
  return "CIRC";
}

template <class StartType, class AuxiliaryType, class GoalType>
inline planning_interface::MotionPlanRequest Circ<StartType, AuxiliaryType, GoalType>::toRequest() const
{
  planning_interface::MotionPlanRequest req{ BaseCmd<StartType, GoalType>::toRequest() };
  req.path_constraints = auxiliary_.toPathConstraints();

  return req;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline AuxiliaryType& Circ<StartType, AuxiliaryType, GoalType>::getAuxiliaryConfiguration()
{
  return auxiliary_;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline const AuxiliaryType& Circ<StartType, AuxiliaryType, GoalType>::getAuxiliaryConfiguration() const
{
  return auxiliary_;
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // CIRC_H
