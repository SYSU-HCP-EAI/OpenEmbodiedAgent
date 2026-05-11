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

#ifndef CIRCAUXILIARY_H
#define CIRCAUXILIARY_H

#include <string>

#include <moveit_msgs/Constraints.h>

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Base class to define an auxiliary point needed to specify
 * circ commands.
 */
template <class ConfigType, class BuilderType>
class CircAuxiliary
{
public:
  void setConfiguration(const ConfigType& auxiliary_config);
  ConfigType& getConfiguration();
  const ConfigType& getConfiguration() const;

public:
  moveit_msgs::Constraints toPathConstraints() const;

private:
  virtual std::string getConstraintName() const = 0;

protected:
  ConfigType auxiliary_config_;
};

template <class ConfigType, class BuilderType>
void CircAuxiliary<ConfigType, BuilderType>::setConfiguration(const ConfigType& auxiliary_config)
{
  auxiliary_config_ = auxiliary_config;
}

template <class ConfigType, class BuilderType>
inline ConfigType& CircAuxiliary<ConfigType, BuilderType>::getConfiguration()
{
  return auxiliary_config_;
}

template <class ConfigType, class BuilderType>
inline const ConfigType& CircAuxiliary<ConfigType, BuilderType>::getConfiguration() const
{
  return auxiliary_config_;
}

template <class ConfigType, class BuilderType>
inline moveit_msgs::Constraints CircAuxiliary<ConfigType, BuilderType>::toPathConstraints() const
{
  return BuilderType().setConstraintName(getConstraintName()).setConfiguration(getConfiguration()).toPathConstraints();
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // CIRCAUXILIARY_H
