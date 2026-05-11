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

#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <stdexcept>
#include <vector>
#include <utility>
#include <typeinfo>

#include <moveit_msgs/MotionSequenceRequest.h>

#include "command_types_typedef.h"
#include "motioncmd.h"

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Data class storing all information regarding a Sequence command.
 */
class Sequence
{
public:
  /**
   * @brief Adds a command to the end of the sequence.
   * @param cmd The command which has to be added.
   */
  void add(const CmdVariant& cmd, const double blend_radius = 0.);

  /**
   * @brief Returns the number of commands.
   */
  size_t size() const;

  template <class T>
  T& getCmd(const size_t index_cmd);

  template <class T>
  const T& getCmd(const size_t index_cmd) const;

  /**
   * @return TRUE if the specified command is of the specified type,
   * otherwise FALSE.
   */
  template <class T>
  bool cmdIsOfType(const size_t index_cmd) const;

  /**
   * @brief Returns the specific command as base class reference.
   * This function allows the user to operate on the sequence without
   * having knowledge of the underlying specific command type.
   */
  MotionCmd& getCmd(const size_t index_cmd);

  void setAllBlendRadiiToZero();
  void setBlendRadius(const size_t index_cmd, const double blend_radius);
  double getBlendRadius(const size_t index_cmd) const;

  /**
   * @brief Deletes all commands from index 'start' to index 'end'.
   */
  void erase(const size_t start, const size_t end);

  moveit_msgs::MotionSequenceRequest toRequest() const;

private:
  using TCmdRadiiPair = std::pair<CmdVariant, double>;
  std::vector<TCmdRadiiPair> cmds_;
};

inline void Sequence::add(const CmdVariant& cmd, const double blend_radius)
{
  cmds_.emplace_back(cmd, blend_radius);
}

inline size_t Sequence::size() const
{
  return cmds_.size();
}

template <class T>
inline T& Sequence::getCmd(const size_t index_cmd)
{
  return boost::get<T>(cmds_.at(index_cmd).first);
}

template <class T>
inline const T& Sequence::getCmd(const size_t index_cmd) const
{
  return boost::get<T>(cmds_.at(index_cmd).first);
}

inline double Sequence::getBlendRadius(const size_t index_cmd) const
{
  return cmds_.at(index_cmd).second;
}

inline void Sequence::setBlendRadius(const size_t index_cmd, const double blend_radius)
{
  cmds_.at(index_cmd).second = blend_radius;
}

inline void Sequence::setAllBlendRadiiToZero()
{
  std::for_each(cmds_.begin(), cmds_.end(), [](TCmdRadiiPair& cmd) { cmd.second = 0.; });
}

template <class T>
inline bool Sequence::cmdIsOfType(const size_t index_cmd) const
{
  return cmds_.at(index_cmd).first.type() == typeid(T);
}
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // SEQUENCE_H
