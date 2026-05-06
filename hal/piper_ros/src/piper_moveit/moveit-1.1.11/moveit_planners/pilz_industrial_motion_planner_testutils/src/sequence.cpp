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

#include "pilz_industrial_motion_planner_testutils/sequence.h"

#include <algorithm>
#include <boost/variant.hpp>

namespace pilz_industrial_motion_planner_testutils
{
/**
 * @brief Visitor transforming the stored command into a MotionPlanRequest.
 */
class ToReqVisitor : public boost::static_visitor<planning_interface::MotionPlanRequest>
{
public:
  template <typename T>
  planning_interface::MotionPlanRequest operator()(T& cmd) const
  {
    return cmd.toRequest();
  }
};

/**
 * @brief Visitor returning not the specific command type but the base type.
 */
class ToBaseVisitor : public boost::static_visitor<MotionCmd&>
{
public:
  template <typename T>
  MotionCmd& operator()(T& cmd) const
  {
    return cmd;
  }
};

moveit_msgs::MotionSequenceRequest Sequence::toRequest() const
{
  moveit_msgs::MotionSequenceRequest req;

  std::vector<std::string> group_names;
  for (const auto& cmd : cmds_)
  {
    moveit_msgs::MotionSequenceItem item;
    item.req = boost::apply_visitor(ToReqVisitor(), cmd.first);

    if (std::find(group_names.begin(), group_names.end(), item.req.group_name) != group_names.end())
    {
      // Remove start state because only the first request of a group
      // is allowed to have a start state in a sequence.
      item.req.start_state = moveit_msgs::RobotState();
    }
    else
    {
      group_names.emplace_back(item.req.group_name);
    }

    item.blend_radius = cmd.second;
    req.items.push_back(item);
  }
  return req;
}

void Sequence::erase(const size_t start, const size_t end)
{
  const size_t orig_n{ size() };
  if (start > orig_n || end > orig_n)
  {
    std::string msg;
    msg.append("Parameter start=").append(std::to_string(start));
    msg.append(" and end=").append(std::to_string(end));
    msg.append(" must not be greater then the number of #commands=");
    msg.append(std::to_string(size()));
    throw std::invalid_argument(msg);
  }
  cmds_.erase(cmds_.begin() + start, cmds_.begin() + end);
  if (end == orig_n)
  {
    // Make sure last radius is set zero
    cmds_.at(size() - 1).second = 0.;
  }
}

MotionCmd& Sequence::getCmd(const size_t index_cmd)
{
  return boost::apply_visitor(ToBaseVisitor(), cmds_.at(index_cmd).first);
}

}  // namespace pilz_industrial_motion_planner_testutils
