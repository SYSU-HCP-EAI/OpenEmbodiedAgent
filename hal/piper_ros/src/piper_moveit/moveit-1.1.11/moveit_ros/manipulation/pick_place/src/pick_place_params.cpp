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

/* Author: Ioan Sucan */

#include <moveit/pick_place/pick_place_params.h>
#include <dynamic_reconfigure/server.h>
#include <moveit_ros_manipulation/PickPlaceDynamicReconfigureConfig.h>

namespace pick_place
{
namespace
{
using namespace moveit_ros_manipulation;

class DynamicReconfigureImpl
{
public:
  DynamicReconfigureImpl() : dynamic_reconfigure_server_(ros::NodeHandle("~/pick_place"))
  {
    dynamic_reconfigure_server_.setCallback(
        [this](auto& config, uint32_t level) { dynamicReconfigureCallback(config, level); });
  }

  const PickPlaceParams& getParams() const
  {
    return params_;
  }

private:
  PickPlaceParams params_;

  void dynamicReconfigureCallback(PickPlaceDynamicReconfigureConfig& config, uint32_t /*level*/)
  {
    params_.max_goal_count_ = config.max_attempted_states_per_pose;
    params_.max_fail_ = config.max_consecutive_fail_attempts;
    params_.max_step_ = config.cartesian_motion_step_size;
    params_.jump_factor_ = config.jump_factor;
  }

  dynamic_reconfigure::Server<PickPlaceDynamicReconfigureConfig> dynamic_reconfigure_server_;
};
}  // namespace
}  // namespace pick_place

pick_place::PickPlaceParams::PickPlaceParams() : max_goal_count_(5), max_fail_(3), max_step_(0.02), jump_factor_(2.0)
{
}

const pick_place::PickPlaceParams& pick_place::GetGlobalPickPlaceParams()
{
  static DynamicReconfigureImpl pick_place_params;
  return pick_place_params.getParams();
}
