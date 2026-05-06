/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <moveit/utils/moveit_error_code.h>

namespace moveit
{
namespace core
{
const char* MoveItErrorCode::toString(const moveit_msgs::MoveItErrorCodes& error_code)
{
  switch (error_code.val)
  {
    case 0:
      return "NOT INITIALIZED";
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE:
      return "FAILURE";
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
      return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
      return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
      return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
      return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
      return "PREEMPTED";
    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
      return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
      return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return "GOAL_CONSTRAINTS_VIOLATED";
    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
      return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
      return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
      return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
      return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
      return "INVALID_OBJECT_NAME";
    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
      return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
      return "SENSOR_INFO_STALE";
    case moveit_msgs::MoveItErrorCodes::COMMUNICATION_FAILURE:
      return "COMMUNICATION_FAILURE";
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
      return "NO_IK_SOLUTION";
    default:
      return "UNKNOWN";
  }
}

std::ostream& operator<<(std::ostream& out, const MoveItErrorCode& e)
{
  return out << MoveItErrorCode::toString(e);
}

}  // namespace core
}  // namespace moveit
