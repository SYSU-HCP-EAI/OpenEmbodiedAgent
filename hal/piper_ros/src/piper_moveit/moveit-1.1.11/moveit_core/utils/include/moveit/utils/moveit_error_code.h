/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
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

#pragma once

#include <moveit_msgs/MoveItErrorCodes.h>

namespace moveit
{
namespace core
{
/**
 * @brief a wrapper around moveit_msgs::MoveItErrorCodes to make it easier to return an error code message from a function
 */
class MoveItErrorCode : public moveit_msgs::MoveItErrorCodes
{
public:
  static const char* toString(const moveit_msgs::MoveItErrorCodes& error_code);

  MoveItErrorCode(int code = 0)
  {
    val = code;
  }
  MoveItErrorCode(const moveit_msgs::MoveItErrorCodes& code)
  {
    val = code.val;
  }
  explicit operator bool() const
  {
    return val == moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  explicit operator std::string() const
  {
    return toString(*this);
  }
  bool operator==(const int c) const
  {
    return val == c;
  }
  bool operator!=(const int c) const
  {
    return val != c;
  }
};

std::ostream& operator<<(std::ostream& out, const MoveItErrorCode& e);

}  // namespace core
}  // namespace moveit
