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

#pragma once

#include <stdexcept>
#include <string>

#include <moveit_msgs/MoveItErrorCodes.h>

namespace pilz_industrial_motion_planner
{
/**
 * @brief Exception storing an moveit_msgs::MoveItErrorCodes value.
 */
class MoveItErrorCodeException : public std::runtime_error
{
public:
  MoveItErrorCodeException(const std::string& msg);

protected:
  MoveItErrorCodeException(const MoveItErrorCodeException&) = default;
  MoveItErrorCodeException(MoveItErrorCodeException&&) = default;
  ~MoveItErrorCodeException() override = default;

  MoveItErrorCodeException& operator=(const MoveItErrorCodeException&) = default;
  MoveItErrorCodeException& operator=(MoveItErrorCodeException&&) = default;

public:
  virtual const moveit_msgs::MoveItErrorCodes::_val_type& getErrorCode() const = 0;
};

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE = moveit_msgs::MoveItErrorCodes::FAILURE>
class TemplatedMoveItErrorCodeException : public MoveItErrorCodeException
{
public:
  TemplatedMoveItErrorCodeException(const std::string& msg);
  TemplatedMoveItErrorCodeException(const std::string& msg, const moveit_msgs::MoveItErrorCodes::_val_type& error_code);

public:
  const moveit_msgs::MoveItErrorCodes::_val_type& getErrorCode() const override;

private:
  const moveit_msgs::MoveItErrorCodes::_val_type error_code_{ ERROR_CODE };
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
inline MoveItErrorCodeException::MoveItErrorCodeException(const std::string& msg) : std::runtime_error(msg)
{
}

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE>
inline TemplatedMoveItErrorCodeException<ERROR_CODE>::TemplatedMoveItErrorCodeException(const std::string& msg)
  : MoveItErrorCodeException(msg)
{
}

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE>
inline TemplatedMoveItErrorCodeException<ERROR_CODE>::TemplatedMoveItErrorCodeException(
    const std::string& msg, const moveit_msgs::MoveItErrorCodes::_val_type& error_code)
  : MoveItErrorCodeException(msg), error_code_(error_code)
{
}

template <moveit_msgs::MoveItErrorCodes::_val_type ERROR_CODE>
inline const moveit_msgs::MoveItErrorCodes::_val_type&
TemplatedMoveItErrorCodeException<ERROR_CODE>::getErrorCode() const
{
  return error_code_;
}

/*
 * @brief Macro to automatically generate a derived class of
 * the MoveItErrorCodeException class.
 */
#define CREATE_MOVEIT_ERROR_CODE_EXCEPTION(EXCEPTION_CLASS_NAME, ERROR_CODE)                                           \
  class EXCEPTION_CLASS_NAME : public TemplatedMoveItErrorCodeException<ERROR_CODE>                                    \
  {                                                                                                                    \
  public:                                                                                                              \
    EXCEPTION_CLASS_NAME(const std::string& msg) : TemplatedMoveItErrorCodeException(msg)                              \
    {                                                                                                                  \
    }                                                                                                                  \
                                                                                                                       \
    EXCEPTION_CLASS_NAME(const std::string& msg, const moveit_msgs::MoveItErrorCodes::_val_type& error_code)           \
      : TemplatedMoveItErrorCodeException(msg, error_code)                                                             \
    {                                                                                                                  \
    }                                                                                                                  \
  }

}  // namespace pilz_industrial_motion_planner
