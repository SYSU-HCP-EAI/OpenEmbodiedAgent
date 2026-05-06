/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include "pilz_industrial_motion_planner/joint_limits_extension.h"

#include <map>
#include <vector>
#include <string>

namespace pilz_industrial_motion_planner
{
/**
 * @brief Container for JointLimits, essentially a map with convenience
 * functions.
 * Adds the ability to as for limits and get a common limit that unifies all
 * given limits
 */
class JointLimitsContainer
{
public:
  /**
   * @brief Add a limit
   * @param joint_name  Name if the joint this limit belongs to
   * @param joint_limit Limit of the joint
   * @return true if the limit was added, false
   *         if joint_limit.has_deceleration_limit &&
   * joint_limit.max_deceleration >= 0
   */
  bool addLimit(const std::string& joint_name, JointLimit joint_limit);

  /**
   * @brief Check if there is a limit for a joint with the given name in this
   * container
   * @param joint_name Name of the joint
   */
  bool hasLimit(const std::string& joint_name) const;

  /**
   * @brief Get Number of limits in the container
   * @return Number of limits in the container
   */
  size_t getCount() const;

  /**
   * @brief Returns wether the container is empty
   * @return true if empty, false otherwise
   */
  bool empty() const;

  /**
   * @brief Returns joint limit fusion of all(position, velocity, acceleration,
   * deceleration) limits for all joint.
   * There are cases where the most strict limit of all limits is needed.
   * If there are no matching limits, the flag
   * has_[position|velocity|...]_limits is set to false.
   *
   * @return joint limit
   */
  JointLimit getCommonLimit() const;

  /**
   * @brief Returns joint limit fusion of all(position, velocity, acceleration,
   * deceleration) limits for given joints.
   * There are cases where the most strict limit of all limits is needed.
   * If there are no matching limits, the flag
   * has_[position|velocity|...]_limits is set to false.
   *
   * @param joint_names
   * @return joint limit
   * @throws std::out_of_range if a joint limit with this name does not exist
   */
  JointLimit getCommonLimit(const std::vector<std::string>& joint_names) const;

  /**
   * @brief getLimit get the limit for the given joint name
   * @param joint_name
   * @return joint limit
   * @throws std::out_of_range if a joint limit with this name does not exist
   */
  JointLimit getLimit(const std::string& joint_name) const;

  /**
   * @brief ConstIterator to the underlying data structure
   * @return
   */
  std::map<std::string, JointLimit>::const_iterator begin() const;

  /**
   * @brief ConstIterator to the underlying data structure
   * @return
   */
  std::map<std::string, JointLimit>::const_iterator end() const;

  /**
   * @brief verify position limit of single joint
   * @param joint_name
   * @param joint_position
   * @return
   */
  bool verifyVelocityLimit(const std::string& joint_name, const double& joint_velocity) const;

  /**
   * @brief verify position limit of single joint
   * @param joint_name
   * @param joint_position
   * @return
   */
  bool verifyPositionLimit(const std::string& joint_name, const double& joint_position) const;

private:
  /**
   * @brief update the most strict limit with given joint limit
   * @param joint_limit
   * @param common_limit the current most strict limit
   */
  static void updateCommonLimit(const JointLimit& joint_limit, JointLimit& common_limit);

protected:
  /// Actual container object containing the data
  std::map<std::string, JointLimit> container_;
};
}  // namespace pilz_industrial_motion_planner
