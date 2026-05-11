/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, SRI International
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

/* Author: Acorn Pooley */

#pragma once

#include <moveit/robot_interaction/kinematic_options.h>
#include <boost/thread.hpp>
#include <boost/function.hpp>

namespace robot_interaction
{
// Maintains a set of KinematicOptions with a key/value mapping and a default
// value.
class KinematicOptionsMap
{
public:
  /// Constructor - set all options to reasonable default values.
  KinematicOptionsMap();

  /// When used as \e key this means the default value
  static const std::string DEFAULT;

  /// When used as \e key this means set ALL keys (including default)
  static const std::string ALL;

  /// Set \e state using inverse kinematics.
  /// @param state the state to set
  /// @param key used to lookup the options to use
  /// @param group name of group whose joints can move
  /// @param tip link that will be posed
  /// @param pose desired pose of tip link
  /// @param result true if IK succeeded.
  bool setStateFromIK(moveit::core::RobotState& state, const std::string& key, const std::string& group,
                      const std::string& tip, const geometry_msgs::Pose& pose) const;

  /// Get the options to use for a particular key.
  /// To get the default values pass key = KinematicOptionsMap::DEFAULT
  KinematicOptions getOptions(const std::string& key) const;

  /// Set some of the options to be used for a particular key.
  ///
  /// @param key set the options for this key.
  ///         To set the default options use key = KinematicOptionsMap::DEFAULT
  ///         To set ALL options use key = KinematicOptionsMap::ALL
  ///
  /// @param options the new value for the options.
  ///
  /// @fields which options to set for the key.
  void setOptions(const std::string& key, const KinematicOptions& options,
                  KinematicOptions::OptionBitmask fields = KinematicOptions::ALL);

  /// Merge all options from \e other into \e this.
  /// Values in \e other (including defaults_) take precedence over values in \e
  /// this.
  void merge(const KinematicOptionsMap& other);

private:
  // this protects all members.
  mutable boost::mutex lock_;

  // default kinematic options.
  // PROTECTED BY lock_
  KinematicOptions defaults_;

  typedef std::map<std::string, KinematicOptions> M_options;

  // per key kinematic options.
  // If key is not here, defaults are used.
  // PROTECTED BY lock_
  M_options options_;
};
}  // namespace robot_interaction
