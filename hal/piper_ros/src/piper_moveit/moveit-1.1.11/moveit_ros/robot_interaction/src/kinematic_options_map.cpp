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

#include <moveit/robot_interaction/kinematic_options_map.h>

// These strings have no content.  They are compared by address.
const std::string robot_interaction::KinematicOptionsMap::DEFAULT = "";
const std::string robot_interaction::KinematicOptionsMap::ALL = "";

robot_interaction::KinematicOptionsMap::KinematicOptionsMap()
{
}

// Returns a copy of the KinematicOptions so that the caller does not need to
// worry about locking.
robot_interaction::KinematicOptions robot_interaction::KinematicOptionsMap::getOptions(const std::string& key) const
{
  boost::mutex::scoped_lock lock(lock_);

  if (&key == &DEFAULT)
    return defaults_;

  M_options::const_iterator it = options_.find(key);
  if (it == options_.end())
    return defaults_;
  return it->second;
}

void robot_interaction::KinematicOptionsMap::setOptions(const std::string& key, const KinematicOptions& options_delta,
                                                        KinematicOptions::OptionBitmask fields)
{
  boost::mutex::scoped_lock lock(lock_);

  if (&key == &ALL)
  {
    if (fields == KinematicOptions::ALL)
    {
      // setting ALL fields for ALL keys
      // so just clear all key-specific fields and set the defaults.
      defaults_ = options_delta;
      options_.clear();
      return;
    }

    defaults_.setOptions(options_delta, fields);
    for (std::pair<const std::string, KinematicOptions>& option : options_)
    {
      option.second.setOptions(options_delta, fields);
    }
    return;
  }

  if (&key == &DEFAULT)
  {
    defaults_.setOptions(options_delta, fields);
    return;
  }

  M_options::iterator it = options_.find(key);
  KinematicOptions* opts;
  if (it == options_.end())
  {
    // create new entry for key and initialize to defaults_
    opts = &options_[key];
    *opts = defaults_;
  }
  else
  {
    opts = &it->second;
  }

  opts->setOptions(options_delta, fields);
}

// merge other into this.  All options in other take precedence over this.
void robot_interaction::KinematicOptionsMap::merge(const KinematicOptionsMap& other)
{
  if (&other == this)
    return;

  // need to lock in consistent order to avoid deadlock.
  // Lock the one with lower address first.
  boost::mutex* m1 = &lock_;
  boost::mutex* m2 = &other.lock_;
  if (m2 < m1)
    std::swap(m1, m2);
  boost::mutex::scoped_lock lock1(*m1);
  boost::mutex::scoped_lock lock2(*m2);

  defaults_ = other.defaults_;
  for (const std::pair<const std::string, KinematicOptions>& option : other.options_)
  {
    options_[option.first] = option.second;
  }
}

// This is intended to be called as a ModifyStateFunction to modify the state
// maintained by a LockedRobotState in place.
bool robot_interaction::KinematicOptionsMap::setStateFromIK(moveit::core::RobotState& state, const std::string& key,
                                                            const std::string& group, const std::string& tip,
                                                            const geometry_msgs::Pose& pose) const
{
  // copy options so lock is not needed during IK solve.
  KinematicOptions options = getOptions(key);
  return options.setStateFromIK(state, group, tip, pose);
}
