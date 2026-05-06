/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Acorn Pooley, Ioan Sucan, Sachin Chitta */

#pragma once

#include <moveit/collision_detection/world.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection
{
MOVEIT_CLASS_FORWARD(WorldDiff);  // Defines WorldDiffPtr, ConstPtr, WeakPtr... etc

/** \brief Maintain a diff list of changes that have happened to a World. */
class WorldDiff
{
public:
  /** \brief Constructor */
  WorldDiff();

  /** \brief Constructor */
  WorldDiff(const WorldPtr& world);

  /** \brief copy constructor. */
  WorldDiff(WorldDiff& other);

  ~WorldDiff();

  /** \brief Set which world to record.  Records all objects in old world (if
   * any) as DESTROYED and all objects in new world as CREATED and ADD_SHAPE
   * */
  void setWorld(const WorldPtr& world);

  /** \brief Set which world to record.  Erases all previously recorded
   * changes.  */
  void reset(const WorldPtr& world);

  /** \brief Turn off recording and erase all previously recorded changes. */
  void reset();

  /** \brief Return all the changes that have been recorded */
  const std::map<std::string, World::Action>& getChanges() const
  {
    return changes_;
  }

  using const_iterator = std::map<std::string, World::Action>::const_iterator;
  /** iterator pointing to first change */
  const_iterator begin() const
  {
    return changes_.begin();
  }
  /** iterator pointing to end of changes */
  const_iterator end() const
  {
    return changes_.end();
  }
  /** number of changes stored */
  size_t size() const
  {
    return changes_.size();
  }
  /** find changes for a named object */
  const_iterator find(const std::string& id) const
  {
    return changes_.find(id);
  }
  /** set the entry for an id */
  void set(const std::string& id, World::Action val)
  {
    if (val)
      changes_[id] = val;
    else
      changes_.erase(id);
  }

  /** \brief Clear the internally maintained vector of changes */
  void clearChanges();

private:
  /** \brief Notification function */
  void notify(const World::ObjectConstPtr& /*obj*/, World::Action /*action*/);

  /** keep changes in a map so they can be coalesced */
  std::map<std::string, World::Action> changes_;

  /* observer handle for world callback */
  World::ObserverHandle observer_handle_;

  /* used to unregister the notifier */
  WorldWeakPtr world_;
};
}  // namespace collision_detection
