/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Acorn Pooley, Ioan Sucan */

#include <moveit/collision_detection/world_diff.h>
#include <functional>

namespace collision_detection
{
WorldDiff::~WorldDiff()
{
  WorldPtr old_world = world_.lock();
  if (old_world)
    old_world->removeObserver(observer_handle_);
}

WorldDiff::WorldDiff()
{
}

WorldDiff::WorldDiff(const WorldPtr& world) : world_(world)
{
  observer_handle_ =
      world->addObserver([this](const World::ObjectConstPtr& object, World::Action action) { notify(object, action); });
}

WorldDiff::WorldDiff(WorldDiff& other)
{
  WorldPtr world = other.world_.lock();
  if (world)
  {
    changes_ = other.changes_;

    WorldWeakPtr(world).swap(world_);
    observer_handle_ = world->addObserver(
        [this](const World::ObjectConstPtr& object, World::Action action) { notify(object, action); });
  }
}

void WorldDiff::reset()
{
  clearChanges();

  WorldPtr old_world = world_.lock();
  if (old_world)
    old_world->removeObserver(observer_handle_);

  world_.reset();
}

void WorldDiff::reset(const WorldPtr& world)
{
  clearChanges();

  WorldPtr old_world = world_.lock();
  if (old_world)
    old_world->removeObserver(observer_handle_);

  WorldWeakPtr(world).swap(world_);
  observer_handle_ =
      world->addObserver([this](const World::ObjectConstPtr& object, World::Action action) { notify(object, action); });
}

void WorldDiff::setWorld(const WorldPtr& world)
{
  WorldPtr old_world = world_.lock();
  if (old_world)
  {
    old_world->notifyObserverAllObjects(observer_handle_, World::DESTROY);
    old_world->removeObserver(observer_handle_);
  }

  WorldWeakPtr(world).swap(world_);

  observer_handle_ =
      world->addObserver([this](const World::ObjectConstPtr& object, World::Action action) { notify(object, action); });
  world->notifyObserverAllObjects(observer_handle_, World::CREATE | World::ADD_SHAPE);
}

void WorldDiff::clearChanges()
{
  changes_.clear();
}

void WorldDiff::notify(const World::ObjectConstPtr& obj, World::Action action)
{
  World::Action& a = changes_[obj->id_];
  if (action == World::DESTROY)
    a = World::DESTROY;
  else
    a = a | action;
}

}  // end of namespace collision_detection
