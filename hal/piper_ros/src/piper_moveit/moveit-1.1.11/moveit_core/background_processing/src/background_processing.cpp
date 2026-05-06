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

#include <moveit/background_processing/background_processing.h>
#include <ros/console.h>

namespace moveit
{
namespace tools
{
BackgroundProcessing::BackgroundProcessing()
{
  // spin a thread that will process user events
  run_processing_thread_ = true;
  processing_ = false;
  processing_thread_ = std::make_unique<boost::thread>([this] { processingThread(); });
}

BackgroundProcessing::~BackgroundProcessing()
{
  run_processing_thread_ = false;
  new_action_condition_.notify_all();
  processing_thread_->join();
}

void BackgroundProcessing::processingThread()
{
  boost::unique_lock<boost::mutex> ulock(action_lock_);

  while (run_processing_thread_)
  {
    while (actions_.empty() && run_processing_thread_)
      new_action_condition_.wait(ulock);

    while (!actions_.empty())
    {
      JobCallback fn = actions_.front();
      std::string action_name = action_names_.front();
      actions_.pop_front();
      action_names_.pop_front();
      processing_ = true;

      // make sure we are unlocked while we process the event
      action_lock_.unlock();
      try
      {
        ROS_DEBUG_NAMED("background_processing", "Begin executing '%s'", action_name.c_str());
        fn();
        ROS_DEBUG_NAMED("background_processing", "Done executing '%s'", action_name.c_str());
      }
      catch (std::exception& ex)
      {
        ROS_ERROR_NAMED("background_processing", "Exception caught while processing action '%s': %s",
                        action_name.c_str(), ex.what());
      }
      processing_ = false;
      if (queue_change_event_)
        queue_change_event_(COMPLETE, action_name);
      action_lock_.lock();
    }
  }
}

void BackgroundProcessing::addJob(const boost::function<void()>& job, const std::string& name)
{
  {
    boost::mutex::scoped_lock _(action_lock_);
    actions_.push_back(job);
    action_names_.push_back(name);
    new_action_condition_.notify_all();
  }
  if (queue_change_event_)
    queue_change_event_(ADD, name);
}

void BackgroundProcessing::clear()
{
  bool update = false;
  std::deque<std::string> removed;
  {
    boost::mutex::scoped_lock _(action_lock_);
    update = !actions_.empty();
    actions_.clear();
    action_names_.swap(removed);
  }
  if (update && queue_change_event_)
    for (const std::string& it : removed)
      queue_change_event_(REMOVE, it);
}

std::size_t BackgroundProcessing::getJobCount() const
{
  boost::mutex::scoped_lock _(action_lock_);
  return actions_.size() + (processing_ ? 1 : 0);
}

void BackgroundProcessing::setJobUpdateEvent(const JobUpdateCallback& event)
{
  boost::mutex::scoped_lock _(action_lock_);
  queue_change_event_ = event;
}

void BackgroundProcessing::clearJobUpdateEvent()
{
  setJobUpdateEvent(JobUpdateCallback());
}

}  // end of namespace tools
}  // end of namespace moveit
