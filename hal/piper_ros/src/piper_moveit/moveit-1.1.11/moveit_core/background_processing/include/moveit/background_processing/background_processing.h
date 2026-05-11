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

#pragma once

#include <deque>
#include <string>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <memory>

namespace moveit
{
/** \brief This namespace includes classes and functions that are
    helpful in the implementation of other MoveIt components. This is
    not code specific to the functionality provided by MoveIt. */
namespace tools
{
/** \brief This class provides simple API for executing background
    jobs. A queue of jobs is created and the specified jobs are
    executed in order, one at a time. */
class BackgroundProcessing : private boost::noncopyable
{
public:
  /** \brief Events for jobs */
  enum JobEvent
  {
    /// Called when a job is added to the queue
    ADD,
    /// Called when a job is removed from the queue without execution
    REMOVE,
    /// Called when a job is completed (and removed from the queue)
    COMPLETE
  };

  /** \brief The signature for callback triggered when job events take place: the event that took place and the name of
   * the job */
  typedef boost::function<void(JobEvent, const std::string&)> JobUpdateCallback;

  /** \brief The signature for job callbacks */
  typedef boost::function<void()> JobCallback;

  /** \brief Constructor. The background thread is activated automatically. */
  BackgroundProcessing();

  /** \brief Finishes currently executing job, clears the remaining queue. */
  ~BackgroundProcessing();

  /** \brief Add a job to the queue of jobs to execute. A name is also specifies for the job */
  void addJob(const JobCallback& job, const std::string& name);

  /** \brief Get the size of the queue of jobs (includes currently processed job). */
  std::size_t getJobCount() const;

  /** \brief Clear the queue of jobs */
  void clear();

  /** \brief Set the callback to be triggered when events in JobEvent take place */
  void setJobUpdateEvent(const JobUpdateCallback& event);

  /** \brief Clear the callback to be triggered when events in JobEvent take place */
  void clearJobUpdateEvent();

private:
  std::unique_ptr<boost::thread> processing_thread_;
  bool run_processing_thread_;

  mutable boost::mutex action_lock_;
  boost::condition_variable new_action_condition_;
  std::deque<JobCallback> actions_;
  std::deque<std::string> action_names_;

  JobUpdateCallback queue_change_event_;

  bool processing_;

  void processingThread();
};
}  // namespace tools
}  // namespace moveit
