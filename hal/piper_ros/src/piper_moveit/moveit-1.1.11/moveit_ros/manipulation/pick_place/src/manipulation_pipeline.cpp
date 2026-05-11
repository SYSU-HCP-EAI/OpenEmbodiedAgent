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

#include <moveit/pick_place/manipulation_pipeline.h>
#include <ros/console.h>

namespace pick_place
{
ManipulationPipeline::ManipulationPipeline(const std::string& name, unsigned int nthreads)
  : name_(name), nthreads_(nthreads), verbose_(false), stop_processing_(true)
{
  processing_threads_.resize(nthreads, nullptr);
}

ManipulationPipeline::~ManipulationPipeline()
{
  reset();
}

ManipulationPipeline& ManipulationPipeline::addStage(const ManipulationStagePtr& next)
{
  next->setVerbose(verbose_);
  stages_.push_back(next);
  return *this;
}

const ManipulationStagePtr& ManipulationPipeline::getFirstStage() const
{
  if (stages_.empty())
  {
    static const ManipulationStagePtr EMPTY;
    return EMPTY;
  }
  else
    return stages_.front();
}

const ManipulationStagePtr& ManipulationPipeline::getLastStage() const
{
  if (stages_.empty())
  {
    static const ManipulationStagePtr EMPTY;
    return EMPTY;
  }
  else
    return stages_.back();
}

void ManipulationPipeline::reset()
{
  clear();
  stages_.clear();
}

void ManipulationPipeline::setVerbose(bool flag)
{
  verbose_ = flag;
  for (pick_place::ManipulationStagePtr& stage : stages_)
    stage->setVerbose(flag);
}

void ManipulationPipeline::clear()
{
  stop();
  {
    boost::mutex::scoped_lock slock(queue_access_lock_);
    queue_.clear();
  }
  {
    boost::mutex::scoped_lock slock(result_lock_);
    success_.clear();
    failed_.clear();
  }
}

void ManipulationPipeline::start()
{
  stop_processing_ = false;
  empty_queue_threads_ = 0;
  for (pick_place::ManipulationStagePtr& stage : stages_)
    stage->resetStopSignal();
  for (std::size_t i = 0; i < processing_threads_.size(); ++i)
    if (!processing_threads_[i])
      processing_threads_[i] = new boost::thread([this, i] { processingThread(i); });
}

void ManipulationPipeline::signalStop()
{
  for (pick_place::ManipulationStagePtr& stage : stages_)
    stage->signalStop();
  stop_processing_ = true;
  queue_access_cond_.notify_all();
}

void ManipulationPipeline::stop()
{
  signalStop();
  for (boost::thread*& processing_thread : processing_threads_)
    if (processing_thread)
    {
      processing_thread->join();
      delete processing_thread;
      processing_thread = nullptr;
    }
}

void ManipulationPipeline::processingThread(unsigned int index)
{
  ROS_DEBUG_STREAM_NAMED("manipulation", "Start thread " << index << " for '" << name_ << "'");

  while (!stop_processing_)
  {
    bool inc_queue = false;
    boost::unique_lock<boost::mutex> ulock(queue_access_lock_);
    // if the queue is empty, we trigger the corresponding event
    if (queue_.empty() && !stop_processing_ && empty_queue_callback_)
    {
      empty_queue_threads_++;
      inc_queue = true;
      if (empty_queue_threads_ == processing_threads_.size())
        empty_queue_callback_();
    }
    while (queue_.empty() && !stop_processing_)
      queue_access_cond_.wait(ulock);
    while (!stop_processing_ && !queue_.empty())
    {
      ManipulationPlanPtr g = queue_.front();
      queue_.pop_front();
      if (inc_queue)
      {
        empty_queue_threads_--;
        inc_queue = false;
      }

      queue_access_lock_.unlock();
      try
      {
        g->error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        for (std::size_t i = 0; !stop_processing_ && i < stages_.size(); ++i)
        {
          bool res = stages_[i]->evaluate(g);
          g->processing_stage_ = i + 1;
          if (!res)
          {
            boost::mutex::scoped_lock slock(result_lock_);
            failed_.push_back(g);
            ROS_INFO_STREAM_NAMED("manipulation", "Manipulation plan " << g->id_ << " failed at stage '"
                                                                       << stages_[i]->getName() << "' on thread "
                                                                       << index);
            break;
          }
        }
        if (g->error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          g->processing_stage_++;
          {
            boost::mutex::scoped_lock slock(result_lock_);
            success_.push_back(g);
          }
          signalStop();
          ROS_INFO_STREAM_NAMED("manipulation", "Found successful manipulation plan!");
          if (solution_callback_)
            solution_callback_();
        }
      }
      catch (std::exception& ex)
      {
        ROS_ERROR_NAMED("manipulation", "[%s:%u] %s", name_.c_str(), index, ex.what());
      }
      queue_access_lock_.lock();
    }
  }
}

void ManipulationPipeline::push(const ManipulationPlanPtr& plan)
{
  boost::mutex::scoped_lock slock(queue_access_lock_);
  queue_.push_back(plan);
  ROS_INFO_STREAM_NAMED("manipulation",
                        "Added plan for pipeline '" << name_ << "'. Queue is now of size " << queue_.size());
  queue_access_cond_.notify_all();
}

void ManipulationPipeline::reprocessLastFailure()
{
  boost::mutex::scoped_lock slock(queue_access_lock_);
  if (failed_.empty())
    return;
  ManipulationPlanPtr plan = failed_.back();
  failed_.pop_back();
  plan->clear();
  queue_.push_back(plan);
  ROS_INFO_STREAM_NAMED("manipulation", "Re-added last failed plan for pipeline '"
                                            << name_ << "'. Queue is now of size " << queue_.size());
  queue_access_cond_.notify_all();
}
}  // namespace pick_place
