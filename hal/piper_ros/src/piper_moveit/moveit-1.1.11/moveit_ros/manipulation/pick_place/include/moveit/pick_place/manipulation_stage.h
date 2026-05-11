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

/* Author: Ioan Sucan, Sachin Chitta */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/pick_place/manipulation_plan.h>
#include <string>

namespace pick_place
{
MOVEIT_CLASS_FORWARD(ManipulationStage);  // Defines ManipulationStagePtr, ConstPtr, WeakPtr... etc

class ManipulationStage
{
public:
  ManipulationStage(const std::string& name) : name_(name), signal_stop_(false), verbose_(false)
  {
  }

  virtual ~ManipulationStage()
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

  void setVerbose(bool flag)
  {
    verbose_ = flag;
  }

  virtual void resetStopSignal()
  {
    signal_stop_ = false;
  }

  virtual void signalStop()
  {
    signal_stop_ = true;
  }

  virtual bool evaluate(const ManipulationPlanPtr& plan) const = 0;

protected:
  std::string name_;
  bool signal_stop_;
  bool verbose_;
};
}  // namespace pick_place
