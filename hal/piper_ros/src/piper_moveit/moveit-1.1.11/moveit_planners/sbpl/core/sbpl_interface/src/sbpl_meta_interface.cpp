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

#include <sbpl_interface/sbpl_meta_interface.h>
#include <sbpl_interface/sbpl_interface.h>
#include <planning_models/conversions.h>

namespace sbpl_interface
{
SBPLMetaInterface::SBPLMetaInterface(const planning_models::RobotModelConstPtr& robot_model)
{
  sbpl_interface_first_ = std::make_shared<sbpl_interface::SBPLInterface>(robot_model);
  sbpl_interface_second_ = std::make_shared<sbpl_interface::SBPLInterface>(robot_model);
}

bool SBPLMetaInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                              const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res)
{
  first_ok_ = false;
  first_done_ = false;
  second_ok_ = false;
  second_done_ = false;

  PlanningParameters param_bfs;
  param_bfs.use_bfs_ = true;
  PlanningParameters param_no_bfs;
  param_no_bfs.use_bfs_ = false;
  moveit_msgs::GetMotionPlan::Response res1, res2;
  boost::thread thread1(
      [this, &planning_scene, &req, &res1, param_bfs] { runSolver(true, planning_scene, req, res1, param_bfs); });
  boost::thread thread2(
      [this, &planning_scene, &req, &res1, param_bfs] { runSolver(false, planning_scene, req, res2, param_no_bfs); });
  boost::mutex::scoped_lock lock(planner_done_mutex_);
  planner_done_condition_.wait(lock);

  if (first_done_)
  {
    std::cerr << "FIRST DONE" << std::endl;
    if (first_ok_)
    {
      std::cerr << "First ok, interrupting second" << std::endl;
      if (!second_done_)
      {
        thread2.interrupt();
        thread2.join();
      }
    }
    else
    {
      if (!second_done_)
      {
        planner_done_condition_.wait(lock);
      }
    }
  }
  if (second_done_)
  {
    std::cerr << "Second done" << std::endl;
    if (second_ok_)
    {
      std::cerr << "Second ok, interrupting first" << std::endl;
      if (!first_done_)
      {
        thread1.interrupt();
        thread1.join();
      }
    }
    else
    {
      if (!first_done_)
      {
        planner_done_condition_.wait(lock);
      }
    }
  }

  if (!first_ok_ && !second_ok_)
  {
    std::cerr << "Both planners failed" << std::endl;
    res = res1;
    return false;
  }
  if (!first_ok_ && second_ok_)
  {
    std::cerr << "Sbpl interface no bfs reports time "
              << sbpl_interface_second_->getLastPlanningStatistics().total_planning_time_ << std::endl;
    last_planning_statistics_ = sbpl_interface_second_->getLastPlanningStatistics();
    res = res2;
    return true;
  }
  else if (first_ok_ && !second_ok_)
  {
    std::cerr << "Sbpl interface bfs reports time "
              << sbpl_interface_first_->getLastPlanningStatistics().total_planning_time_ << std::endl;
    last_planning_statistics_ = sbpl_interface_first_->getLastPlanningStatistics();
    res = res1;
    return true;
  }
  std::cerr << "Sbpl interface bfs reports time "
            << sbpl_interface_first_->getLastPlanningStatistics().total_planning_time_ << std::endl;
  std::cerr << "Sbpl interface no bfs reports time "
            << sbpl_interface_second_->getLastPlanningStatistics().total_planning_time_ << std::endl;
  if (sbpl_interface_first_->getLastPlanningStatistics().total_planning_time_ <
      sbpl_interface_second_->getLastPlanningStatistics().total_planning_time_)
  {
    last_planning_statistics_ = sbpl_interface_first_->getLastPlanningStatistics();
    res = res1;
  }
  else
  {
    last_planning_statistics_ = sbpl_interface_second_->getLastPlanningStatistics();
    res = res2;
  }
  return true;
}

void SBPLMetaInterface::runSolver(bool use_first, const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const moveit_msgs::GetMotionPlan::Request& req,
                                  moveit_msgs::GetMotionPlan::Response& res, const PlanningParameters& params)
{
  try
  {
    if (use_first)
    {
      std::cerr << "Running first planner" << std::endl;
      first_ok_ = sbpl_interface_first_->solve(planning_scene, req, res, params);
      first_done_ = true;
    }
    else
    {
      std::cerr << "Running second planner" << std::endl;
      second_ok_ = sbpl_interface_second_->solve(planning_scene, req, res, params);
      second_done_ = true;
    }
    planner_done_condition_.notify_all();
  }
  catch (...)
  {
    std::cerr << "Interruption requested\n";
  }
}
}  // namespace sbpl_interface
