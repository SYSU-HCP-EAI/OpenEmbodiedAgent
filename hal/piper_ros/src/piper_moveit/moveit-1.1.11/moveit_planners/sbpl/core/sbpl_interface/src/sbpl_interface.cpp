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

#include <sbpl_interface/sbpl_interface.h>
#include <planning_models/conversions.h>
//#include <valgrind/callgrind.h>

namespace sbpl_interface
{
bool SBPLInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res,
                          const PlanningParameters& params) const
{
  res.trajectory.joint_trajectory.points.clear();
  (const_cast<SBPLInterface*>(this))->last_planning_statistics_ = PlanningStatistics();
  planning_models::RobotState* start_state(planning_scene->getCurrentState());
  planning_models::robotStateMsgToRobotState(*planning_scene->getTransforms(), req.motion_plan_request.start_state,
                                             start_state);

  ros::WallTime wt = ros::WallTime::now();
  boost::shared_ptr<EnvironmentChain3D> env_chain(new EnvironmentChain3D(planning_scene));
  if (!env_chain->setupForMotionPlan(planning_scene, req, res, params))
  {
    // std::cerr << "Env chain setup failing" << std::endl;
    return false;
  }
  // std::cerr << "Creation with params " << params.use_bfs_ << " took " << (ros::WallTime::now()-wt).toSec() <<
  // std::endl;
  boost::this_thread::interruption_point();

  // DummyEnvironment* dummy_env = new DummyEnvironment();
  boost::shared_ptr<ARAPlanner> planner(new ARAPlanner(env_chain.get(), true));
  planner->set_initialsolution_eps(100.0);
  planner->set_search_mode(true);
  planner->force_planning_from_scratch();
  planner->set_start(env_chain->getPlanningData().start_hash_entry_->stateID);
  planner->set_goal(env_chain->getPlanningData().goal_hash_entry_->stateID);
  // std::cerr << "Creation took " << (ros::WallTime::now()-wt) << std::endl;
  std::vector<int> solution_state_ids;
  int solution_cost;
  wt = ros::WallTime::now();
  // CALLGRIND_START_INSTRUMENTATION;
  bool b_ret = planner->replan(10.0, &solution_state_ids, &solution_cost);
  // CALLGRIND_STOP_INSTRUMENTATION;
  double el = (ros::WallTime::now() - wt).toSec();
  std::cerr << "B ret is " << b_ret << " planning time " << el << std::endl;
  std::cerr << "Expansions " << env_chain->getPlanningStatistics().total_expansions_ << " average time "
            << (env_chain->getPlanningStatistics().total_expansion_time_.toSec() /
                (env_chain->getPlanningStatistics().total_expansions_ * 1.0))
            << " hz "
            << 1.0 / (env_chain->getPlanningStatistics().total_expansion_time_.toSec() /
                      (env_chain->getPlanningStatistics().total_expansions_ * 1.0))
            << std::endl;
  std::cerr << "Total coll checks " << env_chain->getPlanningStatistics().coll_checks_ << " hz "
            << 1.0 / (env_chain->getPlanningStatistics().total_coll_check_time_.toSec() /
                      (env_chain->getPlanningStatistics().coll_checks_ * 1.0))
            << std::endl;
  std::cerr << "Path length is " << solution_state_ids.size() << std::endl;
  if (!b_ret)
  {
    res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
  if (solution_state_ids.size() == 0)
  {
    std::cerr << "Success but no path" << std::endl;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }
  if (!env_chain->populateTrajectoryFromStateIDSequence(solution_state_ids, res.trajectory.joint_trajectory))
  {
    std::cerr << "Success but path bad" << std::endl;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }
  ros::WallTime pre_short = ros::WallTime::now();
  // std::cerr << "Num traj points before " << res.trajectory.joint_trajectory.points.size() << std::endl;
  trajectory_msgs::JointTrajectory traj = res.trajectory.joint_trajectory;
  env_chain->attemptShortcut(traj, res.trajectory.joint_trajectory);
  // std::cerr << "Num traj points after " << res.trajectory.joint_trajectory.points.size() << std::endl;
  // std::cerr << "Time " << (ros::WallTime::now()-pre_short).toSec() << std::endl;
  // env_chain->getPlaneBFSMarker(mark, env_chain->getGoalPose().translation().z());
  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  PlanningStatistics stats = env_chain->getPlanningStatistics();
  stats.total_planning_time_ = ros::WallDuration(el);
  (const_cast<SBPLInterface*>(this))->last_planning_statistics_ = stats;
  return true;
}
}  // namespace sbpl_interface
