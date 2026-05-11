/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Mark Moll */

#include <chrono>
#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

namespace po = boost::program_options;

/** Benchmark program measuring time to solve inverse kinematics of robot described in robot_description */
int main(int argc, char* argv[])
{
  std::string group;
  std::string tip;
  unsigned int num;
  bool reset_to_default;
  po::options_description desc("Options");
  // clang-format off
  desc.add_options()
      ("help", "show help message")
      ("group", po::value<std::string>(&group)->default_value("all"), "name of planning group")
      ("tip", po::value<std::string>(&tip)->default_value("default"), "name of the end effector in the planning group")
      ("num", po::value<unsigned int>(&num)->default_value(100000), "number of IK solutions to compute")
      ("reset_to_default", po::value<bool>(&reset_to_default)->default_value(true),
       "whether to reset IK seed to default state. If set to false, the seed is the "
       "correct IK solution (to accelerate filling the cache).");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help") != 0u)
  {
    std::cout << desc << "\n";
    return 1;
  }

  ros::init(argc, argv, "benchmark_ik");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader;
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  moveit::core::RobotState& kinematic_state = planning_scene.getCurrentStateNonConst();
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  std::chrono::duration<double> ik_time(0);
  std::chrono::time_point<std::chrono::system_clock> start;
  std::vector<moveit::core::JointModelGroup*> groups;
  std::vector<std::string> end_effectors;

  if (group == "all")
    groups = kinematic_model->getJointModelGroups();
  else
    groups.push_back(kinematic_model->getJointModelGroup(group));

  for (const auto& group : groups)
  {
    // skip group if there's no IK solver
    if (group->getSolverInstance() == nullptr)
      continue;

    if (tip == "default")
      group->getEndEffectorTips(end_effectors);
    else
      end_effectors = std::vector<std::string>(1, tip);

    // perform first IK call to load the cache, so that the time for loading is not included in
    // average IK call time
    kinematic_state.setToDefaultValues();
    EigenSTL::vector_Isometry3d default_eef_states;
    for (const auto& end_effector : end_effectors)
      default_eef_states.push_back(kinematic_state.getGlobalLinkTransform(end_effector));
    if (end_effectors.size() == 1)
      kinematic_state.setFromIK(group, default_eef_states[0], end_effectors[0], 0.1);
    else
      kinematic_state.setFromIK(group, default_eef_states, end_effectors, 0.1);

    bool found_ik;
    unsigned int num_failed_calls = 0, num_self_collisions = 0;
    EigenSTL::vector_Isometry3d end_effector_states(end_effectors.size());
    unsigned int i = 0;
    while (i < num)
    {
      kinematic_state.setToRandomPositions(group);
      collision_result.clear();
      planning_scene.checkSelfCollision(collision_request, collision_result);
      if (collision_result.collision)
      {
        ++num_self_collisions;
        continue;
      }
      for (unsigned j = 0; j < end_effectors.size(); ++j)
        end_effector_states[j] = kinematic_state.getGlobalLinkTransform(end_effectors[j]);
      if (reset_to_default)
        kinematic_state.setToDefaultValues();
      start = std::chrono::system_clock::now();
      if (end_effectors.size() == 1)
        found_ik = kinematic_state.setFromIK(group, end_effector_states[0], end_effectors[0], 0.1);
      else
        found_ik = kinematic_state.setFromIK(group, end_effector_states, end_effectors, 0.1);
      ik_time += std::chrono::system_clock::now() - start;
      if (!found_ik)
        num_failed_calls++;
      ++i;
      if (i % 100 == 0)
        ROS_INFO_NAMED("cached_ik.measure_ik_call_cost",
                       "Avg. time per IK solver call is %g after %d calls. %g%% of calls failed to return a solution. "
                       "%g%% of random joint configurations were ignored due to self-collisions.",
                       ik_time.count() / (double)i, i, 100. * num_failed_calls / i,
                       100. * num_self_collisions / (num_self_collisions + i));
    }
    ROS_INFO_NAMED("cached_ik.measure_ik_call_cost", "Summary for group %s: %g %g %g", group->getName().c_str(),
                   ik_time.count() / (double)i, 100. * num_failed_calls / i,
                   100. * num_self_collisions / (num_self_collisions + i));
  }

  ros::shutdown();
  return 0;
}
