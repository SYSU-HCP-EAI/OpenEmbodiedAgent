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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/thread.hpp>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void runCollisionDetection(unsigned int id, unsigned int trials, const planning_scene::PlanningScene& scene,
                           const moveit::core::RobotState& state)
{
  ROS_INFO("Starting thread %u", id);
  collision_detection::CollisionRequest req;
  ros::WallTime start = ros::WallTime::now();
  for (unsigned int i = 0; i < trials; ++i)
  {
    collision_detection::CollisionResult res;
    scene.checkCollision(req, res, state);
  }
  double duration = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Thread %u performed %lf collision checks per second", id, (double)trials / duration);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "evaluate_collision_checking_speed");

  unsigned int nthreads = 2;
  unsigned int trials = 10000;
  boost::program_options::options_description desc;
  desc.add_options()("nthreads", boost::program_options::value<unsigned int>(&nthreads)->default_value(nthreads),
                     "Number of threads to use")(
      "trials", boost::program_options::value<unsigned int>(&trials)->default_value(trials),
      "Number of collision checks to perform with each thread")("wait",
                                                                "Wait for a user command (so the planning scene can be "
                                                                "updated in thre background)")("help", "this screen");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 0;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (psm.getPlanningScene())
  {
    if (vm.count("wait"))
    {
      psm.startWorldGeometryMonitor();
      psm.startSceneMonitor();
      std::cout << "Listening to planning scene updates. Press Enter to continue ..." << std::endl;
      std::cin.get();
    }
    else
      ros::Duration(0.5).sleep();

    std::vector<moveit::core::RobotStatePtr> states;
    ROS_INFO("Sampling %u valid states...", nthreads);
    for (unsigned int i = 0; i < nthreads; ++i)
    {
      // sample a valid state
      moveit::core::RobotState* state = new moveit::core::RobotState(psm.getPlanningScene()->getRobotModel());
      collision_detection::CollisionRequest req;
      do
      {
        state->setToRandomPositions();
        state->update();
        collision_detection::CollisionResult res;
        psm.getPlanningScene()->checkCollision(req, res);
        if (!res.collision)
          break;
      } while (true);
      states.push_back(moveit::core::RobotStatePtr(state));
    }

    std::vector<boost::thread*> threads;

    for (unsigned int i = 0; i < states.size(); ++i)
      threads.push_back(new boost::thread([i, trials, &scene = *psm.getPlanningScene(), &state = *states[i]] {
        return runCollisionDetection(i, trials, scene, state);
      }));

    for (unsigned int i = 0; i < states.size(); ++i)
    {
      threads[i]->join();
      delete threads[i];
    }
  }
  else
    ROS_ERROR("Planning scene not configured");

  return 0;
}
