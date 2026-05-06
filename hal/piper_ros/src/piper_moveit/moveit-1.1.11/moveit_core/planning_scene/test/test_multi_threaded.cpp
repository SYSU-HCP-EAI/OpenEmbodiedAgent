/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <gtest/gtest.h>
#include <thread>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_plugin_cache.h>

const int TRIALS = 1000;
const int THREADS = 2;

bool runCollisionDetection(unsigned int /*id*/, unsigned int trials, const planning_scene::PlanningScene& scene,
                           const moveit::core::RobotState& state)
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  for (unsigned int i = 0; i < trials; ++i)
  {
    res.clear();
    scene.checkCollision(req, res, state);
  }
  return res.collision;
}

void runCollisionDetectionAssert(unsigned int id, unsigned int trials, const planning_scene::PlanningScene& scene,
                                 const moveit::core::RobotState& state, bool expected_result)
{
  ASSERT_EQ(expected_result, runCollisionDetection(id, trials, scene, state));
}

class CollisionDetectorTests : public testing::TestWithParam<const char*>
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    ASSERT_TRUE(static_cast<bool>(robot_model_));

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  }

  void TearDown() override
  {
  }

  bool robot_model_ok_;

  moveit::core::RobotModelPtr robot_model_;

  collision_detection::CollisionEnvPtr cenv_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  moveit::core::RobotStatePtr robot_state_;

  planning_scene::PlanningScenePtr planning_scene_;
};

/** \brief Tests the collision detector in multiple threads. */
TEST_P(CollisionDetectorTests, Threaded)
{
  const std::string plugin_name = GetParam();
  SCOPED_TRACE(plugin_name);

  std::vector<moveit::core::RobotState> states;
  std::vector<std::thread> threads;
  std::vector<bool> collisions;

  collision_detection::CollisionPluginCache loader;
  if (!loader.activate(plugin_name, planning_scene_, true))
  {
#if defined(GTEST_SKIP_)
    GTEST_SKIP_("Failed to load collision plugin");
#else
    return;
#endif
  }

  for (unsigned int i = 0; i < THREADS; ++i)
  {
    moveit::core::RobotState state{ planning_scene_->getRobotModel() };
    collision_detection::CollisionRequest req;
    state.setToRandomPositions();
    state.update();
    collisions.push_back(runCollisionDetection(0, 1, *planning_scene_, state));
    states.push_back(std::move(state));
  }

  for (unsigned int i = 0; i < THREADS; ++i)
    threads.emplace_back(std::thread([i, &scene = *planning_scene_, &state = states[i], expected = collisions[i]] {
      return runCollisionDetectionAssert(i, TRIALS, scene, state, expected);
    }));

  for (unsigned int i = 0; i < states.size(); ++i)
  {
    threads[i].join();
  }
  threads.clear();

  planning_scene_.reset();
}

#ifndef INSTANTIATE_TEST_SUITE_P  // prior to gtest 1.10
#define INSTANTIATE_TEST_SUITE_P(...) INSTANTIATE_TEST_CASE_P(__VA_ARGS__)
#endif

// instantiate parameterized tests for common collision plugins
INSTANTIATE_TEST_SUITE_P(PluginTests, CollisionDetectorTests, testing::Values("FCL", "Bullet"));

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
