/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <memory>

#include <gtest/gtest.h>

#include "pilz_industrial_motion_planner/pilz_industrial_motion_planner.h"
#include "pilz_industrial_motion_planner/planning_context_loader_ptp.h"
#include "pilz_industrial_motion_planner/planning_exceptions.h"

using namespace pilz_industrial_motion_planner;

TEST(CommandPlannerTestDirect, ExceptionCoverage)
{
  std::shared_ptr<PlanningException> p_ex{ new PlanningException("") };
  std::shared_ptr<ContextLoaderRegistrationException> clr_ex{ new ContextLoaderRegistrationException("") };
}

/**
 *  This test uses pilz_industrial_motion_planner::CommandPlanner directly and
 * is thus seperated from
 * unittest_pilz_industrial_motion_planner.cpp since plugin loading via
 * pluginlib does not allow loading of classes
 * already
 * defined.
 */

/**
 * @brief Check that a exception is thrown if a already loaded
 * PlanningContextLoader is loaded
 *
 * It this point the planning_instance_ has loaded ptp, lin, circ.
 * A additional ptp is loaded which should throw the respective exception.
 */
TEST(CommandPlannerTestDirect, CheckDoubleLoadingException)
{
  /// Registed a found loader
  pilz_industrial_motion_planner::CommandPlanner planner;
  pilz_industrial_motion_planner::PlanningContextLoaderPtr planning_context_loader(
      new pilz_industrial_motion_planner::PlanningContextLoaderPTP());

  EXPECT_NO_THROW(planner.registerContextLoader(planning_context_loader));

  EXPECT_THROW(planner.registerContextLoader(planning_context_loader),
               pilz_industrial_motion_planner::ContextLoaderRegistrationException);
}

/**
 * @brief Check that getPlanningContext() fails if the underlying ContextLoader
 * fails to load the context.
 */
TEST(CommandPlannerTestDirect, FailOnLoadContext)
{
  pilz_industrial_motion_planner::CommandPlanner planner;

  // Mock of failing PlanningContextLoader
  class TestPlanningContextLoader : public pilz_industrial_motion_planner::PlanningContextLoader
  {
  public:
    std::string getAlgorithm() const override
    {
      return "Test_Algorithm";
    }

    bool loadContext(planning_interface::PlanningContextPtr& /* planning_context */, const std::string& /* name */,
                     const std::string& /* group */) const override
    {
      // Mock behaviour: Cannot load planning context.
      return false;
    }
  };

  /// Registed a found loader
  pilz_industrial_motion_planner::PlanningContextLoaderPtr planning_context_loader(new TestPlanningContextLoader());
  planner.registerContextLoader(planning_context_loader);

  moveit_msgs::MotionPlanRequest req;
  req.planner_id = "Test_Algorithm";

  moveit_msgs::MoveItErrorCodes error_code;
  EXPECT_FALSE(planner.getPlanningContext(nullptr, req, error_code));
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::PLANNING_FAILED, error_code.val);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
