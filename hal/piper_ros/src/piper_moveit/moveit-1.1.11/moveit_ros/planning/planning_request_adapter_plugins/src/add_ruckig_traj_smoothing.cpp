/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <class_loader/class_loader.hpp>

namespace default_planner_request_adapters
{
using namespace trajectory_processing;

const std::string LOGGER = "moveit_ros.add_traj_smoothing";

/** @brief This adapter uses the time-optimal trajectory generation method */
class AddRuckigTrajectorySmoothing : public planning_request_adapter::PlanningRequestAdapter
{
public:
  AddRuckigTrajectorySmoothing() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& /*node_handle*/) override
  {
  }

  std::string getDescription() const override
  {
    return "Add Ruckig trajectory smoothing.";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      if (!smoother_.applySmoothing(*res.trajectory_, req.max_velocity_scaling_factor,
                                    req.max_acceleration_scaling_factor))
      {
        ROS_WARN_NAMED(LOGGER, " Trajectory smoothing for the solution path failed.");
        result = false;
      }
    }

    return result;
  }

private:
  RuckigSmoothing smoother_;
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddRuckigTrajectorySmoothing,
                            planning_request_adapter::PlanningRequestAdapter)
