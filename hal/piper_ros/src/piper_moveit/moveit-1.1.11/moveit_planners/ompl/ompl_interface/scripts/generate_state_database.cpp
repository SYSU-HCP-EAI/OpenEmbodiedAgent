/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  Copyright (c) 2018, Michael 'v4hn' Goerner
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

/* Authors: Ioan Sucan, Michael Goerner */

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/detail/constraints_library.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/profiler/profiler.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>

#include <boost/math/constants/constants.hpp>
#include <sstream>

constexpr char LOGNAME[] = "generate_state_database";

static const std::string ROBOT_DESCRIPTION = "robot_description";

static const std::string CONSTRAINT_PARAMETER = "constraints";

struct GenerateStateDatabaseParameters
{
  bool setFromHandle(ros::NodeHandle& nh)
  {
    use_current_scene = nh.param("use_current_scene", false);

    // number of states in joint space approximation
    construction_opts.samples = nh.param("state_cnt", 10000);

    // generate edges together with states?
    construction_opts.edges_per_sample = nh.param("edges_per_sample", 0);
    construction_opts.max_edge_length = nh.param("max_edge_length", 0.2);

    // verify constraint validity on edges
    construction_opts.explicit_motions = nh.param("explicit_motions", true);
    construction_opts.explicit_points_resolution = nh.param("explicit_points_resolution", 0.05);
    construction_opts.max_explicit_points = nh.param("max_explicit_points", 200);

    // local planning in JointModel state space
    construction_opts.state_space_parameterization =
        nh.param<std::string>("state_space_parameterization", "JointModel");

    output_folder = nh.param<std::string>("output_folder", "constraint_approximations_database");

    if (!nh.getParam("planning_group", planning_group))
    {
      ROS_FATAL_NAMED(LOGNAME, "~planning_group parameter has to be specified.");
      return false;
    }

    XmlRpc::XmlRpcValue constraint_description;
    if (!nh.getParam(CONSTRAINT_PARAMETER, constraint_description) ||
        !kinematic_constraints::constructConstraints(constraint_description, constraints))
    {
      ROS_FATAL_STREAM_NAMED(LOGNAME,
                             "Could not find valid constraint description in parameter '"
                                 << nh.resolveName(CONSTRAINT_PARAMETER)
                                 << "'. "
                                    "Please upload correct correct constraint description or remap the parameter.");
      return false;
    }

    return true;
  };

  std::string planning_group;

  // path to folder for generated database
  std::string output_folder;

  // request the current scene via get_planning_scene service
  bool use_current_scene;

  // constraints the approximation should satisfy
  moveit_msgs::Constraints constraints;

  // internal parameters of approximation generator
  ompl_interface::ConstraintApproximationConstructionOptions construction_opts;
};

void computeDB(const planning_scene::PlanningScenePtr& scene, struct GenerateStateDatabaseParameters& params)
{
  // required by addConstraintApproximation
  scene->getCurrentStateNonConst().update();

  ompl_interface::OMPLInterface ompl_interface(scene->getRobotModel());
  planning_interface::MotionPlanRequest req;
  req.group_name = params.planning_group;
  req.path_constraints = params.constraints;
  moveit::core::robotStateToRobotStateMsg(scene->getCurrentState(), req.start_state);
  req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
      scene->getCurrentState(), scene->getRobotModel()->getJointModelGroup(params.planning_group)));

  ompl_interface::ModelBasedPlanningContextPtr context = ompl_interface.getPlanningContext(scene, req);

  ROS_INFO_STREAM_NAMED(LOGNAME, "Generating Joint Space Constraint Approximation Database for constraint:\n"
                                     << params.constraints);

  ompl_interface::ConstraintApproximationConstructionResults result =
      context->getConstraintsLibraryNonConst()->addConstraintApproximation(params.constraints, params.planning_group,
                                                                           scene, params.construction_opts);

  if (!result.approx)
  {
    ROS_FATAL_NAMED(LOGNAME, "Failed to generate approximation.");
    return;
  }
  context->getConstraintsLibraryNonConst()->saveConstraintApproximations(params.output_folder);
  ROS_INFO_STREAM_NAMED(LOGNAME,
                        "Successfully generated Joint Space Constraint Approximation Database for constraint:\n"
                            << params.constraints);
  ROS_INFO_STREAM_NAMED(LOGNAME, "The database has been saved in your local folder '" << params.output_folder << "'");
}

/**
 * Generates a database of states that follow the given constraints.
 * An example of the constraint yaml that should be loaded to rosparam:
 * "name: tool0_upright
 *  constraints:
 *  - type: orientation
 *    frame_id: world
 *    link_name: tool0
 *    orientation: [0, 0, 0]
 *    tolerances: [0.01, 0.01, 3.15]
 *    weight: 1.0
 * "
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "construct_tool_constraint_database", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");

  GenerateStateDatabaseParameters params;
  if (!params.setFromHandle(nh))
    return 1;

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getRobotModel())
    return 1;

  if (params.use_current_scene)
  {
    ROS_INFO_NAMED(LOGNAME, "Requesting current planning scene to generate database");
    if (!psm.requestPlanningSceneState())
    {
      ROS_FATAL_NAMED(LOGNAME, "Abort. The current scene could not be retrieved.");
      return 1;
    }
  }

  if (moveit::core::isEmpty(params.constraints))
  {
    ROS_FATAL_NAMED(LOGNAME, "Abort. Constraint description is an empty set of constraints.");
    return 1;
  }

  computeDB(psm.getPlanningScene(), params);

  return 0;
}
