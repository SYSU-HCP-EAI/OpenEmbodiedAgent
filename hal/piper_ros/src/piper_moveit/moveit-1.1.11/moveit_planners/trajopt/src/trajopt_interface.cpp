/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Omid Heidari */

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <trajopt_sco/sco_common.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/solver_interface.hpp>

#include <trajopt/trajectory_costs.hpp>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include <trajopt_utils/eigen_conversions.hpp>

#include "trajopt_interface/trajopt_interface.h"
#include "trajopt_interface/problem_description.h"

namespace trajopt_interface
{
TrajOptInterface::TrajOptInterface(const ros::NodeHandle& nh) : nh_(nh), name_("TrajOptInterface")
{
  trajopt_problem_ = TrajOptProblemPtr(new TrajOptProblem);
  setDefaultTrajOPtParams();

  // TODO: callbacks should be defined by the user
  optimizer_callbacks_.push_back(callBackFunc);
}

bool TrajOptInterface::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             const planning_interface::MotionPlanRequest& req,
                             moveit_msgs::MotionPlanDetailedResponse& res)
{
  ROS_INFO(" ======================================= From trajopt_interface, solve is called");
  setTrajOptParams(params_);

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED(name_, "No planning scene initialized.");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return false;
  }

  ROS_INFO(" ======================================= Extract current state information");
  ros::WallTime start_time = ros::WallTime::now();
  moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
  bool robot_model_ok = static_cast<bool>(robot_model);
  if (!robot_model_ok)
    ROS_ERROR_STREAM_NAMED(name_, "robot model is not loaded properly");
  moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(robot_model));
  *current_state = planning_scene->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup(req.group_name);
  if (joint_model_group == nullptr)
    ROS_ERROR_STREAM_NAMED(name_, "joint model group is empty");
  std::vector<std::string> group_joint_names = joint_model_group->getActiveJointModelNames();
  int dof = group_joint_names.size();
  std::vector<double> current_joint_values;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

  // Current state is different from star state in general
  ROS_INFO(" ======================================= Extract start state infromation");
  trajopt::DblVec start_joint_values = extractStartJointValues(req, group_joint_names);

  // check the start state for being empty or joint limit violiation
  if (start_joint_values.empty())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Start_state is empty");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  if (not joint_model_group->satisfiesPositionBounds(start_joint_values.data()))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Start state violates joint limits");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  ROS_INFO(" ======================================= Create ProblemInfo");
  ProblemInfo problem_info(planning_scene, req.group_name);

  setProblemInfoParam(problem_info);

  ROS_INFO(" ======================================= Populate init info, hard-coded");
  // TODO: init info should be defined by user. To this end, we need to add seed trajectories to MotionPlanRequest.
  // JOINT_INTERPOLATED: data is the current joint values
  // GIVEN_TRAJ: data is the joint values of the current state copied to all timesteps
  Eigen::VectorXd current_joint_values_eigen(dof);
  for (int joint_index = 0; joint_index < dof; ++joint_index)
  {
    current_joint_values_eigen(joint_index) = current_joint_values[joint_index];
  }

  if (problem_info.init_info.type == InitInfo::JOINT_INTERPOLATED)
  {
    problem_info.init_info.data = current_joint_values_eigen;
  }
  else if (problem_info.init_info.type == InitInfo::GIVEN_TRAJ)
  {
    problem_info.init_info.data = current_joint_values_eigen.transpose().replicate(problem_info.basic_info.n_steps, 1);
  }

  ROS_INFO(" ======================================= Create Constraints");
  if (req.goal_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "No goal constraints specified!");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  ROS_INFO(" ======================================= Cartesian Constraints");
  if (!req.goal_constraints[0].position_constraints.empty() && !req.goal_constraints[0].orientation_constraints.empty())
  {
    CartPoseTermInfoPtr cart_goal_pos(new CartPoseTermInfo);

    // TODO: Feed cart_goal_pos with request information and the needed param to the setup.yaml file
    // TODO: Multiple Cartesian constraints

    // Add the constraint to problem_info
    problem_info.cnt_infos.push_back(cart_goal_pos);
  }
  else if (req.goal_constraints[0].position_constraints.empty() &&
           !req.goal_constraints[0].orientation_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "position constraint is not defined");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }
  else if (!req.goal_constraints[0].position_constraints.empty() &&
           req.goal_constraints[0].orientation_constraints.empty())
  {
    ROS_ERROR_STREAM_NAMED("trajopt_planner", "orientation constraint is not defined");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  ROS_INFO(" ======================================= Constraints from request goal_constraints");
  for (auto goal_cnt : req.goal_constraints)
  {
    JointPoseTermInfoPtr joint_pos_term(new JointPoseTermInfo);
    // When using MotionPlanning Display in RViz, the created request has no name for the constriant
    setJointPoseTermInfoParams(joint_pos_term, (goal_cnt.name != "") ? goal_cnt.name : "goal_tmp");

    trajopt::DblVec joint_goal_constraints;
    for (const moveit_msgs::JointConstraint& joint_goal_constraint : goal_cnt.joint_constraints)
    {
      joint_goal_constraints.push_back(joint_goal_constraint.position);
    }
    joint_pos_term->targets = joint_goal_constraints;
    problem_info.cnt_infos.push_back(joint_pos_term);

    // needed to initialize trajectory
    problem_info.init_info.data = util::toVectorXd(joint_goal_constraints);
  }

  ROS_INFO(" ======================================= Constraints from request start_state");
  // add the start pos from request as a constraint
  JointPoseTermInfoPtr joint_start_pos(new JointPoseTermInfo);

  joint_start_pos->targets = start_joint_values;
  setJointPoseTermInfoParams(joint_start_pos, "start_pos");
  problem_info.cnt_infos.push_back(joint_start_pos);

  ROS_INFO(" ======================================= Velocity Constraints, hard-coded");
  // TODO: should be defined by user, its parametes should be added to setup.yaml
  JointVelTermInfoPtr joint_vel(new JointVelTermInfo);

  joint_vel->coeffs = std::vector<double>(dof, 5.0);
  joint_vel->targets = std::vector<double>(dof, 0.0);
  joint_vel->first_step = 0;
  joint_vel->last_step = problem_info.basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = trajopt_interface::TT_COST;
  problem_info.cost_infos.push_back(joint_vel);

  ROS_INFO(" ======================================= Visibility Constraints");
  if (!req.goal_constraints[0].visibility_constraints.empty())
  {
    // TODO: Add visibility constraint
  }

  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.improve_ratio_threshold: " << params_.improve_ratio_threshold);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.min_trust_box_size: " << params_.min_trust_box_size);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.min_approx_improve: " << params_.min_approx_improve);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.params_.min_approx_improve_frac: " << params_.min_approx_improve_frac);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.max_iter: " << params_.max_iter);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.trust_shrink_ratio: " << params_.trust_shrink_ratio);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.trust_expand_ratio: " << params_.trust_expand_ratio);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.cnt_tolerance: " << params_.cnt_tolerance);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.max_merit_coeff_increases: " << params_.max_merit_coeff_increases);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.merit_coeff_increase_ratio: " << params_.merit_coeff_increase_ratio);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.max_time: " << params_.max_time);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.initial_merit_error_coeff: " << params_.initial_merit_error_coeff);
  ROS_DEBUG_STREAM_NAMED(name_, "trajopt_param.trust_box_size: " << params_.trust_box_size);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.n_steps: " << problem_info.basic_info.n_steps);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.dt_upper_lim: " << problem_info.basic_info.dt_upper_lim);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.dt_lower_lim: " << problem_info.basic_info.dt_lower_lim);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.start_fixed: " << problem_info.basic_info.start_fixed);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.use_time: " << problem_info.basic_info.use_time);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.convex_solver: " << problem_info.basic_info.convex_solver);

  std::string problem_info_type;
  switch (problem_info.init_info.type)
  {
    case InitInfo::STATIONARY:
      problem_info_type = "STATIONARY";
      break;
    case InitInfo::JOINT_INTERPOLATED:
      problem_info_type = "JOINT_INTERPOLATED";
      break;
    case InitInfo::GIVEN_TRAJ:
      problem_info_type = "GIVEN_TRAJ";
      break;
  }
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.type: " << problem_info_type);
  ROS_DEBUG_STREAM_NAMED(name_, "problem_info.basic_info.dt: " << problem_info.init_info.dt);

  ROS_INFO(" ======================================= Construct problem");
  trajopt_problem_ = ConstructProblem(problem_info);

  ROS_INFO_STREAM_NAMED("num_cost", trajopt_problem_->getNumCosts());
  ROS_INFO_STREAM_NAMED("num_constraints", trajopt_problem_->getNumConstraints());

  ROS_INFO(" ======================================= TrajOpt Optimization");
  sco::BasicTrustRegionSQP opt(trajopt_problem_);

  opt.setParameters(params_);
  opt.initialize(trajopt::trajToDblVec(trajopt_problem_->GetInitTraj()));

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : optimizer_callbacks_)
  {
    opt.addCallback(callback);
  }

  // Optimize
  ros::WallTime create_time = ros::WallTime::now();
  opt.optimize();

  ROS_INFO(" ======================================= TrajOpt Solution");
  trajopt::TrajArray opt_solution = trajopt::getTraj(opt.x(), trajopt_problem_->GetVars());

  // assume that the trajectory is now optimized, fill in the output structure:
  ROS_INFO_STREAM_NAMED("num_rows", opt_solution.rows());
  ROS_INFO_STREAM_NAMED("num_cols", opt_solution.cols());

  res.trajectory.resize(1);
  res.trajectory[0].joint_trajectory.joint_names = group_joint_names;
  res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;

  // fill in the entire trajectory
  res.trajectory[0].joint_trajectory.points.resize(opt_solution.rows());
  for (int i = 0; i < opt_solution.rows(); i++)
  {
    res.trajectory[0].joint_trajectory.points[i].positions.resize(opt_solution.cols());
    for (size_t j = 0; j < opt_solution.cols(); j++)
    {
      res.trajectory[0].joint_trajectory.points[i].positions[j] = opt_solution(i, j);
    }
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory[0].joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.processing_time.push_back((ros::WallTime::now() - start_time).toSec());

  ROS_INFO(" ======================================= check if final state is within goal tolerances");
  kinematic_constraints::JointConstraint joint_cnt(planning_scene->getRobotModel());
  moveit::core::RobotState last_state(*current_state);
  last_state.setJointGroupPositions(req.group_name, res.trajectory[0].joint_trajectory.points.back().positions);

  for (int jn = 0; jn < res.trajectory[0].joint_trajectory.points.back().positions.size(); ++jn)
  {
    ROS_INFO_STREAM_NAMED("joint_value", res.trajectory[0].joint_trajectory.points.back().positions[jn]
                                             << "   " << req.goal_constraints.back().joint_constraints[jn].position);
  }

  bool constraints_are_ok = true;
  int joint_cnt_index = 0;
  for (const moveit_msgs::JointConstraint& constraint : req.goal_constraints.back().joint_constraints)
  {
    ROS_INFO("index %i: jc.configure(constraint)=> %d, jc.decide(last_state).satisfied=> %d, tolerance %f",
             joint_cnt_index, joint_cnt.configure(constraint), joint_cnt.decide(last_state).satisfied,
             constraint.tolerance_above);
    constraints_are_ok = constraints_are_ok and joint_cnt.configure(constraint);
    constraints_are_ok = constraints_are_ok and joint_cnt.decide(last_state).satisfied;
    if (not constraints_are_ok)
    {
      ROS_ERROR_STREAM_NAMED("trajopt_planner", "Goal constraints are violated: " << constraint.joint_name);
      res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
      return false;
    }
    joint_cnt_index = joint_cnt_index + 1;
  }

  ROS_INFO(" ==================================== Response");
  res.trajectory_start = req.start_state;

  ROS_INFO(" ==================================== Debug Response");
  ROS_INFO_STREAM_NAMED("group_name", res.group_name);
  ROS_INFO_STREAM_NAMED("start_traj_name_size", res.trajectory_start.joint_state.name.size());
  ROS_INFO_STREAM_NAMED("start_traj_position_size", res.trajectory_start.joint_state.position.size());
  ROS_INFO_STREAM_NAMED("traj_name_size", res.trajectory[0].joint_trajectory.joint_names.size());
  ROS_INFO_STREAM_NAMED("traj_point_size", res.trajectory[0].joint_trajectory.points.size());
  return true;
}

void TrajOptInterface::setDefaultTrajOPtParams()
{
  sco::BasicTrustRegionSQPParameters params;
  params_ = params;
}

void TrajOptInterface::setTrajOptParams(sco::BasicTrustRegionSQPParameters& params)
{
  nh_.param("trajopt_param/improve_ratio_threshold", params.improve_ratio_threshold, 0.25);
  nh_.param("trajopt_param/min_trust_box_size", params.min_trust_box_size, 1e-4);
  nh_.param("trajopt_param/min_approx_improve", params.min_approx_improve, 1e-4);
  nh_.param("trajopt_param/min_approx_improve_frac", params.min_approx_improve_frac, -static_cast<double>(INFINITY));
  nh_.param("trajopt_param/max_iter", params.max_iter, 100.0);
  nh_.param("trajopt_param/trust_shrink_ratio", params.trust_shrink_ratio, 0.1);

  nh_.param("trajopt_param/trust_expand_ratio", params.trust_expand_ratio, 1.5);
  nh_.param("trajopt_param/cnt_tolerance", params.cnt_tolerance, 1e-4);
  nh_.param("trajopt_param/max_merit_coeff_increases", params.max_merit_coeff_increases, 5.0);
  nh_.param("trajopt_param/merit_coeff_increase_ratio", params.merit_coeff_increase_ratio, 10.0);
  nh_.param("trajopt_param/max_time", params.max_time, static_cast<double>(INFINITY));
  nh_.param("trajopt_param/merit_error_coeff", params.initial_merit_error_coeff, 10.0);
  nh_.param("trajopt_param/trust_box_size", params.trust_box_size, 1e-1);
}

void TrajOptInterface::setProblemInfoParam(ProblemInfo& problem_info)
{
  nh_.param("problem_info/basic_info/n_steps", problem_info.basic_info.n_steps, 20);
  nh_.param("problem_info/basic_info/dt_upper_lim", problem_info.basic_info.dt_upper_lim, 2.0);
  nh_.param("problem_info/basic_info/dt_lower_lim", problem_info.basic_info.dt_lower_lim, 100.0);
  nh_.param("problem_info/basic_info/start_fixed", problem_info.basic_info.start_fixed, true);
  nh_.param("problem_info/basic_info/use_time", problem_info.basic_info.use_time, false);
  int convex_solver_index;
  nh_.param("problem_info/basic_info/convex_solver", convex_solver_index, 1);
  switch (convex_solver_index)
  {
    case 1:
      problem_info.basic_info.convex_solver = sco::ModelType::AUTO_SOLVER;
      break;
    case 2:
      problem_info.basic_info.convex_solver = sco::ModelType::BPMPD;
      break;
    case 3:
      problem_info.basic_info.convex_solver = sco::ModelType::OSQP;
      break;
    case 4:
      problem_info.basic_info.convex_solver = sco::ModelType::QPOASES;
      break;
    case 5:
      problem_info.basic_info.convex_solver = sco::ModelType::GUROBI;
      break;
  }

  nh_.param("problem_info/init_info/dt", problem_info.init_info.dt, 0.5);
  int type_index;
  nh_.param("problem_info/init_info/type", type_index, 1);
  switch (type_index)
  {
    case 1:
      problem_info.init_info.type = InitInfo::STATIONARY;
      break;
    case 2:
      problem_info.init_info.type = InitInfo::JOINT_INTERPOLATED;
      break;
    case 3:
      problem_info.init_info.type = InitInfo::GIVEN_TRAJ;
      break;
  }
}

void TrajOptInterface::setJointPoseTermInfoParams(JointPoseTermInfoPtr& jp, std::string name)
{
  int term_type_index;
  std::string term_type_address = "joint_pos_term_info/" + name + "/term_type";
  nh_.param(term_type_address, term_type_index, 1);

  switch (term_type_index)
  {
    case 1:
      jp->term_type = TT_COST;
      break;
    case 2:
      jp->term_type = TT_CNT;
      break;
    case 3:
      jp->term_type = TT_USE_TIME;
      break;
  }

  nh_.getParam("joint_pos_term_info/" + name + "/first_timestep", jp->first_step);
  nh_.getParam("joint_pos_term_info/" + name + "/last_timestep", jp->last_step);
  nh_.getParam("joint_pos_term_info/" + name + "/name", jp->name);
}

trajopt::DblVec TrajOptInterface::extractStartJointValues(const planning_interface::MotionPlanRequest& req,
                                                          const std::vector<std::string>& group_joint_names)
{
  std::unordered_map<std::string, double> all_joints;
  trajopt::DblVec start_joint_vals;

  for (int joint_index = 0; joint_index < req.start_state.joint_state.position.size(); ++joint_index)
  {
    all_joints[req.start_state.joint_state.name[joint_index]] = req.start_state.joint_state.position[joint_index];
  }

  for (auto joint_name : group_joint_names)
  {
    ROS_INFO(" joint position from start state, name: %s, value: %f", joint_name.c_str(), all_joints[joint_name]);
    start_joint_vals.push_back(all_joints[joint_name]);
  }

  return start_joint_vals;
}

void callBackFunc(sco::OptProb* opt_prob, sco::OptResults& opt_res)
{
  // TODO: Create the actuall implementation
}

}  // namespace trajopt_interface
