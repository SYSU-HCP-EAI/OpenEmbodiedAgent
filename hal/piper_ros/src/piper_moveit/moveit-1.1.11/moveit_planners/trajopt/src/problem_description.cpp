/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, John Schulman
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  http://opensource.org/licenses/BSD-2-Clause
 *********************************************************************/

#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/vector_ops.hpp>

#include "trajopt_interface/problem_description.h"
#include "trajopt_interface/kinematic_terms.h"

/**
 * @brief Checks the size of the parameter given and throws if incorrect
 * @param parameter The vector whose size is getting checked
 * @param expected_size The expected size of the vector
 * @param name The name to use when printing an error or warning
 * @param apply_first If true and only one value is given, broadcast value to length of expected_size
 */
void checkParameterSize(trajopt::DblVec& parameter, const unsigned int& expected_size, const std::string& name,
                        const bool& apply_first = true)
{
  if (apply_first == true && parameter.size() == 1)
  {
    parameter = trajopt::DblVec(expected_size, parameter[0]);
    ROS_INFO("1 %s given. Applying to all %i joints", name.c_str(), expected_size);
  }
  else if (parameter.size() != expected_size)
  {
    PRINT_AND_THROW(boost::format("wrong number of %s. expected %i got %i") % name % expected_size % parameter.size());
  }
}

namespace trajopt_interface
{
TrajOptProblem::TrajOptProblem()
{
}

TrajOptProblem::TrajOptProblem(const ProblemInfo& problem_info)
  : OptProb(problem_info.basic_info.convex_solver)
  , planning_scene_(problem_info.planning_scene)
  , planning_group_(problem_info.planning_group_name)
{
  moveit::core::RobotModelConstPtr robot_model = planning_scene_->getRobotModel();
  moveit::core::RobotState current_state = planning_scene_->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(planning_group_);

  moveit::core::JointBoundsVector bounds = joint_model_group->getActiveJointModelsBounds();
  dof_ = joint_model_group->getActiveJointModelNames().size();  // or bounds.size();

  int n_steps = problem_info.basic_info.n_steps;

  ROS_INFO(" ======================================= problem_description: limits");
  Eigen::MatrixX2d limits(dof_, 2);
  for (int k = 0; k < limits.size() / 2; ++k)
  {
    moveit::core::JointModel::Bounds bound = *bounds[k];
    // In MoveIt, joints are considered to have multiple dofs but we only have single dof joints:
    moveit::core::VariableBounds joint_bound = bound.front();

    limits(k, 0) = joint_bound.min_position_;
    limits(k, 1) = joint_bound.max_position_;

    ROS_INFO("joint %i with lower bound: %f, upper bound: %f", k, joint_bound.min_position_, joint_bound.max_position_);
  }

  Eigen::VectorXd lower, upper;
  lower = limits.col(0);
  upper = limits.col(1);

  trajopt::DblVec vlower, vupper;
  std::vector<std::string> names;
  for (int i = 0; i < n_steps; ++i)
  {
    for (int j = 0; j < dof_; ++j)
    {
      names.push_back((boost::format("j_%i_%i") % i % j).str());
    }
    vlower.insert(vlower.end(), lower.data(), lower.data() + lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data() + upper.size());

    if (problem_info.basic_info.use_time == true)
    {
      vlower.insert(vlower.end(), problem_info.basic_info.dt_lower_lim);
      vupper.insert(vupper.end(), problem_info.basic_info.dt_upper_lim);
      names.push_back((boost::format("dt_%i") % i).str());
    }
  }

  sco::VarVector trajvarvec = createVariables(names, vlower, vupper);
  matrix_traj_vars = trajopt::VarArray(n_steps, dof_ + (problem_info.basic_info.use_time ? 1 : 0), trajvarvec.data());
  // matrix_traj_vars is essentialy a matrix of elements like:
  // j_0_0, j_0_1 ...
  // j_1_0, j_1_1 ...
  // its size is n_steps by n_dof
}

TrajOptProblemPtr ConstructProblem(const ProblemInfo& pci)
{
  const BasicInfo& bi = pci.basic_info;
  int n_steps = bi.n_steps;

  bool use_time = false;
  // Check that all costs and constraints support the types that are specified in pci
  for (TermInfoPtr cost : pci.cost_infos)
  {
    if (cost->term_type & TT_CNT)
      ROS_WARN("%s is listed as a type TT_CNT but was added to cost_infos", (cost->name).c_str());
    if (!(cost->getSupportedTypes() & TT_COST))
      PRINT_AND_THROW(boost::format("%s is only a constraint, but you listed it as a cost") % cost->name);
    if (cost->term_type & TT_USE_TIME)
    {
      use_time = true;
      if (!(cost->getSupportedTypes() & TT_USE_TIME))
        PRINT_AND_THROW(boost::format("%s does not support time, but you listed it as a using time") % cost->name);
    }
  }
  for (TermInfoPtr cnt : pci.cnt_infos)
  {
    if (cnt->term_type & TT_COST)
      ROS_WARN("%s is listed as a type TT_COST but was added to cnt_infos", (cnt->name).c_str());
    if (!(cnt->getSupportedTypes() & TT_CNT))
      PRINT_AND_THROW(boost::format("%s is only a cost, but you listed it as a constraint") % cnt->name);
    if (cnt->term_type & TT_USE_TIME)
    {
      use_time = true;
      if (!(cnt->getSupportedTypes() & TT_USE_TIME))
        PRINT_AND_THROW(boost::format("%s does not support time, but you listed it as a using time") % cnt->name);
    }
  }

  // Check that if a cost or constraint uses time, basic_info is set accordingly
  if ((use_time == true) && (pci.basic_info.use_time == false))
    PRINT_AND_THROW("A term is using time and basic_info is not set correctly. Try basic_info.use_time = true");

  // This could be removed in the future once we are sure that all costs are
  if ((use_time == false) && (pci.basic_info.use_time == true))
    PRINT_AND_THROW("No terms use time and basic_info is not set correctly. Try basic_info.use_time = false");

  TrajOptProblemPtr prob(new TrajOptProblem(pci));

  // Generate initial trajectory and check its size
  moveit::core::RobotModelConstPtr robot_model = pci.planning_scene->getRobotModel();
  moveit::core::RobotState current_state = pci.planning_scene->getCurrentState();

  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(pci.planning_group_name);
  int n_dof = prob->GetNumDOF();

  std::vector<double> current_joint_values;
  current_state.copyJointGroupPositions(joint_model_group, current_joint_values);

  trajopt::TrajArray init_traj;
  generateInitialTrajectory(pci, current_joint_values, init_traj);

  if (pci.basic_info.use_time == true)
  {
    prob->SetHasTime(true);
    if (init_traj.rows() != n_steps || init_traj.cols() != n_dof + 1)
    {
      PRINT_AND_THROW(boost::format("Initial trajectory is not the right size matrix\n"
                                    "Expected %i rows (time steps) x %i columns (%i dof + 1 time column)\n"
                                    "Got %i rows and %i columns") %
                      n_steps % (n_dof + 1) % n_dof % init_traj.rows() % init_traj.cols());
    }
  }
  else
  {
    prob->SetHasTime(false);
    if (init_traj.rows() != n_steps || init_traj.cols() != n_dof)
    {
      PRINT_AND_THROW(boost::format("Initial trajectory is not the right size matrix\n"
                                    "Expected %i rows (time steps) x %i columns\n"
                                    "Got %i rows and %i columns") %
                      n_steps % n_dof % init_traj.rows() % init_traj.cols());
    }
  }
  prob->SetInitTraj(init_traj);

  trajopt::VarArray matrix_traj_vars_temp;
  // If start_fixed, constrain the joint values for the first time step to be their initialized values
  if (bi.start_fixed)
  {
    if (init_traj.rows() < 1)
    {
      PRINT_AND_THROW("Initial trajectory must contain at least the start state.");
    }

    if (init_traj.cols() != (n_dof + (use_time ? 1 : 0)))
    {
      PRINT_AND_THROW("robot dof values don't match initialization. I don't "
                      "know what you want me to use for the dof values");
    }

    for (int j = 0; j < static_cast<int>(n_dof); ++j)
    {
      matrix_traj_vars_temp = prob->GetVars();
      prob->addLinearConstraint(sco::exprSub(sco::AffExpr(matrix_traj_vars_temp(0, j)), init_traj(0, j)), sco::EQ);
    }
  }

  // Apply constraint to each fixed dof to its initial value for all timesteps (freeze that column)
  if (!bi.dofs_fixed.empty())
  {
    for (const int& dof_ind : bi.dofs_fixed)
    {
      for (int i = 1; i < prob->GetNumSteps(); ++i)
      {
        matrix_traj_vars_temp = prob->GetVars();
        prob->addLinearConstraint(sco::exprSub(sco::AffExpr(matrix_traj_vars_temp(i, dof_ind)),
                                               sco::AffExpr(init_traj(0, dof_ind))),
                                  sco::EQ);
      }
    }
  }

  for (const TermInfoPtr& ci : pci.cost_infos)
  {
    ci->addObjectiveTerms(*prob);
  }

  for (const TermInfoPtr& ci : pci.cnt_infos)
  {
    ci->addObjectiveTerms(*prob);
  }
  return prob;
}

CartPoseTermInfo::CartPoseTermInfo() : TermInfo(TT_COST | TT_CNT)
{
  pos_coeffs = Eigen::Vector3d::Ones();
  rot_coeffs = Eigen::Vector3d::Ones();
  tcp.setIdentity();
}

void CartPoseTermInfo::addObjectiveTerms(TrajOptProblem& prob)
{
  unsigned int n_dof = prob.GetActiveGroupNumDOF();

  Eigen::Isometry3d input_pose;
  Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  input_pose.linear() = q.matrix();
  input_pose.translation() = xyz;

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    sco::VectorOfVector::Ptr f(new CartPoseErrCalculator(input_pose, prob.GetPlanningScene(), link, tcp));
    prob.addCost(sco::Cost::Ptr(new sco::CostFromErrFunc(f, prob.GetVarRow(timestep, 0, n_dof),
                                                         concatVector(rot_coeffs, pos_coeffs), sco::ABS, name)));
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    sco::VectorOfVector::Ptr f(new CartPoseErrCalculator(input_pose, prob.GetPlanningScene(), link, tcp));
    prob.addConstraint(sco::Constraint::Ptr(new sco::ConstraintFromErrFunc(
        f, prob.GetVarRow(timestep, 0, n_dof), concatVector(rot_coeffs, pos_coeffs), sco::EQ, name)));
  }
  else
  {
    ROS_WARN("CartPoseTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void JointPoseTermInfo::addObjectiveTerms(TrajOptProblem& prob)
{
  unsigned int n_dof = prob.GetActiveGroupNumDOF();

  // If optional parameter not given, set to default
  if (coeffs.empty())
    coeffs = trajopt::DblVec(n_dof, 1);
  if (upper_tols.empty())
    upper_tols = trajopt::DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = trajopt::DblVec(n_dof, 0);
  if (last_step <= -1)
    last_step = prob.GetNumSteps() - 1;

  // Check time step is valid
  if ((prob.GetNumSteps() - 1) <= first_step)
    first_step = prob.GetNumSteps() - 1;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  //  if (last_step == first_step)
  //    last_step += 1;
  if (last_step < first_step)
  {
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointPosTerm comes before first step. Reversing them.");
  }
  if (last_step == -1)  // last_step not set
    last_step = first_step;

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointPoseTermInfo coeffs", true);
  checkParameterSize(targets, n_dof, "JointPoseTermInfo targets", true);
  checkParameterSize(upper_tols, n_dof, "JointPoseTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointPoseTermInfo lower_tols", true);

  // Check if tolerances are all zeros
  bool is_upper_zeros =
      std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });
  bool is_lower_zeros =
      std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });

  // Get vars associated with joints
  trajopt::VarArray vars = prob.GetVars();
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(n_dof));
  if (prob.GetHasTime())
    ROS_INFO("JointPoseTermInfo does not differ based on setting of TT_USE_TIME");

  if (term_type & TT_COST)
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(sco::Cost::Ptr(new trajopt::JointPosEqCost(joint_vars, util::toVectorXd(coeffs),
                                                              util::toVectorXd(targets), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(sco::Cost::Ptr(new trajopt::JointPosIneqCost(joint_vars, util::toVectorXd(coeffs),
                                                                util::toVectorXd(targets), util::toVectorXd(upper_tols),
                                                                util::toVectorXd(lower_tols), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if (term_type & TT_CNT)
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(sco::Constraint::Ptr(new trajopt::JointPosEqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(sco::Constraint::Ptr(new trajopt::JointPosIneqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), util::toVectorXd(upper_tols),
          util::toVectorXd(lower_tols), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
  else
  {
    ROS_WARN("JointPosTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void JointVelTermInfo::addObjectiveTerms(TrajOptProblem& prob)
{
  unsigned int n_dof = prob.GetNumDOF();

  // If optional parameter not given, set to default
  if (coeffs.empty())
    coeffs = trajopt::DblVec(n_dof, 1);
  if (upper_tols.empty())
    upper_tols = trajopt::DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = trajopt::DblVec(n_dof, 0);
  if (last_step <= -1)
    last_step = prob.GetNumSteps() - 1;

  // If only one time step is desired, calculate velocity with next step (2 steps are needed for 1 velocity calculation)
  if ((prob.GetNumSteps() - 2) <= first_step)
    first_step = prob.GetNumSteps() - 2;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  if (last_step == first_step)
    last_step += 1;
  if (last_step < first_step)
  {
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointVelTerm comes before first step. Reversing them.");
  }

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointVelTermInfo coeffs", true);
  checkParameterSize(targets, n_dof, "JointVelTermInfo targets", true);
  checkParameterSize(upper_tols, n_dof, "JointVelTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointVelTermInfo lower_tols", true);
  assert(last_step > first_step);
  assert(first_step >= 0);

  // Check if tolerances are all zeros
  bool is_upper_zeros =
      std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });
  bool is_lower_zeros =
      std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });

  // Get vars associated with joints
  trajopt::VarArray vars = prob.GetVars();
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(n_dof));

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    unsigned num_vels = last_step - first_step;

    // Apply seperate cost to each joint b/c that is how the error function is currently written
    for (size_t j = 0; j < n_dof; j++)
    {
      // Get a vector of a single column of vars
      sco::VarVector joint_vars_vec = joint_vars.cblock(first_step, j, last_step - first_step + 1);
      sco::VarVector time_vars_vec = vars.cblock(first_step, vars.cols() - 1, last_step - first_step + 1);

      // If the tolerances are 0, an equality cost is set
      if (is_upper_zeros && is_lower_zeros)
      {
        trajopt::DblVec single_jnt_coeffs = trajopt::DblVec(num_vels * 2, coeffs[j]);
        prob.addCost(sco::Cost::Ptr(new sco::CostFromErrFunc(
            sco::VectorOfVector::Ptr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVector::Ptr(new JointVelJacobianCalculator()), concatVector(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs), sco::SQUARED, name + "_j" + std::to_string(j))));
      }
      // Otherwise it's a hinged "inequality" cost
      else
      {
        trajopt::DblVec single_jnt_coeffs = trajopt::DblVec(num_vels * 2, coeffs[j]);
        prob.addCost(sco::Cost::Ptr(new sco::CostFromErrFunc(
            sco::VectorOfVector::Ptr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVector::Ptr(new JointVelJacobianCalculator()), concatVector(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs), sco::HINGE, name + "_j" + std::to_string(j))));
      }
    }
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    unsigned num_vels = last_step - first_step;

    // Apply seperate cnt to each joint b/c that is how the error function is currently written
    for (size_t j = 0; j < n_dof; j++)
    {
      // Get a vector of a single column of vars
      sco::VarVector joint_vars_vec = joint_vars.cblock(first_step, j, last_step - first_step + 1);
      sco::VarVector time_vars_vec = vars.cblock(first_step, vars.cols() - 1, last_step - first_step + 1);

      // If the tolerances are 0, an equality cnt is set
      if (is_upper_zeros && is_lower_zeros)
      {
        trajopt::DblVec single_jnt_coeffs = trajopt::DblVec(num_vels * 2, coeffs[j]);
        prob.addConstraint(sco::Constraint::Ptr(new sco::ConstraintFromErrFunc(
            sco::VectorOfVector::Ptr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVector::Ptr(new JointVelJacobianCalculator()), concatVector(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs), sco::EQ, name + "_j" + std::to_string(j))));
      }
      // Otherwise it's a hinged "inequality" constraint
      else
      {
        trajopt::DblVec single_jnt_coeffs = trajopt::DblVec(num_vels * 2, coeffs[j]);
        prob.addConstraint(sco::Constraint::Ptr(new sco::ConstraintFromErrFunc(
            sco::VectorOfVector::Ptr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVector::Ptr(new JointVelJacobianCalculator()), concatVector(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs), sco::INEQ, name + "_j" + std::to_string(j))));
      }
    }
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(sco::Cost::Ptr(new trajopt::JointVelEqCost(joint_vars, util::toVectorXd(coeffs),
                                                              util::toVectorXd(targets), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(sco::Cost::Ptr(new trajopt::JointVelIneqCost(joint_vars, util::toVectorXd(coeffs),
                                                                util::toVectorXd(targets), util::toVectorXd(upper_tols),
                                                                util::toVectorXd(lower_tols), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(sco::Constraint::Ptr(new trajopt::JointVelEqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(sco::Constraint::Ptr(new trajopt::JointVelIneqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), util::toVectorXd(upper_tols),
          util::toVectorXd(lower_tols), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
  else
  {
    ROS_WARN("JointVelTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void generateInitialTrajectory(const ProblemInfo& pci, const std::vector<double>& current_joint_values,
                               trajopt::TrajArray& init_traj)
{
  Eigen::VectorXd current_pos(current_joint_values.size());
  for (int joint_index = 0; joint_index < current_joint_values.size(); ++joint_index)
  {
    current_pos(joint_index) = current_joint_values[joint_index];
  }

  const InitInfo& init_info = pci.init_info;

  // initialize based on type specified
  if (init_info.type == InitInfo::STATIONARY)
  {
    // Initializes all joint values to the initial value (the current value in scene)
    init_traj = current_pos.transpose().replicate(pci.basic_info.n_steps, 1);
  }
  else if (init_info.type == InitInfo::JOINT_INTERPOLATED)
  {
    //  Linearly interpolates between initial value (current values in the scene) and the joint position specified in
    //  InitInfo.data
    Eigen::VectorXd end_pos = init_info.data;
    init_traj.resize(pci.basic_info.n_steps, end_pos.rows());
    for (int dof_index = 0; dof_index < current_pos.rows(); ++dof_index)
    {
      init_traj.col(dof_index) =
          Eigen::VectorXd::LinSpaced(pci.basic_info.n_steps, current_pos(dof_index), end_pos(dof_index));
    }
  }
  else if (init_info.type == InitInfo::GIVEN_TRAJ)
  {
    //  Initializes the matrix to a given trajectory
    init_traj = init_info.data;
  }
  else
  {
    PRINT_AND_THROW("Init Info did not have a valid type. Valid types are "
                    "STATIONARY, JOINT_INTERPOLATED, or GIVEN_TRAJ");
  }

  // Currently all trajectories are generated without time then appended here
  if (pci.basic_info.use_time)
  {
    // add on time (default to 1 sec)
    init_traj.conservativeResize(Eigen::NoChange_t(), init_traj.cols() + 1);

    init_traj.block(0, init_traj.cols() - 1, init_traj.rows(), 1) =
        Eigen::VectorXd::Constant(init_traj.rows(), init_info.dt);
  }
}
}  // namespace trajopt_interface
