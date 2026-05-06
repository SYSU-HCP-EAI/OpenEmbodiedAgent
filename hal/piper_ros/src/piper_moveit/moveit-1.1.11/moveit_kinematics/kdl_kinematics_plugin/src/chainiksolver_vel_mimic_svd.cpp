// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

// Modified to account for "mimic" joints, i.e. joints whose motion has a
// linear relationship to that of another joint.
// Copyright  (C)  2013  Sachin Chitta, Willow Garage

#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp>

namespace
{
unsigned int countMimicJoints(const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints)
{
  unsigned int num_mimic = 0;
  for (const auto& item : mimic_joints)
  {
    if (!item.active)
      ++num_mimic;
  }
  return num_mimic;
}
}  // namespace

namespace KDL
{
ChainIkSolverVelMimicSVD::ChainIkSolverVelMimicSVD(const Chain& chain,
                                                   const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints,
                                                   bool position_ik, double threshold)
  : mimic_joints_(mimic_joints)
  , num_mimic_joints_(countMimicJoints(mimic_joints))
  , chain_(chain)
  , jnt2jac_(chain)
  // Performing a position-only IK, we just need to consider the first 3 rows of the Jacobian for SVD
  // SVD doesn't consider mimic joints, but only their driving joints
  , svd_(position_ik ? 3 : 6, chain_.getNrOfJoints() - num_mimic_joints_, Eigen::ComputeThinU | Eigen::ComputeThinV)
  , jac_(chain_.getNrOfJoints())
  , jac_reduced_(svd_.cols())
{
  assert(mimic_joints_.size() == chain.getNrOfJoints());
#ifndef NDEBUG
  for (const auto& item : mimic_joints)
    assert(item.map_index < chain_.getNrOfJoints());
#endif
  svd_.setThreshold(threshold);
}

void ChainIkSolverVelMimicSVD::updateInternalDataStructures()
{
  // TODO: move (re)allocation of any internal data structures here
  // to react to changes in chain
}

ChainIkSolverVelMimicSVD::~ChainIkSolverVelMimicSVD() = default;

bool ChainIkSolverVelMimicSVD::jacToJacReduced(const Jacobian& jac, Jacobian& jac_reduced)
{
  jac_reduced.data.setZero();
  for (std::size_t i = 0; i < chain_.getNrOfJoints(); ++i)
  {
    Twist vel1 = jac_reduced.getColumn(mimic_joints_[i].map_index);
    Twist vel2 = jac.getColumn(i);
    Twist result = vel1 + (mimic_joints_[i].multiplier * vel2);
    jac_reduced.setColumn(mimic_joints_[i].map_index, result);
  }
  return true;
}

// NOLINTNEXTLINE(readability-identifier-naming)
int ChainIkSolverVelMimicSVD::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out,
                                        const Eigen::VectorXd& joint_weights,
                                        const Eigen::Matrix<double, 6, 1>& cartesian_weights)
{
  // Let the ChainJntToJacSolver calculate the Jacobian for the current joint positions q_in.
  if (num_mimic_joints_ > 0)
  {
    jnt2jac_.JntToJac(q_in, jac_);
    // Now compute the actual jacobian that involves only the active DOFs
    jacToJacReduced(jac_, jac_reduced_);
  }
  else
    jnt2jac_.JntToJac(q_in, jac_reduced_);

  // weight Jacobian
  auto& jac = jac_reduced_.data;
  const Eigen::Index rows = svd_.rows();  // only operate on position rows?
  jac.topRows(rows) *= joint_weights.asDiagonal();
  jac.topRows(rows).transpose() *= cartesian_weights.topRows(rows).asDiagonal();

  // transform v_in to 6D Eigen::Vector
  Eigen::Matrix<double, 6, 1> vin;
  vin.topRows<3>() = Eigen::Map<const Eigen::Array3d>(v_in.vel.data, 3) * cartesian_weights.topRows<3>().array();
  vin.bottomRows<3>() = Eigen::Map<const Eigen::Array3d>(v_in.rot.data, 3) * cartesian_weights.bottomRows<3>().array();

  // Do a singular value decomposition: J = U*S*V^t
  svd_.compute(jac.topRows(rows));

  if (num_mimic_joints_ > 0)
  {
    qdot_out_reduced_.noalias() = svd_.solve(vin.topRows(rows));
    qdot_out_reduced_.array() *= joint_weights.array();
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
      qdot_out(i) = qdot_out_reduced_[mimic_joints_[i].map_index] * mimic_joints_[i].multiplier;
  }
  else
  {
    qdot_out.data.noalias() = svd_.solve(vin.topRows(rows));
    qdot_out.data.array() *= joint_weights.array();
  }

  return 0;
}
}  // namespace KDL
