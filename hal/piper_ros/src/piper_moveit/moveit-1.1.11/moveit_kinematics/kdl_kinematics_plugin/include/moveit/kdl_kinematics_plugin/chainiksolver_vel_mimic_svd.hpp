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

#pragma once

#include <kdl/config.h>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
#include <Eigen/SVD>

namespace KDL
{
/**
 * Implementation of a inverse velocity kinematics algorithm based
 * on the generalize pseudo inverse to calculate the velocity
 * transformation from Cartesian to joint space of a general
 * KDL::Chain. It uses a svd-calculation based on householders
 * rotations.
 *
 * @ingroup KinematicFamily
 */
class ChainIkSolverVelMimicSVD : public ChainIkSolverVel
{
public:
  /**
   * Constructor of the solver
   *
   * @param chain the chain to calculate the inverse velocity kinematics for
   * @param mimic_joints A vector of indices that map each (and every) joint onto the corresponding joint in a
   * reduced set of joints that do not include the mimic joints. This vector must be of size chain.getNrOfJoints().
   * E.g. if an arm has 7 joints: j0 to j6. Say j2 mimics (follows) j0. Then, mimic_joints should be: [0 1 0 3 4 5 6]
   * @param num_mimic_joints The number of joints that are setup to follow other joints
   * @param position_ik false if you want to solve for the full 6 dof end-effector pose,
   *        true if you want to solve only for the 3 dof end-effector position.
   * @param threshold if a singular value is below this value, its inverse is set to zero, default: 0.001
   */
  explicit ChainIkSolverVelMimicSVD(const Chain& chain_,
                                    const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints,
                                    bool position_ik = false, double threshold = 0.001);

// TODO: simplify after kinetic support is dropped
#define KDL_VERSION_LESS(a, b, c) ((KDL_VERSION) < ((a << 16) | (b << 8) | c))
#if KDL_VERSION_LESS(1, 4, 0)
  void updateInternalDataStructures();
#else
  void updateInternalDataStructures() override;
#endif
#undef KDL_VERSION_LESS

  ~ChainIkSolverVelMimicSVD() override;

  int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out) override
  {
    return CartToJnt(q_in, v_in, qdot_out, Eigen::VectorXd::Constant(svd_.cols(), 1.0),
                     Eigen::Matrix<double, 6, 1>::Constant(1.0));
  }

  /** Compute qdot_out = W_q * (W_x * J * W_q)^# * W_x * v_in
   *
   * where W_q and W_x are joint- and Cartesian weights respectively.
   * A smaller joint weight (< 1.0) will reduce the contribution of this joint to the solution. */
  // NOLINTNEXTLINE(readability-identifier-naming)
  int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out, const Eigen::VectorXd& joint_weights,
                const Eigen::Matrix<double, 6, 1>& cartesian_weights);

  /// not implemented.
  int CartToJnt(const JntArray& /*q_init*/, const FrameVel& /*v_in*/, JntArrayVel& /*q_out*/) override
  {
    return -1;
  }

  /// Return true iff we ignore orientation but only consider position for inverse kinematics
  bool isPositionOnly() const
  {
    return svd_.rows() == 3;
  }

private:
  bool jacToJacReduced(const Jacobian& jac, Jacobian& jac_reduced);

  // Mimic joint specific
  const std::vector<kdl_kinematics_plugin::JointMimic>& mimic_joints_;
  int num_mimic_joints_;

  const Chain& chain_;
  ChainJntToJacSolver jnt2jac_;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  Eigen::VectorXd qdot_out_reduced_;

  Jacobian jac_;          // full Jacobian
  Jacobian jac_reduced_;  // reduced Jacobian with contributions of mimic joints mapped onto active DoFs
};
}  // namespace KDL
