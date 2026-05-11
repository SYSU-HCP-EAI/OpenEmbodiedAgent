#include <Eigen/Geometry>
#include <boost/format.hpp>

#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>

#include "trajopt_interface/kinematic_terms.h"

using namespace std;
using namespace sco;
using namespace Eigen;

namespace trajopt_interface
{
VectorXd CartPoseErrCalculator::operator()(const VectorXd& dof_vals) const
{
  // TODO: create the actual error function from information in planning scene
  VectorXd err;
  return err;
}

VectorXd JointVelErrCalculator::operator()(const VectorXd& var_vals) const
{
  assert(var_vals.rows() % 2 == 0);
  // var_vals = (theta_t1, theta_t2, theta_t3 ... 1/dt_1, 1/dt_2, 1/dt_3 ...)
  int half = static_cast<int>(var_vals.rows() / 2);
  int num_vels = half - 1;
  // (x1-x0)*(1/dt)
  VectorXd vel = (var_vals.segment(1, num_vels) - var_vals.segment(0, num_vels)).array() *
                 var_vals.segment(half + 1, num_vels).array();

  // Note that for equality terms tols are 0, so error is effectively doubled
  VectorXd result(vel.rows() * 2);
  result.topRows(vel.rows()) = -(upper_tol_ - (vel.array() - target_));
  result.bottomRows(vel.rows()) = lower_tol_ - (vel.array() - target_);
  return result;
}

MatrixXd JointVelJacobianCalculator::operator()(const VectorXd& var_vals) const
{
  // var_vals = (theta_t1, theta_t2, theta_t3 ... 1/dt_1, 1/dt_2, 1/dt_3 ...)
  int num_vals = static_cast<int>(var_vals.rows());
  int half = num_vals / 2;
  int num_vels = half - 1;
  MatrixXd jac = MatrixXd::Zero(num_vels * 2, num_vals);

  for (int i = 0; i < num_vels; i++)
  {
    // v = (j_i+1 - j_i)*(1/dt)
    // We calculate v with the dt from the second pt
    int time_index = i + half + 1;
    jac(i, i) = -1.0 * var_vals(time_index);
    jac(i, i + 1) = 1.0 * var_vals(time_index);
    jac(i, time_index) = var_vals(i + 1) - var_vals(i);
    // All others are 0
  }

  // bottom half is negative velocities
  jac.bottomRows(num_vels) = -jac.topRows(num_vels);

  return jac;
}

}  // namespace trajopt_interface
