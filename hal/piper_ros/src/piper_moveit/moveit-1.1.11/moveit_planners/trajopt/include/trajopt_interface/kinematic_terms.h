#pragma once

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>

namespace trajopt_interface
{
/**
 * @brief Extracts the vector part of quaternion
 */
inline Eigen::Vector3d quaternionRotationVector(const Eigen::Matrix3d& matrix)
{
  Eigen::Quaterniond quaternion;
  quaternion = matrix;
  return Eigen::Vector3d(quaternion.x(), quaternion.y(), quaternion.z());
}

/**
 * @brief Appends b to a of type VectorXd
 */
inline Eigen::VectorXd concatVector(const Eigen::VectorXd& vector_a, const Eigen::VectorXd& vector_b)
{
  Eigen::VectorXd out(vector_a.size() + vector_b.size());
  out.topRows(vector_a.size()) = vector_a;
  out.middleRows(vector_a.size(), vector_b.size()) = vector_b;
  return out;
}

/**
 * @brief Appends b to a of type T
 */
template <typename T>
inline std::vector<T> concatVector(const std::vector<T>& vector_a, const std::vector<T>& vector_b)
{
  std::vector<T> out;
  out.insert(out.end(), vector_a.begin(), vector_a.end());
  out.insert(out.end(), vector_b.begin(), vector_b.end());
  return out;
}

/**
 * @brief Used to calculate the error for StaticCartPoseTermInfo
 * This is converted to a cost or constraint using TrajOptCostFromErrFunc or TrajOptConstraintFromErrFunc
 */
struct CartPoseErrCalculator : public sco::VectorOfVector
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Isometry3d target_pose_inv_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string link_;
  Eigen::Isometry3d tcp_;

  CartPoseErrCalculator(const Eigen::Isometry3d& pose, const planning_scene::PlanningSceneConstPtr planning_scene,
                        const std::string& link, Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity())
    : target_pose_inv_(pose.inverse()), planning_scene_(planning_scene), link_(link), tcp_(tcp)
  {
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& dof_vals) const override;
};

// TODO(omid): The following should be added and adjusted from trajopt
// JointPosEqCost
// JointPosIneqCost
// JointPosEqConstraint
// JointPosIneqConstraint

struct JointVelErrCalculator : sco::VectorOfVector
{
  /** @brief Velocity target */
  double target_;
  /** @brief Upper tolerance */
  double upper_tol_;
  /** @brief Lower tolerance */
  double lower_tol_;
  JointVelErrCalculator() : target_(0.0), upper_tol_(0.0), lower_tol_(0.0)
  {
  }
  JointVelErrCalculator(double target, double upper_tol, double lower_tol)
    : target_(target), upper_tol_(upper_tol), lower_tol_(lower_tol)
  {
  }

  Eigen::VectorXd operator()(const Eigen::VectorXd& var_vals) const;
};

struct JointVelJacobianCalculator : sco::MatrixOfVector
{
  Eigen::MatrixXd operator()(const Eigen::VectorXd& var_vals) const;
};

}  // namespace trajopt_interface
