/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/* Author: Mrinal Kalakrishnan */

#pragma once

#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_cost.h>
#include <chomp_motion_planner/multivariate_gaussian.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_distance_field/collision_env_hybrid.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

namespace chomp
{
class ChompOptimizer
{
public:
  ChompOptimizer(ChompTrajectory* trajectory, const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const std::string& planning_group, const ChompParameters* parameters,
                 const moveit::core::RobotState& start_state);

  virtual ~ChompOptimizer();

  /**
   * Optimizes the CHOMP cost function and tries to find an optimal path
   * @return true if an optimal collision free path is found else returns false
   */
  bool optimize();

  inline void destroy()
  {
    // Nothing for now.
  }

  bool isInitialized() const
  {
    return initialized_;
  }

  bool isCollisionFree() const
  {
    return is_collision_free_;
  }

private:
  inline double getPotential(double field_distance, double radius, double clearance)
  {
    double d = field_distance - radius;

    if (d >= clearance)  // everything is fine
    {
      return 0.0;
    }
    else if (d >= 0.0)  // transition phase, no collision yet
    {
      const double diff = (d - clearance);
      const double gradient_magnitude = diff / clearance;
      return 0.5 * gradient_magnitude * diff;  // 0.5 * (d - clearance)^2 / clearance
    }
    else  // d < 0.0: collision
    {
      return -d + 0.5 * clearance;  // linearly increase, starting from 0.5 * clearance
    }
  }
  template <typename Derived>
  void getJacobian(int trajectoryPoint, Eigen::Vector3d& collision_point_pos, std::string& jointName,
                   Eigen::MatrixBase<Derived>& jacobian) const;

  // void getRandomState(const moveit::core::RobotState& currentState,
  //                     const std::string& group_name,
  //                     Eigen::VectorXd& state_vec);

  void setRobotStateFromPoint(ChompTrajectory& group_trajectory, int i);

  // collision_proximity::CollisionProximitySpace::TrajectorySafety checkCurrentIterValidity();

  int num_joints_;
  int num_vars_free_;
  int num_vars_all_;
  int num_collision_points_;
  int free_vars_start_;
  int free_vars_end_;
  int iteration_;
  unsigned int collision_free_iteration_;

  ChompTrajectory* full_trajectory_;
  const moveit::core::RobotModelConstPtr& robot_model_;
  std::string planning_group_;
  const ChompParameters* parameters_;
  ChompTrajectory group_trajectory_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit::core::RobotState state_;
  moveit::core::RobotState start_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  const collision_detection::CollisionEnvHybrid* hy_env_;

  std::vector<ChompCost> joint_costs_;
  collision_detection::GroupStateRepresentationPtr gsr_;
  bool initialized_;

  std::vector<std::vector<std::string> > collision_point_joint_names_;
  std::vector<EigenSTL::vector_Vector3d> collision_point_pos_eigen_;
  std::vector<EigenSTL::vector_Vector3d> collision_point_vel_eigen_;
  std::vector<EigenSTL::vector_Vector3d> collision_point_acc_eigen_;
  std::vector<std::vector<double> > collision_point_potential_;
  std::vector<std::vector<double> > collision_point_vel_mag_;
  std::vector<EigenSTL::vector_Vector3d> collision_point_potential_gradient_;
  std::vector<EigenSTL::vector_Vector3d> joint_axes_;
  std::vector<EigenSTL::vector_Vector3d> joint_positions_;
  Eigen::MatrixXd group_trajectory_backup_;
  Eigen::MatrixXd best_group_trajectory_;
  double best_group_trajectory_cost_;
  int last_improvement_iteration_;
  unsigned int num_collision_free_iterations_;

  // HMC stuff:
  Eigen::MatrixXd momentum_;
  Eigen::MatrixXd random_momentum_;
  Eigen::VectorXd random_joint_momentum_;  // temporary variable
  std::vector<MultivariateGaussian> multivariate_gaussian_;
  double stochasticity_factor_;

  std::vector<int> state_is_in_collision_; /**< Array containing a boolean about collision info for each point in the
                                              trajectory */
  std::vector<std::vector<int> > point_is_in_collision_;
  bool is_collision_free_;
  double worst_collision_cost_state_;

  Eigen::MatrixXd smoothness_increments_;
  Eigen::MatrixXd collision_increments_;
  Eigen::MatrixXd final_increments_;

  // temporary variables for all functions:
  Eigen::VectorXd smoothness_derivative_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_pseudo_inverse_;
  Eigen::MatrixXd jacobian_jacobian_tranpose_;
  Eigen::VectorXd random_state_;
  Eigen::VectorXd joint_state_velocities_;

  std::vector<std::string> joint_names_;
  std::map<std::string, std::map<std::string, bool> > joint_parent_map_;

  inline bool isParent(const std::string& childLink, const std::string& parentLink) const
  {
    if (childLink == parentLink)
    {
      return true;
    }

    if (joint_parent_map_.find(childLink) == joint_parent_map_.end())
    {
      // ROS_ERROR("%s was not in joint parent map! for lookup of %s", childLink.c_str(), parentLink.c_str());
      return false;
    }
    const std::map<std::string, bool>& parents = joint_parent_map_.at(childLink);
    return (parents.find(parentLink) != parents.end() && parents.at(parentLink));
  }

  void registerParents(const moveit::core::JointModel* model);
  void initialize();
  void calculateSmoothnessIncrements();
  void calculateCollisionIncrements();
  void calculateTotalIncrements();
  void performForwardKinematics();
  void addIncrementsToTrajectory();
  void updateFullTrajectory();
  void debugCost();
  void handleJointLimits();
  double getTrajectoryCost();
  double getSmoothnessCost();
  double getCollisionCost();
  void perturbTrajectory();
  void getRandomMomentum();
  void updateMomentum();
  void updatePositionFromMomentum();
  void calculatePseudoInverse();
  void computeJointProperties(int trajectoryPoint);
  bool isCurrentTrajectoryMeshToMeshCollisionFree() const;
};
}  // namespace chomp
