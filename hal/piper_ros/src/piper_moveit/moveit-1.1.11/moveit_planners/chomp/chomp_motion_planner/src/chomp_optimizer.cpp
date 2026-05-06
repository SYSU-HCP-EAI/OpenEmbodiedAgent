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

#include <chomp_motion_planner/chomp_optimizer.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <random>

namespace chomp
{
double getRandomDouble()
{
  std::default_random_engine seed;
  std::uniform_real_distribution<> uniform(0.0, 1.0);
  return uniform(seed);
}

ChompOptimizer::ChompOptimizer(ChompTrajectory* trajectory, const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const std::string& planning_group, const ChompParameters* parameters,
                               const moveit::core::RobotState& start_state)
  : full_trajectory_(trajectory)
  , robot_model_(planning_scene->getRobotModel())
  , planning_group_(planning_group)
  , parameters_(parameters)
  , group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH)
  , planning_scene_(planning_scene)
  , state_(start_state)
  , start_state_(start_state)
  , initialized_(false)
{
  std::vector<std::string> cd_names;
  planning_scene->getCollisionDetectorNames(cd_names);

  ROS_INFO_STREAM("The following collision detectors are available in the planning scene.");
  for (const std::string& cd_name : cd_names)
  {
    ROS_INFO_STREAM(cd_name);
  }

  ROS_INFO_STREAM("Active collision detector is: " + planning_scene->getActiveCollisionDetectorName());

  hy_env_ = dynamic_cast<const collision_detection::CollisionEnvHybrid*>(
      planning_scene->getCollisionEnv(planning_scene->getActiveCollisionDetectorName()).get());
  if (!hy_env_)
  {
    ROS_WARN_STREAM("Could not initialize hybrid collision world from planning scene");
    return;
  }

  initialize();
}

void ChompOptimizer::initialize()
{
  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = planning_group_;
  ros::WallTime wt = ros::WallTime::now();
  hy_env_->getCollisionGradients(req, res, state_, &planning_scene_->getAllowedCollisionMatrix(), gsr_);
  ROS_INFO_STREAM("First coll check took " << (ros::WallTime::now() - wt));
  num_collision_points_ = 0;
  for (const collision_detection::GradientInfo& gradient : gsr_->gradients_)
  {
    num_collision_points_ += gradient.gradients.size();
  }

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;

  joint_model_group_ = planning_scene_->getRobotModel()->getJointModelGroup(planning_group_);

  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
  for (size_t i = 0; i < joint_models.size(); i++)
  {
    double joint_cost = 1.0;
    // nh.param("joint_costs/" + joint_models[i]->getName(), joint_cost, 1.0);
    std::vector<double> derivative_costs(3);
    derivative_costs[0] = joint_cost * parameters_->smoothness_cost_velocity_;
    derivative_costs[1] = joint_cost * parameters_->smoothness_cost_acceleration_;
    derivative_costs[2] = joint_cost * parameters_->smoothness_cost_jerk_;
    joint_costs_.push_back(ChompCost(group_trajectory_, i, derivative_costs, parameters_->ridge_factor_));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale)
      max_cost_scale = cost_scale;
  }

  // scale the smoothness costs
  for (int i = 0; i < num_joints_; i++)
  {
    joint_costs_[i].scale(max_cost_scale);
  }

  // allocate memory for matrices:
  smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  final_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
  jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);
  jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(num_joints_, 3);
  jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);
  random_state_ = Eigen::VectorXd::Zero(num_joints_);
  joint_state_velocities_ = Eigen::VectorXd::Zero(num_joints_);

  group_trajectory_backup_ = group_trajectory_.getTrajectory();
  best_group_trajectory_ = group_trajectory_.getTrajectory();

  collision_point_joint_names_.resize(num_vars_all_, std::vector<std::string>(num_collision_points_));
  collision_point_pos_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  collision_point_vel_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  collision_point_acc_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  joint_axes_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));
  joint_positions_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));

  collision_point_potential_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_vel_mag_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_potential_gradient_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));

  collision_free_iteration_ = 0;
  is_collision_free_ = false;
  state_is_in_collision_.resize(num_vars_all_);
  point_is_in_collision_.resize(num_vars_all_, std::vector<int>(num_collision_points_));

  last_improvement_iteration_ = -1;

  /// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC
  /// parameters values in the chomp_planning.yaml file so that CHOMP can find optimal paths

  // HMC initialization:
  // momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  // random_momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  // random_joint_momentum_ = Eigen::VectorXd::Zero(num_vars_free_);
  multivariate_gaussian_.clear();
  stochasticity_factor_ = 1.0;
  for (int i = 0; i < num_joints_; i++)
  {
    multivariate_gaussian_.push_back(
        MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_), joint_costs_[i].getQuadraticCostInverse()));
  }

  std::map<std::string, std::string> fixed_link_resolution_map;
  for (int i = 0; i < num_joints_; i++)
  {
    joint_names_.push_back(joint_model_group_->getActiveJointModels()[i]->getName());
    // ROS_INFO("Got joint %s", joint_names_[i].c_str());
    registerParents(joint_model_group_->getActiveJointModels()[i]);
    fixed_link_resolution_map[joint_names_[i]] = joint_names_[i];
  }

  for (const moveit::core::JointModel* jm : joint_model_group_->getFixedJointModels())
  {
    if (!jm->getParentLinkModel())  // root joint doesn't have a parent
      continue;

    fixed_link_resolution_map[jm->getName()] = jm->getParentLinkModel()->getParentJointModel()->getName();
  }

  // TODO - is this just the joint_roots_?
  for (const moveit::core::LinkModel* link : joint_model_group_->getUpdatedLinkModels())
  {
    if (fixed_link_resolution_map.find(link->getParentJointModel()->getName()) == fixed_link_resolution_map.end())
    {
      const moveit::core::JointModel* parent_model = nullptr;
      bool found_root = false;

      while (!found_root)
      {
        if (parent_model == nullptr)
        {
          parent_model = link->getParentJointModel();
        }
        else
        {
          parent_model = parent_model->getParentLinkModel()->getParentJointModel();
          for (const std::string& joint_name : joint_names_)
          {
            if (parent_model->getName() == joint_name)
            {
              found_root = true;
            }
          }
        }
      }
      fixed_link_resolution_map[link->getParentJointModel()->getName()] = parent_model->getName();
    }
  }

  int start = free_vars_start_;
  int end = free_vars_end_;
  for (int i = start; i <= end; ++i)
  {
    size_t j = 0;
    for (const collision_detection::GradientInfo& info : gsr_->gradients_)
    {
      for (size_t k = 0; k < info.sphere_locations.size(); k++)
      {
        if (fixed_link_resolution_map.find(info.joint_name) != fixed_link_resolution_map.end())
        {
          collision_point_joint_names_[i][j] = fixed_link_resolution_map[info.joint_name];
        }
        else
        {
          ROS_ERROR("Couldn't find joint %s!", info.joint_name.c_str());
        }
        j++;
      }
    }
  }
  initialized_ = true;
}

ChompOptimizer::~ChompOptimizer()
{
  destroy();
}

void ChompOptimizer::registerParents(const moveit::core::JointModel* model)
{
  const moveit::core::JointModel* parent_model = nullptr;
  bool found_root = false;

  if (model == robot_model_->getRootJoint())
    return;

  while (!found_root)
  {
    if (parent_model == nullptr)
    {
      if (model->getParentLinkModel() == nullptr)
      {
        ROS_ERROR_STREAM("Model " << model->getName() << " not root but has NULL link model parent");
        return;
      }
      else if (model->getParentLinkModel()->getParentJointModel() == nullptr)
      {
        ROS_ERROR_STREAM("Model " << model->getName() << " not root but has NULL joint model parent");
        return;
      }
      parent_model = model->getParentLinkModel()->getParentJointModel();
    }
    else
    {
      if (parent_model == robot_model_->getRootJoint())
      {
        found_root = true;
      }
      else
      {
        parent_model = parent_model->getParentLinkModel()->getParentJointModel();
      }
    }
    joint_parent_map_[model->getName()][parent_model->getName()] = true;
  }
}

bool ChompOptimizer::optimize()
{
  bool optimization_result = 0;
  ros::WallTime start_time = ros::WallTime::now();
  // double averageCostVelocity = 0.0;
  // int currentCostIter = 0;
  int cost_window = 10;
  std::vector<double> costs(cost_window, 0.0);
  // double minimaThreshold = 0.05;
  bool should_break_out = false;

  // iterate
  for (iteration_ = 0; iteration_ < parameters_->max_iterations_; iteration_++)
  {
    ros::WallTime for_time = ros::WallTime::now();
    performForwardKinematics();
    ROS_DEBUG_STREAM("Forward kinematics took " << (ros::WallTime::now() - for_time));
    double c_cost = getCollisionCost();
    double s_cost = getSmoothnessCost();
    double cost = c_cost + s_cost;

    // ROS_INFO_STREAM("Collision cost " << cCost << " smoothness cost " << sCost);

    /// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC
    /// parameters values in the chomp_planning.yaml file so that CHOMP can find optimal paths

    // if(parameters_->getAddRandomness() && currentCostIter != -1)
    // {
    //   costs[currentCostIter] = cCost;
    //   currentCostIter++;

    //   if(currentCostIter >= costWindow)
    //   {
    //     for(int i = 1; i < costWindow; i++)
    //     {
    //       averageCostVelocity += (costs.at(i) - costs.at(i - 1));
    //     }

    //     averageCostVelocity /= (double)(costWindow);
    //     currentCostIter = -1;
    //   }
    // }

    if (iteration_ == 0)
    {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = cost;
      last_improvement_iteration_ = iteration_;
    }
    else if (cost < best_group_trajectory_cost_)
    {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = cost;
      last_improvement_iteration_ = iteration_;
    }
    calculateSmoothnessIncrements();
    calculateCollisionIncrements();
    calculateTotalIncrements();

    /// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC
    /// parameters values in the chomp_planning.yaml file so that CHOMP can find optimal paths

    // if(!parameters_->getUseHamiltonianMonteCarlo())
    // {
    //   // non-stochastic version:
    addIncrementsToTrajectory();
    // }
    // else
    // {
    //   // hamiltonian monte carlo updates:
    //   getRandomMomentum();
    //   updateMomentum();
    //   updatePositionFromMomentum();
    //   stochasticity_factor_ *= parameters_->getHmcAnnealingFactor();
    // }

    handleJointLimits();
    updateFullTrajectory();

    if (iteration_ % 10 == 0)
    {
      ROS_INFO("iteration: %d", iteration_);
      if (isCurrentTrajectoryMeshToMeshCollisionFree())
      {
        num_collision_free_iterations_ = 0;
        ROS_INFO("Chomp Got mesh to mesh safety at iter %d. Breaking out early.", iteration_);
        is_collision_free_ = true;
        iteration_++;
        should_break_out = true;
      }
      // } else if(safety == CollisionProximitySpace::InCollisionSafe) {

      /// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC
      /// parameters values in the chomp_planning.yaml file so that CHOMP can find optimal paths

      // ROS_DEBUG("Trajectory cost: %f (s=%f, c=%f)", getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());
      // CollisionProximitySpace::TrajectorySafety safety = checkCurrentIterValidity();
      // if(safety == CollisionProximitySpace::MeshToMeshSafe)
      // {
      //   num_collision_free_iterations_ = 0;
      //   ROS_INFO("Chomp Got mesh to mesh safety at iter %d. Breaking out early.", iteration_);
      //   is_collision_free_ = true;
      //   iteration_++;
      //   should_break_out = true;
      // } else if(safety == CollisionProximitySpace::InCollisionSafe) {
      //   num_collision_free_iterations_ = parameters_->getMaxIterationsAfterCollisionFree();
      //   ROS_INFO("Chomp Got in collision safety at iter %d. Breaking out soon.", iteration_);
      //   is_collision_free_ = true;
      //   iteration_++;
      //   should_break_out = true;
      // }
      // else
      // {
      //   is_collision_free_ = false;
      // }
    }

    if (!parameters_->filter_mode_)
    {
      if (c_cost < parameters_->collision_threshold_)
      {
        num_collision_free_iterations_ = parameters_->max_iterations_after_collision_free_;
        is_collision_free_ = true;
        iteration_++;
        should_break_out = true;
      }
      else
      {
        // ROS_INFO_STREAM("cCost " << cCost << " over threshold " << parameters_->getCollisionThreshold());
      }
    }

    if ((ros::WallTime::now() - start_time).toSec() > parameters_->planning_time_limit_)
    {
      ROS_WARN("Breaking out early due to time limit constraints.");
      break;
    }

    /// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC
    /// parameters values in the chomp_planning.yaml file so that CHOMP can find optimal paths

    // if(fabs(averageCostVelocity) < minimaThreshold && currentCostIter == -1 && !is_collision_free_ &&
    // parameters_->getAddRandomness())
    // {
    //   ROS_INFO("Detected local minima. Attempting to break out!");
    //   int iter = 0;
    //   bool success = false;
    //   while(iter < 20 && !success)
    //   {
    //     performForwardKinematics();
    //     double original_cost = getTrajectoryCost();
    //     group_trajectory_backup_ = group_trajectory_.getTrajectory();
    //     perturbTrajectory();
    //     handleJointLimits();
    //     updateFullTrajectory();
    //     performForwardKinematics();
    //     double new_cost = getTrajectoryCost();
    //     iter ++;
    //     if(new_cost < original_cost)
    //     {
    //       ROS_INFO("Got out of minimum in %d iters!", iter);
    //       averageCostVelocity = 0.0;
    //       currentCostIter = 0;
    //       success = true;
    //     }
    //     else
    //     {
    //       group_trajectory_.getTrajectory() = group_trajectory_backup_;
    //       updateFullTrajectory();
    //       currentCostIter = 0;
    //       averageCostVelocity = 0.0;
    //       success = false;
    //     }

    //   }

    //   if(!success)
    //   {
    //     ROS_INFO("Failed to exit minimum!");
    //   }
    //}
    // else if (currentCostIter == -1)
    //{
    //  currentCostIter = 0;
    //  averageCostVelocity = 0.0;
    //}

    if (should_break_out)
    {
      collision_free_iteration_++;
      if (num_collision_free_iterations_ == 0)
      {
        break;
      }
      else if (collision_free_iteration_ > num_collision_free_iterations_)
      {
        // CollisionProximitySpace::TrajectorySafety safety = checkCurrentIterValidity();
        // if(safety != CollisionProximitySpace::MeshToMeshSafe &&
        //    safety != CollisionProximitySpace::InCollisionSafe) {
        //   ROS_WARN_STREAM("Apparently regressed");
        // }
        break;
      }
    }
  }

  if (is_collision_free_)
  {
    optimization_result = true;
    ROS_INFO("Chomp path is collision free");
  }
  else
  {
    optimization_result = false;
    ROS_ERROR("Chomp path is not collision free!");
  }

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  updateFullTrajectory();

  ROS_INFO("Terminated after %d iterations, using path from iteration %d", iteration_, last_improvement_iteration_);
  ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());
  ROS_INFO_STREAM("Time per iteration " << (ros::WallTime::now() - start_time).toSec() / (iteration_ * 1.0));

  return optimization_result;
}

bool ChompOptimizer::isCurrentTrajectoryMeshToMeshCollisionFree() const
{
  moveit_msgs::RobotTrajectory traj;
  traj.joint_trajectory.joint_names = joint_names_;

  for (size_t i = 0; i < group_trajectory_.getNumPoints(); i++)
  {
    trajectory_msgs::JointTrajectoryPoint point;
    for (size_t j = 0; j < group_trajectory_.getNumJoints(); j++)
    {
      point.positions.push_back(best_group_trajectory_(i, j));
    }
    traj.joint_trajectory.points.push_back(point);
  }
  moveit_msgs::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(start_state_, start_state_msg);
  return planning_scene_->isPathValid(start_state_msg, traj, planning_group_);
}

/// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC parameters
/// values in the chomp_planning.yaml file so that CHOMP can find optimal paths

// CollisionProximitySpace::TrajectorySafety ChompOptimizer::checkCurrentIterValidity()
// {
//   JointTrajectory jointTrajectory;
//   jointTrajectory.joint_names = joint_names_;
//   jointTrajectory.header.frame_id = collision_space_->getCollisionModelsInterface()->getRobotFrameId();
//   jointTrajectory.header.stamp = ros::Time::now();
//   Constraints goalConstraints;
//   Constraints pathConstraints;
//   ArmNavigationErrorCodes errorCode;
//   vector<ArmNavigationErrorCodes> trajectoryErrorCodes;
//   for(int i = 0; i < group_trajectory_.getNumPoints(); i++)
//   {
//     JointTrajectoryPoint point;
//     for(int j = 0; j < group_trajectory_.getNumJoints(); j++)
//     {
//       point.positions.push_back(best_group_trajectory_(i, j));
//     }
//     jointTrajectory.points.push_back(point);
//   }

//   return collision_space_->isTrajectorySafe(jointTrajectory, goalConstraints, pathConstraints, planning_group_);
//   /*
//     bool valid = collision_space_->getCollisionModelsInterface()->isJointTrajectoryValid(*state_,
//     jointTrajectory,
//     goalConstraints,
//     pathConstraints, errorCode,
//     trajectoryErrorCodes, false);
//   */

// }

void ChompOptimizer::calculateSmoothnessIncrements()
{
  for (int i = 0; i < num_joints_; i++)
  {
    joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
    smoothness_increments_.col(i) = -smoothness_derivative_.segment(group_trajectory_.getStartIndex(), num_vars_free_);
  }
}

void ChompOptimizer::calculateCollisionIncrements()
{
  double potential;
  double vel_mag_sq;
  double vel_mag;
  Eigen::Vector3d potential_gradient;
  Eigen::Vector3d normalized_velocity;
  Eigen::Matrix3d orthogonal_projector;
  Eigen::Vector3d curvature_vector;
  Eigen::Vector3d cartesian_gradient;

  collision_increments_.setZero(num_vars_free_, num_joints_);

  int start_point = 0;
  int end_point = free_vars_end_;

  // In stochastic descent, simply use a random point in the trajectory, rather than all the trajectory points.
  // This is faster and guaranteed to converge, but it may take more iterations in the worst case.
  if (parameters_->use_stochastic_descent_)
  {
    start_point = (int)(getRandomDouble() * (free_vars_end_ - free_vars_start_) + free_vars_start_);
    if (start_point < free_vars_start_)
      start_point = free_vars_start_;
    if (start_point > free_vars_end_)
      start_point = free_vars_end_;
    end_point = start_point;
  }
  else
  {
    start_point = free_vars_start_;
  }

  for (int i = start_point; i <= end_point; i++)
  {
    for (int j = 0; j < num_collision_points_; j++)
    {
      potential = collision_point_potential_[i][j];

      if (potential < 0.0001)
        continue;

      potential_gradient = -collision_point_potential_gradient_[i][j];

      vel_mag = collision_point_vel_mag_[i][j];
      vel_mag_sq = vel_mag * vel_mag;

      // all math from the CHOMP paper:

      normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
      orthogonal_projector = Eigen::Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
      curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
      cartesian_gradient = vel_mag * (orthogonal_projector * potential_gradient - potential * curvature_vector);

      // pass it through the jacobian transpose to get the increments
      getJacobian(i, collision_point_pos_eigen_[i][j], collision_point_joint_names_[i][j], jacobian_);

      if (parameters_->use_pseudo_inverse_)
      {
        calculatePseudoInverse();
        collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_pseudo_inverse_ * cartesian_gradient;
      }
      else
      {
        collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_.transpose() * cartesian_gradient;
      }

      /*
        if(point_is_in_collision_[i][j])
        {
        break;
        }
      */
    }
  }
  // cout << collision_increments_ << endl;
}

void ChompOptimizer::calculatePseudoInverse()
{
  jacobian_jacobian_tranpose_ =
      jacobian_ * jacobian_.transpose() + Eigen::MatrixXd::Identity(3, 3) * parameters_->pseudo_inverse_ridge_factor_;
  jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void ChompOptimizer::calculateTotalIncrements()
{
  for (int i = 0; i < num_joints_; i++)
  {
    final_increments_.col(i) =
        parameters_->learning_rate_ * (joint_costs_[i].getQuadraticCostInverse() *
                                       (parameters_->smoothness_cost_weight_ * smoothness_increments_.col(i) +
                                        parameters_->obstacle_cost_weight_ * collision_increments_.col(i)));
  }
}

void ChompOptimizer::addIncrementsToTrajectory()
{
  const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group_->getActiveJointModels();
  for (size_t i = 0; i < joint_models.size(); i++)
  {
    double scale = 1.0;
    double max = final_increments_.col(i).maxCoeff();
    double min = final_increments_.col(i).minCoeff();
    double max_scale = parameters_->joint_update_limit_ / fabs(max);
    double min_scale = parameters_->joint_update_limit_ / fabs(min);
    if (max_scale < scale)
      scale = max_scale;
    if (min_scale < scale)
      scale = min_scale;
    group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
  }
  // ROS_DEBUG("Scale: %f",scale);
  // group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
}

void ChompOptimizer::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void ChompOptimizer::debugCost()
{
  double cost = 0.0;
  for (int i = 0; i < num_joints_; i++)
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  std::cout << "Cost = " << cost << std::endl;
}

double ChompOptimizer::getTrajectoryCost()
{
  return getSmoothnessCost() + getCollisionCost();
}

double ChompOptimizer::getSmoothnessCost()
{
  double smoothness_cost = 0.0;
  // joint costs:
  for (int i = 0; i < num_joints_; i++)
    smoothness_cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));

  return parameters_->smoothness_cost_weight_ * smoothness_cost;
}

double ChompOptimizer::getCollisionCost()
{
  double collision_cost = 0.0;

  double worst_collision_cost = 0.0;
  worst_collision_cost_state_ = -1;

  // collision costs:
  for (int i = free_vars_start_; i <= free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    for (int j = 0; j < num_collision_points_; j++)
    {
      state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
    }
    collision_cost += state_collision_cost;
    if (state_collision_cost > worst_collision_cost)
    {
      worst_collision_cost = state_collision_cost;
      worst_collision_cost_state_ = i;
    }
  }

  return parameters_->obstacle_cost_weight_ * collision_cost;
}

void ChompOptimizer::computeJointProperties(int trajectory_point)
{
  for (int j = 0; j < num_joints_; j++)
  {
    const moveit::core::JointModel* joint_model = state_.getJointModel(joint_names_[j]);
    const moveit::core::RevoluteJointModel* revolute_joint =
        dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
    const moveit::core::PrismaticJointModel* prismatic_joint =
        dynamic_cast<const moveit::core::PrismaticJointModel*>(joint_model);

    std::string parent_link_name = joint_model->getParentLinkModel()->getName();
    std::string child_link_name = joint_model->getChildLinkModel()->getName();
    Eigen::Isometry3d joint_transform = state_.getGlobalLinkTransform(parent_link_name) *
                                        (robot_model_->getLinkModel(child_link_name)->getJointOriginTransform() *
                                         (state_.getJointTransform(joint_model)));

    // joint_transform = inverseWorldTransform * jointTransform;
    Eigen::Vector3d axis;

    if (revolute_joint != nullptr)
    {
      axis = revolute_joint->getAxis();
    }
    else if (prismatic_joint != nullptr)
    {
      axis = prismatic_joint->getAxis();
    }
    else
    {
      axis = Eigen::Vector3d::Identity();
    }

    axis = joint_transform * axis;

    joint_axes_[trajectory_point][j] = axis;
    joint_positions_[trajectory_point][j] = joint_transform.translation();
  }
}

template <typename Derived>
void ChompOptimizer::getJacobian(int trajectory_point, Eigen::Vector3d& collision_point_pos, std::string& joint_name,
                                 Eigen::MatrixBase<Derived>& jacobian) const
{
  for (int j = 0; j < num_joints_; j++)
  {
    if (isParent(joint_name, joint_names_[j]))
    {
      Eigen::Vector3d column = joint_axes_[trajectory_point][j].cross(
          Eigen::Vector3d(collision_point_pos(0), collision_point_pos(1), collision_point_pos(2)) -
          joint_positions_[trajectory_point][j]);

      jacobian.col(j)[0] = column.x();
      jacobian.col(j)[1] = column.y();
      jacobian.col(j)[2] = column.z();
    }
    else
    {
      jacobian.col(j)[0] = 0.0;
      jacobian.col(j)[1] = 0.0;
      jacobian.col(j)[2] = 0.0;
    }
  }
}

void ChompOptimizer::handleJointLimits()
{
  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
  for (size_t joint_i = 0; joint_i < joint_models.size(); joint_i++)
  {
    const moveit::core::JointModel* joint_model = joint_models[joint_i];

    if (joint_model->getType() == moveit::core::JointModel::REVOLUTE)
    {
      const moveit::core::RevoluteJointModel* revolute_joint =
          dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
      if (revolute_joint->isContinuous())
      {
        continue;
      }
    }

    const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();

    double joint_max = -DBL_MAX;
    double joint_min = DBL_MAX;

    for (const moveit::core::VariableBounds& bound : bounds)
    {
      if (bound.min_position_ < joint_min)
      {
        joint_min = bound.min_position_;
      }

      if (bound.max_position_ > joint_max)
      {
        joint_max = bound.max_position_;
      }
    }

    int count = 0;

    bool violation = false;
    do
    {
      double max_abs_violation = 1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      for (int i = free_vars_start_; i <= free_vars_end_; i++)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        if (group_trajectory_(i, joint_i) > joint_max)
        {
          amount = joint_max - group_trajectory_(i, joint_i);
          absolute_amount = fabs(amount);
        }
        else if (group_trajectory_(i, joint_i) < joint_min)
        {
          amount = joint_min - group_trajectory_(i, joint_i);
          absolute_amount = fabs(amount);
        }
        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier =
            max_violation / joint_costs_[joint_i].getQuadraticCostInverse()(free_var_index, free_var_index);
        group_trajectory_.getFreeJointTrajectoryBlock(joint_i) +=
            multiplier * joint_costs_[joint_i].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    } while (violation);
  }
}

void ChompOptimizer::performForwardKinematics()
{
  double inv_time = 1.0 / group_trajectory_.getDiscretization();
  double inv_time_sq = inv_time * inv_time;

  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_ == 0)
  {
    start = 0;
    end = num_vars_all_ - 1;
  }

  is_collision_free_ = true;

  ros::WallDuration total_dur(0.0);

  // for each point in the trajectory
  for (int i = start; i <= end; ++i)
  {
    // Set Robot state from trajectory point...
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = planning_group_;
    setRobotStateFromPoint(group_trajectory_, i);
    ros::WallTime grad = ros::WallTime::now();

    hy_env_->getCollisionGradients(req, res, state_, nullptr, gsr_);
    total_dur += (ros::WallTime::now() - grad);
    computeJointProperties(i);
    state_is_in_collision_[i] = false;

    // Keep vars in scope
    {
      size_t j = 0;
      for (const collision_detection::GradientInfo& info : gsr_->gradients_)
      {
        for (size_t k = 0; k < info.sphere_locations.size(); k++)
        {
          collision_point_pos_eigen_[i][j][0] = info.sphere_locations[k].x();
          collision_point_pos_eigen_[i][j][1] = info.sphere_locations[k].y();
          collision_point_pos_eigen_[i][j][2] = info.sphere_locations[k].z();

          collision_point_potential_[i][j] =
              getPotential(info.distances[k], info.sphere_radii[k], parameters_->min_clearance_);
          collision_point_potential_gradient_[i][j][0] = info.gradients[k].x();
          collision_point_potential_gradient_[i][j][1] = info.gradients[k].y();
          collision_point_potential_gradient_[i][j][2] = info.gradients[k].z();

          point_is_in_collision_[i][j] = (info.distances[k] - info.sphere_radii[k] < info.sphere_radii[k]);

          if (point_is_in_collision_[i][j])
          {
            state_is_in_collision_[i] = true;
            // if(is_collision_free_ == true) {
            //   ROS_INFO_STREAM("We know it's not collision free " << g);
            //   ROS_INFO_STREAM("Sphere location " << info.sphere_locations[k].x() << " " <<
            //   info.sphere_locations[k].y() << " " << info.sphere_locations[k].z());
            //   ROS_INFO_STREAM("Gradient " << info.gradients[k].x() << " " << info.gradients[k].y() << " " <<
            //   info.gradients[k].z() << " distance " << info.distances[k] << " radii " << info.sphere_radii[k]);
            //   ROS_INFO_STREAM("Radius " << info.sphere_radii[k] << " potential " <<
            //   collision_point_potential_[i][j]);
            // }

            is_collision_free_ = false;
          }
          j++;
        }
      }
    }
  }

  // ROS_INFO_STREAM("Total dur " << total_dur << " total checks " << end-start+1);

  // now, get the vel and acc for each collision point (using finite differencing)
  for (int i = free_vars_start_; i <= free_vars_end_; i++)
  {
    for (int j = 0; j < num_collision_points_; j++)
    {
      collision_point_vel_eigen_[i][j] = Eigen::Vector3d(0, 0, 0);
      collision_point_acc_eigen_[i][j] = Eigen::Vector3d(0, 0, 0);
      for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
      {
        collision_point_vel_eigen_[i][j] +=
            (inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[i + k][j];
        collision_point_acc_eigen_[i][j] +=
            (inv_time_sq * DIFF_RULES[1][k + DIFF_RULE_LENGTH / 2]) * collision_point_pos_eigen_[i + k][j];
      }

      // get the norm of the velocity:
      collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
    }
  }
}

void ChompOptimizer::setRobotStateFromPoint(ChompTrajectory& group_trajectory, int i)
{
  const Eigen::MatrixXd::RowXpr& point = group_trajectory.getTrajectoryPoint(i);

  std::vector<double> joint_states;
  joint_states.reserve(group_trajectory.getNumJoints());
  for (size_t j = 0; j < group_trajectory.getNumJoints(); j++)
    joint_states.emplace_back(point(0, j));

  state_.setJointGroupPositions(planning_group_, joint_states);
  state_.update();
}

void ChompOptimizer::perturbTrajectory()
{
  // int mid_point = (free_vars_start_ + free_vars_end_) / 2;
  if (worst_collision_cost_state_ < 0)
    return;
  int mid_point = worst_collision_cost_state_;
  moveit::core::RobotState random_state = state_;
  const moveit::core::JointModelGroup* planning_group = state_.getJointModelGroup(planning_group_);
  random_state.setToRandomPositions(planning_group);
  std::vector<double> vals;
  random_state.copyJointGroupPositions(planning_group_, vals);
  double* ptr = &vals[0];
  Eigen::Map<Eigen::VectorXd> random_matrix(ptr, vals.size());
  // Eigen::VectorXd random_matrix = vals;

  // convert the state into an increment
  random_matrix -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

  // project the increment orthogonal to joint velocities
  group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
  joint_state_velocities_.normalize();
  random_matrix = (Eigen::MatrixXd::Identity(num_joints_, num_joints_) -
                   joint_state_velocities_ * joint_state_velocities_.transpose()) *
                  random_matrix;

  int mp_free_vars_index = mid_point - free_vars_start_;
  for (int i = 0; i < num_joints_; i++)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) +=
        joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
  }
}

/// TODO: HMC BASED COMMENTED CODE BELOW, Need to uncomment and perform extensive testing by varying the HMC parameters
/// values in the chomp_planning.yaml file so that CHOMP can find optimal paths
// void ChompOptimizer::getRandomState(const RobotState currentState, const string& groupName, Eigen::VectorXd&
// state_vec)
// {
//   const vector<RobotState *::JointState*>& jointStates =
//     currentState->getJointStateGroup(groupName)->getJointStateVector();
//   for(size_t i = 0; i < jointStates.size(); i++)
//   {

//     bool continuous = false;

//     RobotState *::JointState* jointState = jointStates[i];
//     const RevoluteJointModel* revolute_joint
//       = dynamic_cast<const RevoluteJointModel*>(jointState->getJointModel());
//     if(revolute_joint && revolute_joint->continuous_) {
//       continuous = true;
//     }

//     map<string, pair<double, double> > bounds = jointState->getJointModel()->getAllVariableBounds();
//     int j = 0;
//     for(map<string, pair<double, double> >::iterator it = bounds.begin(); it != bounds.end(); it++)
//     {
//       double randVal = jointState->getJointStateValues()[j] + (getRandomDouble()
//                                                                * (parameters_->getRandomJumpAmount()) -
//                                                                getRandomDouble() *
//                                                                (parameters_->getRandomJumpAmount()));

//       if(!continuous)
//       {
//         if(randVal > it->second.second)
//         {
//           randVal = it->second.second;
//         }
//         else if(randVal < it->second.first)
//         {
//           randVal = it->second.first;
//         }
//       }

//       ROS_DEBUG_STREAM("Joint " << it->first << " old value " << jointState->getJointStateValues()[j] << " new value
//       " << randVal);
//       state_vec(i) = randVal;

//       j++;
//     }
//   }
// }

/*
void ChompOptimizer::getRandomMomentum()
{
  if (is_collision_free_)
    random_momentum_.setZero(num_vars_free_, num_joints_);
  else
    for (int i = 0; i < num_joints_; ++i)
    {
      multivariate_gaussian_[i].sample(random_joint_momentum_);
      random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
    }
}

void ChompOptimizer::updateMomentum()
{
  // double alpha = 1.0 - parameters_->getHmcStochasticity();
  double eps = parameters_->getHmcDiscretization();
  if (iteration_ > 0)
    momentum_ = (momentum_ + eps * final_increments_);
  else
    momentum_ = random_momentum_;
  // momentum_ = alpha * (momentum_ + eps*final_increments_) + sqrt(1.0-alpha*alpha)*random_momentum_;
}

void ChompOptimizer::updatePositionFromMomentum()
{
  double eps = parameters_->getHmcDiscretization();
  group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
}
*/

}  // namespace chomp
