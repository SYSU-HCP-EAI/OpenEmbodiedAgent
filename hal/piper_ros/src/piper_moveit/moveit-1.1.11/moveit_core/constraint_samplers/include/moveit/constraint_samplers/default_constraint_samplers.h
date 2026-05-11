/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/macros/class_forward.h>
#include <random_numbers/random_numbers.h>

namespace constraint_samplers
{
random_numbers::RandomNumberGenerator createSeededRNG(const std::string& seed_param);

MOVEIT_CLASS_FORWARD(JointConstraintSampler);  // Defines JointConstraintSamplerPtr, ConstPtr, WeakPtr... etc

/**
 * \brief JointConstraintSampler is a class that allows the sampling
 * of joints in a particular group of the robot, subject to a set of individual joint constraints.
 *
 * The set of individual joint constraint reduce the allowable bounds
 * used in the sampling.  Unconstrained values will be sampled within
 * their limits.
 *
 */
class JointConstraintSampler : public ConstraintSampler
{
public:
  /**
   * Constructor
   *
   * @param [in] scene The planning scene used to check the constraint
   *
   * @param [in] group_name The group name associated with the
   * constraint.  Will be invalid if no group name is passed in or the
   * joint model group cannot be found in the kinematic model
   *
   */
  JointConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name)
    : ConstraintSampler(scene, group_name)
    , random_number_generator_(createSeededRNG("~joint_constraint_sampler_random_seed"))
  {
  }

  /**
   * \brief Configures a joint constraint given a Constraints message.
   *
   * If more than one constraint for a particular joint is specified,
   * the most restrictive set of bounds will be used (highest minimum
   * value, lowest maximum value).  For the configuration to be
   * successful, the following condition must be met, in addition to
   * the conditions specified in \ref configure(const std::vector<kinematic_constraints::JointConstraint> &jc) :

   * \li The Constraints message must contain one or more valid joint
   * constraints (where validity is judged by the ability to configure
   * a \ref JointConstraint)
   *
   * @param [in] constr The message containing the constraints
   *
   * @return True if the conditions are met, otherwise false
   */
  bool configure(const moveit_msgs::Constraints& constr) override;

  /**
   * \brief Configures a joint constraint given a vector of constraints.
   *
   * If more than one constraint for a particular joint is specified,
   * the most restrictive set of bounds will be used (highest minimum
   * value, lowest_maximum value.  For the configuration to be
   * successful, the following conditions must be met:

   * \li The vector must contain one or more valid, enabled joint
   * constraints
   *
   * \li At least one constraint must reference a joint in the
   * indicated group.  If no additional bounds exist for this group,
   * then RobotState::setToRandomPositions() can be
   * used to generate a sample independently from the
   * constraint_samplers infrastructure.
   *
   * \li The constraints must allow a sampleable region for all
   * joints, where the most restrictive minimum bound is less than the
   * most restrictive maximum bound
   *
   * @param [in] jc The vector of joint constraints
   *
   * @return True if the conditions are met, otherwise false
   */
  bool configure(const std::vector<kinematic_constraints::JointConstraint>& jc);

  bool sample(moveit::core::RobotState& state, const moveit::core::RobotState& ks, unsigned int max_attempts) override;

  /**
   * \brief Gets the number of constrained joints - joints that have an
   * additional bound beyond the joint limits.
   *
   *
   * @return The number of constrained joints.
   */
  std::size_t getConstrainedJointCount() const
  {
    return bounds_.size();
  }

  /**
   * \brief Gets the number of unconstrained joints - joint that have
   * no additional bound beyond the joint limits.
   *
   * @return The number of unconstrained joints.
   */
  std::size_t getUnconstrainedJointCount() const
  {
    return unbounded_.size();
  }

  /**
   * \brief Get the name of the constraint sampler, for debugging purposes
   * should be in CamelCase format.
   * \return string of name
   */
  const std::string& getName() const override
  {
    static const std::string SAMPLER_NAME = "JointConstraintSampler";
    return SAMPLER_NAME;
  }

protected:
  /// \brief An internal structure used for maintaining constraints on a particular joint
  struct JointInfo
  {
    /**
     * \brief Constructor
     *
     * @return
     */
    JointInfo()
    {
      min_bound_ = -std::numeric_limits<double>::max();
      max_bound_ = std::numeric_limits<double>::max();
    }

    /**
     * \brief Function that adjusts the joints only if they are more
     * restrictive.  This means that the min limit is higher than the
     * current limit, or the max limit is lower than the current max
     * limit.
     *
     * @param min The min limit for potential adjustment
     * @param max The max limit for potential adjustment
     */
    void potentiallyAdjustMinMaxBounds(double min, double max)
    {
      min_bound_ = std::max(min, min_bound_);
      max_bound_ = std::min(max, max_bound_);
    }

    double min_bound_;  /**< The most restrictive min value of those set */
    double max_bound_;  /**< The most restrictive max value of those set */
    std::size_t index_; /**< The index within the joint state vector for this joint */
  };

  void clear() override;

  random_numbers::RandomNumberGenerator random_number_generator_; /**< \brief Random number generator used to sample */
  std::vector<JointInfo> bounds_; /**< \brief The bounds for any joint with bounds that are more restrictive than the
                                     joint limits */

  std::vector<const moveit::core::JointModel*> unbounded_; /**< \brief The joints that are not bounded except by joint
                                                             limits */
  std::vector<unsigned int> uindex_; /**< \brief The index of the unbounded joints in the joint state vector */
  std::vector<double> values_;       /**< \brief Values associated with this group to avoid continuously reallocating */
};

/**
 * \brief A structure for potentially holding a position constraint
 * and an orientation constraint for use during Ik Sampling
 *
 */
struct IKSamplingPose
{
  /**
   * \brief Empty constructor.
   *
   * @return
   */
  IKSamplingPose();

  /**
   * \brief Constructor that takes a single pose constraint, doing a copy
   *
   * @param pc The pose constraint that will be copied into the internal variable
   *
   */
  IKSamplingPose(const kinematic_constraints::PositionConstraint& pc);

  /**
   * \brief Constructor that takes a single orientation constraint, doing a copy
   *
   * @param oc The orientation constraint that will be copied into the internal variable
   *
   * @return
   */
  IKSamplingPose(const kinematic_constraints::OrientationConstraint& oc);

  /**
   * \brief Constructor that takes both a position and an orientation
   * constraint, copying both into the internal variables
   *
   * @param pc The pose constraint that will be copied into the internal variable
   * @param oc The orientation constraint that will be copied into the internal variable
   *
   * @return
   */
  IKSamplingPose(const kinematic_constraints::PositionConstraint& pc,
                 const kinematic_constraints::OrientationConstraint& oc);

  /**
   * \brief Constructor that takes a pointer to a position constraint.
   *
   * @param pc Pointer for copying into internal variable
   *
   * @return
   */
  IKSamplingPose(const kinematic_constraints::PositionConstraintPtr& pc);

  /**
   * \brief Constructor that takes a pointer to a orientation constraint.
   *
   * @param oc Pointer for copying into internal variable
   *
   * @return
   */
  IKSamplingPose(const kinematic_constraints::OrientationConstraintPtr& oc);

  /**
   * \brief Constructor that takes a pointer to both position and orientation constraints.
   *
   * @param pc Pointer for copying into internal variables
   * @param oc Pointer for copying into internal variable
   *
   * @return
   */
  IKSamplingPose(const kinematic_constraints::PositionConstraintPtr& pc,
                 const kinematic_constraints::OrientationConstraintPtr& oc);

  kinematic_constraints::PositionConstraintPtr position_constraint_; /**< \brief Holds the position constraint for
                                                                        sampling */
  kinematic_constraints::OrientationConstraintPtr
      orientation_constraint_; /**< \brief Holds the orientation constraint for sampling */
};

MOVEIT_CLASS_FORWARD(IKConstraintSampler);  // Defines IKConstraintSamplerPtr, ConstPtr, WeakPtr... etc

/**
 * \brief A class that allows the sampling of IK constraints.
 *
 * An IK constraint can have a position constraint, and orientation
 * constraint, or both.  The constraint will attempt to sample a pose
 * that adheres to the constraint, and then solves IK for that pose.
 *
 */
class IKConstraintSampler : public ConstraintSampler
{
public:
  /**
   * \brief Constructor
   *
   * @param [in] scene The planning scene used to check the constraint
   *
   * @param [in] group_name The group name associated with the
   * constraint.  Will be invalid if no group name is passed in or the
   * joint model group cannot be found in the kinematic model
   *
   */
  IKConstraintSampler(const planning_scene::PlanningSceneConstPtr& scene, const std::string& group_name)
    : ConstraintSampler(scene, group_name)
    , random_number_generator_(createSeededRNG("~ik_constraint_sampler_random_seed"))
  {
  }

  /**
   * \brief Configures the IK constraint given a constraints message.
   *
   * If the constraints message contains both orientation constraints
   * and positions constraints, the function iterates through each
   * potential pair until it finds a pair of position orientation
   * constraints that lead to valid configuration of kinematic
   * constraints.  It creates an IKSamplingPose from these and calls
   * \ref configure(const IKSamplingPose &sp).  If no pair leads to
   * both having valid configuration, it will attempt to iterate
   * through the position constraints in the Constraints message,
   * calling \ref configure(const IKSamplingPose &sp) on the resulting
   * IKSamplingPose.  Finally, if no valid position constraints exist
   * it will attempt the same procedure with the orientation
   * constraints.  If no valid position or orientation constraints
   * exist, it will return false.  For more information, see the docs
   * for \ref configure(const IKSamplingPose &sp).
   *
   * @param constr The Constraint message
   *
   * @return True if some valid position and orientation constraints
   * exist and the overloaded configuration function returns true.
   * Otherwise, returns false.
   */
  bool configure(const moveit_msgs::Constraints& constr) override;

  /**
   * \brief Configures the Constraint given a IKSamplingPose.
   *
   *
   * This function performs the actual constraint configuration.  It returns true if the following are true:
   * \li The \ref IKSamplingPose has either a valid orientation or position constraint
   * \li The position and orientation constraints are specified for the same link
   *
   * \li There is a valid IK solver instance for the indicated group.
   * This will be only be the case if a group has a specific solver
   * associated with it.  For situations where the super-group doesn't
   * have a solver, but all subgroups have solvers, then use the
   * \ref ConstraintSamplerManager.
   *
   * \li The kinematic model has both the links associated with the IK
   * solver's tip and base frames.
   *
   * \li The link specified in the constraints is the tip link of the IK solver
   *
   * @param [in] sp The variable that contains the position and orientation constraints
   *
   * @return True if all conditions are met and the group specified in
   * the constructor is valid.  Otherwise, false.
   */
  bool configure(const IKSamplingPose& sp);

  /**
   * \brief Gets the timeout argument passed to the IK solver
   *
   *
   * @return The IK timeout
   */
  double getIKTimeout() const
  {
    return ik_timeout_;
  }

  /**
   * \brief Sets the timeout argument passed to the IK solver
   *
   * @param timeout The timeout argument that will be used in future IK calls
   */
  void setIKTimeout(double timeout)
  {
    ik_timeout_ = timeout;
  }

  /**
   * \brief Gets the position constraint associated with this sampler.
   *
   *
   * @return The position constraint, or an empty shared_ptr if none has been specified
   */
  const kinematic_constraints::PositionConstraintPtr& getPositionConstraint() const
  {
    return sampling_pose_.position_constraint_;
  }
  /**
   * \brief Gets the orientation constraint associated with this sampler.
   *
   *
   * @return The orientation constraint, or an empty shared_ptr if none has been specified
   */
  const kinematic_constraints::OrientationConstraintPtr& getOrientationConstraint() const
  {
    return sampling_pose_.orientation_constraint_;
  }

  /**
   * \brief Gets the volume associated with the position and orientation constraints.
   *
   * This function computes the volume of the sampling constraint.
   * The volume associated with the position constraint is either the
   * product of the volume of all position constraint regions, or 1.0
   * otherwise.  The volume associated with the orientation constraint
   * is the product of all the axis tolerances, or 1.0 otherwise.  If
   * both are specified, the product of the volumes is returned.

   * @return Returns the sum of the volumes of all constraint regions
   * associated with the position and orientation constraints.
   */
  double getSamplingVolume() const;

  /**
   * \brief Gets the link name associated with this sampler
   *
   *
   * @return The associated link name
   */
  const std::string& getLinkName() const;

  /**
   * \brief Produces an IK sample.
   *
   * This function first calls the \ref samplePose function to produce
   * a position and orientation in the constraint region.  It then
   * calls IK on that pose.  If a pose that satisfies the constraints
   * can be determined, and IK returns a solution for that pose, then
   * the joint values associated with the joint group will be
   * populated with the results of the IK, and the function will
   * return true.  The function will attempt to sample a pose up to
   * max_attempts number of times, and then call IK on that value.  If
   * IK is not successful, it will repeat the pose sample and IK
   * procedure max_attempt times.  If in any iteration a valid pose
   * cannot be sample within max_attempts time, it will return false.
   *
   * @param jsg The joint state group in question.  Must match the group passed in the constructor or will return false.
   * @param ks A reference state that will be used for transforming the IK poses
   * @param max_attempts The number of attempts to both sample and try IK
   *
   * @return True if a valid sample pose was produced and valid IK found for that pose.  Otherwise false.
   */
  bool sample(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state,
              unsigned int max_attempts) override;

  /**
   * \brief Returns a pose that falls within the constraint regions.
   *
   * If a position constraint is specified, then a position is
   * produced by selecting a random region among the constraint
   * regions and taking a sample in that region.  If no region is
   * valid the function returns false.  If no position constraint is
   * specified, a position is produced by assigning a random valid
   * value to each group joint, performing forward kinematics, and
   * taking the resulting pose.  If an orientation constraint is
   * specified, then an quaternion is produced by sampling a
   * difference value within the axis tolerances and applying the
   * difference rotation to the orientation constraint target.
   * Otherwise, a random quaternion is produced.
   *
   * @param [out] pos The position component of the sample
   * @param [out] quat The orientation component of the sample. It will always be a normalized quaternion.
   * @param [in] ks The reference state used for performing transforms
   * @param [in] max_attempts The maximum attempts to try to sample - applies only to the position constraint
   *
   * @return True if a sample was successfully produced, otherwise false
   */
  bool samplePose(Eigen::Vector3d& pos, Eigen::Quaterniond& quat, const moveit::core::RobotState& ks,
                  unsigned int max_attempts);

  /**
   * \brief Get the name of the constraint sampler, for debugging purposes
   * should be in CamelCase format.
   * \return string of name
   */
  const std::string& getName() const override
  {
    static const std::string SAMPLER_NAME = "IKConstraintSampler";
    return SAMPLER_NAME;
  }

protected:
  void clear() override;

  /**
   * \brief Performs checks and sets various internal values associated with the IK solver
   *
   * @return True if the IK solver exists and if it associated with the expected base frame and tip frames.  Otherwise
   *false.
   */
  bool loadIKSolver();

  /**
   * \brief Actually calls IK on the given pose, generating a random seed state.
   *
   * @param ik_query The pose for solving IK, assumed to be for the tip frame in the base frame
   * @param timeout The timeout for the IK search
   * @param jsg The joint state group into which to place the solution
   * @param use_as_seed If true, the state values in jsg are used as seed for the IK
   *
   * @return True if IK returns successfully with the timeout, and otherwise false.
   */
  bool callIK(const geometry_msgs::Pose& ik_query,
              const kinematics::KinematicsBase::IKCallbackFn& adapted_ik_validity_callback, double timeout,
              moveit::core::RobotState& state, bool use_as_seed);
  bool sampleHelper(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state,
                    unsigned int max_attempts);
  bool validate(moveit::core::RobotState& state) const;

  random_numbers::RandomNumberGenerator random_number_generator_; /**< \brief Random generator used by the sampler */
  IKSamplingPose sampling_pose_;                                  /**< \brief Holder for the pose used for sampling */
  kinematics::KinematicsBaseConstPtr kb_;                         /**< \brief Holds the kinematics solver */
  double ik_timeout_;                                             /**< \brief Holds the timeout associated with IK */
  std::string ik_frame_;                                          /**< \brief Holds the base from of the IK solver */
  bool transform_ik_; /**< \brief True if the frame associated with the kinematic model is different than the base frame
                         of the IK solver */
  bool need_eef_to_ik_tip_transform_; /**< \brief True if the tip frame of the inverse kinematic is different than the
                                        frame of the end effector */
  Eigen::Isometry3d eef_to_ik_tip_transform_; /**< \brief Holds the transformation from end effector to IK tip frame */
};
}  // namespace constraint_samplers
