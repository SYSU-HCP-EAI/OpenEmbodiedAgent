/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <moveit_msgs/JointLimits.h>
#include <random_numbers/random_numbers.h>
#include <Eigen/Geometry>

namespace moveit
{
namespace core
{
struct VariableBounds
{
  VariableBounds()
    : min_position_(0.0)
    , max_position_(0.0)
    , position_bounded_(false)
    , min_velocity_(0.0)
    , max_velocity_(0.0)
    , velocity_bounded_(false)
    , min_acceleration_(0.0)
    , max_acceleration_(0.0)
    , acceleration_bounded_(false)
  {
  }

  double min_position_;
  double max_position_;
  bool position_bounded_;

  double min_velocity_;
  double max_velocity_;
  bool velocity_bounded_;

  double min_acceleration_;
  double max_acceleration_;
  bool acceleration_bounded_;
};

class LinkModel;
class JointModel;

/** \brief Data type for holding mappings from variable names to their position in a state vector */
typedef std::map<std::string, int> VariableIndexMap;

/** \brief Data type for holding mappings from variable names to their bounds */
using VariableBoundsMap = std::map<std::string, VariableBounds>;

/** \brief Map of names to instances for JointModel */
using JointModelMap = std::map<std::string, JointModel*>;

/** \brief Map of names to const instances for JointModel */
using JointModelMapConst = std::map<std::string, const JointModel*>;

/** \brief A joint from the robot. Models the transform that
    this joint applies in the kinematic chain. A joint
    consists of multiple variables. In the simplest case, when
    the joint is a single DOF, there is only one variable and
    its name is the same as the joint's name. For multi-DOF
    joints, each variable has a local name (e.g., \e x, \e y)
    but the full variable name as seen from the outside of
    this class is a concatenation of the "joint name"/"local
    name" (e.g., a joint named 'base' with local variables 'x'
    and 'y' will store its full variable names as 'base/x' and
    'base/y'). Local names are never used to reference
    variables directly. */
class JointModel
{
public:
  /** \brief The different types of joints we support */
  enum JointType
  {
    UNKNOWN,
    REVOLUTE,
    PRISMATIC,
    PLANAR,
    FLOATING,
    FIXED
  };

  /** \brief The datatype for the joint bounds */
  using Bounds = std::vector<VariableBounds>;

  /** \brief Construct a joint named \e name */
  JointModel(const std::string& name);

  virtual ~JointModel();

  /** \brief Get the name of the joint */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief Get the type of joint */
  JointType getType() const
  {
    return type_;
  }

  /** \brief Get the type of joint as a string */
  std::string getTypeName() const;

  /** \brief Get the link that this joint connects to. The
      robot is assumed to start with a joint, so the root
      joint will return a NULL pointer here. */
  const LinkModel* getParentLinkModel() const
  {
    return parent_link_model_;
  }

  /** \brief Get the link that this joint connects to. There will always be such a link */
  const LinkModel* getChildLinkModel() const
  {
    return child_link_model_;
  }

  void setParentLinkModel(const LinkModel* link)
  {
    parent_link_model_ = link;
  }

  void setChildLinkModel(const LinkModel* link)
  {
    child_link_model_ = link;
  }

  /** @name Reason about the variables that make up this joint
      @{ */

  /** \brief Get the names of the variables that make up this joint, in the order they appear in corresponding states.
      For single DOF joints, this will be just the joint name. For multi-DOF joints these will be the joint name
     followed by "/", followed by
      the local names of the variables */
  const std::vector<std::string>& getVariableNames() const
  {
    return variable_names_;
  }

  /** \brief Get the local names of the variable that make up the joint (suffixes that are attached to joint names to
     construct the variable names).
      For single DOF joints, this will be empty. */
  const std::vector<std::string>& getLocalVariableNames() const
  {
    return local_variable_names_;
  }

  /** \brief Check if a particular variable is known to this joint */
  bool hasVariable(const std::string& variable) const
  {
    return variable_index_map_.find(variable) != variable_index_map_.end();
  }

  /** \brief Get the number of variables that describe this joint */
  std::size_t getVariableCount() const
  {
    return variable_names_.size();
  }

  /** \brief Get the index of this joint's first variable within the full robot state */
  int getFirstVariableIndex() const
  {
    return first_variable_index_;
  }

  /** \brief Set the index of this joint's first variable within the full robot state */
  void setFirstVariableIndex(int index)
  {
    first_variable_index_ = index;
  }

  /** \brief Get the index of this joint within the robot model */
  int getJointIndex() const
  {
    return joint_index_;
  }

  /** \brief Set the index of this joint within the robot model */
  void setJointIndex(int index)
  {
    joint_index_ = index;
  }

  /** \brief Get the index of the variable within this joint */
  int getLocalVariableIndex(const std::string& variable) const;

  /** @} */

  /** @name Functionality specific to computing state values
      @{ */

  /** \brief Provide a default value for the joint given the default joint variable bounds (maintained internally).
      Most joints will use the default implementation provided in this base class, but the quaternion
      for example needs a different implementation. Enough memory is assumed to be allocated. */
  void getVariableDefaultPositions(double* values) const
  {
    getVariableDefaultPositions(values, variable_bounds_);
  }

  /** \brief Provide a default value for the joint given the joint variable bounds.
      Most joints will use the default implementation provided in this base class, but the quaternion
      for example needs a different implementation. Enough memory is assumed to be allocated. */
  virtual void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const = 0;

  /** \brief Provide random values for the joint variables (within default bounds). Enough memory is assumed to be
   * allocated. */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const
  {
    getVariableRandomPositions(rng, values, variable_bounds_);
  }

  /** \brief Provide random values for the joint variables (within specified bounds). Enough memory is assumed to be
   * allocated. */
  virtual void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values,
                                          const Bounds& other_bounds) const = 0;

  /** \brief Provide random values for the joint variables (within default bounds). Enough memory is assumed to be
   * allocated. */
  void getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values, const double* seed,
                                        const double distance) const
  {
    getVariableRandomPositionsNearBy(rng, values, variable_bounds_, seed, distance);
  }

  /** \brief Provide random values for the joint variables (within specified bounds). Enough memory is assumed to be
   * allocated. */
  virtual void getVariableRandomPositionsNearBy(random_numbers::RandomNumberGenerator& rng, double* values,
                                                const Bounds& other_bounds, const double* seed,
                                                const double distance) const = 0;

  /** @} */

  /** @name Functionality specific to verifying bounds
      @{ */

  /** \brief Check if the set of values for the variables of this joint are within bounds. */
  bool satisfiesPositionBounds(const double* values, double margin = 0.0) const
  {
    return satisfiesPositionBounds(values, variable_bounds_, margin);
  }

  /** \brief Check if the set of position values for the variables of this joint are within bounds, up to some margin.
   */
  virtual bool satisfiesPositionBounds(const double* values, const Bounds& other_bounds, double margin) const = 0;

  /** \brief Force the specified values to be inside bounds and normalized. Quaternions are normalized, continuous
     joints are made between -Pi and Pi.
      Returns true if changes were made. */
  bool enforcePositionBounds(double* values) const
  {
    return enforcePositionBounds(values, variable_bounds_);
  }

  /** \brief Force the specified values to be inside bounds and normalized. Quaternions are normalized, continuous
     joints are made between -Pi and Pi.
      Return true if changes were made. */
  virtual bool enforcePositionBounds(double* values, const Bounds& other_bounds) const = 0;

  /** Harmonize position of revolute joints, adding/subtracting multiples of 2*Pi to bring them back into bounds.
   *  Return true if changes were made. */
  virtual bool harmonizePosition(double* values, const Bounds& other_bounds) const;
  bool harmonizePosition(double* values) const
  {
    return harmonizePosition(values, variable_bounds_);
  }

  /** \brief Check if the set of velocities for the variables of this joint are within bounds. */
  bool satisfiesVelocityBounds(const double* values, double margin = 0.0) const
  {
    return satisfiesVelocityBounds(values, variable_bounds_, margin);
  }

  /** \brief Check if the set of velocities for the variables of this joint are within bounds, up to some margin. */
  virtual bool satisfiesVelocityBounds(const double* values, const Bounds& other_bounds, double margin) const;

  /** \brief Force the specified velocities to be within bounds. Return true if changes were made. */
  bool enforceVelocityBounds(double* values) const
  {
    return enforceVelocityBounds(values, variable_bounds_);
  }

  /** \brief Force the specified velocities to be inside bounds. Return true if changes were made. */
  virtual bool enforceVelocityBounds(double* values, const Bounds& other_bounds) const;

  /** \brief Get the bounds for a variable. Throw an exception if the variable was not found */
  const VariableBounds& getVariableBounds(const std::string& variable) const;

  /** \brief Get the variable bounds for this joint, in the same order as the names returned by getVariableNames() */
  const Bounds& getVariableBounds() const
  {
    return variable_bounds_;
  }

  /** \brief Set the lower and upper bounds for a variable. Throw an exception if the variable was not found. */
  void setVariableBounds(const std::string& variable, const VariableBounds& bounds);

  /** \brief Override joint limits loaded from URDF. Unknown variables are ignored. */
  void setVariableBounds(const std::vector<moveit_msgs::JointLimits>& jlim);

  /** \brief Get the joint limits known to this model, as a message. */
  const std::vector<moveit_msgs::JointLimits>& getVariableBoundsMsg() const
  {
    return variable_bounds_msg_;
  }

  /** @} */

  /** \brief Compute the distance between two joint states of the same model (represented by the variable values) */
  virtual double distance(const double* value1, const double* value2) const = 0;

  /** \brief Get the factor that should be applied to the value returned by distance() when that value is used in
   * compound distances */
  double getDistanceFactor() const
  {
    return distance_factor_;
  }

  /** \brief Set the factor that should be applied to the value returned by distance() when that value is used in
   * compound distances */
  void setDistanceFactor(double factor)
  {
    distance_factor_ = factor;
  }

  /** \brief Get the dimension of the state space that corresponds to this joint */
  virtual unsigned int getStateSpaceDimension() const = 0;

  /** \brief Get the joint this one is mimicking */
  const JointModel* getMimic() const
  {
    return mimic_;
  }

  /** \brief If mimicking a joint, this is the offset added to that joint's value */
  double getMimicOffset() const
  {
    return mimic_offset_;
  }

  /** \brief If mimicking a joint, this is the multiplicative factor for that joint's value */
  double getMimicFactor() const
  {
    return mimic_factor_;
  }

  /** \brief Mark this joint as mimicking \e mimic using \e factor and \e offset */
  void setMimic(const JointModel* mimic, double factor, double offset);

  /** \brief The joint models whose values would be modified if the value of this joint changed */
  const std::vector<const JointModel*>& getMimicRequests() const
  {
    return mimic_requests_;
  }

  /** \brief Notify this joint that there is another joint that mimics it */
  void addMimicRequest(const JointModel* joint);
  void addDescendantJointModel(const JointModel* joint);
  void addDescendantLinkModel(const LinkModel* link);

  /** \brief Get all the link models that descend from this joint, in the kinematic tree */
  const std::vector<const LinkModel*>& getDescendantLinkModels() const
  {
    return descendant_link_models_;
  }

  /** \brief Get all the joint models that descend from this joint, in the kinematic tree */
  const std::vector<const JointModel*>& getDescendantJointModels() const
  {
    return descendant_joint_models_;
  }

  /** \brief Get all the non-fixed joint models that descend from this joint, in the kinematic tree */
  const std::vector<const JointModel*>& getNonFixedDescendantJointModels() const
  {
    return non_fixed_descendant_joint_models_;
  }

  /** \brief Check if this joint is passive */
  bool isPassive() const
  {
    return passive_;
  }

  void setPassive(bool flag)
  {
    passive_ = flag;
  }

  /** \brief Computes the state that lies at time @e t in [0, 1] on the segment that connects @e from state to @e to
     state.
      The memory location of @e state is not required to be different from the memory of either
      @e from or @e to. */
  virtual void interpolate(const double* from, const double* to, const double t, double* state) const = 0;

  /** \brief Get the extent of the state space (the maximum value distance() can ever report) */
  virtual double getMaximumExtent(const Bounds& other_bounds) const = 0;

  double getMaximumExtent() const
  {
    return getMaximumExtent(variable_bounds_);
  }

  /** @name Computing transforms
      @{ */

  /** \brief Given the joint values for a joint, compute the corresponding transform. The computed transform is
   *  guaranteed to be a valid isometry. */
  virtual void computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const = 0;

  /** \brief Given the transform generated by joint, compute the corresponding joint values. Make sure the passed
   *  transform is a valid isometry. */
  virtual void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const = 0;

  /** @} */

protected:
  void computeVariableBoundsMsg();

  /** \brief Name of the joint */
  std::string name_;

  /** \brief The type of joint */
  JointType type_;

  /** \brief The local names to use for the variables that make up this joint */
  std::vector<std::string> local_variable_names_;

  /** \brief The full names to use for the variables that make up this joint */
  std::vector<std::string> variable_names_;

  /** \brief The bounds for each variable (low, high) in the same order as variable_names_ */
  Bounds variable_bounds_;

  std::vector<moveit_msgs::JointLimits> variable_bounds_msg_;

  /** \brief Map from variable names to the corresponding index in variable_names_ (indexing makes sense within the
   * JointModel only) */
  VariableIndexMap variable_index_map_;

  /** \brief The link before this joint */
  const LinkModel* parent_link_model_;

  /** \brief The link after this joint */
  const LinkModel* child_link_model_;

  /** \brief The joint this one mimics (NULL for joints that do not mimic) */
  const JointModel* mimic_;

  /** \brief The offset to the mimic joint */
  double mimic_factor_;

  /** \brief The multiplier to the mimic joint */
  double mimic_offset_;

  /** \brief The set of joints that should get a value copied to them when this joint changes */
  std::vector<const JointModel*> mimic_requests_;

  /** \brief Pointers to all the links that will be moved if this joint changes value */
  std::vector<const LinkModel*> descendant_link_models_;

  /** \brief Pointers to all the joints that follow this one in the kinematic tree (including mimic joints) */
  std::vector<const JointModel*> descendant_joint_models_;

  /** \brief Pointers to all the joints that follow this one in the kinematic tree, including mimic joints, but
   * excluding fixed joints */
  std::vector<const JointModel*> non_fixed_descendant_joint_models_;

  /** \brief Specify whether this joint is marked as passive in the SRDF */
  bool passive_;

  /** \brief The factor applied to the distance between two joint states */
  double distance_factor_;

  /** \brief The index of this joint's first variable, in the complete robot state */
  int first_variable_index_;

  /** \brief Index for this joint in the array of joints of the complete model */
  int joint_index_;
};

/** \brief Operator overload for printing variable bounds to a stream */
std::ostream& operator<<(std::ostream& out, const VariableBounds& b);
}  // namespace core
}  // namespace moveit
