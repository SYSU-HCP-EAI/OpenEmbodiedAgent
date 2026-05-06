/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2008-2013, Willow Garage, Inc.
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

#include <moveit/macros/class_forward.h>
#include <moveit/exceptions/exceptions.h>
#include <urdf/model.h>
#include <srdfdom/model.h>

// joint types
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/fixed_joint_model.h>
#include <moveit/robot_model/floating_joint_model.h>
#include <moveit/robot_model/planar_joint_model.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_model/prismatic_joint_model.h>

#include <Eigen/Geometry>
#include <iostream>

/** \brief Main namespace for MoveIt */
namespace moveit
{
/** \brief Core components of MoveIt */
namespace core
{
MOVEIT_CLASS_FORWARD(RobotModel);  // Defines RobotModelPtr, ConstPtr, WeakPtr... etc

static inline void checkInterpolationParamBounds(const char LOGNAME[], double t)
{
  if (std::isnan(t) || std::isinf(t))
  {
    throw Exception("Interpolation parameter is NaN or inf.");
  }
  ROS_WARN_STREAM_COND_NAMED(t < 0. || t > 1., LOGNAME, "Interpolation parameter is not in the range [0, 1]: " << t);
}

/** \brief Definition of a kinematic model. This class is not thread
    safe, however multiple instances can be created */
class RobotModel
{
public:
  /** \brief Construct a kinematic model from a parsed description and a list of planning groups */
  RobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model, const srdf::ModelConstSharedPtr& srdf_model);

  /** \brief Destructor. Clear all memory. */
  ~RobotModel();

  /** \brief Get the model name. */
  const std::string& getName() const
  {
    return model_name_;
  }

  /** \brief Get the frame in which the transforms for this model are
      computed (when using a RobotState). This frame depends on the
      root joint. As such, the frame is either extracted from SRDF, or
      it is assumed to be the name of the root link */
  const std::string& getModelFrame() const
  {
    return model_frame_;
  }

  /** \brief Return true if the model is empty (has no root link, no joints) */
  bool isEmpty() const
  {
    return root_link_ == nullptr;
  }

  /** \brief Get the parsed URDF model */
  const urdf::ModelInterfaceSharedPtr& getURDF() const
  {
    return urdf_;
  }

  /** \brief Get the parsed SRDF model */
  const srdf::ModelConstSharedPtr& getSRDF() const
  {
    return srdf_;
  }

  /** \brief Print information about the constructed model */
  void printModelInfo(std::ostream& out) const;

  /** \name Access to joint models
   *  @{
   */

  /** \brief Get the root joint. There will be one root joint
      unless the model is empty. This is either extracted from the
      SRDF, or a fixed joint is assumed, if no specification is
      given. */
  const JointModel* getRootJoint() const;

  /** \brief Return the name of the root joint. Throws an exception if there is no root joint. */
  const std::string& getRootJointName() const
  {
    return getRootJoint()->getName();
  }

  /** \brief Check if a joint exists. Return true if it does. */
  bool hasJointModel(const std::string& name) const;

  /** \brief Get a joint by its name. Output error and return NULL when the joint is missing. */
  const JointModel* getJointModel(const std::string& joint) const;

  /** \brief Get a joint by its index. Output error and return NULL when the link is missing. */
  const JointModel* getJointModel(int index) const;

  /** \brief Get a joint by its name. Output error and return NULL when the joint is missing. */
  JointModel* getJointModel(const std::string& joint);

  /** \brief Get the array of joints, in the order they appear
      in the robot state. */
  const std::vector<const JointModel*>& getJointModels() const
  {
    return joint_model_vector_const_;
  }

  /** \brief Get the array of joints, in the order they appear in the
      robot state. This includes all types of joints (including mimic
      & fixed), as opposed to JointModelGroup::getJointModels(). */
  const std::vector<JointModel*>& getJointModels()
  {
    return joint_model_vector_;
  }

  /** \brief Get the array of joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getJointModelNames() const
  {
    return joint_model_names_vector_;
  }

  /** \brief Get the array of joints that are active (not fixed, not mimic) in this model. */
  const std::vector<const JointModel*>& getActiveJointModels() const
  {
    return active_joint_model_vector_const_;
  }

  /** \brief Get the array of active joint names, in the order they appear in the robot state. */
  const std::vector<std::string>& getActiveJointModelNames() const
  {
    return active_joint_model_names_vector_;
  }

  /** \brief Get the array of joints that are active (not fixed, not mimic) in this model */
  const std::vector<JointModel*>& getActiveJointModels()
  {
    return active_joint_model_vector_;
  }

  /** \brief This is a list of all single-dof joints (including mimic joints) */
  const std::vector<const JointModel*>& getSingleDOFJointModels() const
  {
    return single_dof_joints_;
  }

  /** \brief This is a list of all multi-dof joints */
  const std::vector<const JointModel*>& getMultiDOFJointModels() const
  {
    return multi_dof_joints_;
  }

  /** \brief Get the array of continuous joints, in the order they appear
      in the robot state. */
  const std::vector<const JointModel*>& getContinuousJointModels() const
  {
    return continuous_joint_model_vector_;
  }

  /** \brief Get the array of mimic joints, in the order they appear
      in the robot state. */
  const std::vector<const JointModel*>& getMimicJointModels() const
  {
    return mimic_joints_;
  }

  const JointModel* getJointOfVariable(int variable_index) const
  {
    return joints_of_variable_[variable_index];
  }

  const JointModel* getJointOfVariable(const std::string& variable) const
  {
    return joints_of_variable_[getVariableIndex(variable)];
  }

  std::size_t getJointModelCount() const
  {
    return joint_model_vector_.size();
  }

  /** @} */

  /** \name Access to link models
   *  @{
   */

  /** \brief Get the physical root link of the robot. */
  const LinkModel* getRootLink() const;

  /** \brief Get the name of the root link of the robot. */
  const std::string& getRootLinkName() const
  {
    return getRootLink()->getName();
  }

  /** \brief Check if a link exists. Return true if it does.
   *
   * If this is followed by a call to getLinkModel(), better use the latter with the has_link argument */
  bool hasLinkModel(const std::string& name) const;

  /** \brief Get a link by its name. Output error and return NULL when the link is missing. */
  const LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr) const;

  /** \brief Get a link by its index. Output error and return NULL when the link is missing. */
  const LinkModel* getLinkModel(int index) const;

  /** \brief Get a link by its name. Output error and return NULL when the link is missing. */
  LinkModel* getLinkModel(const std::string& link, bool* has_link = nullptr);

  /** \brief Get the latest link upwards the kinematic tree, which is only connected via fixed joints
   *
   * This is useful, if the link should be warped to a specific pose using updateStateWithLinkAt().
   * As updateStateWithLinkAt() warps only the specified link and its descendants, you might not
   * achieve what you expect, if link is an abstract frame name. Considering the following example:
   * root -> arm0 -> ... -> armN -> wrist -- grasp_frame
   *                                      -- palm -> end effector ...
   * Calling updateStateWithLinkAt(grasp_frame), will not warp the end effector, which is probably
   * what you went for. Instead, updateStateWithLinkAt(getRigidlyConnectedParentLinkModel(grasp_frame), ...)
   * will actually warp wrist (and all its descendants).
   */
  static const moveit::core::LinkModel* getRigidlyConnectedParentLinkModel(const LinkModel* link);

  /** \brief Get the array of links  */
  const std::vector<const LinkModel*>& getLinkModels() const
  {
    return link_model_vector_const_;
  }

  /** \brief Get the array of links  */
  const std::vector<LinkModel*>& getLinkModels()
  {
    return link_model_vector_;
  }

  /** \brief Get the link names (of all links) */
  const std::vector<std::string>& getLinkModelNames() const
  {
    return link_model_names_vector_;
  }

  /** \brief Get the link models that have some collision geometry associated to themselves */
  const std::vector<const LinkModel*>& getLinkModelsWithCollisionGeometry() const
  {
    return link_models_with_collision_geometry_vector_;
  }

  /** \brief Get the names of the link models that have some collision geometry associated to themselves */
  const std::vector<std::string>& getLinkModelNamesWithCollisionGeometry() const
  {
    return link_model_names_with_collision_geometry_vector_;
  }

  std::size_t getLinkModelCount() const
  {
    return link_model_vector_.size();
  }

  std::size_t getLinkGeometryCount() const
  {
    return link_geometry_count_;
  }

  /** @} */

  /** \brief Compute the random values for a RobotState */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const;

  /** \brief Compute the default values for a RobotState */
  void getVariableDefaultPositions(double* values) const;

  /** \brief Compute the random values for a RobotState */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, std::vector<double>& values) const
  {
    values.resize(variable_count_);
    getVariableRandomPositions(rng, &values[0]);
  }

  /** \brief Compute the default values for a RobotState */
  void getVariableDefaultPositions(std::vector<double>& values) const
  {
    values.resize(variable_count_);
    getVariableDefaultPositions(&values[0]);
  }

  /** \brief Compute the random values for a RobotState */
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng,
                                  std::map<std::string, double>& values) const;

  /** \brief Compute the default values for a RobotState */
  void getVariableDefaultPositions(std::map<std::string, double>& values) const;

  bool enforcePositionBounds(double* state) const
  {
    return enforcePositionBounds(state, active_joint_models_bounds_);
  }
  bool enforcePositionBounds(double* state, const JointBoundsVector& active_joint_bounds) const;
  bool satisfiesPositionBounds(const double* state, double margin = 0.0) const
  {
    return satisfiesPositionBounds(state, active_joint_models_bounds_, margin);
  }
  bool satisfiesPositionBounds(const double* state, const JointBoundsVector& active_joint_bounds,
                               double margin = 0.0) const;
  double getMaximumExtent() const
  {
    return getMaximumExtent(active_joint_models_bounds_);
  }
  double getMaximumExtent(const JointBoundsVector& active_joint_bounds) const;

  /** \brief Return the sum of joint distances between two states. Only considers active joints. */
  double distance(const double* state1, const double* state2) const;

  /**
   * Interpolate between "from" state, to "to" state. Mimic joints are correctly updated.
   *
   * @param from interpolate from this state
   * @param to to this state
   * @param t a fraction in the range [0 1]. If 1, the result matches "to" state exactly.
   * @param state holds the result
   */
  void interpolate(const double* from, const double* to, double t, double* state) const;

  /** \name Access to joint groups
   *  @{
   */

  /** \brief Check if the JointModelGroup \e group exists */
  bool hasJointModelGroup(const std::string& group) const;

  /** \brief Get a joint group from this model (by name) */
  const JointModelGroup* getJointModelGroup(const std::string& name) const;

  /** \brief Get a joint group from this model (by name) */
  JointModelGroup* getJointModelGroup(const std::string& name);

  /** \brief Get the available joint groups */
  const std::vector<const JointModelGroup*>& getJointModelGroups() const
  {
    return joint_model_groups_const_;
  }

  /** \brief Get the available joint groups */
  const std::vector<JointModelGroup*>& getJointModelGroups()
  {
    return joint_model_groups_;
  }

  /** \brief Get the names of all groups that are defined for this model */
  const std::vector<std::string>& getJointModelGroupNames() const
  {
    return joint_model_group_names_;
  }

  /** \brief Check if an end effector exists */
  bool hasEndEffector(const std::string& eef) const;

  /** \brief Get the joint group that corresponds to a given end-effector name */
  const JointModelGroup* getEndEffector(const std::string& name) const;

  /** \brief Get the joint group that corresponds to a given end-effector name */
  JointModelGroup* getEndEffector(const std::string& name);

  /** \brief Get the map between end effector names and the groups they correspond to */
  const std::vector<const JointModelGroup*>& getEndEffectors() const
  {
    return end_effectors_;
  }

  /** @} */

  /** \brief Get the number of variables that describe this model */
  std::size_t getVariableCount() const
  {
    return variable_count_;
  }

  /** \brief Get the names of the variables that make up the joints that form this state. Fixed joints have no DOF, so
     they are not here,
      but the variables for mimic joints are included. The number of returned elements is always equal to
     getVariableCount() */
  const std::vector<std::string>& getVariableNames() const
  {
    return variable_names_;
  }

  /** \brief Get the bounds for a specific variable. Throw an exception of variable is not found. */
  const VariableBounds& getVariableBounds(const std::string& variable) const
  {
    return getJointOfVariable(variable)->getVariableBounds(variable);
  }

  /** \brief Get the bounds for all the active joints */
  const JointBoundsVector& getActiveJointModelsBounds() const
  {
    return active_joint_models_bounds_;
  }

  void getMissingVariableNames(const std::vector<std::string>& variables,
                               std::vector<std::string>& missing_variables) const;

  /** \brief Get the index of a variable in the robot state */
  int getVariableIndex(const std::string& variable) const;

  /** \brief Get the deepest joint in the kinematic tree that is a common parent of both joints passed as argument */
  const JointModel* getCommonRoot(const JointModel* a, const JointModel* b) const
  {
    if (!a)
      return b;
    if (!b)
      return a;
    return joint_model_vector_[common_joint_roots_[a->getJointIndex() * joint_model_vector_.size() + b->getJointIndex()]];
  }

  /// A map of known kinematics solvers (associated to their group name)
  void setKinematicsAllocators(const std::map<std::string, SolverAllocatorFn>& allocators);

protected:
  /** \brief Get the transforms between link and all its rigidly attached descendants */
  void computeFixedTransforms(const LinkModel* link, const Eigen::Isometry3d& transform,
                              LinkTransformMap& associated_transforms);

  /** \brief Given two joints, find their common root */
  const JointModel* computeCommonRoot(const JointModel* a, const JointModel* b) const;

  /** \brief Update the variable values for the state of a group with respect to the mimic joints. */
  void updateMimicJoints(double* values) const;

  // GENERIC INFO

  /** \brief The name of the robot */
  std::string model_name_;

  /** \brief The reference (base) frame for this model. The frame is either extracted from the SRDF as a virtual joint,
   * or it is assumed to be the name of the root link in the URDF */
  std::string model_frame_;

  srdf::ModelConstSharedPtr srdf_;

  urdf::ModelInterfaceSharedPtr urdf_;

  // LINKS

  /** \brief The first physical link for the robot */
  const LinkModel* root_link_;

  /** \brief A map from link names to their instances */
  LinkModelMap link_model_map_;

  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<LinkModel*> link_model_vector_;

  /** \brief The vector of links that are updated when computeTransforms() is called, in the order they are updated */
  std::vector<const LinkModel*> link_model_vector_const_;

  /** \brief The vector of link names that corresponds to link_model_vector_ */
  std::vector<std::string> link_model_names_vector_;

  /** \brief Only links that have collision geometry specified */
  std::vector<const LinkModel*> link_models_with_collision_geometry_vector_;

  /** \brief The vector of link names that corresponds to link_models_with_collision_geometry_vector_ */
  std::vector<std::string> link_model_names_with_collision_geometry_vector_;

  /** \brief Total number of geometric shapes in this model */
  std::size_t link_geometry_count_;

  // JOINTS

  /** \brief The root joint */
  const JointModel* root_joint_;

  /** \brief A map from joint names to their instances */
  JointModelMap joint_model_map_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*> joint_model_vector_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<const JointModel*> joint_model_vector_const_;

  /** \brief The vector of joint names that corresponds to joint_model_vector_ */
  std::vector<std::string> joint_model_names_vector_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<JointModel*> active_joint_model_vector_;

  /** \brief The vector of joint names that corresponds to active_joint_model_vector_ */
  std::vector<std::string> active_joint_model_names_vector_;

  /** \brief The vector of joints in the model, in the order they appear in the state vector */
  std::vector<const JointModel*> active_joint_model_vector_const_;

  /** \brief The set of continuous joints this model contains */
  std::vector<const JointModel*> continuous_joint_model_vector_;

  /** \brief The set of mimic joints this model contains */
  std::vector<const JointModel*> mimic_joints_;

  std::vector<const JointModel*> single_dof_joints_;

  std::vector<const JointModel*> multi_dof_joints_;

  /** \brief For every two joints, the index of the common root for thw joints is stored.

      for jointA, jointB
      the index of the common root is located in the array at location
      jointA->getJointIndex() * nr.joints + jointB->getJointIndex().
      The size of this array is nr.joints * nr.joints
   */
  std::vector<int> common_joint_roots_;

  // INDEXING

  /** \brief The names of the DOF that make up this state (this is just a sequence of joint variable names; not
   * necessarily joint names!) */
  std::vector<std::string> variable_names_;

  /** \brief Get the number of variables necessary to describe this model */
  std::size_t variable_count_;

  /** \brief The state includes all the joint variables that make up the joints the state consists of.
      This map gives the position in the state vector of the group for each of these variables.
      Additionaly, it includes the names of the joints and the index for the first variable of that joint. */
  VariableIndexMap joint_variables_index_map_;

  std::vector<int> active_joint_model_start_index_;

  /** \brief The bounds for all the active joint models */
  JointBoundsVector active_joint_models_bounds_;

  /** \brief The joints that correspond to each variable index */
  std::vector<const JointModel*> joints_of_variable_;

  // GROUPS

  /** \brief A map from group names to joint groups */
  JointModelGroupMap joint_model_group_map_;

  /** \brief The known end effectors */
  JointModelGroupMap end_effectors_map_;

  /** \brief The array of joint model groups, in alphabetical order */
  std::vector<JointModelGroup*> joint_model_groups_;

  /** \brief The array of joint model groups, in alphabetical order */
  std::vector<const JointModelGroup*> joint_model_groups_const_;

  /** \brief A vector of all group names, in alphabetical order */
  std::vector<std::string> joint_model_group_names_;

  /** \brief The array of end-effectors, in alphabetical order */
  std::vector<const JointModelGroup*> end_effectors_;

  /** \brief Given an URDF model and a SRDF model, build a full kinematic model */
  void buildModel(const urdf::ModelInterface& urdf_model, const srdf::Model& srdf_model);

  /** \brief Given a SRDF model describing the groups, build up the groups in this kinematic model */
  void buildGroups(const srdf::Model& srdf_model);

  /** \brief Compute helpful information about groups (that can be queried later) */
  void buildGroupsInfoSubgroups();

  /** \brief Compute helpful information about groups (that can be queried later) */
  void buildGroupsInfoEndEffectors(const srdf::Model& srdf_model);

  /** \brief Given the URDF model, build up the mimic joints (mutually constrained joints) */
  void buildMimic(const urdf::ModelInterface& urdf_model);

  /** \brief Given a SRDF model describing the groups, build the default states defined in the SRDF */
  void buildGroupStates(const srdf::Model& srdf_model);

  /** \brief Compute helpful information about joints */
  void buildJointInfo();

  /** \brief For every joint, pre-compute the list of descendant joints & links */
  void computeDescendants();

  /** \brief For every pair of joints, pre-compute the common roots of the joints */
  void computeCommonRoots();

  /** \brief (This function is mostly intended for internal use). Given a parent link, build up (recursively),
      the kinematic model by walking  down the tree*/
  JointModel* buildRecursive(LinkModel* parent, const urdf::Link* link, const srdf::Model& srdf_model);

  /** \brief Construct a JointModelGroup given a SRDF description \e group */
  bool addJointModelGroup(const srdf::Model::Group& group);

  /** \brief Given a urdf joint model, a child link and a set of virtual joints,
      build up the corresponding JointModel object*/
  JointModel* constructJointModel(const urdf::Joint* urdf_joint_model, const urdf::Link* child_link,
                                  const srdf::Model& srdf_model);

  /** \brief Given a urdf link, build the corresponding LinkModel object*/
  LinkModel* constructLinkModel(const urdf::Link* urdf_link);

  /** \brief Given a geometry spec from the URDF and a filename (for a mesh), construct the corresponding shape object*/
  shapes::ShapePtr constructShape(const urdf::Geometry* geom);
};
}  // namespace core
}  // namespace moveit

namespace robot_model = moveit::core;
namespace robot_state = moveit::core;
