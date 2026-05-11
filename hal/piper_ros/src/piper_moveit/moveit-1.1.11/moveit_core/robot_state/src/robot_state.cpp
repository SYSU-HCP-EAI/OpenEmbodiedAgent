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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Sachin Chitta, Acorn Pooley, Mario Prats, Dave Coleman */

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/transforms/transforms.h>
#include <geometric_shapes/check_isometry.h>
#include <geometric_shapes/shape_operations.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/backtrace/backtrace.h>
#include <moveit/profiler/profiler.h>
#include <moveit/macros/console_colors.h>
#include <functional>
#include <moveit/robot_model/aabb.h>

namespace moveit
{
namespace core
{
namespace
{
constexpr char LOGNAME[] = "robot_state";
}  // namespace

RobotState::RobotState(const RobotModelConstPtr& robot_model)
  : robot_model_(robot_model)
  , has_velocity_(false)
  , has_acceleration_(false)
  , has_effort_(false)
  , dirty_link_transforms_(nullptr)
  , dirty_collision_body_transforms_(nullptr)
  , rng_(nullptr)
{
  if (robot_model == nullptr)
  {
    throw std::invalid_argument("RobotState cannot be constructed with nullptr RobotModelConstPtr");
  }

  dirty_link_transforms_ = robot_model_->getRootJoint();
  allocMemory();
  initTransforms();
}

RobotState::RobotState(const RobotState& other) : rng_(nullptr)
{
  robot_model_ = other.robot_model_;
  allocMemory();
  copyFrom(other);
}

RobotState::~RobotState()
{
  clearAttachedBodies();
  free(memory_);
  if (rng_)
    delete rng_;
}

void RobotState::allocMemory()
{
  static_assert((sizeof(Eigen::Isometry3d) / EIGEN_MAX_ALIGN_BYTES) * EIGEN_MAX_ALIGN_BYTES == sizeof(Eigen::Isometry3d),
                "sizeof(Eigen::Isometry3d) should be a multiple of EIGEN_MAX_ALIGN_BYTES");

  constexpr unsigned int extra_alignment_bytes = EIGEN_MAX_ALIGN_BYTES - 1;
  // memory for the dirty joint transforms
  const int nr_doubles_for_dirty_joint_transforms =
      1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
  const size_t bytes =
      sizeof(Eigen::Isometry3d) * (robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() +
                                   robot_model_->getLinkGeometryCount()) +
      sizeof(double) * (robot_model_->getVariableCount() * 3 + nr_doubles_for_dirty_joint_transforms) +
      extra_alignment_bytes;
  memory_ = malloc(bytes);

  // make the memory for transforms align at EIGEN_MAX_ALIGN_BYTES
  // https://eigen.tuxfamily.org/dox/classEigen_1_1aligned__allocator.html
  variable_joint_transforms_ = reinterpret_cast<Eigen::Isometry3d*>(((uintptr_t)memory_ + extra_alignment_bytes) &
                                                                    ~(uintptr_t)extra_alignment_bytes);
  global_link_transforms_ = variable_joint_transforms_ + robot_model_->getJointModelCount();
  global_collision_body_transforms_ = global_link_transforms_ + robot_model_->getLinkModelCount();
  dirty_joint_transforms_ =
      reinterpret_cast<unsigned char*>(global_collision_body_transforms_ + robot_model_->getLinkGeometryCount());
  position_ = reinterpret_cast<double*>(dirty_joint_transforms_) + nr_doubles_for_dirty_joint_transforms;
  velocity_ = position_ + robot_model_->getVariableCount();
  // acceleration and effort share the memory (not both can be specified)
  effort_ = acceleration_ = velocity_ + robot_model_->getVariableCount();
}

void RobotState::initTransforms()
{
  // mark all transforms as dirty
  const int nr_doubles_for_dirty_joint_transforms =
      1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
  memset(dirty_joint_transforms_, 1, sizeof(double) * nr_doubles_for_dirty_joint_transforms);

  // initialize last row of transformation matrices, which will not be modified by transform updates anymore
  for (size_t i = 0, end = robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() +
                           robot_model_->getLinkGeometryCount();
       i != end; ++i)
    variable_joint_transforms_[i].makeAffine();
}

RobotState& RobotState::operator=(const RobotState& other)
{
  if (this != &other)
    copyFrom(other);
  return *this;
}

void RobotState::copyFrom(const RobotState& other)
{
  has_velocity_ = other.has_velocity_;
  has_acceleration_ = other.has_acceleration_;
  has_effort_ = other.has_effort_;

  dirty_collision_body_transforms_ = other.dirty_collision_body_transforms_;
  dirty_link_transforms_ = other.dirty_link_transforms_;

  if (dirty_link_transforms_ == robot_model_->getRootJoint())
  {
    // everything is dirty; no point in copying transforms; copy positions, potentially velocity & acceleration
    memcpy(position_, other.position_,
           robot_model_->getVariableCount() * sizeof(double) *
               (1 + (has_velocity_ ? 1 : 0) + ((has_acceleration_ || has_effort_) ? 1 : 0)));
    // and just initialize transforms
    initTransforms();
  }
  else
  {
    // copy all the memory; maybe avoid copying velocity and acceleration if possible
    const int nr_doubles_for_dirty_joint_transforms =
        1 + robot_model_->getJointModelCount() / (sizeof(double) / sizeof(unsigned char));
    const size_t bytes =
        sizeof(Eigen::Isometry3d) * (robot_model_->getJointModelCount() + robot_model_->getLinkModelCount() +
                                     robot_model_->getLinkGeometryCount()) +
        sizeof(double) *
            (robot_model_->getVariableCount() * (1 + ((has_velocity_ || has_acceleration_ || has_effort_) ? 1 : 0) +
                                                 ((has_acceleration_ || has_effort_) ? 1 : 0)) +
             nr_doubles_for_dirty_joint_transforms);
    memcpy((void*)variable_joint_transforms_, (void*)other.variable_joint_transforms_, bytes);
  }

  // copy attached bodies
  clearAttachedBodies();
  for (const auto& attached_body : other.attached_body_map_)
    attachBody(std::make_unique<AttachedBody>(*attached_body.second));
}

bool RobotState::checkJointTransforms(const JointModel* joint) const
{
  if (dirtyJointTransform(joint))
  {
    ROS_WARN_NAMED(LOGNAME, "Returning dirty joint transforms for joint '%s'", joint->getName().c_str());
    return false;
  }
  return true;
}

bool RobotState::checkLinkTransforms() const
{
  if (dirtyLinkTransforms())
  {
    ROS_WARN_NAMED(LOGNAME, "Returning dirty link transforms");
    return false;
  }
  return true;
}

bool RobotState::checkCollisionTransforms() const
{
  if (dirtyCollisionBodyTransforms())
  {
    ROS_WARN_NAMED(LOGNAME, "Returning dirty collision body transforms");
    return false;
  }
  return true;
}

void RobotState::markVelocity()
{
  if (!has_velocity_)
  {
    has_velocity_ = true;
    memset(velocity_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void RobotState::markAcceleration()
{
  if (!has_acceleration_)
  {
    has_acceleration_ = true;
    has_effort_ = false;
    memset(acceleration_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void RobotState::markEffort()
{
  if (!has_effort_)
  {
    has_acceleration_ = false;
    has_effort_ = true;
    memset(effort_, 0, sizeof(double) * robot_model_->getVariableCount());
  }
}

void RobotState::zeroVelocities()
{
  has_velocity_ = false;
  markVelocity();
}

void RobotState::zeroAccelerations()
{
  has_acceleration_ = false;
  markAcceleration();
}

void RobotState::zeroEffort()
{
  has_effort_ = false;
  markEffort();
}

void RobotState::dropVelocities()
{
  has_velocity_ = false;
}

void RobotState::dropAccelerations()
{
  has_acceleration_ = false;
}

void RobotState::dropEffort()
{
  has_effort_ = false;
}

void RobotState::dropDynamics()
{
  dropVelocities();
  dropAccelerations();
  dropEffort();
}

void RobotState::setToRandomPositions()
{
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  robot_model_->getVariableRandomPositions(rng, position_);
  memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
  dirty_link_transforms_ = robot_model_->getRootJoint();
  // mimic values are correctly set in RobotModel
}

void RobotState::setToRandomPositions(const JointModelGroup* group)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  setToRandomPositions(group, rng);
}
void RobotState::setToRandomPositions(const JointModelGroup* group, random_numbers::RandomNumberGenerator& rng)
{
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (const JointModel* joint : joints)
    joint->getVariableRandomPositions(rng, position_ + joint->getFirstVariableIndex());
  updateMimicJoints(group);
}

void RobotState::setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& seed,
                                            const std::vector<double>& distances)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  setToRandomPositionsNearBy(group, seed, distances, rng);
}

void RobotState::setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& seed,
                                            const std::vector<double>& distances,
                                            random_numbers::RandomNumberGenerator& rng)
{
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  assert(distances.size() == joints.size());
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    const int idx = joints[i]->getFirstVariableIndex();
    joints[i]->getVariableRandomPositionsNearBy(rng, position_ + joints[i]->getFirstVariableIndex(),
                                                seed.position_ + idx, distances[i]);
  }
  updateMimicJoints(group);
}

void RobotState::setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& seed, double distance)
{
  // we do not make calls to RobotModel for random number generation because mimic joints
  // could trigger updates outside the state of the group itself
  random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
  setToRandomPositionsNearBy(group, seed, distance, rng);
}

void RobotState::setToRandomPositionsNearBy(const JointModelGroup* group, const RobotState& seed, double distance,
                                            random_numbers::RandomNumberGenerator& rng)
{
  const std::vector<const JointModel*>& joints = group->getActiveJointModels();
  for (const JointModel* joint : joints)
  {
    const int idx = joint->getFirstVariableIndex();
    joint->getVariableRandomPositionsNearBy(rng, position_ + joint->getFirstVariableIndex(), seed.position_ + idx,
                                            distance);
  }
  updateMimicJoints(group);
}

bool RobotState::setToDefaultValues(const JointModelGroup* group, const std::string& name)
{
  std::map<std::string, double> m;
  bool r = group->getVariableDefaultPositions(name, m);  // mimic values are updated
  setVariablePositions(m);
  return r;
}

void RobotState::setToDefaultValues()
{
  robot_model_->getVariableDefaultPositions(position_);  // mimic values are updated
  // set velocity & acceleration to 0
  memset(velocity_, 0, sizeof(double) * 2 * robot_model_->getVariableCount());
  memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const double* position)
{
  // assume everything is in order in terms of array lengths (for efficiency reasons)
  memcpy(position_, position, robot_model_->getVariableCount() * sizeof(double));

  // the full state includes mimic joint values, so no need to update mimic here

  // Since all joint values have potentially changed, we will need to recompute all transforms
  memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map)
{
  for (const std::pair<const std::string, double>& it : variable_map)
  {
    const int index = robot_model_->getVariableIndex(it.first);
    position_[index] = it.second;
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::getMissingKeys(const std::map<std::string, double>& variable_map,
                                std::vector<std::string>& missing_variables) const
{
  missing_variables.clear();
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (const std::string& variable_name : nm)
    if (variable_map.find(variable_name) == variable_map.end())
      if (robot_model_->getJointOfVariable(variable_name)->getMimic() == nullptr)
        missing_variables.push_back(variable_name);
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map,
                                      std::vector<std::string>& missing_variables)
{
  setVariablePositions(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariablePositions(const std::vector<std::string>& variable_names,
                                      const std::vector<double>& variable_position)
{
  for (std::size_t i = 0; i < variable_names.size(); ++i)
  {
    const int index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map)
{
  markVelocity();
  for (const std::pair<const std::string, double>& it : variable_map)
    velocity_[robot_model_->getVariableIndex(it.first)] = it.second;
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map,
                                       std::vector<std::string>& missing_variables)
{
  setVariableVelocities(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableVelocities(const std::vector<std::string>& variable_names,
                                       const std::vector<double>& variable_velocity)
{
  markVelocity();
  assert(variable_names.size() == variable_velocity.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i)
    velocity_[robot_model_->getVariableIndex(variable_names[i])] = variable_velocity[i];
}

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map)
{
  markAcceleration();
  for (const std::pair<const std::string, double>& it : variable_map)
    acceleration_[robot_model_->getVariableIndex(it.first)] = it.second;
}

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map,
                                          std::vector<std::string>& missing_variables)
{
  setVariableAccelerations(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableAccelerations(const std::vector<std::string>& variable_names,
                                          const std::vector<double>& variable_acceleration)
{
  markAcceleration();
  assert(variable_names.size() == variable_acceleration.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i)
    acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_acceleration[i];
}

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map)
{
  markEffort();
  for (const std::pair<const std::string, double>& it : variable_map)
    effort_[robot_model_->getVariableIndex(it.first)] = it.second;
}

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map,
                                   std::vector<std::string>& missing_variables)
{
  setVariableEffort(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableEffort(const std::vector<std::string>& variable_names,
                                   const std::vector<double>& variable_effort)
{
  markEffort();
  assert(variable_names.size() == variable_effort.size());
  for (std::size_t i = 0; i < variable_names.size(); ++i)
    effort_[robot_model_->getVariableIndex(variable_names[i])] = variable_effort[i];
}

void RobotState::invertVelocity()
{
  if (has_velocity_)
  {
    for (size_t i = 0; i < robot_model_->getVariableCount(); ++i)
      velocity_[i] *= -1;
  }
}

void RobotState::setJointEfforts(const JointModel* joint, const double* effort)
{
  if (has_acceleration_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to set joint efforts because array is being used for accelerations");
    return;
  }
  has_effort_ = true;

  memcpy(effort_ + joint->getFirstVariableIndex(), effort, joint->getVariableCount() * sizeof(double));
}

void RobotState::setJointGroupPositions(const JointModelGroup* group, const double* gstate)
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(position_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0; i < il.size(); ++i)
      position_[il[i]] = gstate[i];
  }
  updateMimicJoints(group);
}

void RobotState::setJointGroupPositions(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  const std::vector<int>& il = group->getVariableIndexList();
  for (std::size_t i = 0; i < il.size(); ++i)
    position_[il[i]] = values(i);
  updateMimicJoints(group);
}

void RobotState::setJointGroupActivePositions(const JointModelGroup* group, const std::vector<double>& gstate)
{
  assert(gstate.size() == group->getActiveVariableCount());
  std::size_t i = 0;
  for (const JointModel* jm : group->getActiveJointModels())
  {
    setJointPositions(jm, &gstate[i]);
    i += jm->getVariableCount();
  }
  updateMimicJoints(group);
}

void RobotState::setJointGroupActivePositions(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  assert(values.size() == group->getActiveVariableCount());
  std::size_t i = 0;
  for (const JointModel* jm : group->getActiveJointModels())
  {
    setJointPositions(jm, &values(i));
    i += jm->getVariableCount();
  }
  updateMimicJoints(group);
}

void RobotState::copyJointGroupPositions(const JointModelGroup* group, double* gstate) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, position_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0; i < il.size(); ++i)
      gstate[i] = position_[il[i]];
}

void RobotState::copyJointGroupPositions(const JointModelGroup* group, Eigen::VectorXd& values) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0; i < il.size(); ++i)
    values(i) = position_[il[i]];
}

void RobotState::setJointGroupVelocities(const JointModelGroup* group, const double* gstate)
{
  markVelocity();
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(velocity_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0; i < il.size(); ++i)
      velocity_[il[i]] = gstate[i];
  }
}

void RobotState::setJointGroupVelocities(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  markVelocity();
  const std::vector<int>& il = group->getVariableIndexList();
  for (std::size_t i = 0; i < il.size(); ++i)
    velocity_[il[i]] = values(i);
}

void RobotState::copyJointGroupVelocities(const JointModelGroup* group, double* gstate) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, velocity_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0; i < il.size(); ++i)
      gstate[i] = velocity_[il[i]];
}

void RobotState::copyJointGroupVelocities(const JointModelGroup* group, Eigen::VectorXd& values) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0; i < il.size(); ++i)
    values(i) = velocity_[il[i]];
}

void RobotState::setJointGroupAccelerations(const JointModelGroup* group, const double* gstate)
{
  markAcceleration();
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(acceleration_ + il[0], gstate, group->getVariableCount() * sizeof(double));
  else
  {
    for (std::size_t i = 0; i < il.size(); ++i)
      acceleration_[il[i]] = gstate[i];
  }
}

void RobotState::setJointGroupAccelerations(const JointModelGroup* group, const Eigen::VectorXd& values)
{
  markAcceleration();
  const std::vector<int>& il = group->getVariableIndexList();
  for (std::size_t i = 0; i < il.size(); ++i)
    acceleration_[il[i]] = values(i);
}

void RobotState::copyJointGroupAccelerations(const JointModelGroup* group, double* gstate) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  if (group->isContiguousWithinState())
    memcpy(gstate, acceleration_ + il[0], group->getVariableCount() * sizeof(double));
  else
    for (std::size_t i = 0; i < il.size(); ++i)
      gstate[i] = acceleration_[il[i]];
}

void RobotState::copyJointGroupAccelerations(const JointModelGroup* group, Eigen::VectorXd& values) const
{
  const std::vector<int>& il = group->getVariableIndexList();
  values.resize(il.size());
  for (std::size_t i = 0; i < il.size(); ++i)
    values(i) = acceleration_[il[i]];
}

void RobotState::update(bool force)
{
  // make sure we do everything from scratch if needed
  if (force)
  {
    memset(dirty_joint_transforms_, 1, robot_model_->getJointModelCount() * sizeof(unsigned char));
    dirty_link_transforms_ = robot_model_->getRootJoint();
  }

  // this actually triggers all needed updates
  updateCollisionBodyTransforms();
}

void RobotState::updateCollisionBodyTransforms()
{
  if (dirty_link_transforms_ != nullptr)
    updateLinkTransforms();

  if (dirty_collision_body_transforms_ != nullptr)
  {
    const std::vector<const LinkModel*>& links = dirty_collision_body_transforms_->getDescendantLinkModels();
    dirty_collision_body_transforms_ = nullptr;

    for (const LinkModel* link : links)
    {
      const EigenSTL::vector_Isometry3d& ot = link->getCollisionOriginTransforms();
      const std::vector<int>& ot_id = link->areCollisionOriginTransformsIdentity();
      const int index_co = link->getFirstCollisionBodyTransformIndex();
      const int index_l = link->getLinkIndex();
      for (std::size_t j = 0, end = ot.size(); j != end; ++j)
      {
        if (ot_id[j])
          global_collision_body_transforms_[index_co + j] = global_link_transforms_[index_l];
        else
          global_collision_body_transforms_[index_co + j].affine().noalias() =
              global_link_transforms_[index_l].affine() * ot[j].matrix();
      }
    }
  }
}

void RobotState::updateLinkTransforms()
{
  if (dirty_link_transforms_ != nullptr)
  {
    updateLinkTransformsInternal(dirty_link_transforms_);
    if (dirty_collision_body_transforms_)
      dirty_collision_body_transforms_ =
          robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    else
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    dirty_link_transforms_ = nullptr;
  }
}

void RobotState::updateLinkTransformsInternal(const JointModel* start)
{
  for (const LinkModel* link : start->getDescendantLinkModels())
  {
    int idx_link = link->getLinkIndex();
    const LinkModel* parent = link->getParentLinkModel();
    if (parent)  // root JointModel will not have a parent
    {
      int idx_parent = parent->getLinkIndex();
      if (link->parentJointIsFixed())  // fixed joint
        global_link_transforms_[idx_link].affine().noalias() =
            global_link_transforms_[idx_parent].affine() * link->getJointOriginTransform().matrix();
      else  // non-fixed joint
      {
        if (link->jointOriginTransformIsIdentity())  // Link has identity transform
          global_link_transforms_[idx_link].affine().noalias() =
              global_link_transforms_[idx_parent].affine() * getJointTransform(link->getParentJointModel()).matrix();
        else  // Link has non-identity transform
          global_link_transforms_[idx_link].affine().noalias() =
              global_link_transforms_[idx_parent].affine() * link->getJointOriginTransform().matrix() *
              getJointTransform(link->getParentJointModel()).matrix();
      }
    }
    else  // is the origin / root / 'model frame'
    {
      if (link->jointOriginTransformIsIdentity())
        global_link_transforms_[idx_link] = getJointTransform(link->getParentJointModel());
      else
        global_link_transforms_[idx_link].affine().noalias() =
            link->getJointOriginTransform().affine() * getJointTransform(link->getParentJointModel()).matrix();
    }
  }

  // update attached bodies tf; these are usually very few, so we update them all
  for (const auto& attached_body : attached_body_map_)
    attached_body.second->computeTransform(
        global_link_transforms_[attached_body.second->getAttachedLink()->getLinkIndex()]);
}

void RobotState::updateStateWithLinkAt(const LinkModel* link, const Eigen::Isometry3d& transform, bool backward)
{
  updateLinkTransforms();  // no link transforms must be dirty, otherwise the transform we set will be overwritten

  // update the fact that collision body transforms are out of date
  if (dirty_collision_body_transforms_)
    dirty_collision_body_transforms_ =
        robot_model_->getCommonRoot(dirty_collision_body_transforms_, link->getParentJointModel());
  else
    dirty_collision_body_transforms_ = link->getParentJointModel();

  global_link_transforms_[link->getLinkIndex()] = transform;

  // update link transforms for descendant links only (leaving the transform for the current link untouched)
  const std::vector<const JointModel*>& cj = link->getChildJointModels();
  for (const JointModel* joint : cj)
    updateLinkTransformsInternal(joint);

  // if we also need to go backward
  if (backward)
  {
    const LinkModel* parent_link = link;
    const LinkModel* child_link;
    while (parent_link->getParentJointModel()->getParentLinkModel())
    {
      child_link = parent_link;
      parent_link = parent_link->getParentJointModel()->getParentLinkModel();

      // update the transform of the parent
      global_link_transforms_[parent_link->getLinkIndex()] =
          global_link_transforms_[child_link->getLinkIndex()] *
          (child_link->getJointOriginTransform() *
           variable_joint_transforms_[child_link->getParentJointModel()->getJointIndex()])
              .inverse();

      // update link transforms for descendant links only (leaving the transform for the current link untouched)
      // with the exception of the child link we are coming backwards from
      const std::vector<const JointModel*>& cj = parent_link->getChildJointModels();
      for (const JointModel* joint : cj)
        if (joint != child_link->getParentJointModel())
          updateLinkTransformsInternal(joint);
    }
    // all collision body transforms are invalid now
    dirty_collision_body_transforms_ = parent_link->getParentJointModel();
  }

  // update attached bodies tf; these are usually very few, so we update them all
  for (const auto& attached_body : attached_body_map_)
    attached_body.second->computeTransform(
        global_link_transforms_[attached_body.second->getAttachedLink()->getLinkIndex()]);
}

const LinkModel* RobotState::getRigidlyConnectedParentLinkModel(const std::string& frame) const
{
  const moveit::core::LinkModel* link{ nullptr };

  size_t idx = 0;
  if ((idx = frame.find('/')) != std::string::npos)
  {  // resolve sub frame
    std::string object{ frame.substr(0, idx) };
    if (!hasAttachedBody(object))
      return nullptr;
    auto body{ getAttachedBody(object) };
    if (!body->hasSubframeTransform(frame))
      return nullptr;
    link = body->getAttachedLink();
  }
  else if (hasAttachedBody(frame))
  {
    link = getAttachedBody(frame)->getAttachedLink();
  }
  else if (getRobotModel()->hasLinkModel(frame))
    link = getLinkModel(frame);

  return getRobotModel()->getRigidlyConnectedParentLinkModel(link);
}

bool RobotState::satisfiesBounds(double margin) const
{
  const std::vector<const JointModel*>& jm = robot_model_->getActiveJointModels();
  for (const JointModel* joint : jm)
    if (!satisfiesBounds(joint, margin))
      return false;
  return true;
}

bool RobotState::satisfiesBounds(const JointModelGroup* group, double margin) const
{
  const std::vector<const JointModel*>& jm = group->getActiveJointModels();
  for (const JointModel* joint : jm)
    if (!satisfiesBounds(joint, margin))
      return false;
  return true;
}

void RobotState::enforceBounds()
{
  const std::vector<const JointModel*>& jm = robot_model_->getActiveJointModels();
  for (const JointModel* joint : jm)
    enforceBounds(joint);
}

void RobotState::enforceBounds(const JointModelGroup* joint_group)
{
  const std::vector<const JointModel*>& jm = joint_group->getActiveJointModels();
  for (const JointModel* joint : jm)
    enforceBounds(joint);
}

void RobotState::harmonizePositions()
{
  for (const JointModel* jm : robot_model_->getActiveJointModels())
    harmonizePosition(jm);
}

void RobotState::harmonizePositions(const JointModelGroup* joint_group)
{
  for (const JointModel* jm : joint_group->getActiveJointModels())
    harmonizePosition(jm);
}

std::pair<double, const JointModel*> RobotState::getMinDistanceToPositionBounds() const
{
  return getMinDistanceToPositionBounds(robot_model_->getActiveJointModels());
}

std::pair<double, const JointModel*> RobotState::getMinDistanceToPositionBounds(const JointModelGroup* group) const
{
  return getMinDistanceToPositionBounds(group->getActiveJointModels());
}

std::pair<double, const JointModel*>
RobotState::getMinDistanceToPositionBounds(const std::vector<const JointModel*>& joints) const
{
  double distance = std::numeric_limits<double>::max();
  const JointModel* index = nullptr;
  for (const JointModel* joint : joints)
  {
    if (joint->getType() == JointModel::PLANAR || joint->getType() == JointModel::FLOATING)
      continue;
    if (joint->getType() == JointModel::REVOLUTE)
      if (static_cast<const RevoluteJointModel*>(joint)->isContinuous())
        continue;

    const double* joint_values = getJointPositions(joint);
    const JointModel::Bounds& bounds = joint->getVariableBounds();
    std::vector<double> lower_bounds(bounds.size()), upper_bounds(bounds.size());
    for (std::size_t j = 0; j < bounds.size(); ++j)
    {
      lower_bounds[j] = bounds[j].min_position_;
      upper_bounds[j] = bounds[j].max_position_;
    }
    double new_distance = joint->distance(joint_values, &lower_bounds[0]);
    if (new_distance < distance)
    {
      index = joint;
      distance = new_distance;
    }
    new_distance = joint->distance(joint_values, &upper_bounds[0]);
    if (new_distance < distance)
    {
      index = joint;
      distance = new_distance;
    }
  }
  return std::make_pair(distance, index);
}

bool RobotState::isValidVelocityMove(const RobotState& other, const JointModelGroup* group, double dt) const
{
  const std::vector<const JointModel*>& jm = group->getActiveJointModels();
  for (const JointModel* joint_id : jm)
  {
    const int idx = joint_id->getFirstVariableIndex();
    const std::vector<VariableBounds>& bounds = joint_id->getVariableBounds();

    // Check velocity for each joint variable
    for (std::size_t var_id = 0; var_id < joint_id->getVariableCount(); ++var_id)
    {
      const double dtheta = std::abs(*(position_ + idx + var_id) - *(other.getVariablePositions() + idx + var_id));

      if (dtheta > dt * bounds[var_id].max_velocity_)
        return false;
    }
  }
  return true;
}

double RobotState::distance(const RobotState& other, const JointModelGroup* joint_group) const
{
  double d = 0.0;
  const std::vector<const JointModel*>& jm = joint_group->getActiveJointModels();
  for (const JointModel* joint : jm)
  {
    const int idx = joint->getFirstVariableIndex();
    d += joint->getDistanceFactor() * joint->distance(position_ + idx, other.position_ + idx);
  }
  return d;
}

void RobotState::interpolate(const RobotState& to, double t, RobotState& state) const
{
  moveit::core::checkInterpolationParamBounds(LOGNAME, t);
  robot_model_->interpolate(getVariablePositions(), to.getVariablePositions(), t, state.getVariablePositions());

  memset(state.dirty_joint_transforms_, 1, state.robot_model_->getJointModelCount() * sizeof(unsigned char));
  state.dirty_link_transforms_ = state.robot_model_->getRootJoint();
}

void RobotState::interpolate(const RobotState& to, double t, RobotState& state, const JointModelGroup* joint_group) const
{
  moveit::core::checkInterpolationParamBounds(LOGNAME, t);
  const std::vector<const JointModel*>& jm = joint_group->getActiveJointModels();
  for (const JointModel* joint : jm)
  {
    const int idx = joint->getFirstVariableIndex();
    joint->interpolate(position_ + idx, to.position_ + idx, t, state.position_ + idx);
  }
  state.updateMimicJoints(joint_group);
}

void RobotState::setAttachedBodyUpdateCallback(const AttachedBodyCallback& callback)
{
  attached_body_update_callback_ = callback;
}

bool RobotState::hasAttachedBody(const std::string& id) const
{
  return attached_body_map_.find(id) != attached_body_map_.end();
}

const AttachedBody* RobotState::getAttachedBody(const std::string& id) const
{
  const auto it = attached_body_map_.find(id);
  if (it == attached_body_map_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "Attached body '%s' not found", id.c_str());
    return nullptr;
  }
  else
    return it->second.get();
}

void RobotState::attachBody(std::unique_ptr<AttachedBody> attached_body)
{
  // If an attached body with the same id exists, remove it
  clearAttachedBody(attached_body->getName());

  attached_body->computeTransform(getGlobalLinkTransform(attached_body->getAttachedLink()));
  if (attached_body_update_callback_)
    attached_body_update_callback_(attached_body.get(), true);
  attached_body_map_[attached_body->getName()] = std::move(attached_body);
}

void RobotState::attachBody(AttachedBody* attached_body)
{
  attachBody(std::unique_ptr<AttachedBody>(attached_body));
}

void RobotState::attachBody(const std::string& id, const Eigen::Isometry3d& pose,
                            const std::vector<shapes::ShapeConstPtr>& shapes,
                            const EigenSTL::vector_Isometry3d& shape_poses, const std::set<std::string>& touch_links,
                            const std::string& link, const trajectory_msgs::JointTrajectory& detach_posture,
                            const moveit::core::FixedTransformsMap& subframe_poses)
{
  attachBody(std::make_unique<AttachedBody>(robot_model_->getLinkModel(link), id, pose, shapes, shape_poses,
                                            touch_links, detach_posture, subframe_poses));
}

void RobotState::getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies) const
{
  attached_bodies.clear();
  attached_bodies.reserve(attached_body_map_.size());
  for (const auto& it : attached_body_map_)
    attached_bodies.push_back(it.second.get());
}

void RobotState::getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const JointModelGroup* group) const
{
  attached_bodies.clear();
  for (const auto& it : attached_body_map_)
    if (group->hasLinkModel(it.second->getAttachedLinkName()))
      attached_bodies.push_back(it.second.get());
}

void RobotState::getAttachedBodies(std::vector<const AttachedBody*>& attached_bodies, const LinkModel* link_model) const
{
  attached_bodies.clear();
  for (const auto& it : attached_body_map_)
    if (it.second->getAttachedLink() == link_model)
      attached_bodies.push_back(it.second.get());
}

void RobotState::clearAttachedBodies()
{
  for (const auto& it : attached_body_map_)
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it.second.get(), false);
  }
  attached_body_map_.clear();
}

void RobotState::clearAttachedBodies(const LinkModel* link)
{
  for (auto it = attached_body_map_.cbegin(); it != attached_body_map_.cend(); ++it)
  {
    if (it->second->getAttachedLink() != link)
    {
      continue;
    }
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second.get(), false);
    const auto del = it++;
    attached_body_map_.erase(del);
  }
}

void RobotState::clearAttachedBodies(const JointModelGroup* group)
{
  for (auto it = attached_body_map_.cbegin(); it != attached_body_map_.cend(); ++it)
  {
    if (!group->hasLinkModel(it->second->getAttachedLinkName()))
    {
      continue;
    }
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second.get(), false);
    const auto del = it++;
    attached_body_map_.erase(del);
  }
}

bool RobotState::clearAttachedBody(const std::string& id)
{
  const auto it = attached_body_map_.find(id);
  if (it != attached_body_map_.end())
  {
    if (attached_body_update_callback_)
      attached_body_update_callback_(it->second.get(), false);
    attached_body_map_.erase(it);
    return true;
  }
  else
    return false;
}

const Eigen::Isometry3d& RobotState::getFrameTransform(const std::string& frame_id, bool* frame_found)
{
  updateLinkTransforms();
  return static_cast<const RobotState*>(this)->getFrameTransform(frame_id, frame_found);
}

const Eigen::Isometry3d& RobotState::getFrameTransform(const std::string& frame_id, bool* frame_found) const
{
  const LinkModel* ignored_link;
  bool found;
  const auto& result = getFrameInfo(frame_id, ignored_link, found);

  if (frame_found)
    *frame_found = found;
  else if (!found)
    ROS_WARN_NAMED(LOGNAME, "getFrameTransform() did not find a frame with name %s.", frame_id.c_str());

  return result;
}

const Eigen::Isometry3d& RobotState::getFrameInfo(const std::string& frame_id, const LinkModel*& robot_link,
                                                  bool& frame_found) const
{
  if (!frame_id.empty() && frame_id[0] == '/')
    return getFrameInfo(frame_id.substr(1), robot_link, frame_found);

  static const Eigen::Isometry3d IDENTITY_TRANSFORM = Eigen::Isometry3d::Identity();
  if (frame_id == robot_model_->getModelFrame())
  {
    robot_link = robot_model_->getRootLink();
    frame_found = true;
    return IDENTITY_TRANSFORM;
  }
  if ((robot_link = robot_model_->getLinkModel(frame_id, &frame_found)))
  {
    BOOST_VERIFY(checkLinkTransforms());
    return global_link_transforms_[robot_link->getLinkIndex()];
  }
  robot_link = nullptr;

  // Check names of the attached bodies
  const auto jt = attached_body_map_.find(frame_id);
  if (jt != attached_body_map_.end())
  {
    const Eigen::Isometry3d& transform = jt->second->getGlobalPose();
    robot_link = jt->second->getAttachedLink();
    frame_found = true;
    BOOST_VERIFY(checkLinkTransforms());
    return transform;
  }

  // Check if an AttachedBody has a subframe with name frame_id
  for (const auto& body : attached_body_map_)
  {
    const Eigen::Isometry3d& transform = body.second->getGlobalSubframeTransform(frame_id, &frame_found);
    if (frame_found)
    {
      robot_link = body.second->getAttachedLink();
      BOOST_VERIFY(checkLinkTransforms());
      return transform;
    }
  }

  robot_link = nullptr;
  frame_found = false;
  return IDENTITY_TRANSFORM;
}

bool RobotState::knowsFrameTransform(const std::string& frame_id) const
{
  if (!frame_id.empty() && frame_id[0] == '/')
    return knowsFrameTransform(frame_id.substr(1));
  if (robot_model_->hasLinkModel(frame_id))
    return true;

  // Check if an AttachedBody with name frame_id exists
  const auto it = attached_body_map_.find(frame_id);
  if (it != attached_body_map_.end())
    return !it->second->getGlobalCollisionBodyTransforms().empty();

  // Check if an AttachedBody has a subframe with name frame_id
  for (const auto& body : attached_body_map_)
  {
    if (body.second->hasSubframeTransform(frame_id))
      return true;
  }
  return false;
}

void RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                                 const std_msgs::ColorRGBA& color, const std::string& ns, const ros::Duration& dur,
                                 bool include_attached) const
{
  std::size_t cur_num = arr.markers.size();
  getRobotMarkers(arr, link_names, include_attached);
  unsigned int id = cur_num;
  for (std::size_t i = cur_num; i < arr.markers.size(); ++i, ++id)
  {
    arr.markers[i].ns = ns;
    arr.markers[i].id = id;
    arr.markers[i].lifetime = dur;
    arr.markers[i].color = color;
  }
}

void RobotState::getRobotMarkers(visualization_msgs::MarkerArray& arr, const std::vector<std::string>& link_names,
                                 bool include_attached) const
{
  ros::Time tm = ros::Time::now();
  for (const std::string& link_name : link_names)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Trying to get marker for link '%s'", link_name.c_str());
    const LinkModel* link_model = robot_model_->getLinkModel(link_name);
    if (!link_model)
      continue;
    if (include_attached)
      for (const auto& it : attached_body_map_)
        if (it.second->getAttachedLink() == link_model)
        {
          for (std::size_t j = 0; j < it.second->getShapes().size(); ++j)
          {
            visualization_msgs::Marker att_mark;
            att_mark.header.frame_id = robot_model_->getModelFrame();
            att_mark.header.stamp = tm;
            if (shapes::constructMarkerFromShape(it.second->getShapes()[j].get(), att_mark))
            {
              // if the object is invisible (0 volume) we skip it
              if (fabs(att_mark.scale.x * att_mark.scale.y * att_mark.scale.z) < std::numeric_limits<float>::epsilon())
                continue;
              att_mark.pose = tf2::toMsg(it.second->getGlobalCollisionBodyTransforms()[j]);
              arr.markers.push_back(att_mark);
            }
          }
        }

    if (link_model->getShapes().empty())
      continue;

    for (std::size_t j = 0; j < link_model->getShapes().size(); ++j)
    {
      visualization_msgs::Marker mark;
      mark.header.frame_id = robot_model_->getModelFrame();
      mark.header.stamp = tm;

      // we prefer using the visual mesh, if a mesh is available and we have one body to render
      const std::string& mesh_resource = link_model->getVisualMeshFilename();
      if (mesh_resource.empty() || link_model->getShapes().size() > 1)
      {
        if (!shapes::constructMarkerFromShape(link_model->getShapes()[j].get(), mark))
          continue;
        // if the object is invisible (0 volume) we skip it
        if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon())
          continue;
        mark.pose =
            tf2::toMsg(global_collision_body_transforms_[link_model->getFirstCollisionBodyTransformIndex() + j]);
      }
      else
      {
        mark.type = mark.MESH_RESOURCE;
        mark.mesh_use_embedded_materials = false;
        mark.mesh_resource = mesh_resource;
        const Eigen::Vector3d& mesh_scale = link_model->getVisualMeshScale();

        mark.scale.x = mesh_scale[0];
        mark.scale.y = mesh_scale[1];
        mark.scale.z = mesh_scale[2];
        mark.pose = tf2::toMsg(global_link_transforms_[link_model->getLinkIndex()] * link_model->getVisualMeshOrigin());
      }

      arr.markers.push_back(mark);
    }
  }
}

Eigen::MatrixXd RobotState::getJacobian(const JointModelGroup* group,
                                        const Eigen::Vector3d& reference_point_position) const
{
  Eigen::MatrixXd result;
  if (!getJacobian(group, group->getLinkModels().back(), reference_point_position, result, false))
    throw Exception("Unable to compute Jacobian");
  return result;
}

bool RobotState::getJacobian(const JointModelGroup* group, const LinkModel* link,
                             const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& jacobian,
                             bool use_quaternion_representation) const
{
  BOOST_VERIFY(checkLinkTransforms());

  if (!group->isChain())
  {
    ROS_ERROR_NAMED(LOGNAME, "The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
    return false;
  }

  if (!group->isLinkUpdated(link->getName()))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link name '%s' does not exist in the chain '%s' or is not a child for this chain",
                    link->getName().c_str(), group->getName().c_str());
    return false;
  }

  const moveit::core::JointModel* root_joint_model = group->getJointModels()[0];  // group->getJointRoots()[0];
  const moveit::core::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
  // getGlobalLinkTransform() returns a valid isometry by contract
  Eigen::Isometry3d reference_transform =
      root_link_model ? getGlobalLinkTransform(root_link_model).inverse() : Eigen::Isometry3d::Identity();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = group->getVariableCount();
  jacobian = Eigen::MatrixXd::Zero(rows, columns);

  // getGlobalLinkTransform() returns a valid isometry by contract
  Eigen::Isometry3d link_transform = reference_transform * getGlobalLinkTransform(link);  // valid isometry
  Eigen::Vector3d point_transform = link_transform * reference_point_position;

  /*
  ROS_DEBUG_NAMED(LOGNAME, "Point from reference origin expressed in world coordinates: %f %f %f",
           point_transform.x(),
           point_transform.y(),
           point_transform.z());
  */

  Eigen::Vector3d joint_axis;
  Eigen::Isometry3d joint_transform;

  while (link)
  {
    /*
    ROS_DEBUG_NAMED(LOGNAME, "Link: %s, %f %f %f",link_state->getName().c_str(),
             link_state->getGlobalLinkTransform().translation().x(),
             link_state->getGlobalLinkTransform().translation().y(),
             link_state->getGlobalLinkTransform().translation().z());
    ROS_DEBUG_NAMED(LOGNAME, "Joint: %s",link_state->getParentJointState()->getName().c_str());
    */
    const JointModel* pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      if (!group->hasJointModel(pjm->getName()))
      {
        link = pjm->getParentLinkModel();
        continue;
      }
      unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
      // getGlobalLinkTransform() returns a valid isometry by contract
      joint_transform = reference_transform * getGlobalLinkTransform(link);  // valid isometry
      if (pjm->getType() == moveit::core::JointModel::REVOLUTE)
      {
        joint_axis = joint_transform.linear() * static_cast<const moveit::core::RevoluteJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) =
            jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
      }
      else if (pjm->getType() == moveit::core::JointModel::PRISMATIC)
      {
        joint_axis = joint_transform.linear() * static_cast<const moveit::core::PrismaticJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
      }
      else if (pjm->getType() == moveit::core::JointModel::PLANAR)
      {
        joint_axis = joint_transform * Eigen::Vector3d(1.0, 0.0, 0.0);
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 1.0, 0.0);
        jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
        joint_axis = joint_transform * Eigen::Vector3d(0.0, 0.0, 1.0);
        jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) +
                                                   joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
      }
      else
        ROS_ERROR_NAMED(LOGNAME, "Unknown type of joint in Jacobian computation");
    }
    if (pjm == root_joint_model)
      break;
    link = pjm->getParentLinkModel();
  }
  if (use_quaternion_representation)
  {  // Quaternion representation
    // From "Advanced Dynamics and Motion Simulation" by Paul Mitiguy
    // d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
    //        [x]           [  w -z  y ]    [ omega_2 ]
    //        [y]           [  z  w -x ]    [ omega_3 ]
    //        [z]           [ -y  x  w ]
    Eigen::Quaterniond q(link_transform.linear());
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();
    Eigen::MatrixXd quaternion_update_matrix(4, 3);
    quaternion_update_matrix << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;
    jacobian.block(3, 0, 4, columns) = 0.5 * quaternion_update_matrix * jacobian.block(3, 0, 3, columns);
  }
  return true;
}

bool RobotState::setFromDiffIK(const JointModelGroup* jmg, const Eigen::VectorXd& twist, const std::string& tip,
                               double dt, const GroupStateValidityCallbackFn& constraint)
{
  Eigen::VectorXd qdot;
  computeVariableVelocity(jmg, qdot, twist, getLinkModel(tip));
  return integrateVariableVelocity(jmg, qdot, dt, constraint);
}

bool RobotState::setFromDiffIK(const JointModelGroup* jmg, const geometry_msgs::Twist& twist, const std::string& tip,
                               double dt, const GroupStateValidityCallbackFn& constraint)
{
  Eigen::Matrix<double, 6, 1> t;
  tf2::fromMsg(twist, t);
  return setFromDiffIK(jmg, t, tip, dt, constraint);
}

void RobotState::computeVariableVelocity(const JointModelGroup* jmg, Eigen::VectorXd& qdot,
                                         const Eigen::VectorXd& twist, const LinkModel* tip) const
{
  // Get the Jacobian of the group at the current configuration
  Eigen::MatrixXd j(6, jmg->getVariableCount());
  Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
  getJacobian(jmg, tip, reference_point, j, false);

  // Rotate the jacobian to the end-effector frame
  Eigen::Isometry3d e_mb = getGlobalLinkTransform(tip).inverse();
  Eigen::MatrixXd e_wb = Eigen::ArrayXXd::Zero(6, 6);
  e_wb.block(0, 0, 3, 3) = e_mb.matrix().block(0, 0, 3, 3);
  e_wb.block(3, 3, 3, 3) = e_mb.matrix().block(0, 0, 3, 3);
  j = e_wb * j;

  // Do the Jacobian moore-penrose pseudo-inverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_of_j(j, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd& u = svd_of_j.matrixU();
  const Eigen::MatrixXd& v = svd_of_j.matrixV();
  const Eigen::VectorXd& s = svd_of_j.singularValues();

  Eigen::VectorXd sinv = s;
  static const double PINVTOLER = std::numeric_limits<float>::epsilon();
  double maxsv = 0.0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(s.rows()); ++i)
    if (fabs(s(i)) > maxsv)
      maxsv = fabs(s(i));
  for (std::size_t i = 0; i < static_cast<std::size_t>(s.rows()); ++i)
  {
    // Those singular values smaller than a percentage of the maximum singular value are removed
    if (fabs(s(i)) > maxsv * PINVTOLER)
      sinv(i) = 1.0 / s(i);
    else
      sinv(i) = 0.0;
  }
  Eigen::MatrixXd jinv = (v * sinv.asDiagonal() * u.transpose());

  // Compute joint velocity
  qdot = jinv * twist;
}

bool RobotState::integrateVariableVelocity(const JointModelGroup* jmg, const Eigen::VectorXd& qdot, double dt,
                                           const GroupStateValidityCallbackFn& constraint)
{
  Eigen::VectorXd q(jmg->getVariableCount());
  copyJointGroupPositions(jmg, q);
  q = q + dt * qdot;
  setJointGroupPositions(jmg, q);
  enforceBounds(jmg);

  if (constraint)
  {
    std::vector<double> values;
    copyJointGroupPositions(jmg, values);
    return constraint(this, jmg, &values[0]);
  }
  else
    return true;
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const geometry_msgs::Pose& pose, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    ROS_ERROR_NAMED(LOGNAME, "No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
    return false;
  }
  return setFromIK(jmg, pose, solver->getTipFrame(), timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const geometry_msgs::Pose& pose, const std::string& tip,
                           double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  Eigen::Isometry3d mat;
  tf2::fromMsg(pose, mat);
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, mat, tip, consistency_limits, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const Eigen::Isometry3d& pose, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
  if (!solver)
  {
    ROS_ERROR_NAMED(LOGNAME, "No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
    return false;
  }
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, pose, solver->getTipFrame(), consistency_limits, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const Eigen::Isometry3d& pose_in, const std::string& tip_in,
                           double timeout, const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  static std::vector<double> consistency_limits;
  return setFromIK(jmg, pose_in, tip_in, consistency_limits, timeout, constraint, options);
}

namespace
{
void ikCallbackFnAdapter(RobotState* state, const JointModelGroup* group,
                         const GroupStateValidityCallbackFn& constraint, const geometry_msgs::Pose& /*unused*/,
                         const std::vector<double>& ik_sol, moveit_msgs::MoveItErrorCodes& error_code)
{
  const std::vector<unsigned int>& bij = group->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[bij[i]] = ik_sol[i];
  if (constraint(state, group, &solution[0]))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
}
}  // namespace

bool RobotState::setToIKSolverFrame(Eigen::Isometry3d& pose, const kinematics::KinematicsBaseConstPtr& solver)
{
  return setToIKSolverFrame(pose, solver->getBaseFrame());
}

bool RobotState::setToIKSolverFrame(Eigen::Isometry3d& pose, const std::string& ik_frame)
{
  // Bring the pose to the frame of the IK solver
  if (!Transforms::sameFrame(ik_frame, robot_model_->getModelFrame()))
  {
    const LinkModel* link_model =
        getLinkModel((!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);
    if (!link_model)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "IK frame '" << ik_frame << "' does not exist.");
      return false;
    }
    pose = getGlobalLinkTransform(link_model).inverse() * pose;
  }
  return true;
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const Eigen::Isometry3d& pose_in, const std::string& tip_in,
                           const std::vector<double>& consistency_limits_in, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  // Convert from single pose and tip to vectors
  EigenSTL::vector_Isometry3d poses;
  poses.push_back(pose_in);

  std::vector<std::string> tips;
  tips.push_back(tip_in);

  std::vector<std::vector<double> > consistency_limits;
  consistency_limits.push_back(consistency_limits_in);

  return setFromIK(jmg, poses, tips, consistency_limits, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const EigenSTL::vector_Isometry3d& poses_in,
                           const std::vector<std::string>& tips_in, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  const std::vector<std::vector<double> > consistency_limits;
  return setFromIK(jmg, poses_in, tips_in, consistency_limits, timeout, constraint, options);
}

bool RobotState::setFromIK(const JointModelGroup* jmg, const EigenSTL::vector_Isometry3d& poses_in,
                           const std::vector<std::string>& tips_in,
                           const std::vector<std::vector<double> >& consistency_limit_sets, double timeout,
                           const GroupStateValidityCallbackFn& constraint,
                           const kinematics::KinematicsQueryOptions& options)
{
  // Error check
  if (poses_in.size() != tips_in.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of poses must be the same as number of tips");
    return false;
  }

  // Load solver
  const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();

  // Check if this jmg has a solver
  bool valid_solver = true;
  if (!solver)
  {
    valid_solver = false;
  }
  // Check if this jmg's IK solver can handle multiple tips (non-chain solver)
  else if (poses_in.size() > 1)
  {
    std::string error_msg;
    if (!solver->supportsGroup(jmg, &error_msg))
    {
      // skirt around clang-diagnostic-potentially-evaluated-expression
      const kinematics::KinematicsBase& solver_ref = *solver;
      ROS_ERROR_NAMED(LOGNAME, "Kinematics solver %s does not support joint group %s.  Error: %s",
                      typeid(solver_ref).name(), jmg->getName().c_str(), error_msg.c_str());
      valid_solver = false;
    }
  }

  if (!valid_solver)
  {
    // Check if there are subgroups that can solve this for us (non-chains)
    if (poses_in.size() > 1)
    {
      // Forward to setFromIKSubgroups() to allow different subgroup IK solvers to work together
      return setFromIKSubgroups(jmg, poses_in, tips_in, consistency_limit_sets, timeout, constraint, options);
    }
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "No kinematics solver instantiated for group '%s'", jmg->getName().c_str());
      return false;
    }
  }

  // Check that no, or only one set of consistency limits has been passed in, and choose that one
  std::vector<double> consistency_limits;
  if (consistency_limit_sets.size() > 1)
  {
    ROS_ERROR_NAMED(LOGNAME,
                    "Invalid number (%zu) of sets of consistency limits for a setFromIK request "
                    "that is being solved by a single IK solver",
                    consistency_limit_sets.size());
    return false;
  }
  else if (consistency_limit_sets.size() == 1)
    consistency_limits = consistency_limit_sets[0];

  const std::vector<std::string>& solver_tip_frames = solver->getTipFrames();

  // Track which possible tips frames we have filled in so far
  std::vector<bool> tip_frames_used(solver_tip_frames.size(), false);

  // Create vector to hold the output frames in the same order as solver_tip_frames
  std::vector<geometry_msgs::Pose> ik_queries(solver_tip_frames.size());

  // Bring each pose to the frame of the IK solver
  for (std::size_t i = 0; i < poses_in.size(); ++i)
  {
    // Make non-const
    Eigen::Isometry3d pose = poses_in[i];
    std::string pose_frame = tips_in[i];

    // Remove extra slash
    if (!pose_frame.empty() && pose_frame[0] == '/')
      pose_frame = pose_frame.substr(1);

    // bring the pose to the frame of the IK solver
    if (!setToIKSolverFrame(pose, solver))
      return false;

    // try all of the solver's possible tip frames to see if they uniquely align with any of our passed in pose tip
    // frames
    bool found_valid_frame = false;
    std::size_t solver_tip_id;  // our current index
    for (solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
    {
      // Check if this tip frame is already accounted for
      if (tip_frames_used[solver_tip_id])
        continue;  // already has a pose

      // check if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
      std::string solver_tip_frame = solver_tip_frames[solver_tip_id];

      // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings
      // more often that we need to
      if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
        solver_tip_frame = solver_tip_frame.substr(1);

      if (pose_frame != solver_tip_frame)
      {
        if (hasAttachedBody(pose_frame))
        {
          const AttachedBody* body = getAttachedBody(pose_frame);
          pose_frame = body->getAttachedLinkName();
          pose = pose * body->getPose().inverse();
        }
        if (pose_frame != solver_tip_frame)
        {
          const moveit::core::LinkModel* link_model = getLinkModel(pose_frame);
          if (!link_model)
          {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Pose frame '" << pose_frame << "' does not exist.");
            return false;
          }
          const moveit::core::LinkTransformMap& fixed_links = link_model->getAssociatedFixedTransforms();
          for (const std::pair<const LinkModel* const, Eigen::Isometry3d>& fixed_link : fixed_links)
            if (Transforms::sameFrame(fixed_link.first->getName(), solver_tip_frame))
            {
              pose_frame = solver_tip_frame;
              pose = pose * fixed_link.second;
              break;
            }
        }

      }  // end if pose_frame

      // Check if this pose frame works
      if (pose_frame == solver_tip_frame)
      {
        found_valid_frame = true;
        break;
      }

    }  // end for solver_tip_frames

    // Make sure one of the tip frames worked
    if (!found_valid_frame)
    {
      ROS_ERROR_NAMED(LOGNAME, "Cannot compute IK for query %zu pose reference frame '%s'", i, pose_frame.c_str());
      // Debug available tip frames
      std::stringstream ss;
      for (solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
        ss << solver_tip_frames[solver_tip_id] << ", ";
      ROS_ERROR_NAMED(LOGNAME, "Available tip frames: [%s]", ss.str().c_str());
      return false;
    }

    // Remove that tip from the list of available tip frames because each can only have one pose
    tip_frames_used[solver_tip_id] = true;

    // Convert Eigen pose to geometry_msgs pose
    geometry_msgs::Pose ik_query;
    ik_query = tf2::toMsg(pose);

    // Save into vectors
    ik_queries[solver_tip_id] = ik_query;
  }  // end for poses_in

  // Create poses for all remaining tips a solver expects, even if not passed into this function
  for (std::size_t solver_tip_id = 0; solver_tip_id < solver_tip_frames.size(); ++solver_tip_id)
  {
    // Check if this tip frame is already accounted for
    if (tip_frames_used[solver_tip_id])
      continue;  // already has a pose

    // Process this tip
    std::string solver_tip_frame = solver_tip_frames[solver_tip_id];

    // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more
    // often that we need to
    if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
      solver_tip_frame = solver_tip_frame.substr(1);

    // Get the pose of a different EE tip link
    Eigen::Isometry3d current_pose = getGlobalLinkTransform(solver_tip_frame);

    // bring the pose to the frame of the IK solver
    if (!setToIKSolverFrame(current_pose, solver))
      return false;

    // Convert Eigen pose to geometry_msgs pose
    geometry_msgs::Pose ik_query;
    ik_query = tf2::toMsg(current_pose);

    // Save into vectors - but this needs to be ordered in the same order as the IK solver expects its tip frames
    ik_queries[solver_tip_id] = ik_query;

    // Remove that tip from the list of available tip frames because each can only have one pose
    tip_frames_used[solver_tip_id] = true;
  }

  // if no timeout has been specified, use the default one
  if (timeout < std::numeric_limits<double>::epsilon())
    timeout = jmg->getDefaultIKTimeout();

  // set callback function
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn;
  if (constraint)
    ik_callback_fn = [this, jmg, constraint](const geometry_msgs::Pose& pose, const std::vector<double>& joints,
                                             moveit_msgs::MoveItErrorCodes& error_code) {
      ikCallbackFnAdapter(this, jmg, constraint, pose, joints, error_code);
    };

  // Bijection
  const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();

  std::vector<double> initial_values;
  copyJointGroupPositions(jmg, initial_values);
  std::vector<double> seed(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    seed[i] = initial_values[bij[i]];

  // compute the IK solution
  std::vector<double> ik_sol;
  moveit_msgs::MoveItErrorCodes error;

  if (solver->searchPositionIK(ik_queries, seed, timeout, consistency_limits, ik_sol, ik_callback_fn, error, options,
                               this))
  {
    std::vector<double> solution(bij.size());
    for (std::size_t i = 0; i < bij.size(); ++i)
      solution[bij[i]] = ik_sol[i];
    setJointGroupPositions(jmg, solution);
    return true;
  }
  return false;
}

bool RobotState::setFromIKSubgroups(const JointModelGroup* jmg, const EigenSTL::vector_Isometry3d& poses_in,
                                    const std::vector<std::string>& tips_in,
                                    const std::vector<std::vector<double> >& consistency_limits, double timeout,
                                    const GroupStateValidityCallbackFn& constraint,
                                    const kinematics::KinematicsQueryOptions& /*options*/)
{
  // Assume we have already ran setFromIK() and those checks

  // Get containing subgroups
  std::vector<const JointModelGroup*> sub_groups;
  jmg->getSubgroups(sub_groups);

  // Error check
  if (poses_in.size() != sub_groups.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of poses (%zu) must be the same as number of sub-groups (%zu)", poses_in.size(),
                    sub_groups.size());
    return false;
  }

  if (tips_in.size() != sub_groups.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of tip names (%zu) must be same as number of sub-groups (%zu)", tips_in.size(),
                    sub_groups.size());
    return false;
  }

  if (!consistency_limits.empty() && consistency_limits.size() != sub_groups.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "Number of consistency limit vectors must be the same as number of sub-groups");
    return false;
  }

  for (std::size_t i = 0; i < consistency_limits.size(); ++i)
  {
    if (consistency_limits[i].size() != sub_groups[i]->getVariableCount())
    {
      ROS_ERROR_NAMED(LOGNAME, "Number of joints in consistency_limits is %zu but it should be should be %u", i,
                      sub_groups[i]->getVariableCount());
      return false;
    }
  }

  // Populate list of kin solvers for the various subgroups
  std::vector<kinematics::KinematicsBaseConstPtr> solvers;
  for (std::size_t i = 0; i < poses_in.size(); ++i)
  {
    kinematics::KinematicsBaseConstPtr solver = sub_groups[i]->getSolverInstance();
    if (!solver)
    {
      ROS_ERROR_NAMED(LOGNAME, "Could not find solver for group '%s'", sub_groups[i]->getName().c_str());
      return false;
    }
    solvers.push_back(solver);
  }

  // Make non-const versions
  EigenSTL::vector_Isometry3d transformed_poses = poses_in;
  std::vector<std::string> pose_frames = tips_in;

  // Each each pose's tip frame naming
  for (std::size_t i = 0; i < poses_in.size(); ++i)
  {
    ASSERT_ISOMETRY(transformed_poses[i])  // unsanitized input, could contain a non-isometry
    Eigen::Isometry3d& pose = transformed_poses[i];
    std::string& pose_frame = pose_frames[i];

    // bring the pose to the frame of the IK solver
    if (!setToIKSolverFrame(pose, solvers[i]))
      return false;

    // see if the tip frame can be transformed via fixed transforms to the frame known to the IK solver
    std::string solver_tip_frame = solvers[i]->getTipFrame();

    // remove the frame '/' if there is one, so we can avoid calling Transforms::sameFrame() which may copy strings more
    // often that we need to
    if (!solver_tip_frame.empty() && solver_tip_frame[0] == '/')
      solver_tip_frame = solver_tip_frame.substr(1);

    if (pose_frame != solver_tip_frame)
    {
      if (hasAttachedBody(pose_frame))
      {
        const AttachedBody* body = getAttachedBody(pose_frame);
        pose_frame = body->getAttachedLinkName();
        pose = pose * body->getPose().inverse();  // valid isometry
      }
      if (pose_frame != solver_tip_frame)
      {
        const moveit::core::LinkModel* link_model = getLinkModel(pose_frame);
        if (!link_model)
          return false;
        // getAssociatedFixedTransforms() returns valid isometries by contract
        const moveit::core::LinkTransformMap& fixed_links = link_model->getAssociatedFixedTransforms();
        for (const std::pair<const LinkModel* const, Eigen::Isometry3d>& fixed_link : fixed_links)
          if (fixed_link.first->getName() == solver_tip_frame)
          {
            pose_frame = solver_tip_frame;
            pose = pose * fixed_link.second;  // valid isometry
            break;
          }
      }
    }

    if (pose_frame != solver_tip_frame)
    {
      ROS_ERROR_NAMED(LOGNAME, "Cannot compute IK for query pose reference frame '%s', desired: '%s'",
                      pose_frame.c_str(), solver_tip_frame.c_str());
      return false;
    }
  }

  // Convert Eigen poses to geometry_msg format
  std::vector<geometry_msgs::Pose> ik_queries(poses_in.size());
  for (std::size_t i = 0; i < transformed_poses.size(); ++i)
  {
    Eigen::Quaterniond quat(transformed_poses[i].linear());
    Eigen::Vector3d point(transformed_poses[i].translation());
    ik_queries[i].position.x = point.x();
    ik_queries[i].position.y = point.y();
    ik_queries[i].position.z = point.z();
    ik_queries[i].orientation.x = quat.x();
    ik_queries[i].orientation.y = quat.y();
    ik_queries[i].orientation.z = quat.z();
    ik_queries[i].orientation.w = quat.w();
  }

  // if no timeout has been specified, use the default one
  if (timeout < std::numeric_limits<double>::epsilon())
    timeout = jmg->getDefaultIKTimeout();
  ros::WallTime start = ros::WallTime::now();
  double elapsed = 0;

  bool first_seed = true;
  unsigned int attempts = 0;
  do
  {
    ++attempts;
    ROS_DEBUG_NAMED(LOGNAME, "IK attempt: %d", attempts);
    bool found_solution = true;
    for (std::size_t sg = 0; sg < sub_groups.size(); ++sg)
    {
      const std::vector<unsigned int>& bij = sub_groups[sg]->getKinematicsSolverJointBijection();
      std::vector<double> seed(bij.size());
      // the first seed is the initial state
      if (first_seed)
      {
        std::vector<double> initial_values;
        copyJointGroupPositions(sub_groups[sg], initial_values);
        for (std::size_t i = 0; i < bij.size(); ++i)
          seed[i] = initial_values[bij[i]];
      }
      else
      {
        // sample a random seed
        random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();
        std::vector<double> random_values;
        sub_groups[sg]->getVariableRandomPositions(rng, random_values);
        for (std::size_t i = 0; i < bij.size(); ++i)
          seed[i] = random_values[bij[i]];
      }

      // compute the IK solution
      std::vector<double> ik_sol;
      moveit_msgs::MoveItErrorCodes error;
      const std::vector<double>& climits = consistency_limits.empty() ? std::vector<double>() : consistency_limits[sg];
      if (solvers[sg]->searchPositionIK(ik_queries[sg], seed, (timeout - elapsed) / sub_groups.size(), climits, ik_sol,
                                        error))
      {
        std::vector<double> solution(bij.size());
        for (std::size_t i = 0; i < bij.size(); ++i)
          solution[bij[i]] = ik_sol[i];
        setJointGroupPositions(sub_groups[sg], solution);
      }
      else
      {
        found_solution = false;
        break;
      }
    }
    if (found_solution)
    {
      std::vector<double> full_solution;
      copyJointGroupPositions(jmg, full_solution);
      if (constraint ? constraint(this, jmg, &full_solution[0]) : true)
      {
        ROS_DEBUG_NAMED(LOGNAME, "Found IK solution");
        return true;
      }
    }
    elapsed = (ros::WallTime::now() - start).toSec();
    first_seed = false;
  } while (elapsed < timeout);
  return false;
}

double RobotState::computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                        const LinkModel* link, const Eigen::Vector3d& direction,
                                        bool global_reference_frame, double distance, double max_step,
                                        double jump_threshold_factor, const GroupStateValidityCallbackFn& validCallback,
                                        const kinematics::KinematicsQueryOptions& options)
{
  return CartesianInterpolator::computeCartesianPath(this, group, traj, link, direction, global_reference_frame,
                                                     distance, MaxEEFStep(max_step),
                                                     JumpThreshold(jump_threshold_factor), validCallback, options);
}

double RobotState::computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                        const LinkModel* link, const Eigen::Isometry3d& target,
                                        bool global_reference_frame, double max_step, double jump_threshold_factor,
                                        const GroupStateValidityCallbackFn& validCallback,
                                        const kinematics::KinematicsQueryOptions& options)
{
  return CartesianInterpolator::computeCartesianPath(this, group, traj, link, target, global_reference_frame,
                                                     MaxEEFStep(max_step), JumpThreshold(jump_threshold_factor),
                                                     validCallback, options);
}

double RobotState::computeCartesianPath(const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
                                        const LinkModel* link, const EigenSTL::vector_Isometry3d& waypoints,
                                        bool global_reference_frame, double max_step, double jump_threshold_factor,
                                        const GroupStateValidityCallbackFn& validCallback,
                                        const kinematics::KinematicsQueryOptions& options)
{
  return CartesianInterpolator::computeCartesianPath(this, group, traj, link, waypoints, global_reference_frame,
                                                     MaxEEFStep(max_step), JumpThreshold(jump_threshold_factor),
                                                     validCallback, options);
}

void RobotState::computeAABB(std::vector<double>& aabb) const
{
  BOOST_VERIFY(checkLinkTransforms());

  core::AABB bounding_box;
  std::vector<const LinkModel*> links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (const LinkModel* link : links)
  {
    Eigen::Isometry3d transform = getGlobalLinkTransform(link);  // intentional copy, we will translate
    const Eigen::Vector3d& extents = link->getShapeExtentsAtOrigin();
    transform.translate(link->getCenteredBoundingBoxOffset());
    bounding_box.extendWithTransformedBox(transform, extents);
  }
  for (const auto& it : attached_body_map_)
  {
    const EigenSTL::vector_Isometry3d& transforms = it.second->getGlobalCollisionBodyTransforms();
    const std::vector<shapes::ShapeConstPtr>& shapes = it.second->getShapes();
    for (std::size_t i = 0; i < transforms.size(); ++i)
    {
      Eigen::Vector3d extents = shapes::computeShapeExtents(shapes[i].get());
      bounding_box.extendWithTransformedBox(transforms[i], extents);
    }
  }

  aabb.clear();
  aabb.resize(6, 0.0);
  if (!bounding_box.isEmpty())
  {
    // The following is a shorthand for something like:
    // aabb[0, 2, 4] = bounding_box.min(); aabb[1, 3, 5] = bounding_box.max();
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned, Eigen::InnerStride<2> >(aabb.data(), 3) = bounding_box.min();
    Eigen::Map<Eigen::VectorXd, Eigen::Unaligned, Eigen::InnerStride<2> >(aabb.data() + 1, 3) = bounding_box.max();
  }
}

void RobotState::printStatePositions(std::ostream& out) const
{
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (std::size_t i = 0; i < nm.size(); ++i)
    out << nm[i] << "=" << position_[i] << std::endl;
}

void RobotState::printStatePositionsWithJointLimits(const moveit::core::JointModelGroup* jmg, std::ostream& out) const
{
  // TODO(davetcoleman): support joints with multiple variables / multiple DOFs such as floating joints
  // TODO(davetcoleman): support unbounded joints

  const std::vector<const moveit::core::JointModel*>& joints = jmg->getActiveJointModels();

  // Loop through joints
  for (const JointModel* joint : joints)
  {
    // Ignore joints with more than one variable
    if (joint->getVariableCount() > 1)
      continue;

    double current_value = getVariablePosition(joint->getName());

    // check if joint is beyond limits
    bool out_of_bounds = !satisfiesBounds(joint);

    const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

    if (out_of_bounds)
      out << MOVEIT_CONSOLE_COLOR_RED;

    out << "   " << std::fixed << std::setprecision(5) << bound.min_position_ << "\t";
    double delta = bound.max_position_ - bound.min_position_;
    double step = delta / 20.0;

    bool marker_shown = false;
    for (double value = bound.min_position_; value < bound.max_position_; value += step)
    {
      // show marker of current value
      if (!marker_shown && current_value < value)
      {
        out << "|";
        marker_shown = true;
      }
      else
        out << "-";
    }
    if (!marker_shown)
      out << "|";

    // show max position
    out << " \t" << std::fixed << std::setprecision(5) << bound.max_position_ << "  \t" << joint->getName()
        << " current: " << std::fixed << std::setprecision(5) << current_value << std::endl;

    if (out_of_bounds)
      out << MOVEIT_CONSOLE_COLOR_RESET;
  }
}

void RobotState::printDirtyInfo(std::ostream& out) const
{
  out << "  * Dirty Joint Transforms: " << std::endl;
  const std::vector<const JointModel*>& jm = robot_model_->getJointModels();
  for (const JointModel* joint : jm)
    if (joint->getVariableCount() > 0 && dirtyJointTransform(joint))
      out << "    " << joint->getName() << std::endl;
  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL")
      << std::endl;
  out << "  * Dirty Collision Body Transforms: "
      << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL") << std::endl;
}

void RobotState::printStateInfo(std::ostream& out) const
{
  out << "Robot State @" << this << std::endl;

  std::size_t n = robot_model_->getVariableCount();
  if (position_)
  {
    out << "  * Position: ";
    for (std::size_t i = 0; i < n; ++i)
      out << position_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Position: NULL" << std::endl;

  if (velocity_)
  {
    out << "  * Velocity: ";
    for (std::size_t i = 0; i < n; ++i)
      out << velocity_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Velocity: NULL" << std::endl;

  if (acceleration_)
  {
    out << "  * Acceleration: ";
    for (std::size_t i = 0; i < n; ++i)
      out << acceleration_[i] << " ";
    out << std::endl;
  }
  else
    out << "  * Acceleration: NULL" << std::endl;

  out << "  * Dirty Link Transforms: " << (dirty_link_transforms_ ? dirty_link_transforms_->getName() : "NULL")
      << std::endl;
  out << "  * Dirty Collision Body Transforms: "
      << (dirty_collision_body_transforms_ ? dirty_collision_body_transforms_->getName() : "NULL") << std::endl;

  printTransforms(out);
}

void RobotState::printTransform(const Eigen::Isometry3d& transform, std::ostream& out) const
{
  if (checkIsometry(transform, CHECK_ISOMETRY_PRECISION, false))
  {
    Eigen::Quaterniond q(transform.linear());
    out << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", "
        << transform.translation().z() << "], Q.xyzw = [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w()
        << "]";
  }
  else
  {
    out << "[NON-ISOMETRY] "
        << transform.matrix().format(
               Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]"));
  }
  out << std::endl;
}

void RobotState::printTransforms(std::ostream& out) const
{
  if (!variable_joint_transforms_)
  {
    out << "No transforms computed" << std::endl;
    return;
  }

  out << "Joint transforms:" << std::endl;
  const std::vector<const JointModel*>& jm = robot_model_->getJointModels();
  for (const JointModel* joint : jm)
  {
    out << "  " << joint->getName();
    const int idx = joint->getJointIndex();
    if (dirty_joint_transforms_[idx])
      out << " [dirty]";
    out << ": ";
    printTransform(variable_joint_transforms_[idx], out);
  }

  out << "Link poses:" << std::endl;
  const std::vector<const LinkModel*>& link_model = robot_model_->getLinkModels();
  for (const LinkModel* link : link_model)
  {
    out << "  " << link->getName() << ": ";
    printTransform(global_link_transforms_[link->getLinkIndex()], out);
  }
}

std::string RobotState::getStateTreeString() const
{
  std::stringstream ss;
  ss << "ROBOT: " << robot_model_->getName() << std::endl;
  getStateTreeJointString(ss, robot_model_->getRootJoint(), "   ", true);
  return ss.str();
}

namespace
{
void getPoseString(std::ostream& ss, const Eigen::Isometry3d& pose, const std::string& pfx)
{
  ss.precision(3);
  for (int y = 0; y < 4; ++y)
  {
    ss << pfx;
    for (int x = 0; x < 4; ++x)
    {
      ss << std::setw(8) << pose(y, x) << " ";
    }
    ss << std::endl;
  }
}
}  // namespace

void RobotState::getStateTreeJointString(std::ostream& ss, const JointModel* jm, const std::string& pfx0,
                                         bool last) const
{
  std::string pfx = pfx0 + "+--";

  ss << pfx << "Joint: " << jm->getName() << std::endl;

  pfx = pfx0 + (last ? "   " : "|  ");

  for (std::size_t i = 0; i < jm->getVariableCount(); ++i)
  {
    ss.precision(3);
    ss << pfx << jm->getVariableNames()[i] << std::setw(12) << position_[jm->getFirstVariableIndex() + i] << std::endl;
  }

  const LinkModel* link_model = jm->getChildLinkModel();

  ss << pfx << "Link: " << link_model->getName() << std::endl;
  getPoseString(ss, link_model->getJointOriginTransform(), pfx + "joint_origin:");
  if (variable_joint_transforms_)
  {
    getPoseString(ss, variable_joint_transforms_[jm->getJointIndex()], pfx + "joint_variable:");
    getPoseString(ss, global_link_transforms_[link_model->getLinkIndex()], pfx + "link_global:");
  }

  for (std::vector<const JointModel*>::const_iterator it = link_model->getChildJointModels().begin();
       it != link_model->getChildJointModels().end(); ++it)
    getStateTreeJointString(ss, *it, pfx, it + 1 == link_model->getChildJointModels().end());
}

std::ostream& operator<<(std::ostream& out, const RobotState& s)
{
  s.printStateInfo(out);
  return out;
}

bool haveSameAttachedObjects(const RobotState& left, const RobotState& right, const std::string& prefix)
{
  std::vector<const moveit::core::AttachedBody*> left_attached;
  std::vector<const moveit::core::AttachedBody*> right_attached;
  left.getAttachedBodies(left_attached);
  right.getAttachedBodies(right_attached);
  if (left_attached.size() != right_attached.size())
  {
    ROS_DEBUG_STREAM(prefix << "different number of objects");
    return false;
  }

  for (const moveit::core::AttachedBody* left_object : left_attached)
  {
    auto it = std::find_if(right_attached.cbegin(), right_attached.cend(),
                           [left_object](const moveit::core::AttachedBody* object) {
                             return object->getName() == left_object->getName();
                           });
    if (it == right_attached.cend())
    {
      ROS_DEBUG_STREAM(prefix << "object missing: " << left_object->getName());
      return false;
    }
    const moveit::core::AttachedBody* right_object = *it;
    if (left_object->getAttachedLink() != right_object->getAttachedLink())
    {
      ROS_DEBUG_STREAM(prefix << "different attach links: " << left_object->getName() << " attached to "
                              << left_object->getAttachedLinkName() << " / " << right_object->getAttachedLinkName());
      return false;  // links not matching
    }
    if (left_object->getShapes().size() != right_object->getShapes().size())
    {
      ROS_DEBUG_STREAM(prefix << "different object shapes: " << left_object->getName());
      return false;  // shapes not matching
    }

    auto left_it = left_object->getShapePosesInLinkFrame().cbegin();
    auto left_end = left_object->getShapePosesInLinkFrame().cend();
    auto right_it = right_object->getShapePosesInLinkFrame().cbegin();
    for (; left_it != left_end; ++left_it, ++right_it)
      if (!(left_it->matrix() - right_it->matrix()).isZero(1e-4))
      {
        ROS_DEBUG_STREAM(prefix << "different pose of attached object shape: " << left_object->getName());
        return false;  // transforms do not match
      }
  }
  return true;
}

}  // end of namespace core
}  // end of namespace moveit
