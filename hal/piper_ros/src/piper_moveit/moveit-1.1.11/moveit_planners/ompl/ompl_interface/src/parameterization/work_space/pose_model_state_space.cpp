/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <moveit/profiler/profiler.h>

#include <utility>

namespace ompl_interface
{
constexpr char LOGNAME[] = "pose_model_state_space";
}  // namespace ompl_interface

const std::string ompl_interface::PoseModelStateSpace::PARAMETERIZATION_TYPE = "PoseModel";

ompl_interface::PoseModelStateSpace::PoseModelStateSpace(const ModelBasedStateSpaceSpecification& spec)
  : ModelBasedStateSpace(spec)
{
  jump_factor_ = 3;  // \todo make this a param

  if (spec.joint_model_group_->getGroupKinematics().first)
    poses_.emplace_back(spec.joint_model_group_, spec.joint_model_group_->getGroupKinematics().first);
  else if (!spec.joint_model_group_->getGroupKinematics().second.empty())
  {
    const moveit::core::JointModelGroup::KinematicsSolverMap& m = spec.joint_model_group_->getGroupKinematics().second;
    for (const auto& it : m)
      poses_.emplace_back(it.first, it.second);
  }
  if (poses_.empty())
    ROS_ERROR_NAMED(LOGNAME, "No kinematics solvers specified. Unable to construct a "
                             "PoseModelStateSpace");
  else
    std::sort(poses_.begin(), poses_.end());
  setName(getName() + "_" + PARAMETERIZATION_TYPE);
}

ompl_interface::PoseModelStateSpace::~PoseModelStateSpace() = default;

double ompl_interface::PoseModelStateSpace::distance(const ompl::base::State* state1,
                                                     const ompl::base::State* state2) const
{
  double total = 0;
  for (std::size_t i = 0; i < poses_.size(); ++i)
    total += poses_[i].state_space_->distance(state1->as<StateType>()->poses[i], state2->as<StateType>()->poses[i]);
  return total;
}

double ompl_interface::PoseModelStateSpace::getMaximumExtent() const
{
  double total = 0.0;
  for (const auto& pose : poses_)
    total += pose.state_space_->getMaximumExtent();
  return total;
}

ompl::base::State* ompl_interface::PoseModelStateSpace::allocState() const
{
  auto* state = new StateType();
  state->values =
      new double[variable_count_];  // need to allocate this here since ModelBasedStateSpace::allocState() is not called
  state->poses = new ompl::base::SE3StateSpace::StateType*[poses_.size()];
  for (std::size_t i = 0; i < poses_.size(); ++i)
    state->poses[i] = poses_[i].state_space_->allocState()->as<ompl::base::SE3StateSpace::StateType>();
  return state;
}

void ompl_interface::PoseModelStateSpace::freeState(ompl::base::State* state) const
{
  for (std::size_t i = 0; i < poses_.size(); ++i)
    poses_[i].state_space_->freeState(state->as<StateType>()->poses[i]);
  delete[] state->as<StateType>()->poses;
  ModelBasedStateSpace::freeState(state);
}

void ompl_interface::PoseModelStateSpace::copyState(ompl::base::State* destination,
                                                    const ompl::base::State* source) const
{
  // copy the state data
  ModelBasedStateSpace::copyState(destination, source);

  for (std::size_t i = 0; i < poses_.size(); ++i)
    poses_[i].state_space_->copyState(destination->as<StateType>()->poses[i], source->as<StateType>()->poses[i]);

  // compute additional stuff if needed
  computeStateK(destination);
}

void ompl_interface::PoseModelStateSpace::sanityChecks() const
{
  ModelBasedStateSpace::sanityChecks(std::numeric_limits<double>::epsilon(), std::numeric_limits<float>::epsilon(),
                                     ~ompl::base::StateSpace::STATESPACE_TRIANGLE_INEQUALITY);
}

void ompl_interface::PoseModelStateSpace::interpolate(const ompl::base::State* from, const ompl::base::State* to,
                                                      const double t, ompl::base::State* state) const
{
  //  moveit::Profiler::ScopedBlock sblock("interpolate");

  // we want to interpolate in Cartesian space; we do not have a guarantee that from and to
  // have their poses computed, but this is very unlikely to happen (depends how the planner gets its input states)

  // interpolate in joint space
  ModelBasedStateSpace::interpolate(from, to, t, state);

  // interpolate SE3 components
  for (std::size_t i = 0; i < poses_.size(); ++i)
    poses_[i].state_space_->interpolate(from->as<StateType>()->poses[i], to->as<StateType>()->poses[i], t,
                                        state->as<StateType>()->poses[i]);

  // the call above may reset all flags for state; but we know the pose we want flag should be set
  state->as<StateType>()->setPoseComputed(true);

  /*
  std::cout << "*********** interpolate\n";
  printState(from, std::cout);
  printState(to, std::cout);
  printState(state, std::cout);
  std::cout << "\n\n";
  */

  // after interpolation we cannot be sure about the joint values (we use them as seed only)
  // so we recompute IK if needed
  if (computeStateIK(state))
  {
    double dj = jump_factor_ * ModelBasedStateSpace::distance(from, to);
    double d_from = ModelBasedStateSpace::distance(from, state);
    double d_to = ModelBasedStateSpace::distance(state, to);

    // if the joint value jumped too much
    if (d_from + d_to > std::max(0.2, dj))  // \todo make 0.2 a param
      state->as<StateType>()->markInvalid();
  }
}

void ompl_interface::PoseModelStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY,
                                                            double minZ, double maxZ)
{
  ModelBasedStateSpace::setPlanningVolume(minX, maxX, minY, maxY, minZ, maxZ);
  ompl::base::RealVectorBounds b(3);
  b.low[0] = minX;
  b.low[1] = minY;
  b.low[2] = minZ;
  b.high[0] = maxX;
  b.high[1] = maxY;
  b.high[2] = maxZ;
  for (auto& pose : poses_)
    pose.state_space_->as<ompl::base::SE3StateSpace>()->setBounds(b);
}

ompl_interface::PoseModelStateSpace::PoseComponent::PoseComponent(
    const moveit::core::JointModelGroup* subgroup, const moveit::core::JointModelGroup::KinematicsSolver& k)
  : subgroup_(subgroup), kinematics_solver_(k.allocator_(subgroup)), bijection_(k.bijection_)
{
  state_space_ = std::make_shared<ompl::base::SE3StateSpace>();
  state_space_->setName(subgroup_->getName() + "_Workspace");
  fk_link_.resize(1, kinematics_solver_->getTipFrame());
  if (!fk_link_[0].empty() && fk_link_[0][0] == '/')
    fk_link_[0] = fk_link_[0].substr(1);
}

bool ompl_interface::PoseModelStateSpace::PoseComponent::computeStateFK(StateType* full_state, unsigned int idx) const
{
  // read the values from the joint state, in the order expected by the kinematics solver
  std::vector<double> values(bijection_.size());
  for (unsigned int i = 0; i < bijection_.size(); ++i)
    values[i] = full_state->values[bijection_[i]];

  // compute forward kinematics for the link of interest
  std::vector<geometry_msgs::Pose> poses;
  if (!kinematics_solver_->getPositionFK(fk_link_, values, poses))
    return false;

  // copy the resulting data to the desired location in the state
  ompl::base::SE3StateSpace::StateType* se3_state = full_state->poses[idx];
  se3_state->setXYZ(poses[0].position.x, poses[0].position.y, poses[0].position.z);
  ompl::base::SO3StateSpace::StateType& so3_state = se3_state->rotation();
  so3_state.x = poses[0].orientation.x;
  so3_state.y = poses[0].orientation.y;
  so3_state.z = poses[0].orientation.z;
  so3_state.w = poses[0].orientation.w;

  return true;
}

bool ompl_interface::PoseModelStateSpace::PoseComponent::computeStateIK(StateType* full_state, unsigned int idx) const
{
  // read the values from the joint state, in the order expected by the kinematics solver; use these as the seed
  std::vector<double> seed_values(bijection_.size());
  for (std::size_t i = 0; i < bijection_.size(); ++i)
    seed_values[i] = full_state->values[bijection_[i]];

  /*
  std::cout << "seed: ";
  for (std::size_t i = 0 ; i < seed_values.size() ; ++i)
    std::cout << seed_values[i] << " ";
  std::cout << std::endl;
  */

  // construct the pose
  geometry_msgs::Pose pose;
  const ompl::base::SE3StateSpace::StateType* se3_state = full_state->poses[idx];
  pose.position.x = se3_state->getX();
  pose.position.y = se3_state->getY();
  pose.position.z = se3_state->getZ();
  const ompl::base::SO3StateSpace::StateType& so3_state = se3_state->rotation();
  pose.orientation.x = so3_state.x;
  pose.orientation.y = so3_state.y;
  pose.orientation.z = so3_state.z;
  pose.orientation.w = so3_state.w;

  // run IK
  std::vector<double> solution(bijection_.size());
  moveit_msgs::MoveItErrorCodes err_code;
  if (!kinematics_solver_->getPositionIK(pose, seed_values, solution, err_code))
  {
    if (err_code.val != moveit_msgs::MoveItErrorCodes::TIMED_OUT ||
        !kinematics_solver_->searchPositionIK(pose, seed_values, kinematics_solver_->getDefaultTimeout() * 2.0,
                                              solution, err_code))
      return false;
  }

  for (std::size_t i = 0; i < bijection_.size(); ++i)
    full_state->values[bijection_[i]] = solution[i];

  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateFK(ompl::base::State* state) const
{
  if (state->as<StateType>()->poseComputed())
    return true;
  for (std::size_t i = 0; i < poses_.size(); ++i)
    if (!poses_[i].computeStateFK(state->as<StateType>(), i))
    {
      state->as<StateType>()->markInvalid();
      return false;
    }
  state->as<StateType>()->setPoseComputed(true);
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateIK(ompl::base::State* state) const
{
  if (state->as<StateType>()->jointsComputed())
    return true;
  for (std::size_t i = 0; i < poses_.size(); ++i)
    if (!poses_[i].computeStateIK(state->as<StateType>(), i))
    {
      state->as<StateType>()->markInvalid();
      return false;
    }
  state->as<StateType>()->setJointsComputed(true);
  return true;
}

bool ompl_interface::PoseModelStateSpace::computeStateK(ompl::base::State* state) const
{
  if (state->as<StateType>()->jointsComputed() && !state->as<StateType>()->poseComputed())
    return computeStateFK(state);
  if (!state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
    return computeStateIK(state);
  if (state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
    return true;
  state->as<StateType>()->markInvalid();
  return false;
}

ompl::base::StateSamplerPtr ompl_interface::PoseModelStateSpace::allocDefaultStateSampler() const
{
  class PoseModelStateSampler : public ompl::base::StateSampler
  {
  public:
    PoseModelStateSampler(const ompl::base::StateSpace* space, ompl::base::StateSamplerPtr sampler)
      : ompl::base::StateSampler(space), sampler_(std::move(sampler))
    {
    }

    void sampleUniform(ompl::base::State* state) override
    {
      sampler_->sampleUniform(state);
      afterStateSample(state);
    }

    void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override
    {
      sampler_->sampleUniformNear(state, near, distance);
      afterStateSample(state);
    }

    void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev) override
    {
      sampler_->sampleGaussian(state, mean, stdDev);
      afterStateSample(state);
    }

  protected:
    void afterStateSample(ompl::base::State* sample) const
    {
      sample->as<StateType>()->setJointsComputed(true);
      sample->as<StateType>()->setPoseComputed(false);
      space_->as<PoseModelStateSpace>()->computeStateFK(sample);
    }

    ompl::base::StateSamplerPtr sampler_;
  };

  return ompl::base::StateSamplerPtr(static_cast<ompl::base::StateSampler*>(
      new PoseModelStateSampler(this, ModelBasedStateSpace::allocDefaultStateSampler())));
}

void ompl_interface::PoseModelStateSpace::copyToOMPLState(ompl::base::State* state,
                                                          const moveit::core::RobotState& rstate) const
{
  ModelBasedStateSpace::copyToOMPLState(state, rstate);
  state->as<StateType>()->setJointsComputed(true);
  state->as<StateType>()->setPoseComputed(false);
  computeStateFK(state);
  /*
  std::cout << "COPY STATE IN:\n";
  printState(state, std::cout);
  std::cout << "---------- COPY STATE IN\n"; */
}
