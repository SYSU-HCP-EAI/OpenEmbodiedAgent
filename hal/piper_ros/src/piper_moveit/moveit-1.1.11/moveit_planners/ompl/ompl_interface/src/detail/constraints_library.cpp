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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/ompl_interface/detail/constraints_library.h>
#include <moveit/profiler/profiler.h>
#include <ompl/tools/config/SelfConfig.h>
#include <utility>

namespace ompl_interface
{
constexpr char LOGNAME[] = "constraints_library";

namespace
{
template <typename T>
void msgToHex(const T& msg, std::string& hex)
{
  static const char SYMBOL[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  const size_t serial_size_arg = ros::serialization::serializationLength(msg);

  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  ros::serialization::OStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::serialize(stream_arg, msg);
  hex.resize(serial_size_arg * 2);
  for (std::size_t i = 0; i < serial_size_arg; ++i)
  {
    hex[i * 2] = SYMBOL[buffer_arg[i] / 16];
    hex[i * 2 + 1] = SYMBOL[buffer_arg[i] % 16];
  }
}

template <typename T>
void hexToMsg(const std::string& hex, T& msg)
{
  const size_t serial_size_arg = hex.length() / 2;
  boost::shared_array<uint8_t> buffer_arg(new uint8_t[serial_size_arg]);
  for (std::size_t i = 0; i < serial_size_arg; ++i)
  {
    buffer_arg[i] = (hex[i * 2] <= '9' ? (hex[i * 2] - '0') : (hex[i * 2] - 'A' + 10)) * 16 +
                    (hex[i * 2 + 1] <= '9' ? (hex[i * 2 + 1] - '0') : (hex[i * 2 + 1] - 'A' + 10));
  }
  ros::serialization::IStream stream_arg(buffer_arg.get(), serial_size_arg);
  ros::serialization::deserialize(stream_arg, msg);
}
}  // namespace

class ConstraintApproximationStateSampler : public ob::StateSampler
{
public:
  ConstraintApproximationStateSampler(const ob::StateSpace* space,
                                      const ConstraintApproximationStateStorage* state_storage, std::size_t milestones)
    : ob::StateSampler(space), state_storage_(state_storage)
  {
    max_index_ = milestones - 1;
    inv_dim_ = space->getDimension() > 0 ? 1.0 / (double)space->getDimension() : 1.0;
  }

  void sampleUniform(ob::State* state) override
  {
    space_->copyState(state, state_storage_->getState(rng_.uniformInt(0, max_index_)));
  }

  void sampleUniformNear(ob::State* state, const ob::State* near, const double distance) override
  {
    int index = -1;
    int tag = near->as<ModelBasedStateSpace::StateType>()->tag;

    if (tag >= 0)
    {
      const ConstrainedStateMetadata& md = state_storage_->getMetadata(tag);
      if (!md.first.empty())
      {
        std::size_t matt = md.first.size() / 3;
        std::size_t att = 0;
        do
        {
          index = md.first[rng_.uniformInt(0, md.first.size() - 1)];
        } while (dirty_.find(index) != dirty_.end() && ++att < matt);
        if (att >= matt)
          index = -1;
        else
          dirty_.insert(index);
      }
    }
    if (index < 0)
      index = rng_.uniformInt(0, max_index_);

    double dist = space_->distance(near, state_storage_->getState(index));

    if (dist > distance)
    {
      double d = pow(rng_.uniform01(), inv_dim_) * distance;
      space_->interpolate(near, state_storage_->getState(index), d / dist, state);
    }
    else
      space_->copyState(state, state_storage_->getState(index));
  }

  void sampleGaussian(ob::State* state, const ob::State* mean, const double stdDev) override
  {
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }

protected:
  /** \brief The states to sample from */
  const ConstraintApproximationStateStorage* state_storage_;
  std::set<std::size_t> dirty_;
  unsigned int max_index_;
  double inv_dim_;
};

bool interpolateUsingStoredStates(const ConstraintApproximationStateStorage* state_storage, const ob::State* from,
                                  const ob::State* to, const double t, ob::State* state)
{
  int tag_from = from->as<ModelBasedStateSpace::StateType>()->tag;
  int tag_to = to->as<ModelBasedStateSpace::StateType>()->tag;

  if (tag_from < 0 || tag_to < 0)
    return false;

  if (tag_from == tag_to)
    state_storage->getStateSpace()->copyState(state, to);
  else
  {
    const ConstrainedStateMetadata& md = state_storage->getMetadata(tag_from);

    auto it = md.second.find(tag_to);
    if (it == md.second.end())
      return false;
    const std::pair<std::size_t, std::size_t>& istates = it->second;
    std::size_t index = (std::size_t)((istates.second - istates.first + 2) * t + 0.5);

    if (index == 0)
      state_storage->getStateSpace()->copyState(state, from);
    else
    {
      --index;
      if (index >= istates.second - istates.first)
        state_storage->getStateSpace()->copyState(state, to);
      else
        state_storage->getStateSpace()->copyState(state, state_storage->getState(istates.first + index));
    }
  }
  return true;
}

ompl_interface::InterpolationFunction ompl_interface::ConstraintApproximation::getInterpolationFunction() const
{
  if (explicit_motions_ && milestones_ > 0 && milestones_ < state_storage_->size())
    return
        [this](const ompl::base::State* from, const ompl::base::State* to, const double t, ompl::base::State* state) {
          return interpolateUsingStoredStates(state_storage_, from, to, t, state);
        };
  return InterpolationFunction();
}

ompl::base::StateSamplerPtr
allocConstraintApproximationStateSampler(const ob::StateSpace* space, const std::vector<int>& expected_signature,
                                         const ConstraintApproximationStateStorage* state_storage,
                                         std::size_t milestones)
{
  std::vector<int> sig;
  space->computeSignature(sig);
  if (sig != expected_signature)
    return ompl::base::StateSamplerPtr();
  else
    return ompl::base::StateSamplerPtr(new ConstraintApproximationStateSampler(space, state_storage, milestones));
}
}  // namespace ompl_interface

ompl_interface::ConstraintApproximation::ConstraintApproximation(
    std::string group, std::string state_space_parameterization, bool explicit_motions, moveit_msgs::Constraints msg,
    std::string filename, ompl::base::StateStoragePtr storage, std::size_t milestones)
  : group_(std::move(group))
  , state_space_parameterization_(std::move(state_space_parameterization))
  , explicit_motions_(explicit_motions)
  , constraint_msg_(std::move(msg))
  , ompldb_filename_(std::move(filename))
  , state_storage_ptr_(std::move(storage))
  , milestones_(milestones)
{
  state_storage_ = static_cast<ConstraintApproximationStateStorage*>(state_storage_ptr_.get());
  state_storage_->getStateSpace()->computeSignature(space_signature_);
  if (milestones_ == 0)
    milestones_ = state_storage_->size();
}

ompl::base::StateSamplerAllocator
ompl_interface::ConstraintApproximation::getStateSamplerAllocator(const moveit_msgs::Constraints& /*unused*/) const
{
  if (state_storage_->size() == 0)
    return ompl::base::StateSamplerAllocator();
  return [this](const ompl::base::StateSpace* ss) {
    return allocConstraintApproximationStateSampler(ss, space_signature_, state_storage_, milestones_);
  };
}
/*
void ompl_interface::ConstraintApproximation::visualizeDistribution(const
std::string &link_name, unsigned int count,
visualization_msgs::MarkerArray &arr) const
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  ompl::RNG rng;
  std_msgs::ColorRGBA color;
  color.r = 0.0f;
  color.g = 1.0f;
  color.b = 1.0f;
  color.a = 1.0f;
  if (state_storage_->size() < count)
    count = state_storage_->size();

  for (std::size_t i = 0 ; i < count ; ++i)
  {
    state_storage_->getStateSpace()->as<ModelBasedStateSpace>()->copyToRobotState(robot_state,
state_storage_->getState(rng.uniformInt(0, state_storage_->size() - 1)));
    const Eigen::Vector3d &pos =
robot_state.getLinkState(link_name)->getGlobalLinkTransform().translation();

    visualization_msgs::Marker mk;
    mk.header.stamp = ros::Time::now();
    mk.header.frame_id = robot_model_->getModelFrame();
    mk.ns = "stored_constraint_data";
    mk.id = i;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = pos.x();
    mk.pose.position.y = pos.y();
    mk.pose.position.z = pos.z();
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.035;
    mk.color = color;
    mk.lifetime = ros::Duration(30.0);
    arr.markers.push_back(mk);
  }
  }
*/

void ompl_interface::ConstraintsLibrary::loadConstraintApproximations(const std::string& path)
{
  constraint_approximations_.clear();
  std::ifstream fin((path + "/manifest").c_str());
  if (!fin.good())
  {
    ROS_WARN_NAMED(LOGNAME,
                   "Manifest not found in folder '%s'. Not loading "
                   "constraint approximations.",
                   path.c_str());
    return;
  }

  ROS_INFO_NAMED(LOGNAME, "Loading constrained space approximations from '%s'...", path.c_str());

  while (fin.good() && !fin.eof())
  {
    std::string group, state_space_parameterization, serialization, filename;
    bool explicit_motions;
    unsigned int milestones;
    fin >> group;
    if (fin.eof())
      break;
    fin >> state_space_parameterization;
    if (fin.eof())
      break;
    fin >> explicit_motions;
    if (fin.eof())
      break;
    fin >> milestones;
    if (fin.eof())
      break;
    fin >> serialization;
    if (fin.eof())
      break;
    fin >> filename;

    if (context_->getGroupName() != group &&
        context_->getOMPLStateSpace()->getParameterizationType() != state_space_parameterization)
    {
      ROS_INFO_NAMED(LOGNAME,
                     "Ignoring constraint approximation of type '%s' "
                     "for group '%s' from '%s'...",
                     state_space_parameterization.c_str(), group.c_str(), filename.c_str());
      continue;
    }

    ROS_INFO_NAMED(LOGNAME,
                   "Loading constraint approximation of type '%s' for "
                   "group '%s' from '%s'...",
                   state_space_parameterization.c_str(), group.c_str(), filename.c_str());
    moveit_msgs::Constraints msg;
    hexToMsg(serialization, msg);
    auto* cass = new ConstraintApproximationStateStorage(context_->getOMPLSimpleSetup()->getStateSpace());
    cass->load((std::string{ path }.append("/").append(filename)).c_str());
    ConstraintApproximationPtr cap(new ConstraintApproximation(group, state_space_parameterization, explicit_motions,
                                                               msg, filename, ompl::base::StateStoragePtr(cass),
                                                               milestones));
    if (constraint_approximations_.find(cap->getName()) != constraint_approximations_.end())
      ROS_WARN_NAMED(LOGNAME, "Overwriting constraint approximation named '%s'", cap->getName().c_str());
    constraint_approximations_[cap->getName()] = cap;
    std::size_t sum = 0;
    for (std::size_t i = 0; i < cass->size(); ++i)
      sum += cass->getMetadata(i).first.size();
    ROS_INFO_NAMED(LOGNAME,
                   "Loaded %lu states (%lu milestones) and %lu "
                   "connections (%0.1lf per state) "
                   "for constraint named '%s'%s",
                   cass->size(), cap->getMilestoneCount(), sum, (double)sum / (double)cap->getMilestoneCount(),
                   msg.name.c_str(), explicit_motions ? ". Explicit motions included." : "");
  }
  ROS_INFO_NAMED(LOGNAME, "Done loading constrained space approximations.");
}

void ompl_interface::ConstraintsLibrary::saveConstraintApproximations(const std::string& path)
{
  ROS_INFO_NAMED(LOGNAME, "Saving %u constrained space approximations to '%s'",
                 (unsigned int)constraint_approximations_.size(), path.c_str());
  try
  {
    boost::filesystem::create_directories(path);
  }
  catch (...)
  {
  }

  std::ofstream fout((path + "/manifest").c_str());
  if (fout.good())
    for (std::map<std::string, ConstraintApproximationPtr>::const_iterator it = constraint_approximations_.begin();
         it != constraint_approximations_.end(); ++it)
    {
      fout << it->second->getGroup() << std::endl;
      fout << it->second->getStateSpaceParameterization() << std::endl;
      fout << it->second->hasExplicitMotions() << std::endl;
      fout << it->second->getMilestoneCount() << std::endl;
      std::string serialization;
      msgToHex(it->second->getConstraintsMsg(), serialization);
      fout << serialization << std::endl;
      fout << it->second->getFilename() << std::endl;
      if (it->second->getStateStorage())
        it->second->getStateStorage()->store((path + "/" + it->second->getFilename()).c_str());
    }
  else
    ROS_ERROR_NAMED(LOGNAME, "Unable to save constraint approximation to '%s'", path.c_str());
  fout.close();
}

void ompl_interface::ConstraintsLibrary::clearConstraintApproximations()
{
  constraint_approximations_.clear();
}

void ompl_interface::ConstraintsLibrary::printConstraintApproximations(std::ostream& out) const
{
  for (const std::pair<const std::string, ConstraintApproximationPtr>& constraint_approximation :
       constraint_approximations_)
  {
    out << constraint_approximation.second->getGroup() << std::endl;
    out << constraint_approximation.second->getStateSpaceParameterization() << std::endl;
    out << constraint_approximation.second->hasExplicitMotions() << std::endl;
    out << constraint_approximation.second->getMilestoneCount() << std::endl;
    out << constraint_approximation.second->getFilename() << std::endl;
    out << constraint_approximation.second->getConstraintsMsg() << std::endl;
  }
}

const ompl_interface::ConstraintApproximationPtr&
ompl_interface::ConstraintsLibrary::getConstraintApproximation(const moveit_msgs::Constraints& msg) const
{
  auto it = constraint_approximations_.find(msg.name);
  if (it != constraint_approximations_.end())
    return it->second;

  static ConstraintApproximationPtr empty;
  return empty;
}

ompl_interface::ConstraintApproximationConstructionResults
ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints& constr,
                                                               const std::string& group,
                                                               const planning_scene::PlanningSceneConstPtr& scene,
                                                               const ConstraintApproximationConstructionOptions& options)
{
  return addConstraintApproximation(constr, constr, group, scene, options);
}

ompl_interface::ConstraintApproximationConstructionResults
ompl_interface::ConstraintsLibrary::addConstraintApproximation(const moveit_msgs::Constraints& constr_sampling,
                                                               const moveit_msgs::Constraints& constr_hard,
                                                               const std::string& group,
                                                               const planning_scene::PlanningSceneConstPtr& scene,
                                                               const ConstraintApproximationConstructionOptions& options)
{
  ConstraintApproximationConstructionResults res;
  if (context_->getGroupName() != group &&
      context_->getOMPLStateSpace()->getParameterizationType() != options.state_space_parameterization)
  {
    ROS_INFO_NAMED(LOGNAME, "Ignoring constraint approximation of type '%s' for group '%s'...",
                   options.state_space_parameterization.c_str(), group.c_str());
    return res;
  }

  context_->clear();
  context_->setPlanningScene(scene);
  context_->setCompleteInitialState(scene->getCurrentState());

  ros::WallTime start = ros::WallTime::now();
  ompl::base::StateStoragePtr state_storage =
      constructConstraintApproximation(context_, constr_sampling, constr_hard, options, res);
  ROS_INFO_NAMED(LOGNAME, "Spent %lf seconds constructing the database", (ros::WallTime::now() - start).toSec());
  if (state_storage)
  {
    ConstraintApproximationPtr constraint_approx(new ConstraintApproximation(
        group, options.state_space_parameterization, options.explicit_motions, constr_hard,
        group + "_" + boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time()) +
            ".ompldb",
        state_storage, res.milestones));
    if (constraint_approximations_.find(constraint_approx->getName()) != constraint_approximations_.end())
      ROS_WARN_NAMED(LOGNAME, "Overwriting constraint approximation named '%s'", constraint_approx->getName().c_str());
    constraint_approximations_[constraint_approx->getName()] = constraint_approx;
    res.approx = constraint_approx;
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "Unable to construct constraint approximation for group '%s'", group.c_str());
  return res;
}

ompl::base::StateStoragePtr ompl_interface::ConstraintsLibrary::constructConstraintApproximation(
    ModelBasedPlanningContext* pcontext, const moveit_msgs::Constraints& constr_sampling,
    const moveit_msgs::Constraints& constr_hard, const ConstraintApproximationConstructionOptions& options,
    ConstraintApproximationConstructionResults& result)
{
  // state storage structure
  ConstraintApproximationStateStorage* cass = new ConstraintApproximationStateStorage(pcontext->getOMPLStateSpace());
  ob::StateStoragePtr state_storage(cass);

  // construct a sampler for the sampling constraints
  kinematic_constraints::KinematicConstraintSet kset(pcontext->getRobotModel());
  moveit::core::Transforms no_transforms(pcontext->getRobotModel()->getModelFrame());
  kset.add(constr_hard, no_transforms);

  const moveit::core::RobotState& default_state = pcontext->getCompleteInitialRobotState();

  unsigned int attempts = 0;

  double bounds_val = std::numeric_limits<double>::max() / 2.0 - 1.0;
  pcontext->getOMPLStateSpace()->setPlanningVolume(-bounds_val, bounds_val, -bounds_val, bounds_val, -bounds_val,
                                                   bounds_val);
  pcontext->getOMPLStateSpace()->setup();

  // construct the constrained states

  moveit::core::RobotState robot_state(default_state);
  const constraint_samplers::ConstraintSamplerManagerPtr& csmng = pcontext->getConstraintSamplerManager();
  ConstrainedSampler* constrained_sampler = nullptr;
  if (csmng)
  {
    constraint_samplers::ConstraintSamplerPtr constraint_sampler =
        csmng->selectSampler(pcontext->getPlanningScene(), pcontext->getJointModelGroup()->getName(), constr_sampling);
    if (constraint_sampler)
      constrained_sampler = new ConstrainedSampler(pcontext, constraint_sampler);
  }

  ob::StateSamplerPtr ss(constrained_sampler ? ob::StateSamplerPtr(constrained_sampler) :
                                               pcontext->getOMPLStateSpace()->allocDefaultStateSampler());

  ompl::base::ScopedState<> temp(pcontext->getOMPLStateSpace());
  int done = -1;
  bool slow_warn = false;
  ompl::time::point start = ompl::time::now();
  while (state_storage->size() < options.samples)
  {
    ++attempts;
    int done_now = 100 * state_storage->size() / options.samples;
    if (done != done_now)
    {
      done = done_now;
      ROS_INFO_NAMED(LOGNAME, "%d%% complete (kept %0.1lf%% sampled states)", done,
                     100.0 * (double)state_storage->size() / (double)attempts);
    }

    if (!slow_warn && attempts > 10 && attempts > state_storage->size() * 100)
    {
      slow_warn = true;
      ROS_WARN_NAMED(LOGNAME, "Computation of valid state database is very slow...");
    }

    if (attempts > options.samples && state_storage->size() == 0)
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to generate any samples");
      break;
    }

    ss->sampleUniform(temp.get());
    pcontext->getOMPLStateSpace()->copyToRobotState(robot_state, temp.get());
    if (kset.decide(robot_state).satisfied)
    {
      if (state_storage->size() < options.samples)
      {
        temp->as<ModelBasedStateSpace::StateType>()->tag = state_storage->size();
        state_storage->addState(temp.get());
      }
    }
  }

  result.state_sampling_time = ompl::time::seconds(ompl::time::now() - start);
  ROS_INFO_NAMED(LOGNAME, "Generated %u states in %lf seconds", (unsigned int)state_storage->size(),
                 result.state_sampling_time);
  if (constrained_sampler)
  {
    result.sampling_success_rate = constrained_sampler->getConstrainedSamplingRate();
    ROS_INFO_NAMED(LOGNAME, "Constrained sampling rate: %lf", result.sampling_success_rate);
  }

  result.milestones = state_storage->size();
  if (options.edges_per_sample > 0)
  {
    ROS_INFO_NAMED(LOGNAME, "Computing graph connections (max %u edges per sample) ...", options.edges_per_sample);

    // construct connections
    const ob::StateSpacePtr& space = pcontext->getOMPLSimpleSetup()->getStateSpace();
    unsigned int milestones = state_storage->size();
    std::vector<ob::State*> int_states(options.max_explicit_points, nullptr);
    pcontext->getOMPLSimpleSetup()->getSpaceInformation()->allocStates(int_states);

    ompl::time::point start = ompl::time::now();
    int good = 0;
    int done = -1;

    for (std::size_t j = 0; j < milestones; ++j)
    {
      int done_now = 100 * j / milestones;
      if (done != done_now)
      {
        done = done_now;
        ROS_INFO_NAMED(LOGNAME, "%d%% complete", done);
      }
      if (cass->getMetadata(j).first.size() >= options.edges_per_sample)
        continue;

      const ob::State* sj = state_storage->getState(j);

      for (std::size_t i = j + 1; i < milestones; ++i)
      {
        if (cass->getMetadata(i).first.size() >= options.edges_per_sample)
          continue;
        double d = space->distance(state_storage->getState(i), sj);
        if (d >= options.max_edge_length)
          continue;
        unsigned int isteps =
            std::min<unsigned int>(options.max_explicit_points, d / options.explicit_points_resolution);
        double step = 1.0 / (double)isteps;
        bool ok = true;
        space->interpolate(state_storage->getState(i), sj, step, int_states[0]);
        for (unsigned int k = 1; k < isteps; ++k)
        {
          double this_step = step / (1.0 - (k - 1) * step);
          space->interpolate(int_states[k - 1], sj, this_step, int_states[k]);
          pcontext->getOMPLStateSpace()->copyToRobotState(robot_state, int_states[k]);
          if (!kset.decide(robot_state).satisfied)
          {
            ok = false;
            break;
          }
        }

        if (ok)
        {
          cass->getMetadata(i).first.push_back(j);
          cass->getMetadata(j).first.push_back(i);

          if (options.explicit_motions)
          {
            cass->getMetadata(i).second[j].first = state_storage->size();
            for (unsigned int k = 0; k < isteps; ++k)
            {
              int_states[k]->as<ModelBasedStateSpace::StateType>()->tag = -1;
              state_storage->addState(int_states[k]);
            }
            cass->getMetadata(i).second[j].second = state_storage->size();
            cass->getMetadata(j).second[i] = cass->getMetadata(i).second[j];
          }

          good++;
          if (cass->getMetadata(j).first.size() >= options.edges_per_sample)
            break;
        }
      }
    }

    result.state_connection_time = ompl::time::seconds(ompl::time::now() - start);
    ROS_INFO_NAMED(LOGNAME, "Computed possible connections in %lf seconds. Added %d connections",
                   result.state_connection_time, good);
    pcontext->getOMPLSimpleSetup()->getSpaceInformation()->freeStates(int_states);

    return state_storage;
  }

  // TODO(davetcoleman): this function did not originally return a value,
  // causing compiler warnings in ROS Melodic
  // Update with more intelligent logic as needed
  ROS_ERROR_NAMED(LOGNAME, "No StateStoragePtr found - implement better solution here.");
  return state_storage;
}
