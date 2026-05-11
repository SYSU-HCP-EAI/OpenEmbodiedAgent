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

/* Author: Ioan Sucan */

#include <moveit/robot_model/robot_model.h>
#include <geometric_shapes/shape_operations.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/profiler/profiler.h>
#include <algorithm>
#include <limits>
#include <cmath>
#include <memory>
#include "order_robot_model_items.inc"

namespace moveit
{
namespace core
{
namespace
{
constexpr char LOGNAME[] = "robot_model";
}  // namespace

RobotModel::RobotModel(const urdf::ModelInterfaceSharedPtr& urdf_model, const srdf::ModelConstSharedPtr& srdf_model)
{
  root_joint_ = nullptr;
  urdf_ = urdf_model;
  srdf_ = srdf_model;
  buildModel(*urdf_model, *srdf_model);
}

RobotModel::~RobotModel()
{
  for (std::pair<const std::string, JointModelGroup*>& it : joint_model_group_map_)
    delete it.second;
  for (JointModel* joint_model : joint_model_vector_)
    delete joint_model;
  for (LinkModel* link_model : link_model_vector_)
    delete link_model;
}

const JointModel* RobotModel::getRootJoint() const
{
  return root_joint_;
}

const LinkModel* RobotModel::getRootLink() const
{
  return root_link_;
}

void RobotModel::buildModel(const urdf::ModelInterface& urdf_model, const srdf::Model& srdf_model)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModel::buildModel");

  root_joint_ = nullptr;
  root_link_ = nullptr;
  link_geometry_count_ = 0;
  variable_count_ = 0;
  model_name_ = urdf_model.getName();
  ROS_INFO_NAMED(LOGNAME, "Loading robot model '%s'...", model_name_.c_str());

  if (urdf_model.getRoot())
  {
    const urdf::Link* root_link_ptr = urdf_model.getRoot().get();
    model_frame_ = root_link_ptr->name;

    ROS_DEBUG_NAMED(LOGNAME, "... building kinematic chain");
    root_joint_ = buildRecursive(nullptr, root_link_ptr, srdf_model);
    if (root_joint_)
      root_link_ = root_joint_->getChildLinkModel();
    ROS_DEBUG_NAMED(LOGNAME, "... building mimic joints");
    buildMimic(urdf_model);

    ROS_DEBUG_NAMED(LOGNAME, "... computing joint indexing");
    buildJointInfo();

    if (link_models_with_collision_geometry_vector_.empty())
      ROS_WARN_NAMED(LOGNAME, "No geometry is associated to any robot links");

    // build groups

    ROS_DEBUG_NAMED(LOGNAME, "... constructing joint groups");
    buildGroups(srdf_model);

    ROS_DEBUG_NAMED(LOGNAME, "... constructing joint group states");
    buildGroupStates(srdf_model);

    // For debugging entire model
    // printModelInfo(std::cout);
  }
  else
    ROS_WARN_NAMED(LOGNAME, "No root link found");
}

namespace
{
typedef std::map<const JointModel*, std::pair<std::set<const LinkModel*, OrderLinksByIndex>,
                                              std::set<const JointModel*, OrderJointsByIndex>>>
    DescMap;

void computeDescendantsHelper(const JointModel* joint, std::vector<const JointModel*>& parents,
                              std::set<const JointModel*>& seen, DescMap& descendants)
{
  if (!joint)
    return;
  if (seen.find(joint) != seen.end())
    return;
  seen.insert(joint);

  for (const JointModel* parent : parents)
    descendants[parent].second.insert(joint);

  const LinkModel* lm = joint->getChildLinkModel();
  if (!lm)
    return;

  for (const JointModel* parent : parents)
    descendants[parent].first.insert(lm);
  descendants[joint].first.insert(lm);

  parents.push_back(joint);
  const std::vector<const JointModel*>& ch = lm->getChildJointModels();
  for (const JointModel* child_joint_model : ch)
    computeDescendantsHelper(child_joint_model, parents, seen, descendants);
  const std::vector<const JointModel*>& mim = joint->getMimicRequests();
  for (const JointModel* mimic_joint_model : mim)
    computeDescendantsHelper(mimic_joint_model, parents, seen, descendants);
  parents.pop_back();
}

void computeCommonRootsHelper(const JointModel* joint, std::vector<int>& common_roots, int size)
{
  if (!joint)
    return;
  const LinkModel* lm = joint->getChildLinkModel();
  if (!lm)
    return;

  const std::vector<const JointModel*>& ch = lm->getChildJointModels();
  for (std::size_t i = 0; i < ch.size(); ++i)
  {
    const std::vector<const JointModel*>& a = ch[i]->getDescendantJointModels();
    for (std::size_t j = i + 1; j < ch.size(); ++j)
    {
      const std::vector<const JointModel*>& b = ch[j]->getDescendantJointModels();
      for (const JointModel* m : b)
        common_roots[ch[i]->getJointIndex() * size + m->getJointIndex()] =
            common_roots[ch[i]->getJointIndex() + m->getJointIndex() * size] = joint->getJointIndex();
      for (const JointModel* k : a)
      {
        common_roots[k->getJointIndex() * size + ch[j]->getJointIndex()] =
            common_roots[k->getJointIndex() + ch[j]->getJointIndex() * size] = joint->getJointIndex();
        for (const JointModel* m : b)
          common_roots[k->getJointIndex() * size + m->getJointIndex()] =
              common_roots[k->getJointIndex() + m->getJointIndex() * size] = joint->getJointIndex();
      }
    }
    computeCommonRootsHelper(ch[i], common_roots, size);
  }
}
}  // namespace

void RobotModel::computeCommonRoots()
{
  // compute common roots for all pairs of joints;
  // there are 3 cases of pairs (X, Y):
  //    X != Y && X and Y are not descendants of one another
  //    X == Y
  //    X != Y && X and Y are descendants of one another

  // by default, the common root is always the global root;
  common_joint_roots_.resize(joint_model_vector_.size() * joint_model_vector_.size(), 0);

  // look at all descendants recursively; for two sibling nodes A, B, both children of X, all the pairs of respective
  // descendants of A and B
  // have X as the common root.
  computeCommonRootsHelper(root_joint_, common_joint_roots_, joint_model_vector_.size());

  for (const JointModel* joint_model : joint_model_vector_)
  {
    // the common root of a joint and itself is the same joint:
    common_joint_roots_[joint_model->getJointIndex() * (1 + joint_model_vector_.size())] = joint_model->getJointIndex();

    // a node N and one of its descendants have as common root the node N itself:
    const std::vector<const JointModel*>& d = joint_model->getDescendantJointModels();
    for (const JointModel* descendant_joint_model : d)
      common_joint_roots_[descendant_joint_model->getJointIndex() * joint_model_vector_.size() +
                          joint_model->getJointIndex()] =
          common_joint_roots_[descendant_joint_model->getJointIndex() +
                              joint_model->getJointIndex() * joint_model_vector_.size()] = joint_model->getJointIndex();
  }
}

void RobotModel::computeDescendants()
{
  // compute the list of descendants for all joints
  std::vector<const JointModel*> parents;
  std::set<const JointModel*> seen;

  DescMap descendants;
  computeDescendantsHelper(root_joint_, parents, seen, descendants);
  for (std::pair<const JointModel* const, std::pair<std::set<const LinkModel*, OrderLinksByIndex>,
                                                    std::set<const JointModel*, OrderJointsByIndex>>>& descendant :
       descendants)
  {
    JointModel* jm = const_cast<JointModel*>(descendant.first);
    for (const JointModel* jt : descendant.second.second)
      jm->addDescendantJointModel(jt);
    for (const LinkModel* jt : descendant.second.first)
      jm->addDescendantLinkModel(jt);
  }
}

void RobotModel::buildJointInfo()
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModel::buildJointInfo");

  // construct additional maps for easy access by name
  variable_count_ = 0;
  active_joint_model_start_index_.reserve(joint_model_vector_.size());
  variable_names_.reserve(joint_model_vector_.size());
  joints_of_variable_.reserve(joint_model_vector_.size());

  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
  {
    joint_model_vector_[i]->setJointIndex(i);
    const std::vector<std::string>& name_order = joint_model_vector_[i]->getVariableNames();

    // compute index map
    if (!name_order.empty())
    {
      for (std::size_t j = 0; j < name_order.size(); ++j)
      {
        joint_variables_index_map_[name_order[j]] = variable_count_ + j;
        variable_names_.push_back(name_order[j]);
        joints_of_variable_.push_back(joint_model_vector_[i]);
      }
      if (joint_model_vector_[i]->getMimic() == nullptr)
      {
        active_joint_model_start_index_.push_back(variable_count_);
        active_joint_model_vector_.push_back(joint_model_vector_[i]);
        active_joint_model_names_vector_.push_back(joint_model_vector_[i]->getName());
        active_joint_model_vector_const_.push_back(joint_model_vector_[i]);
        active_joint_models_bounds_.push_back(&joint_model_vector_[i]->getVariableBounds());
      }

      if (joint_model_vector_[i]->getType() == JointModel::REVOLUTE &&
          static_cast<const RevoluteJointModel*>(joint_model_vector_[i])->isContinuous())
        continuous_joint_model_vector_.push_back(joint_model_vector_[i]);

      joint_model_vector_[i]->setFirstVariableIndex(variable_count_);
      joint_variables_index_map_[joint_model_vector_[i]->getName()] = variable_count_;

      // compute variable count
      std::size_t vc = joint_model_vector_[i]->getVariableCount();
      variable_count_ += vc;
      if (vc == 1)
        single_dof_joints_.push_back(joint_model_vector_[i]);
      else
        multi_dof_joints_.push_back(joint_model_vector_[i]);
    }
  }

  std::vector<bool> link_considered(link_model_vector_.size(), false);
  for (const LinkModel* link : link_model_vector_)
  {
    if (link_considered[link->getLinkIndex()])
      continue;

    LinkTransformMap associated_transforms;
    computeFixedTransforms(link, link->getJointOriginTransform().inverse(), associated_transforms);
    for (auto& tf_base : associated_transforms)
    {
      link_considered[tf_base.first->getLinkIndex()] = true;
      for (auto& tf_target : associated_transforms)
      {
        if (&tf_base != &tf_target)
          const_cast<LinkModel*>(tf_base.first)  // regain write access to base LinkModel*
              ->addAssociatedFixedTransform(tf_target.first, tf_base.second.inverse() * tf_target.second);
      }
    }
  }

  computeDescendants();
  computeCommonRoots();  // must be called _after_ list of descendants was computed
}

void RobotModel::buildGroupStates(const srdf::Model& srdf_model)
{
  // copy the default states to the groups
  const std::vector<srdf::Model::GroupState>& ds = srdf_model.getGroupStates();
  for (const srdf::Model::GroupState& group_state : ds)
  {
    if (hasJointModelGroup(group_state.group_))
    {
      JointModelGroup* jmg = getJointModelGroup(group_state.group_);
      std::vector<const JointModel*> remaining_joints = jmg->getActiveJointModels();
      std::map<std::string, double> state;
      for (std::map<std::string, std::vector<double>>::const_iterator jt = group_state.joint_values_.begin();
           jt != group_state.joint_values_.end(); ++jt)
      {
        if (jmg->hasJointModel(jt->first))
        {
          const JointModel* jm = jmg->getJointModel(jt->first);
          const std::vector<std::string>& vn = jm->getVariableNames();
          // Remove current joint name from remaining list.
          auto it_found = std::find(remaining_joints.begin(), remaining_joints.end(), jm);
          if (it_found != remaining_joints.end())
            remaining_joints.erase(it_found);
          if (vn.size() == jt->second.size())
            for (std::size_t j = 0; j < vn.size(); ++j)
              state[vn[j]] = jt->second[j];
          else
            ROS_ERROR_NAMED(LOGNAME,
                            "The model for joint '%s' requires %d variable values, "
                            "but only %d variable values were supplied in default state '%s' for group '%s'",
                            jt->first.c_str(), (int)vn.size(), (int)jt->second.size(), group_state.name_.c_str(),
                            jmg->getName().c_str());
        }
        else
          ROS_ERROR_NAMED(LOGNAME,
                          "Group state '%s' specifies value for joint '%s', "
                          "but that joint is not part of group '%s'",
                          group_state.name_.c_str(), jt->first.c_str(), jmg->getName().c_str());
      }
      if (!remaining_joints.empty())
      {
        std::stringstream missing;
        missing << (*remaining_joints.begin())->getName();
        for (auto j = ++remaining_joints.begin(); j != remaining_joints.end(); j++)
        {
          missing << ", " << (*j)->getName();
        }
        ROS_WARN_STREAM_NAMED(LOGNAME, "Group state '" << group_state.name_
                                                       << "' doesn't specify all group joints in group '"
                                                       << group_state.group_ << "'. " << missing.str() << " "
                                                       << (remaining_joints.size() > 1 ? "are" : "is") << " missing.");
      }
      if (!state.empty())
        jmg->addDefaultState(group_state.name_, state);
    }
    else
      ROS_ERROR_NAMED(LOGNAME, "Group state '%s' specified for group '%s', but that group does not exist",
                      group_state.name_.c_str(), group_state.group_.c_str());
  }
}

void RobotModel::buildMimic(const urdf::ModelInterface& urdf_model)
{
  // compute mimic joints
  for (JointModel* joint_model : joint_model_vector_)
  {
    const urdf::Joint* jm = urdf_model.getJoint(joint_model->getName()).get();
    if (jm)
      if (jm->mimic)
      {
        JointModelMap::const_iterator jit = joint_model_map_.find(jm->mimic->joint_name);
        if (jit != joint_model_map_.end())
        {
          if (joint_model->getVariableCount() == jit->second->getVariableCount())
            joint_model->setMimic(jit->second, jm->mimic->multiplier, jm->mimic->offset);
          else
            ROS_ERROR_NAMED(LOGNAME, "Join '%s' cannot mimic joint '%s' because they have different number of DOF",
                            joint_model->getName().c_str(), jm->mimic->joint_name.c_str());
        }
        else
          ROS_ERROR_NAMED(LOGNAME, "Joint '%s' cannot mimic unknown joint '%s'", joint_model->getName().c_str(),
                          jm->mimic->joint_name.c_str());
      }
  }

  // in case we have a joint that mimics a joint that already mimics another joint, we can simplify things:
  bool change = true;
  while (change)
  {
    change = false;
    for (JointModel* joint_model : joint_model_vector_)
      if (joint_model->getMimic())
      {
        if (joint_model->getMimic()->getMimic())
        {
          joint_model->setMimic(joint_model->getMimic()->getMimic(),
                                joint_model->getMimicFactor() * joint_model->getMimic()->getMimicFactor(),
                                joint_model->getMimicOffset() +
                                    joint_model->getMimicFactor() * joint_model->getMimic()->getMimicOffset());
          change = true;
        }
        if (joint_model == joint_model->getMimic())
        {
          ROS_ERROR_NAMED(LOGNAME, "Cycle found in joint that mimic each other. Ignoring all mimic joints.");
          for (JointModel* joint_model_recal : joint_model_vector_)
            joint_model_recal->setMimic(nullptr, 0.0, 0.0);
          change = false;
          break;
        }
      }
  }
  // build mimic requests
  for (JointModel* joint_model : joint_model_vector_)
    if (joint_model->getMimic())
    {
      const_cast<JointModel*>(joint_model->getMimic())->addMimicRequest(joint_model);
      mimic_joints_.push_back(joint_model);
    }
}

bool RobotModel::hasEndEffector(const std::string& eef) const
{
  return end_effectors_map_.find(eef) != end_effectors_map_.end();
}

const JointModelGroup* RobotModel::getEndEffector(const std::string& name) const
{
  JointModelGroupMap::const_iterator it = end_effectors_map_.find(name);
  if (it == end_effectors_map_.end())
  {
    it = joint_model_group_map_.find(name);
    if (it != joint_model_group_map_.end() && it->second->isEndEffector())
      return it->second;
    ROS_ERROR_NAMED(LOGNAME, "End-effector '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return nullptr;
  }
  return it->second;
}

JointModelGroup* RobotModel::getEndEffector(const std::string& name)
{
  JointModelGroupMap::const_iterator it = end_effectors_map_.find(name);
  if (it == end_effectors_map_.end())
  {
    it = joint_model_group_map_.find(name);
    if (it != joint_model_group_map_.end() && it->second->isEndEffector())
      return it->second;
    ROS_ERROR_NAMED(LOGNAME, "End-effector '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return nullptr;
  }
  return it->second;
}

bool RobotModel::hasJointModelGroup(const std::string& name) const
{
  return joint_model_group_map_.find(name) != joint_model_group_map_.end();
}

const JointModelGroup* RobotModel::getJointModelGroup(const std::string& name) const
{
  JointModelGroupMap::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "Group '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return nullptr;
  }
  return it->second;
}

JointModelGroup* RobotModel::getJointModelGroup(const std::string& name)
{
  JointModelGroupMap::const_iterator it = joint_model_group_map_.find(name);
  if (it == joint_model_group_map_.end())
  {
    ROS_ERROR_NAMED(LOGNAME, "Group '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
    return nullptr;
  }
  return it->second;
}

void RobotModel::buildGroups(const srdf::Model& srdf_model)
{
  const std::vector<srdf::Model::Group>& group_configs = srdf_model.getGroups();

  // the only thing tricky is dealing with subgroups
  std::vector<bool> processed(group_configs.size(), false);

  bool added = true;
  while (added)
  {
    added = false;

    // going to make passes until we can't do anything else
    for (std::size_t i = 0; i < group_configs.size(); ++i)
      if (!processed[i])
      {
        // if we haven't processed, check and see if the dependencies are met yet
        bool all_subgroups_added = true;
        for (const std::string& subgroup : group_configs[i].subgroups_)
          if (joint_model_group_map_.find(subgroup) == joint_model_group_map_.end())
          {
            all_subgroups_added = false;
            break;
          }
        if (all_subgroups_added)
        {
          added = true;
          processed[i] = true;
          if (!addJointModelGroup(group_configs[i]))
            ROS_WARN_NAMED(LOGNAME, "Failed to add group '%s'", group_configs[i].name_.c_str());
        }
      }
  }

  for (std::size_t i = 0; i < processed.size(); ++i)
    if (!processed[i])
      ROS_WARN_NAMED(LOGNAME, "Could not process group '%s' due to unmet subgroup dependencies",
                     group_configs[i].name_.c_str());

  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin(); it != joint_model_group_map_.end(); ++it)
    joint_model_groups_.push_back(it->second);
  std::sort(joint_model_groups_.begin(), joint_model_groups_.end(), OrderGroupsByName());
  for (JointModelGroup* joint_model_group : joint_model_groups_)
  {
    joint_model_groups_const_.push_back(joint_model_group);
    joint_model_group_names_.push_back(joint_model_group->getName());
  }

  buildGroupsInfoSubgroups();
  buildGroupsInfoEndEffectors(srdf_model);
}

void RobotModel::buildGroupsInfoSubgroups()
{
  // compute subgroups
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin(); it != joint_model_group_map_.end(); ++it)
  {
    JointModelGroup* jmg = it->second;
    std::vector<std::string> subgroup_names;
    std::set<const JointModel*> joints(jmg->getJointModels().begin(), jmg->getJointModels().end());
    for (JointModelGroupMap::const_iterator jt = joint_model_group_map_.begin(); jt != joint_model_group_map_.end();
         ++jt)
      if (jt->first != it->first)
      {
        bool ok = true;
        JointModelGroup* sub_jmg = jt->second;
        const std::vector<const JointModel*>& sub_joints = sub_jmg->getJointModels();
        for (const JointModel* sub_joint : sub_joints)
          if (joints.find(sub_joint) == joints.end())
          {
            ok = false;
            break;
          }
        if (ok)
          subgroup_names.push_back(sub_jmg->getName());
      }
    if (!subgroup_names.empty())
      jmg->setSubgroupNames(subgroup_names);
  }
}

void RobotModel::buildGroupsInfoEndEffectors(const srdf::Model& srdf_model)
{
  // set the end-effector flags
  const std::vector<srdf::Model::EndEffector>& eefs = srdf_model.getEndEffectors();
  for (JointModelGroupMap::const_iterator it = joint_model_group_map_.begin(); it != joint_model_group_map_.end(); ++it)
  {
    // check if this group is a known end effector
    for (const srdf::Model::EndEffector& eef : eefs)
      if (eef.component_group_ == it->first)
      {
        // if it is, mark it as such
        it->second->setEndEffectorName(eef.name_);
        end_effectors_map_[eef.name_] = it->second;
        end_effectors_.push_back(it->second);

        // check to see if there are groups that contain the parent link of this end effector.
        // record this information if found;
        std::vector<JointModelGroup*> possible_parent_groups;
        for (JointModelGroupMap::const_iterator jt = joint_model_group_map_.begin(); jt != joint_model_group_map_.end();
             ++jt)
          if (jt->first != it->first)
          {
            if (jt->second->hasLinkModel(eef.parent_link_))
            {
              jt->second->attachEndEffector(eef.name_);
              possible_parent_groups.push_back(jt->second);
            }
          }

        JointModelGroup* eef_parent_group = nullptr;
        // if a parent group is specified in SRDF, try to use it
        if (!eef.parent_group_.empty())
        {
          JointModelGroupMap::const_iterator jt = joint_model_group_map_.find(eef.parent_group_);
          if (jt != joint_model_group_map_.end())
          {
            if (jt->second->hasLinkModel(eef.parent_link_))
            {
              if (jt->second != it->second)
                eef_parent_group = jt->second;
              else
                ROS_ERROR_NAMED(LOGNAME, "Group '%s' for end-effector '%s' cannot be its own parent",
                                eef.parent_group_.c_str(), eef.name_.c_str());
            }
            else
              ROS_ERROR_NAMED(LOGNAME,
                              "Group '%s' was specified as parent group for end-effector '%s' "
                              "but it does not include the parent link '%s'",
                              eef.parent_group_.c_str(), eef.name_.c_str(), eef.parent_link_.c_str());
          }
          else
            ROS_ERROR_NAMED(LOGNAME, "Group name '%s' not found (specified as parent group for end-effector '%s')",
                            eef.parent_group_.c_str(), eef.name_.c_str());
        }

        // if no parent group was specified, use a default one
        if (eef_parent_group == nullptr)
          if (!possible_parent_groups.empty())
          {
            // if there are multiple options for the group that contains this end-effector,
            // we pick the group with fewest joints.
            std::size_t best = 0;
            for (std::size_t g = 1; g < possible_parent_groups.size(); ++g)
              if (possible_parent_groups[g]->getJointModels().size() <
                  possible_parent_groups[best]->getJointModels().size())
                best = g;
            eef_parent_group = possible_parent_groups[best];
          }

        if (eef_parent_group)
        {
          it->second->setEndEffectorParent(eef_parent_group->getName(), eef.parent_link_);
        }
        else
        {
          ROS_WARN_NAMED(LOGNAME, "Could not identify parent group for end-effector '%s'", eef.name_.c_str());
          it->second->setEndEffectorParent("", eef.parent_link_);
        }
      }
  }
  std::sort(end_effectors_.begin(), end_effectors_.end(), OrderGroupsByName());
}

bool RobotModel::addJointModelGroup(const srdf::Model::Group& gc)
{
  if (joint_model_group_map_.find(gc.name_) != joint_model_group_map_.end())
  {
    ROS_WARN_NAMED(LOGNAME, "A group named '%s' already exists. Not adding.", gc.name_.c_str());
    return false;
  }

  std::set<const JointModel*> jset;

  // add joints from chains
  for (const std::pair<std::string, std::string>& chain : gc.chains_)
  {
    const LinkModel* base_link = getLinkModel(chain.first);
    const LinkModel* tip_link = getLinkModel(chain.second);
    if (base_link && tip_link)
    {
      // go from tip, up the chain, until we hit the root or we find the base_link
      const LinkModel* lm = tip_link;
      std::vector<const JointModel*> cj;
      while (lm)
      {
        if (lm == base_link)
          break;
        cj.push_back(lm->getParentJointModel());
        lm = lm->getParentJointModel()->getParentLinkModel();
      }
      // if we did not find the base_link, we could have a chain like e.g.,
      // from one end-effector to another end-effector, so the root is in between
      if (lm != base_link)
      {
        // we go up the chain from the base this time, and see where we intersect the other chain
        lm = base_link;
        std::size_t index = 0;
        std::vector<const JointModel*> cj2;
        while (lm)
        {
          for (std::size_t j = 0; j < cj.size(); ++j)
            if (cj[j] == lm->getParentJointModel())
            {
              index = j + 1;
              break;
            }
          if (index > 0)
            break;
          cj2.push_back(lm->getParentJointModel());
          lm = lm->getParentJointModel()->getParentLinkModel();
        }
        if (index > 0)
        {
          jset.insert(cj.begin(), cj.begin() + index);
          jset.insert(cj2.begin(), cj2.end());
        }
      }
      else
        // if we have a simple chain, just add the joints
        jset.insert(cj.begin(), cj.end());
    }
  }

  // add joints
  for (const std::string& joint : gc.joints_)
  {
    const JointModel* j = getJointModel(joint);
    if (j)
      jset.insert(j);
  }

  // add joints that are parents of included links
  for (const std::string& link : gc.links_)
  {
    const LinkModel* l = getLinkModel(link);
    if (l)
      jset.insert(l->getParentJointModel());
  }

  // add joints from subgroups
  for (const std::string& subgroup : gc.subgroups_)
  {
    const JointModelGroup* sg = getJointModelGroup(subgroup);
    if (sg)
    {
      // active joints
      const std::vector<const JointModel*>& js = sg->getJointModels();
      for (const JointModel* j : js)
        jset.insert(j);

      // fixed joints
      const std::vector<const JointModel*>& fs = sg->getFixedJointModels();
      for (const JointModel* f : fs)
        jset.insert(f);

      // mimic joints
      const std::vector<const JointModel*>& ms = sg->getMimicJointModels();
      for (const JointModel* m : ms)
        jset.insert(m);
    }
  }

  if (jset.empty())
  {
    ROS_WARN_NAMED(LOGNAME, "Group '%s' must have at least one valid joint", gc.name_.c_str());
    return false;
  }

  std::vector<const JointModel*> joints;
  joints.reserve(jset.size());
  for (const JointModel* it : jset)
    joints.push_back(it);

  JointModelGroup* jmg = new JointModelGroup(gc.name_, gc, joints, this);
  joint_model_group_map_[gc.name_] = jmg;

  return true;
}

JointModel* RobotModel::buildRecursive(LinkModel* parent, const urdf::Link* urdf_link, const srdf::Model& srdf_model)
{
  // construct the joint
  JointModel* joint = urdf_link->parent_joint ?
                          constructJointModel(urdf_link->parent_joint.get(), urdf_link, srdf_model) :
                          constructJointModel(nullptr, urdf_link, srdf_model);
  if (joint == nullptr)
    return nullptr;

  // bookkeeping for the joint
  joint_model_map_[joint->getName()] = joint;
  joint->setJointIndex(joint_model_vector_.size());
  joint_model_vector_.push_back(joint);
  joint_model_vector_const_.push_back(joint);
  joint_model_names_vector_.push_back(joint->getName());
  joint->setParentLinkModel(parent);

  // construct the link
  LinkModel* link = constructLinkModel(urdf_link);
  joint->setChildLinkModel(link);
  link->setParentLinkModel(parent);

  // bookkeeping for the link
  link_model_map_[joint->getChildLinkModel()->getName()] = link;
  link->setLinkIndex(link_model_vector_.size());
  link_model_vector_.push_back(link);
  link_model_vector_const_.push_back(link);
  link_model_names_vector_.push_back(link->getName());
  if (!link->getShapes().empty())
  {
    link_models_with_collision_geometry_vector_.push_back(link);
    link_model_names_with_collision_geometry_vector_.push_back(link->getName());
    link->setFirstCollisionBodyTransformIndex(link_geometry_count_);
    link_geometry_count_ += link->getShapes().size();
  }
  link->setParentJointModel(joint);

  // recursively build child links (and joints)
  for (const urdf::LinkSharedPtr& child_link : urdf_link->child_links)
  {
    JointModel* jm = buildRecursive(link, child_link.get(), srdf_model);
    if (jm)
      link->addChildJointModel(jm);
  }
  return joint;
}

namespace
{
// construct bounds for 1DOF joint
static inline VariableBounds jointBoundsFromURDF(const urdf::Joint* urdf_joint)
{
  VariableBounds b;
  if (urdf_joint->safety)
  {
    b.position_bounded_ = true;
    b.min_position_ = urdf_joint->safety->soft_lower_limit;
    b.max_position_ = urdf_joint->safety->soft_upper_limit;
    if (urdf_joint->limits)
    {
      if (urdf_joint->limits->lower > b.min_position_)
        b.min_position_ = urdf_joint->limits->lower;
      if (urdf_joint->limits->upper < b.max_position_)
        b.max_position_ = urdf_joint->limits->upper;
    }
  }
  else
  {
    if (urdf_joint->limits)
    {
      b.position_bounded_ = true;
      b.min_position_ = urdf_joint->limits->lower;
      b.max_position_ = urdf_joint->limits->upper;
    }
  }
  if (urdf_joint->limits)
  {
    b.max_velocity_ = fabs(urdf_joint->limits->velocity);
    b.min_velocity_ = -b.max_velocity_;
    b.velocity_bounded_ = b.max_velocity_ > std::numeric_limits<double>::epsilon();
  }
  return b;
}
}  // namespace

JointModel* RobotModel::constructJointModel(const urdf::Joint* urdf_joint, const urdf::Link* child_link,
                                            const srdf::Model& srdf_model)
{
  JointModel* new_joint_model = nullptr;

  // if urdf_joint exists, must be the root link transform
  if (urdf_joint)
  {
    switch (urdf_joint->type)
    {
      case urdf::Joint::REVOLUTE:
      {
        RevoluteJointModel* j = new RevoluteJointModel(urdf_joint->name);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(urdf_joint));
        j->setContinuous(false);
        j->setAxis(Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        new_joint_model = j;
      }
      break;
      case urdf::Joint::CONTINUOUS:
      {
        RevoluteJointModel* j = new RevoluteJointModel(urdf_joint->name);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(urdf_joint));
        j->setContinuous(true);
        j->setAxis(Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        new_joint_model = j;
      }
      break;
      case urdf::Joint::PRISMATIC:
      {
        PrismaticJointModel* j = new PrismaticJointModel(urdf_joint->name);
        j->setVariableBounds(j->getName(), jointBoundsFromURDF(urdf_joint));
        j->setAxis(Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        new_joint_model = j;
      }
      break;
      case urdf::Joint::FLOATING:
        new_joint_model = new FloatingJointModel(urdf_joint->name);
        break;
      case urdf::Joint::PLANAR:
        new_joint_model = new PlanarJointModel(urdf_joint->name);
        break;
      case urdf::Joint::FIXED:
        new_joint_model = new FixedJointModel(urdf_joint->name);
        break;
      default:
        ROS_ERROR_NAMED(LOGNAME, "Unknown joint type: %d", (int)urdf_joint->type);
        break;
    }
  }
  else  // if urdf_joint passed in as null, then we're at root of URDF model
  {
    const std::vector<srdf::Model::VirtualJoint>& virtual_joints = srdf_model.getVirtualJoints();
    for (const srdf::Model::VirtualJoint& virtual_joint : virtual_joints)
    {
      if (virtual_joint.child_link_ != child_link->name)
      {
        if (child_link->name == "world" && virtual_joint.type_ == "fixed" && child_link->collision_array.empty() &&
            !child_link->collision && child_link->visual_array.empty() && !child_link->visual)
          // Gazebo requires a fixed link from a dummy world link to the first robot's link
          // Skip warning in this case and create a fixed joint with given name
          new_joint_model = new FixedJointModel(virtual_joint.name_);
        else
          ROS_WARN_NAMED(LOGNAME,
                         "Skipping virtual joint '%s' because its child frame '%s' "
                         "does not match the URDF frame '%s'",
                         virtual_joint.name_.c_str(), virtual_joint.child_link_.c_str(), child_link->name.c_str());
      }
      else if (virtual_joint.parent_frame_.empty())
      {
        ROS_WARN_NAMED(LOGNAME, "Skipping virtual joint '%s' because its parent frame is empty",
                       virtual_joint.name_.c_str());
      }
      else
      {
        if (virtual_joint.type_ == "fixed")
          new_joint_model = new FixedJointModel(virtual_joint.name_);
        else if (virtual_joint.type_ == "planar")
          new_joint_model = new PlanarJointModel(virtual_joint.name_);
        else if (virtual_joint.type_ == "floating")
          new_joint_model = new FloatingJointModel(virtual_joint.name_);
        if (new_joint_model)
        {
          // for fixed frames we still use the robot root link
          if (virtual_joint.type_ != "fixed")
          {
            model_frame_ = virtual_joint.parent_frame_;
          }
          break;
        }
      }
    }
    if (!new_joint_model)
    {
      ROS_INFO_NAMED(LOGNAME, "No root/virtual joint specified in SRDF. Assuming fixed joint");
      new_joint_model = new FixedJointModel("ASSUMED_FIXED_ROOT_JOINT");
    }
  }

  if (new_joint_model)
  {
    new_joint_model->setDistanceFactor(new_joint_model->getStateSpaceDimension());
    const std::vector<srdf::Model::PassiveJoint>& pjoints = srdf_model.getPassiveJoints();
    for (const srdf::Model::PassiveJoint& pjoint : pjoints)
    {
      if (new_joint_model->getName() == pjoint.name_)
      {
        new_joint_model->setPassive(true);
        break;
      }
    }
  }

  return new_joint_model;
}

namespace
{
static inline Eigen::Isometry3d urdfPose2Isometry3d(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d af(Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) * q);
  return af;
}
}  // namespace

LinkModel* RobotModel::constructLinkModel(const urdf::Link* urdf_link)
{
  LinkModel* new_link_model = new LinkModel(urdf_link->name);

  const std::vector<urdf::CollisionSharedPtr>& col_array =
      urdf_link->collision_array.empty() ? std::vector<urdf::CollisionSharedPtr>(1, urdf_link->collision) :
                                           urdf_link->collision_array;

  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Isometry3d poses;

  for (const urdf::CollisionSharedPtr& col : col_array)
  {
    if (col && col->geometry)
    {
      shapes::ShapeConstPtr s = constructShape(col->geometry.get());
      if (s)
      {
        shapes.push_back(s);
        poses.push_back(urdfPose2Isometry3d(col->origin));
      }
    }
  }

  // Should we warn that old (melodic) behaviour has changed, not copying visual to collision geometries anymore?
  bool warn_about_missing_collision = false;
  if (shapes.empty())
  {
    const auto& vis_array = urdf_link->visual_array.empty() ? std::vector<urdf::VisualSharedPtr>{ urdf_link->visual } :
                                                              urdf_link->visual_array;
    for (const urdf::VisualSharedPtr& vis : vis_array)
    {
      if (vis && vis->geometry)
        warn_about_missing_collision = true;
    }
  }
  if (warn_about_missing_collision)
  {
    ROS_WARN_STREAM_NAMED(LOGNAME + ".empty_collision_geometry",
                          "Link " << urdf_link->name
                                  << " has visual geometry but no collision geometry. "
                                     "Collision geometry will be left empty. "
                                     "Fix your URDF file by explicitly specifying collision geometry.");
  }

  new_link_model->setGeometry(shapes, poses);

  // figure out visual mesh (try visual urdf tag first, collision tag otherwise
  if (urdf_link->visual && urdf_link->visual->geometry)
  {
    if (urdf_link->visual->geometry->type == urdf::Geometry::MESH)
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(urdf_link->visual->geometry.get());
      if (!mesh->filename.empty())
        new_link_model->setVisualMesh(mesh->filename, urdfPose2Isometry3d(urdf_link->visual->origin),
                                      Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z));
    }
  }
  else if (urdf_link->collision && urdf_link->collision->geometry)
  {
    if (urdf_link->collision->geometry->type == urdf::Geometry::MESH)
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(urdf_link->collision->geometry.get());
      if (!mesh->filename.empty())
        new_link_model->setVisualMesh(mesh->filename, urdfPose2Isometry3d(urdf_link->collision->origin),
                                      Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z));
    }
  }

  if (urdf_link->parent_joint)
    new_link_model->setJointOriginTransform(
        urdfPose2Isometry3d(urdf_link->parent_joint->parent_to_joint_origin_transform));

  return new_link_model;
}

shapes::ShapePtr RobotModel::constructShape(const urdf::Geometry* geom)
{
  moveit::tools::Profiler::ScopedBlock prof_block("RobotModel::constructShape");

  shapes::Shape* new_shape = nullptr;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      new_shape = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      new_shape = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      new_shape = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                       static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        new_shape = m;
      }
    }
    break;
    default:
      ROS_ERROR_NAMED(LOGNAME, "Unknown geometry type: %d", (int)geom->type);
      break;
  }

  return shapes::ShapePtr(new_shape);
}

bool RobotModel::hasJointModel(const std::string& name) const
{
  return joint_model_map_.find(name) != joint_model_map_.end();
}

bool RobotModel::hasLinkModel(const std::string& name) const
{
  return link_model_map_.find(name) != link_model_map_.end();
}

const JointModel* RobotModel::getJointModel(const std::string& name) const
{
  JointModelMap::const_iterator it = joint_model_map_.find(name);
  if (it != joint_model_map_.end())
    return it->second;
  ROS_ERROR_NAMED(LOGNAME, "Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return nullptr;
}

const JointModel* RobotModel::getJointModel(int index) const
{
  if (index < 0 || index >= static_cast<int>(joint_model_vector_.size()))
  {
    ROS_ERROR_NAMED(LOGNAME, "Joint index '%i' out of bounds of joints in model '%s'", index, model_name_.c_str());
    return nullptr;
  }
  assert(joint_model_vector_[index]->getJointIndex() == index);
  return joint_model_vector_[index];
}

JointModel* RobotModel::getJointModel(const std::string& name)
{
  JointModelMap::const_iterator it = joint_model_map_.find(name);
  if (it != joint_model_map_.end())
    return it->second;
  ROS_ERROR_NAMED(LOGNAME, "Joint '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return nullptr;
}

const LinkModel* RobotModel::getLinkModel(const std::string& name, bool* has_link) const
{
  return const_cast<RobotModel*>(this)->getLinkModel(name, has_link);
}

const LinkModel* RobotModel::getLinkModel(int index) const
{
  if (index < 0 || index >= static_cast<int>(link_model_vector_.size()))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link index '%i' out of bounds of links in model '%s'", index, model_name_.c_str());
    return nullptr;
  }
  assert(link_model_vector_[index]->getLinkIndex() == index);
  return link_model_vector_[index];
}

LinkModel* RobotModel::getLinkModel(const std::string& name, bool* has_link)
{
  if (has_link)
    *has_link = true;  // Start out optimistic
  LinkModelMap::const_iterator it = link_model_map_.find(name);
  if (it != link_model_map_.end())
    return it->second;

  if (has_link)
    *has_link = false;  // Report failure via argument
  else                  // Otherwise print error
    ROS_ERROR_NAMED(LOGNAME, "Link '%s' not found in model '%s'", name.c_str(), model_name_.c_str());
  return nullptr;
}

const LinkModel* RobotModel::getRigidlyConnectedParentLinkModel(const LinkModel* link)
{
  if (!link)
    return link;
  const moveit::core::LinkModel* parent_link = link->getParentLinkModel();
  const moveit::core::JointModel* joint = link->getParentJointModel();

  while (parent_link && joint->getType() == moveit::core::JointModel::FIXED)
  {
    link = parent_link;
    joint = link->getParentJointModel();
    parent_link = joint->getParentLinkModel();
  }
  return link;
}

void RobotModel::updateMimicJoints(double* values) const
{
  for (const JointModel* mimic_joint : mimic_joints_)
  {
    int src = mimic_joint->getMimic()->getFirstVariableIndex();
    int dest = mimic_joint->getFirstVariableIndex();
    values[dest] = values[src] * mimic_joint->getMimicFactor() + mimic_joint->getMimicOffset();
  }
}

void RobotModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values) const
{
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->getVariableRandomPositions(rng, values + active_joint_model_start_index_[i]);
  updateMimicJoints(values);
}

void RobotModel::getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng,
                                            std::map<std::string, double>& values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableRandomPositions(rng, &tmp[0]);
  values.clear();
  for (std::size_t i = 0; i < variable_names_.size(); ++i)
    values[variable_names_[i]] = tmp[i];
}

void RobotModel::getVariableDefaultPositions(double* values) const
{
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->getVariableDefaultPositions(values + active_joint_model_start_index_[i]);
  updateMimicJoints(values);
}

void RobotModel::getVariableDefaultPositions(std::map<std::string, double>& values) const
{
  std::vector<double> tmp(variable_count_);
  getVariableDefaultPositions(&tmp[0]);
  values.clear();
  for (std::size_t i = 0; i < variable_names_.size(); ++i)
    values[variable_names_[i]] = tmp[i];
}

void RobotModel::getMissingVariableNames(const std::vector<std::string>& variables,
                                         std::vector<std::string>& missing_variables) const
{
  missing_variables.clear();
  std::set<std::string> keys(variables.begin(), variables.end());
  for (const std::string& variable_name : variable_names_)
    if (keys.find(variable_name) == keys.end())
      if (getJointOfVariable(variable_name)->getMimic() == nullptr)
        missing_variables.push_back(variable_name);
}

int RobotModel::getVariableIndex(const std::string& variable) const
{
  VariableIndexMap::const_iterator it = joint_variables_index_map_.find(variable);
  if (it == joint_variables_index_map_.end())
    throw Exception("Variable '" + variable + "' is not known to model '" + model_name_ + "'");
  return it->second;
}

double RobotModel::getMaximumExtent(const JointBoundsVector& active_joint_bounds) const
{
  double max_distance = 0.0;
  for (std::size_t j = 0; j < active_joint_model_vector_.size(); ++j)
    max_distance += active_joint_model_vector_[j]->getMaximumExtent(*active_joint_bounds[j]) *
                    active_joint_model_vector_[j]->getDistanceFactor();
  return max_distance;
}

bool RobotModel::satisfiesPositionBounds(const double* state, const JointBoundsVector& active_joint_bounds,
                                         double margin) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    if (!active_joint_model_vector_[i]->satisfiesPositionBounds(state + active_joint_model_start_index_[i],
                                                                *active_joint_bounds[i], margin))
      return false;
  return true;
}

bool RobotModel::enforcePositionBounds(double* state, const JointBoundsVector& active_joint_bounds) const
{
  assert(active_joint_bounds.size() == active_joint_model_vector_.size());
  bool change = false;
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    if (active_joint_model_vector_[i]->enforcePositionBounds(state + active_joint_model_start_index_[i],
                                                             *active_joint_bounds[i]))
      change = true;
  if (change)
    updateMimicJoints(state);
  return change;
}

double RobotModel::distance(const double* state1, const double* state2) const
{
  double d = 0.0;
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    d += active_joint_model_vector_[i]->getDistanceFactor() *
         active_joint_model_vector_[i]->distance(state1 + active_joint_model_start_index_[i],
                                                 state2 + active_joint_model_start_index_[i]);
  return d;
}

void RobotModel::interpolate(const double* from, const double* to, double t, double* state) const
{
  moveit::core::checkInterpolationParamBounds(LOGNAME, t);
  // we interpolate values only for active joint models (non-mimic)
  for (std::size_t i = 0; i < active_joint_model_vector_.size(); ++i)
    active_joint_model_vector_[i]->interpolate(from + active_joint_model_start_index_[i],
                                               to + active_joint_model_start_index_[i], t,
                                               state + active_joint_model_start_index_[i]);
  // now we update mimic as needed
  updateMimicJoints(state);
}

void RobotModel::setKinematicsAllocators(const std::map<std::string, SolverAllocatorFn>& allocators)
{
  // we first set all the "simple" allocators -- where a group has one IK solver
  for (JointModelGroup* jmg : joint_model_groups_)
  {
    std::map<std::string, SolverAllocatorFn>::const_iterator jt = allocators.find(jmg->getName());
    if (jt != allocators.end())
    {
      std::pair<SolverAllocatorFn, SolverAllocatorMapFn> solver_allocator_pair;
      solver_allocator_pair.first = jt->second;
      jmg->setSolverAllocators(solver_allocator_pair);
    }
  }

  // now we set compound IK solvers; we do this later because we need the index maps computed by the previous calls to
  // setSolverAllocators()
  for (JointModelGroup* jmg : joint_model_groups_)
  {
    std::pair<SolverAllocatorFn, SolverAllocatorMapFn> solver_allocator_pair;
    std::map<std::string, SolverAllocatorFn>::const_iterator jt = allocators.find(jmg->getName());
    if (jt == allocators.end())
    {
      // if an kinematics allocator is NOT available for this group, we try to see if we can use subgroups for IK
      std::set<const JointModel*> joints;
      joints.insert(jmg->getJointModels().begin(), jmg->getJointModels().end());

      std::vector<const JointModelGroup*> subs;

      // go through the groups that have IK allocators and see if they are part of jmg; collect them in subs
      for (const std::pair<const std::string, SolverAllocatorFn>& allocator : allocators)
      {
        const JointModelGroup* sub = getJointModelGroup(allocator.first);
        if (!sub)  // this should actually not happen, all groups should be well defined
        {
          subs.clear();
          break;
        }
        std::set<const JointModel*> sub_joints;
        sub_joints.insert(sub->getJointModels().begin(), sub->getJointModels().end());

        if (std::includes(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end()))
        {  // sub_joints included in joints: add sub, remove sub_joints from joints set
          std::set<const JointModel*> joint_model_set;
          std::set_difference(joints.begin(), joints.end(), sub_joints.begin(), sub_joints.end(),
                              std::inserter(joint_model_set, joint_model_set.end()));
          // TODO: instead of maintaining disjoint joint sets here,
          // should we leave that work to JMG's setSolverAllocators() / computeIKIndexBijection()?
          // There, a disjoint bijection from joints to solvers is computed anyway.
          // Underlying question: How do we resolve overlaps? Now the first considered sub group "wins"
          // But, if the overlap only involves fixed joints, we could consider all sub groups
          subs.push_back(sub);
          joints.swap(joint_model_set);
        }
      }

      // if we found subgroups, pass that information to the planning group
      if (!subs.empty())
      {
        std::stringstream ss;
        for (const JointModelGroup* sub : subs)
        {
          ss << sub->getName() << " ";
          solver_allocator_pair.second[sub] = allocators.find(sub->getName())->second;
        }
        ROS_DEBUG_NAMED(LOGNAME, "Added sub-group IK allocators for group '%s': [ %s]", jmg->getName().c_str(),
                        ss.str().c_str());
      }
      jmg->setSolverAllocators(solver_allocator_pair);
    }
  }
}

void RobotModel::printModelInfo(std::ostream& out) const
{
  out << "Model " << model_name_ << " in frame " << model_frame_ << ", using " << getVariableCount() << " variables"
      << std::endl;

  std::ios_base::fmtflags old_flags = out.flags();
  out.setf(std::ios::fixed, std::ios::floatfield);
  std::streamsize old_prec = out.precision();
  out.precision(5);
  out << "Joints: " << std::endl;
  for (JointModel* joint_model : joint_model_vector_)
  {
    out << " '" << joint_model->getName() << "' (" << joint_model->getTypeName() << ")" << std::endl;
    out << "  * Joint Index: " << joint_model->getJointIndex() << std::endl;
    const std::vector<std::string>& vn = joint_model->getVariableNames();
    out << "  * " << vn.size() << (vn.size() > 1 ? " variables:" : (vn.empty() ? " variables" : " variable:"))
        << std::endl;
    int idx = joint_model->getFirstVariableIndex();
    for (const std::string& it : vn)
    {
      out << "     * '" << it << "', index " << idx++ << " in full state";
      if (joint_model->getMimic())
        out << ", mimic '" << joint_model->getMimic()->getName() << "'";
      if (joint_model->isPassive())
        out << ", passive";
      out << std::endl;
      out << "        " << joint_model->getVariableBounds(it) << std::endl;
    }
  }
  out << std::endl;
  out.precision(old_prec);
  out.flags(old_flags);
  out << "Links: " << std::endl;
  for (LinkModel* link_model : link_model_vector_)
  {
    out << " '" << link_model->getName() << "' with " << link_model->getShapes().size() << " geoms" << std::endl;
    if (link_model->parentJointIsFixed())
      out << "   * "
          << "parent joint is fixed" << std::endl;
    if (link_model->jointOriginTransformIsIdentity())
      out << "   * "
          << "joint origin transform is identity" << std::endl;
  }

  out << "Available groups: " << std::endl;
  for (JointModelGroup* joint_model_group : joint_model_groups_)
    joint_model_group->printGroupInfo(out);
}

void RobotModel::computeFixedTransforms(const LinkModel* link, const Eigen::Isometry3d& transform,
                                        LinkTransformMap& associated_transforms)
{
  associated_transforms[link] = transform * link->getJointOriginTransform();
  for (std::size_t i = 0; i < link->getChildJointModels().size(); ++i)
    if (link->getChildJointModels()[i]->getType() == JointModel::FIXED)
      computeFixedTransforms(link->getChildJointModels()[i]->getChildLinkModel(),
                             transform * link->getJointOriginTransform(), associated_transforms);
}

}  // end of namespace core
}  // end of namespace moveit
