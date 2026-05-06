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

#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/constraint_samplers/union_constraint_sampler.h>
#include <sstream>

constraint_samplers::ConstraintSamplerPtr
constraint_samplers::ConstraintSamplerManager::selectSampler(const planning_scene::PlanningSceneConstPtr& scene,
                                                             const std::string& group_name,
                                                             const moveit_msgs::Constraints& constr) const
{
  for (const ConstraintSamplerAllocatorPtr& sampler : sampler_alloc_)
    if (sampler->canService(scene, group_name, constr))
      return sampler->alloc(scene, group_name, constr);

  // if no default sampler was used, try a default one
  return selectDefaultSampler(scene, group_name, constr);
}

constraint_samplers::ConstraintSamplerPtr
constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(const planning_scene::PlanningSceneConstPtr& scene,
                                                                    const std::string& group_name,
                                                                    const moveit_msgs::Constraints& constr)
{
  const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getJointModelGroup(group_name);
  if (!jmg)
    return constraint_samplers::ConstraintSamplerPtr();
  std::stringstream ss;
  ss << constr;
  ROS_DEBUG_NAMED("constraint_samplers",
                  "Attempting to construct constrained state sampler for group '%s', using constraints:\n%s.\n",
                  jmg->getName().c_str(), ss.str().c_str());

  ConstraintSamplerPtr joint_sampler;  // location to put chosen joint sampler if needed
  // if there are joint constraints, we could possibly get a sampler from those
  if (!constr.joint_constraints.empty())
  {
    ROS_DEBUG_NAMED("constraint_samplers",
                    "There are joint constraints specified. "
                    "Attempting to construct a JointConstraintSampler for group '%s'",
                    jmg->getName().c_str());

    std::map<std::string, bool> joint_coverage;
    for (const std::string& joint : jmg->getVariableNames())
      joint_coverage[joint] = false;

    // construct the constraints
    std::vector<kinematic_constraints::JointConstraint> jc;
    for (const moveit_msgs::JointConstraint& joint_constraint : constr.joint_constraints)
    {
      kinematic_constraints::JointConstraint j(scene->getRobotModel());
      if (j.configure(joint_constraint))
      {
        if (joint_coverage.find(j.getJointVariableName()) != joint_coverage.end())
        {
          joint_coverage[j.getJointVariableName()] = true;
          jc.push_back(j);
        }
      }
    }

    // check if every joint is covered (constrained) by just joint samplers
    bool full_coverage = true;
    for (const std::pair<const std::string, bool>& it : joint_coverage)
      if (!it.second)
      {
        full_coverage = false;
        break;
      }

    // if we have constrained every joint, then we just use a sampler using these constraints
    if (full_coverage)
    {
      JointConstraintSamplerPtr sampler(new JointConstraintSampler(scene, jmg->getName()));
      if (sampler->configure(jc))
      {
        ROS_DEBUG_NAMED("constraint_samplers", "Allocated a sampler satisfying joint constraints for group '%s'",
                        jmg->getName().c_str());
        return sampler;
      }
    }
    else
        // if a smaller set of joints has been specified, keep the constraint sampler around, but use it only if no IK
        // sampler has been specified.
        if (!jc.empty())
    {
      JointConstraintSamplerPtr sampler(new JointConstraintSampler(scene, jmg->getName()));
      if (sampler->configure(jc))
      {
        ROS_DEBUG_NAMED("constraint_samplers",
                        "Temporary sampler satisfying joint constraints for group '%s' allocated. "
                        "Looking for different types of constraints before returning though.",
                        jmg->getName().c_str());
        joint_sampler = sampler;
      }
    }
  }

  std::vector<ConstraintSamplerPtr> samplers;
  if (joint_sampler)  // Start making a union of constraint samplers
    samplers.push_back(joint_sampler);

  // read the ik allocators, if any
  const moveit::core::JointModelGroup::KinematicsSolver& ik_alloc = jmg->getGroupKinematics().first;
  const moveit::core::JointModelGroup::KinematicsSolverMap& ik_subgroup_alloc = jmg->getGroupKinematics().second;

  // if we have a means of computing complete states for the group using IK, then we try to see if any IK constraints
  // should be used
  if (ik_alloc)
  {
    ROS_DEBUG_NAMED("constraint_samplers",
                    "There is an IK allocator for '%s'. "
                    "Checking for corresponding position and/or orientation constraints",
                    jmg->getName().c_str());

    // keep track of which links we constrained
    std::map<std::string, IKConstraintSamplerPtr> used_l;

    // if we have position and/or orientation constraints on links that we can perform IK for,
    // we will use a sampleable goal region that employs IK to sample goals;
    // if there are multiple constraints for the same link, we keep the one with the smallest
    // volume for sampling
    for (std::size_t p = 0; p < constr.position_constraints.size(); ++p)
      for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o)
        if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
        {
          kinematic_constraints::PositionConstraintPtr pc(
              new kinematic_constraints::PositionConstraint(scene->getRobotModel()));
          kinematic_constraints::OrientationConstraintPtr oc(
              new kinematic_constraints::OrientationConstraint(scene->getRobotModel()));
          if (pc->configure(constr.position_constraints[p], scene->getTransforms()) &&
              oc->configure(constr.orientation_constraints[o], scene->getTransforms()))
          {
            IKConstraintSamplerPtr iks(new IKConstraintSampler(scene, jmg->getName()));
            if (iks->configure(IKSamplingPose(pc, oc)))
            {
              bool use = true;
              // Check if there already is a constraint on this link
              if (used_l.find(constr.position_constraints[p].link_name) != used_l.end())
                // If there is, check if the previous one has a smaller volume for sampling
                if (used_l[constr.position_constraints[p].link_name]->getSamplingVolume() < iks->getSamplingVolume())
                  use = false;  // only use new constraint if it has a smaller sampling volume
              if (use)
              {
                // assign the link to a new constraint sampler
                used_l[constr.position_constraints[p].link_name] = iks;
                ROS_DEBUG_NAMED("constraint_samplers",
                                "Allocated an IK-based sampler for group '%s' "
                                "satisfying position and orientation constraints on link '%s'",
                                jmg->getName().c_str(), constr.position_constraints[p].link_name.c_str());
              }
            }
          }
        }

    // keep track of links constrained with a full pose
    std::map<std::string, IKConstraintSamplerPtr> used_l_full_pose = used_l;

    for (const moveit_msgs::PositionConstraint& position_constraint : constr.position_constraints)
    {
      // if we are constraining this link with a full pose, we do not attempt to constrain it with a position constraint
      // only
      if (used_l_full_pose.find(position_constraint.link_name) != used_l_full_pose.end())
        continue;

      kinematic_constraints::PositionConstraintPtr pc(
          new kinematic_constraints::PositionConstraint(scene->getRobotModel()));
      if (pc->configure(position_constraint, scene->getTransforms()))
      {
        IKConstraintSamplerPtr iks(new IKConstraintSampler(scene, jmg->getName()));
        if (iks->configure(IKSamplingPose(pc)))
        {
          bool use = true;
          if (used_l.find(position_constraint.link_name) != used_l.end())
            if (used_l[position_constraint.link_name]->getSamplingVolume() < iks->getSamplingVolume())
              use = false;
          if (use)
          {
            used_l[position_constraint.link_name] = iks;
            ROS_DEBUG_NAMED("constraint_samplers",
                            "Allocated an IK-based sampler for group '%s' "
                            "satisfying position constraints on link '%s'",
                            jmg->getName().c_str(), position_constraint.link_name.c_str());
          }
        }
      }
    }

    for (const moveit_msgs::OrientationConstraint& orientation_constraint : constr.orientation_constraints)
    {
      // if we are constraining this link with a full pose, we do not attempt to constrain it with an orientation
      // constraint only
      if (used_l_full_pose.find(orientation_constraint.link_name) != used_l_full_pose.end())
        continue;

      kinematic_constraints::OrientationConstraintPtr oc(
          new kinematic_constraints::OrientationConstraint(scene->getRobotModel()));
      if (oc->configure(orientation_constraint, scene->getTransforms()))
      {
        IKConstraintSamplerPtr iks(new IKConstraintSampler(scene, jmg->getName()));
        if (iks->configure(IKSamplingPose(oc)))
        {
          bool use = true;
          if (used_l.find(orientation_constraint.link_name) != used_l.end())
            if (used_l[orientation_constraint.link_name]->getSamplingVolume() < iks->getSamplingVolume())
              use = false;
          if (use)
          {
            used_l[orientation_constraint.link_name] = iks;
            ROS_DEBUG_NAMED("constraint_samplers",
                            "Allocated an IK-based sampler for group '%s' "
                            "satisfying orientation constraints on link '%s'",
                            jmg->getName().c_str(), orientation_constraint.link_name.c_str());
          }
        }
      }
    }

    if (used_l.size() == 1)
    {
      if (samplers.empty())
        return used_l.begin()->second;
      else
      {
        samplers.push_back(used_l.begin()->second);
        return ConstraintSamplerPtr(new UnionConstraintSampler(scene, jmg->getName(), samplers));
      }
    }
    else if (used_l.size() > 1)
    {
      ROS_DEBUG_NAMED("constraint_samplers",
                      "Too many IK-based samplers for group '%s'. Keeping the one with minimal sampling volume",
                      jmg->getName().c_str());
      // find the sampler with the smallest sampling volume; delete the rest
      IKConstraintSamplerPtr iks = used_l.begin()->second;
      double msv = iks->getSamplingVolume();
      for (std::map<std::string, IKConstraintSamplerPtr>::const_iterator it = ++used_l.begin(); it != used_l.end(); ++it)
      {
        double v = it->second->getSamplingVolume();
        if (v < msv)
        {
          iks = it->second;
          msv = v;
        }
      }
      if (samplers.empty())
      {
        return iks;
      }
      else
      {
        samplers.push_back(iks);
        return ConstraintSamplerPtr(new UnionConstraintSampler(scene, jmg->getName(), samplers));
      }
    }
  }

  // if we got to this point, we have not decided on a sampler.
  // we now check to see if we can use samplers from subgroups
  if (!ik_subgroup_alloc.empty())
  {
    ROS_DEBUG_NAMED("constraint_samplers",
                    "There are IK allocators for subgroups of group '%s'. "
                    "Checking for corresponding position and/or orientation constraints",
                    jmg->getName().c_str());

    bool some_sampler_valid = false;

    std::set<std::size_t> used_p, used_o;
    for (moveit::core::JointModelGroup::KinematicsSolverMap::const_iterator it = ik_subgroup_alloc.begin();
         it != ik_subgroup_alloc.end(); ++it)
    {
      // construct a sub-set of constraints that operate on the sub-group for which we have an IK allocator
      moveit_msgs::Constraints sub_constr;
      for (std::size_t p = 0; p < constr.position_constraints.size(); ++p)
        if (it->first->hasLinkModel(constr.position_constraints[p].link_name))
          if (used_p.find(p) == used_p.end())
          {
            sub_constr.position_constraints.push_back(constr.position_constraints[p]);
            used_p.insert(p);
          }

      for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o)
        if (it->first->hasLinkModel(constr.orientation_constraints[o].link_name))
          if (used_o.find(o) == used_o.end())
          {
            sub_constr.orientation_constraints.push_back(constr.orientation_constraints[o]);
            used_o.insert(o);
          }

      // if some matching constraints were found, construct the allocator
      if (!sub_constr.orientation_constraints.empty() || !sub_constr.position_constraints.empty())
      {
        ROS_DEBUG_NAMED("constraint_samplers", "Attempting to construct a sampler for the '%s' subgroup of '%s'",
                        it->first->getName().c_str(), jmg->getName().c_str());
        ConstraintSamplerPtr cs = selectDefaultSampler(scene, it->first->getName(), sub_constr);
        if (cs)
        {
          ROS_DEBUG_NAMED("constraint_samplers",
                          "Constructed a sampler for the joints corresponding to group '%s', "
                          "but part of group '%s'",
                          it->first->getName().c_str(), jmg->getName().c_str());
          some_sampler_valid = true;
          samplers.push_back(cs);
        }
      }
    }
    if (some_sampler_valid)
    {
      ROS_DEBUG_NAMED("constraint_samplers", "Constructing sampler for group '%s' as a union of %zu samplers",
                      jmg->getName().c_str(), samplers.size());
      return ConstraintSamplerPtr(new UnionConstraintSampler(scene, jmg->getName(), samplers));
    }
  }

  // if we've gotten here, just return joint sampler
  if (joint_sampler)
  {
    ROS_DEBUG_NAMED("constraint_samplers", "Allocated a sampler satisfying joint constraints for group '%s'",
                    jmg->getName().c_str());
    return joint_sampler;
  }

  ROS_DEBUG_NAMED("constraint_samplers", "No constraints sampler allocated for group '%s'", jmg->getName().c_str());

  return ConstraintSamplerPtr();
}
