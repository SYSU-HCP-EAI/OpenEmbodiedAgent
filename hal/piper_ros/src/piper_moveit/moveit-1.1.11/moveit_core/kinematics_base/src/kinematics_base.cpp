/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>

static const std::string LOGNAME = "kinematics_base";

namespace kinematics
{
const double KinematicsBase::DEFAULT_SEARCH_DISCRETIZATION = 0.1;
const double KinematicsBase::DEFAULT_TIMEOUT = 1.0;

static void noDeleter(const moveit::core::RobotModel* /*unused*/)
{
}

void KinematicsBase::storeValues(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                 const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                 double search_discretization)
{
  // The RobotModel is passed in as a borrowed reference from the JointModelGroup belonging to this RobotModel.
  // Hence, it is ensured that the RobotModel will not be destroyed before the JMG and its associated
  // kinematics solvers. To keep RobotModelPtr API (instead of storing the reference here only), but break
  // the circular reference (RM => JMG => KS -> RM), here we create a new shared_ptr that doesn't delete the RM.
  robot_model_ = moveit::core::RobotModelConstPtr(&robot_model, &noDeleter);
  robot_description_ = "";
  group_name_ = group_name;
  base_frame_ = removeSlash(base_frame);
  tip_frames_.clear();
  for (const std::string& name : tip_frames)
    tip_frames_.push_back(removeSlash(name));
  setSearchDiscretization(search_discretization);
  search_discretization_ = search_discretization;
}

void KinematicsBase::setValues(const std::string& robot_description, const std::string& group_name,
                               const std::string& base_frame, const std::vector<std::string>& tip_frames,
                               double search_discretization)
{
  robot_model_.reset();
  robot_description_ = robot_description;
  group_name_ = group_name;
  base_frame_ = removeSlash(base_frame);
  tip_frames_.clear();
  for (const std::string& name : tip_frames)
    tip_frames_.push_back(removeSlash(name));
  setSearchDiscretization(search_discretization);

  // store deprecated values for backwards compatibility
  search_discretization_ = search_discretization;
  if (tip_frames_.size() == 1)
    tip_frame_ = tip_frames_[0];
  else
    tip_frame_.clear();
}

void KinematicsBase::setValues(const std::string& robot_description, const std::string& group_name,
                               const std::string& base_frame, const std::string& tip_frame,
                               double search_discretization)
{
  setValues(robot_description, group_name, base_frame, std::vector<std::string>({ tip_frame }), search_discretization);
}

bool KinematicsBase::initialize(const std::string& /*robot_description*/, const std::string& /*group_name*/,
                                const std::string& /*base_frame*/, const std::string& /*tip_frame*/,
                                double /*search_discretization*/)
{
  return false;  // default implementation returns false
}

bool KinematicsBase::initialize(const std::string& robot_description, const std::string& group_name,
                                const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                double search_discretization)
{
  // For IK solvers that do not support multiple tip frames, fall back to single pose call
  if (tip_frames.size() == 1)
  {
    return initialize(robot_description, group_name, base_frame, tip_frames[0], search_discretization);
  }

  ROS_ERROR_NAMED(LOGNAME, "This solver does not support multiple tip frames");
  return false;
}

bool KinematicsBase::initialize(const moveit::core::RobotModel& /*robot_model*/, const std::string& group_name,
                                const std::string& /*base_frame*/, const std::vector<std::string>& /*tip_frames*/,
                                double /*search_discretization*/)
{
  ROS_WARN_NAMED(LOGNAME,
                 "IK plugin for group '%s' relies on deprecated API. "
                 "Please implement initialize(RobotModel, ...).",
                 group_name.c_str());
  return false;
}

bool KinematicsBase::setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices)
{
  for (const unsigned int& redundant_joint_index : redundant_joint_indices)
  {
    if (redundant_joint_index >= getJointNames().size())
    {
      return false;
    }
  }
  redundant_joint_indices_ = redundant_joint_indices;
  setSearchDiscretization(DEFAULT_SEARCH_DISCRETIZATION);

  return true;
}

bool KinematicsBase::setRedundantJoints(const std::vector<std::string>& redundant_joint_names)
{
  const std::vector<std::string>& jnames = getJointNames();
  std::vector<unsigned int> redundant_joint_indices;
  for (const std::string& redundant_joint_name : redundant_joint_names)
    for (std::size_t j = 0; j < jnames.size(); ++j)
      if (jnames[j] == redundant_joint_name)
      {
        redundant_joint_indices.push_back(j);
        break;
      }
  return redundant_joint_indices.size() == redundant_joint_names.size() ? setRedundantJoints(redundant_joint_indices) :
                                                                          false;
}

std::string KinematicsBase::removeSlash(const std::string& str) const
{
  return (!str.empty() && str[0] == '/') ? removeSlash(str.substr(1)) : str;
}

bool KinematicsBase::supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out) const
{
  // Default implementation for legacy solvers:
  if (!jmg->isChain())
  {
    if (error_text_out)
    {
      *error_text_out = "This plugin only supports joint groups which are chains";
    }
    return false;
  }

  return true;
}

KinematicsBase::KinematicsBase()
  : tip_frame_("DEPRECATED")
  // help users understand why this variable might not be set
  // (if multiple tip frames provided, this variable will be unset)
  , search_discretization_(DEFAULT_SEARCH_DISCRETIZATION)
  , default_timeout_(DEFAULT_TIMEOUT)
{
  supported_methods_.push_back(DiscretizationMethods::NO_DISCRETIZATION);
}

KinematicsBase::~KinematicsBase() = default;

bool KinematicsBase::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                   const std::vector<double>& ik_seed_state,
                                   std::vector<std::vector<double> >& solutions, KinematicsResult& result,
                                   const KinematicsQueryOptions& options) const
{
  std::vector<double> solution;
  result.solution_percentage = 0.0;

  if (std::find(supported_methods_.begin(), supported_methods_.end(), options.discretization_method) ==
      supported_methods_.end())
  {
    result.kinematic_error = KinematicErrors::UNSUPORTED_DISCRETIZATION_REQUESTED;
    return false;
  }

  if (ik_poses.size() != 1)
  {
    ROS_ERROR_NAMED(LOGNAME, "This kinematic solver does not support getPositionIK for multiple tips");
    result.kinematic_error = KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
    return false;
  }

  if (ik_poses.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Input ik_poses array is empty");
    result.kinematic_error = KinematicErrors::EMPTY_TIP_POSES;
    return false;
  }

  moveit_msgs::MoveItErrorCodes error_code;
  if (getPositionIK(ik_poses[0], ik_seed_state, solution, error_code, options))
  {
    solutions.resize(1);
    solutions[0] = solution;
    result.kinematic_error = KinematicErrors::OK;
    result.solution_percentage = 1.0f;
  }
  else
  {
    result.kinematic_error = KinematicErrors::NO_SOLUTION;
    return false;
  }

  return true;
}

}  // end of namespace kinematics
