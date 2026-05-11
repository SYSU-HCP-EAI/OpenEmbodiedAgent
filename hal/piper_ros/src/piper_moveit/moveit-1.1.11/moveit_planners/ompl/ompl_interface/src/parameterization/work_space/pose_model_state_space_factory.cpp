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

#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space_factory.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>

ompl_interface::PoseModelStateSpaceFactory::PoseModelStateSpaceFactory() : ModelBasedStateSpaceFactory()
{
  type_ = PoseModelStateSpace::PARAMETERIZATION_TYPE;
}

int ompl_interface::PoseModelStateSpaceFactory::canRepresentProblem(
    const std::string& group, const moveit_msgs::MotionPlanRequest& req,
    const moveit::core::RobotModelConstPtr& robot_model) const
{
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
  if (jmg)
  {
    const std::pair<moveit::core::JointModelGroup::KinematicsSolver,
                    moveit::core::JointModelGroup::KinematicsSolverMap>& slv = jmg->getGroupKinematics();
    bool ik = false;
    // check that we have a direct means to compute IK
    if (slv.first)
      ik = jmg->getVariableCount() == slv.first.bijection_.size();
    else if (!slv.second.empty())
    {
      // or an IK solver for each of the subgroups
      unsigned int vc = 0;
      unsigned int bc = 0;
      for (const auto& jt : slv.second)
      {
        vc += jt.first->getVariableCount();
        bc += jt.second.bijection_.size();
      }
      if (vc == jmg->getVariableCount() && vc == bc)
        ik = true;
    }

    if (ik)
    {
      // if we have path constraints, we prefer interpolating in pose space
      if ((!req.path_constraints.position_constraints.empty() ||
           !req.path_constraints.orientation_constraints.empty()) &&
          req.path_constraints.joint_constraints.empty() && req.path_constraints.visibility_constraints.empty())
        return 150;
      else
        return 50;
    }
  }
  return -1;
}

ompl_interface::ModelBasedStateSpacePtr
ompl_interface::PoseModelStateSpaceFactory::allocStateSpace(const ModelBasedStateSpaceSpecification& space_spec) const
{
  return std::make_shared<PoseModelStateSpace>(space_spec);
}
