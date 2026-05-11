/*********************************************************************
 *
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
 *
 * Author: Sachin Chitta
 *********************************************************************/

#include <kinematics_cache_ros/kinematics_cache_ros.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_cache");
  kinematics_cache::KinematicsCache::Options opt;
  opt.origin.x = 0.0;
  opt.origin.y = -1.0;
  opt.origin.z = -1.0;

  opt.workspace_size[0] = 2.0;
  opt.workspace_size[1] = 2.0;
  opt.workspace_size[2] = 2.0;

  opt.resolution[0] = 0.01;
  opt.resolution[1] = 0.01;
  opt.resolution[2] = 0.01;
  opt.max_solutions_per_grid_location = 2;

  kinematics_cache_ros::KinematicsCacheROS pr2_kinematics_cache;
  ros::NodeHandle private_handle("~");

  if (!pr2_kinematics_cache.init(opt, "pr2_arm_kinematics/PR2ArmKinematicsPlugin", "right_arm", "torso_lift_link",
                                 "r_wrist_roll_link", 0.01))
    return (0);

  pr2_kinematics_cache.generateCacheMap(10.0);

  geometry_msgs::Pose point;
  point.position.x = 0.0;
  point.position.y = -1.0;
  point.position.z = -1.0;

  unsigned int max_num = 200;

  for (unsigned int i = 0; i < max_num; ++i)
  {
    point.position.x = opt.origin.x + i / 100.0;
    for (unsigned int j = 0; j < max_num; ++j)
    {
      point.position.y = opt.origin.y + j / 100.0;
      for (unsigned int k = 0; k < max_num; ++k)
      {
        point.position.z = opt.origin.z + k / 100.0;
        unsigned int num_solutions(0);
        if (!pr2_kinematics_cache.getNumSolutions(point, num_solutions))
          ROS_ERROR("Outside grid");
        else if (num_solutions > 0)
          ROS_INFO("Num solutions: %d", num_solutions);
      }
    }
  }
  return (0);
}
