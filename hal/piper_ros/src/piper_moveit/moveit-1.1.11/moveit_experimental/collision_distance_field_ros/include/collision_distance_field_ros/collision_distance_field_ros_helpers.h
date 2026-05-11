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

/** \author E. Gil Jones */

#pragma once

#include <ros/ros.h>
#include <planning_models/robot_model.h>
#include <collision_distance_field/collision_distance_field_types.h>

namespace collision_detection
{
static inline bool loadLinkBodySphereDecompositions(
    ros::NodeHandle& nh, const planning_models::RobotModelConstPtr& robot_model,
    std::map<std::string, std::vector<collision_detection::CollisionSphere> >& link_body_spheres)
{
  if (!nh.hasParam("link_spheres"))
  {
    ROS_INFO_STREAM("No parameter for link spheres");
    return false;
  }
  XmlRpc::XmlRpcValue link_spheres;
  nh.getParam("link_spheres", link_spheres);

  if (link_spheres.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN_STREAM("Link spheres not an array");
    return false;
  }

  if (link_spheres.size() == 0)
  {
    ROS_WARN_STREAM("Link spheres has no entries");
    return false;
  }
  for (int i = 0; i < link_spheres.size(); i++)
  {
    if (!link_spheres[i].hasMember("link"))
    {
      ROS_WARN_STREAM("All link spheres must have link");
      continue;
    }
    if (!link_spheres[i].hasMember("spheres"))
    {
      ROS_WARN_STREAM("All link spheres must have spheres");
      continue;
    }
    std::string link = link_spheres[i]["link"];
    XmlRpc::XmlRpcValue spheres = link_spheres[i]["spheres"];
    if (spheres.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      if (std::string(spheres) == "none")
      {
        ROS_DEBUG_STREAM("No spheres for " << link);
        std::vector<collision_detection::CollisionSphere> coll_spheres;
        link_body_spheres[link_spheres[i]["link"]] = coll_spheres;
        continue;
      }
    }
    std::vector<collision_detection::CollisionSphere> coll_spheres;
    for (int j = 0; j < spheres.size(); j++)
    {
      if (!spheres[j].hasMember("x"))
      {
        ROS_WARN_STREAM("All spheres must specify a value for x");
        continue;
      }
      if (!spheres[j].hasMember("y"))
      {
        ROS_WARN_STREAM("All spheres must specify a value for y");
        continue;
      }
      if (!spheres[j].hasMember("z"))
      {
        ROS_WARN_STREAM("All spheres must specify a value for z");
        continue;
      }
      if (!spheres[j].hasMember("radius"))
      {
        ROS_WARN_STREAM("All spheres must specify a value for radius");
        continue;
      }
      Eigen::Vector3d rel(spheres[j]["x"], spheres[j]["y"], spheres[j]["z"]);
      ROS_DEBUG_STREAM("Link " << link_spheres[i]["link"] << " sphere " << coll_spheres.size() << " " << rel.x() << " "
                               << rel.y() << " " << rel.z());
      collision_detection::CollisionSphere cs(rel, spheres[j]["radius"]);
      coll_spheres.push_back(cs);
    }
    link_body_spheres[link_spheres[i]["link"]] = coll_spheres;
    // std::cerr << "For link " << link_spheres[i]["link"] << " adding " << coll_spheres.size() << " spheres " <<
    // std::endl;
  }
  return true;
}
}  // namespace collision_detection
