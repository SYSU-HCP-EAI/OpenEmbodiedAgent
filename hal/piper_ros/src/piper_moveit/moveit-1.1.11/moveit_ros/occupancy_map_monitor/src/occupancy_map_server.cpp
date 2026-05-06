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

/* Author: Jon Binney, Ioan Sucan */

#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <octomap_msgs/conversions.h>

static const std::string LOGNAME = "occupancy_map_server";

static void publishOctomap(ros::Publisher& octree_binary_pub, occupancy_map_monitor::OccupancyMapMonitor& server)
{
  octomap_msgs::Octomap map;

  map.header.frame_id = server.getMapFrame();
  map.header.stamp = ros::Time::now();

  server.getOcTreePtr()->lockRead();
  try
  {
    if (!octomap_msgs::binaryMapToMsgData(*server.getOcTreePtr(), map.data))
      ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "Could not generate OctoMap message");
  }
  catch (...)
  {
    ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "Exception thrown while generating OctoMap message");
  }
  server.getOcTreePtr()->unlockRead();

  octree_binary_pub.publish(map);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_map_server");
  ros::NodeHandle nh;
  ros::Publisher octree_binary_pub = nh.advertise<octomap_msgs::Octomap>("octomap_binary", 1);
  std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(5.0));
  std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*buffer, nh);
  occupancy_map_monitor::OccupancyMapMonitor server(buffer);
  server.setUpdateCallback([&octree_binary_pub, &server] { return publishOctomap(octree_binary_pub, server); });
  server.startMonitor();

  ros::spin();
  return 0;
}
