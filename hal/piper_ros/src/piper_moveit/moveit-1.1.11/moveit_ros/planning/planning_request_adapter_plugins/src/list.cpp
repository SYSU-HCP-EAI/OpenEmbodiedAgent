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

#include <pluginlib/class_loader.hpp>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <ros/ros.h>
#include <memory>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "list_planning_adapter_plugins");

  std::unique_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>> loader;
  try
  {
    loader = std::make_unique<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>>(
        "moveit_core", "planning_request_adapter::PlanningRequestAdapter");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::cout << "Exception while creating class loader " << ex.what() << std::endl;
  }

  const std::vector<std::string>& classes = loader->getDeclaredClasses();
  std::cout << "Available planning request adapter plugins:" << std::endl;
  for (const std::string& adapter_plugin_name : classes)
  {
    std::cout << " \t " << adapter_plugin_name << std::endl;
    planning_request_adapter::PlanningRequestAdapterConstPtr ad;
    try
    {
      ad = loader->createUniqueInstance(adapter_plugin_name);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      std::cout << " \t\t  Exception while planning adapter plugin '" << adapter_plugin_name << "': " << ex.what()
                << std::endl;
    }
    if (ad)
      std::cout << " \t\t  " << ad->getDescription() << std::endl;
    std::cout << std::endl << std::endl;
  }

  return 0;
}
