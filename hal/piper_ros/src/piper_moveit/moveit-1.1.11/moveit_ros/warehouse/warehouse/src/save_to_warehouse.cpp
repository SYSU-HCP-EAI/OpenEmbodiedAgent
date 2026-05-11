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

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void onSceneUpdate(planning_scene_monitor::PlanningSceneMonitor& psm, moveit_warehouse::PlanningSceneStorage& pss)
{
  ROS_INFO("Received an update to the planning scene...");

  if (!psm.getPlanningScene()->getName().empty())
  {
    if (!pss.hasPlanningScene(psm.getPlanningScene()->getName()))
    {
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      pss.addPlanningScene(psmsg);
    }
    else
      ROS_INFO("Scene '%s' was previously added. Not adding again.", psm.getPlanningScene()->getName().c_str());
  }
  else
    ROS_INFO("Scene name is empty. Not saving.");
}

void onMotionPlanRequest(const moveit_msgs::MotionPlanRequest& req, planning_scene_monitor::PlanningSceneMonitor& psm,
                         moveit_warehouse::PlanningSceneStorage& pss)
{
  if (psm.getPlanningScene()->getName().empty())
  {
    ROS_INFO("Scene name is empty. Not saving planning request.");
    return;
  }
  pss.addPlanningQuery(req, psm.getPlanningScene()->getName());
}

void onConstraints(const moveit_msgs::Constraints& msg, moveit_warehouse::ConstraintsStorage& cs)
{
  if (msg.name.empty())
  {
    ROS_INFO("No name specified for constraints. Not saving.");
    return;
  }

  if (cs.hasConstraints(msg.name))
  {
    ROS_INFO("Replacing constraints '%s'", msg.name.c_str());
    cs.removeConstraints(msg.name);
    cs.addConstraints(msg);
  }
  else
  {
    ROS_INFO("Adding constraints '%s'", msg.name.c_str());
    cs.addConstraints(msg);
  }
}

void onRobotState(const moveit_msgs::RobotState& msg, moveit_warehouse::RobotStateStorage& rs)
{
  std::vector<std::string> names;
  rs.getKnownRobotStates(names);
  std::set<std::string> names_set(names.begin(), names.end());
  std::size_t n = names.size();
  while (names_set.find("S" + boost::lexical_cast<std::string>(n)) != names_set.end())
    n++;
  std::string name = "S" + boost::lexical_cast<std::string>(n);
  ROS_INFO("Adding robot state '%s'", name.c_str());
  rs.addRobotState(msg, name);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_to_warehouse", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(),
                                                  "Host for the "
                                                  "DB.")("port", boost::program_options::value<std::size_t>(),
                                                         "Port for the DB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  // Set up db
  warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
  if (vm.count("host") && vm.count("port"))
    conn->setParams(vm["host"].as<std::string>(), vm["port"].as<std::size_t>());
  if (!conn->connect())
    return 1;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer, nh);
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf_buffer);
  if (!psm.getPlanningScene())
  {
    ROS_ERROR("Unable to initialize PlanningSceneMonitor");
    return 1;
  }

  psm.startSceneMonitor();
  psm.startWorldGeometryMonitor();
  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);
  moveit_warehouse::RobotStateStorage rs(conn);
  std::vector<std::string> names;
  pss.getPlanningSceneNames(names);
  if (names.empty())
    ROS_INFO("There are no previously stored scenes");
  else
  {
    ROS_INFO("Previously stored scenes:");
    for (const std::string& name : names)
      ROS_INFO(" * %s", name.c_str());
  }
  cs.getKnownConstraints(names);
  if (names.empty())
    ROS_INFO("There are no previously stored constraints");
  else
  {
    ROS_INFO("Previously stored constraints:");
    for (const std::string& name : names)
      ROS_INFO(" * %s", name.c_str());
  }
  rs.getKnownRobotStates(names);
  if (names.empty())
    ROS_INFO("There are no previously stored robot states");
  else
  {
    ROS_INFO("Previously stored robot states:");
    for (const std::string& name : names)
      ROS_INFO(" * %s", name.c_str());
  }

  psm.addUpdateCallback([&psm, &pss](auto&& /*unused*/) { return onSceneUpdate(psm, pss); });

  ros::Subscriber mplan_req_sub = nh.subscribe<moveit_msgs::MotionPlanRequest>(
      "motion_plan_request", 100, [&psm, &pss](const auto& req) { onMotionPlanRequest(*req, psm, pss); });

  ros::Subscriber constr_sub = nh.subscribe<moveit_msgs::Constraints>(
      "constraints", 100, [&cs](const auto& constraints) { return onConstraints(*constraints, cs); });

  ros::Subscriber state_sub = nh.subscribe<moveit_msgs::RobotState>("robot_state", 100, [&rs](const auto& state) {
    return onRobotState(*state, rs);
  });

  std::vector<std::string> topics;
  psm.getMonitoredTopics(topics);
  ROS_INFO_STREAM("Listening for scene updates on topics " << boost::algorithm::join(topics, ", "));
  ROS_INFO_STREAM("Listening for planning requests on topic " << mplan_req_sub.getTopic());
  ROS_INFO_STREAM("Listening for named constraints on topic " << constr_sub.getTopic());
  ROS_INFO_STREAM("Listening for states on topic " << state_sub.getTopic());

  ros::waitForShutdown();
  return 0;
}
