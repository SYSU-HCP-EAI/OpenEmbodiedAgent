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

#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <ros/ros.h>

static const std::string PLANNING_SCENE_TOPIC = "planning_scene";
static const std::string PLANNING_REQUEST_TOPIC = "motion_plan_request";
static const std::string PLANNING_RESULTS_TOPIC = "motion_plan_results";

static const std::string CONSTRAINTS_TOPIC = "constraints";

static const std::string STATES_TOPIC = "robot_states";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_warehouse_data", ros::init_options::AnonymousName);

  // time to wait in between publishing messages
  double delay = 0.001;

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("host", boost::program_options::value<std::string>(),
                                                  "Host for the "
                                                  "DB.")("port", boost::program_options::value<std::size_t>(),
                                                         "Port for the DB.")(
      "scene", boost::program_options::value<std::string>(), "Name of scene to publish.")(
      "planning_requests", "Also publish the planning requests that correspond to the scene")(
      "planning_results", "Also publish the planning results that correspond to the scene")(
      "constraint", boost::program_options::value<std::string>(), "Name of constraint to publish.")(
      "state", boost::program_options::value<std::string>(),
      "Name of the robot state to publish.")("delay", boost::program_options::value<double>()->default_value(delay),
                                             "Time to wait in between publishing messages (s)");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || (!vm.count("scene") && !vm.count("constraint") && !vm.count("state")))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  try
  {
    delay = vm["delay"].as<double>();
  }
  catch (...)
  {
    std::cout << desc << std::endl;
    return 2;
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
  ros::Publisher pub_scene, pub_req, pub_res, pub_constr, pub_state;
  ros::Duration wait_time(delay);

  // publish the scene
  if (vm.count("scene"))
  {
    pub_scene = nh.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 10);
    bool req = vm.count("planning_requests");
    bool res = vm.count("planning_results");
    if (req)
      pub_req = nh.advertise<moveit_msgs::MotionPlanRequest>(PLANNING_REQUEST_TOPIC, 100);
    if (res)
      pub_res = nh.advertise<moveit_msgs::RobotTrajectory>(PLANNING_RESULTS_TOPIC, 100);

    moveit_warehouse::PlanningSceneStorage pss(conn);
    ros::spinOnce();

    std::vector<std::string> scene_names;
    pss.getPlanningSceneNames(vm["scene"].as<std::string>(), scene_names);

    for (const std::string& scene_name : scene_names)
    {
      moveit_warehouse::PlanningSceneWithMetadata pswm;
      if (pss.getPlanningScene(pswm, scene_name))
      {
        ROS_INFO("Publishing scene '%s'",
                 pswm->lookupString(moveit_warehouse::PlanningSceneStorage::PLANNING_SCENE_ID_NAME).c_str());
        pub_scene.publish(static_cast<const moveit_msgs::PlanningScene&>(*pswm));
        ros::spinOnce();

        // publish optional data associated to the scene
        if (req || res)
        {
          std::vector<moveit_warehouse::MotionPlanRequestWithMetadata> planning_queries;
          std::vector<std::string> query_names;
          pss.getPlanningQueries(planning_queries, query_names, pswm->name);
          ROS_INFO("There are %d planning queries associated to the scene", (int)planning_queries.size());
          ros::WallDuration(0.5).sleep();
          for (std::size_t i = 0; i < planning_queries.size(); ++i)
          {
            if (req)
            {
              ROS_INFO("Publishing query '%s'", query_names[i].c_str());
              pub_req.publish(static_cast<const moveit_msgs::MotionPlanRequest&>(*planning_queries[i]));
              ros::spinOnce();
            }
            if (res)
            {
              std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
              pss.getPlanningResults(planning_results, query_names[i], pswm->name);
              for (moveit_warehouse::RobotTrajectoryWithMetadata& planning_result : planning_results)
              {
                pub_res.publish(static_cast<const moveit_msgs::RobotTrajectory&>(*planning_result));
                ros::spinOnce();
              }
            }
          }
        }
        wait_time.sleep();
      }
    }
  }

  // publish constraints
  if (vm.count("constraint"))
  {
    moveit_warehouse::ConstraintsStorage cs(conn);
    pub_constr = nh.advertise<moveit_msgs::Constraints>(CONSTRAINTS_TOPIC, 100);
    std::vector<std::string> cnames;
    cs.getKnownConstraints(vm["constraint"].as<std::string>(), cnames);

    for (const std::string& cname : cnames)
    {
      moveit_warehouse::ConstraintsWithMetadata cwm;
      if (cs.getConstraints(cwm, cname))
      {
        ROS_INFO("Publishing constraints '%s'",
                 cwm->lookupString(moveit_warehouse::ConstraintsStorage::CONSTRAINTS_ID_NAME).c_str());
        pub_constr.publish(static_cast<const moveit_msgs::Constraints&>(*cwm));
        ros::spinOnce();
        wait_time.sleep();
      }
    }
  }

  // publish constraints
  if (vm.count("state"))
  {
    moveit_warehouse::RobotStateStorage rs(conn);
    pub_state = nh.advertise<moveit_msgs::RobotState>(STATES_TOPIC, 100);
    std::vector<std::string> rnames;
    rs.getKnownRobotStates(vm["state"].as<std::string>(), rnames);

    for (const std::string& rname : rnames)
    {
      moveit_warehouse::RobotStateWithMetadata rswm;
      if (rs.getRobotState(rswm, rname))
      {
        ROS_INFO("Publishing state '%s'", rswm->lookupString(moveit_warehouse::RobotStateStorage::STATE_NAME).c_str());
        pub_state.publish(static_cast<const moveit_msgs::RobotState&>(*rswm));
        ros::spinOnce();
        wait_time.sleep();
      }
    }
  }

  ros::WallDuration(1.0).sleep();
  ROS_INFO("Done.");

  return 0;
}
