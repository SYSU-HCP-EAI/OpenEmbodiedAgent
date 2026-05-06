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
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void parseStart(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                moveit_warehouse::RobotStateStorage* rs)
{
  int count;
  in >> count;
  if (in.good() && !in.eof())
  {
    for (int i = 0; i < count; ++i)
    {
      std::map<std::string, double> v;
      std::string name;
      in >> name;
      if (in.good() && !in.eof())
      {
        std::string joint;
        std::string marker;
        double value;
        in >> joint;
        while (joint != "." && in.good() && !in.eof())
        {
          in >> marker;
          if (marker != "=")
            joint = ".";
          else
          {
            in >> value;
            v[joint] = value;
          }
          if (joint != ".")
            in >> joint;
        }
      }
      if (!v.empty())
      {
        moveit::core::RobotState st = psm->getPlanningScene()->getCurrentState();
        st.setVariablePositions(v);
        moveit_msgs::RobotState msg;
        moveit::core::robotStateToRobotStateMsg(st, msg);
        ROS_INFO("Parsed start state '%s'", name.c_str());
        rs->addRobotState(msg, name);
      }
    }
  }
}

void parseLinkConstraint(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                         moveit_warehouse::ConstraintsStorage* cs)
{
  Eigen::Translation3d pos(Eigen::Vector3d::Zero());
  Eigen::Quaterniond rot(Eigen::Quaterniond::Identity());

  bool have_position = false;
  bool have_orientation = false;

  std::string name;
  std::getline(in, name);

  // The link name is optional, in which case there would be a blank line
  std::string link_name;
  std::getline(in, link_name);

  std::string type;
  in >> type;

  while (type != "." && in.good() && !in.eof())
  {
    if (type == "xyz")
    {
      have_position = true;
      double x, y, z;
      in >> x >> y >> z;
      pos = Eigen::Translation3d(x, y, z);
    }
    else if (type == "rpy")
    {
      have_orientation = true;
      double r, p, y;
      in >> r >> p >> y;
      rot = Eigen::Quaterniond(Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));
    }
    else
      ROS_ERROR("Unknown link constraint element: '%s'", type.c_str());
    in >> type;
  }

  // Convert to getLine method by eating line break
  std::string end_link;
  std::getline(in, end_link);

  if (have_position && have_orientation)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose = tf2::toMsg(pos * rot);
    pose.header.frame_id = psm->getRobotModel()->getModelFrame();
    moveit_msgs::Constraints constr = kinematic_constraints::constructGoalConstraints(link_name, pose);
    constr.name = name;
    ROS_INFO("Parsed link constraint '%s'", name.c_str());
    cs->addConstraints(constr);
  }
}

void parseGoal(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
               moveit_warehouse::ConstraintsStorage* cs)
{
  int count;
  in >> count;

  // Convert to getLine method from here-on, so eat the line break.
  std::string end_link;
  std::getline(in, end_link);

  if (in.good() && !in.eof())
  {
    for (int i = 0; i < count; ++i)
    {
      std::string type;
      std::getline(in, type);

      if (in.good() && !in.eof())
      {
        if (type == "link_constraint")
          parseLinkConstraint(in, psm, cs);
        else
          ROS_ERROR("Unknown goal type: '%s'", type.c_str());
      }
    }
  }
}

void parseQueries(std::istream& in, planning_scene_monitor::PlanningSceneMonitor* psm,
                  moveit_warehouse::RobotStateStorage* rs, moveit_warehouse::ConstraintsStorage* cs)
{
  std::string scene_name;
  in >> scene_name;
  while (in.good() && !in.eof())
  {
    std::string type;
    in >> type;

    if (in.good() && !in.eof())
    {
      if (type == "start")
        parseStart(in, psm, rs);
      else if (type == "goal")
        parseGoal(in, psm, cs);
      else
        ROS_ERROR("Unknown query type: '%s'", type.c_str());
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "import_from_text_to_warehouse", ros::init_options::AnonymousName);

  boost::program_options::options_description desc;
  desc.add_options()("help", "Show help message")("queries", boost::program_options::value<std::string>(),
                                                  "Name of file containing motion planning queries.")(
      "scene", boost::program_options::value<std::string>(), "Name of file containing motion planning scene.")(
      "host", boost::program_options::value<std::string>(),
      "Host for the DB.")("port", boost::program_options::value<std::size_t>(), "Port for the DB.");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || argc == 1)  // show help if no parameters passed
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
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  if (!psm.getPlanningScene())
  {
    ROS_ERROR("Unable to initialize PlanningSceneMonitor");
    return 1;
  }

  moveit_warehouse::PlanningSceneStorage pss(conn);
  moveit_warehouse::ConstraintsStorage cs(conn);
  moveit_warehouse::RobotStateStorage rs(conn);

  if (vm.count("scene"))
  {
    std::ifstream fin(vm["scene"].as<std::string>().c_str());
    psm.getPlanningScene()->loadGeometryFromStream(fin);
    fin.close();
    moveit_msgs::PlanningScene psmsg;
    psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
    pss.addPlanningScene(psmsg);
  }

  if (vm.count("queries"))
  {
    std::ifstream fin(vm["queries"].as<std::string>().c_str());
    if (fin.good() && !fin.eof())
      parseQueries(fin, &psm, &rs, &cs);
    fin.close();
  }

  return 0;
}
