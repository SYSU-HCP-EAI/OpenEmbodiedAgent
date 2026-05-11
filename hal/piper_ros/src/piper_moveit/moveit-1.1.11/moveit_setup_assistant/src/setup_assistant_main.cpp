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

/* Author: Dave Coleman */

#include "widgets/setup_assistant_widget.h"
#include <ros/ros.h>
#include <QApplication>
#include <QMessageBox>
#include <boost/program_options.hpp>
#include <signal.h>
#include <locale.h>

static void siginthandler(int /*param*/)
{
  QApplication::quit();
}

void usage(boost::program_options::options_description& desc, int exit_code)
{
  std::cout << desc << std::endl;
  exit(exit_code);
}

int main(int argc, char** argv)
{
  // Parse parameters
  namespace po = boost::program_options;

  // Declare the supported options
  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Show help message")("debug,g", "Run in debug/test mode")(
      "urdf_path,u", po::value<std::string>(), "Optional, path to URDF file in ROS package")(
      "config_pkg,c", po::value<std::string>(), "Optional, pass in existing config package to load");

  // Process options
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
      usage(desc, 0);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    usage(desc, 1);
  }
  // Start ROS Node
  ros::init(argc, argv, "moveit_setup_assistant", ros::init_options::NoSigintHandler);

  // ROS Spin
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  // Create Qt Application
  QApplication qt_app(argc, argv);
  // numeric values should always be POSIX
  setlocale(LC_NUMERIC, "C");

  // Load Qt Widget
  moveit_setup_assistant::SetupAssistantWidget saw(nullptr, vm);
  saw.setMinimumWidth(1090);
  saw.setMinimumHeight(600);
  //  saw.setWindowState( Qt::WindowMaximized );

  saw.show();

  signal(SIGINT, siginthandler);

  // Wait here until Qt App is finished
  const int result = qt_app.exec();

  // Shutdown ROS
  ros::shutdown();

  return result;
}
