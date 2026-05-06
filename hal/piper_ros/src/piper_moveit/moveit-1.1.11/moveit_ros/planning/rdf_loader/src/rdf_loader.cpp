/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Mathias Lüdtke, Dave Coleman */

// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/profiler/profiler.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Boost
#include <boost/filesystem.hpp>

// C++
#include <fstream>
#include <streambuf>
#include <algorithm>

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif

rdf_loader::RDFLoader::RDFLoader(const std::string& robot_description)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(robot_description)");

  ros::WallTime start = ros::WallTime::now();
  ros::NodeHandle nh("~");
  std::string content;

  if (!nh.searchParam(robot_description, robot_description_) || !nh.getParam(robot_description_, content))
  {
    ROS_ERROR_NAMED("rdf_loader", "Robot model parameter not found! Did you remap '%s'?", robot_description.c_str());
    return;
  }

  std::unique_ptr<urdf::Model> urdf(new urdf::Model());
  if (!urdf->initString(content))
  {
    ROS_ERROR_NAMED("rdf_loader", "Unable to parse URDF from parameter '%s'", robot_description_.c_str());
    return;
  }
  urdf_ = std::move(urdf);

  const std::string srdf_description(robot_description_ + "_semantic");
  std::string scontent;
  if (!nh.getParam(srdf_description, scontent))
  {
    ROS_ERROR_NAMED("rdf_loader", "Robot semantic description not found. Did you forget to define or remap '%s'?",
                    srdf_description.c_str());
    return;
  }

  srdf::ModelSharedPtr srdf(new srdf::Model());
  if (!srdf->initString(*urdf_, scontent))
  {
    ROS_ERROR_NAMED("rdf_loader", "Unable to parse SRDF from parameter '%s'", srdf_description.c_str());
    return;
  }
  srdf_ = std::move(srdf);

  ROS_DEBUG_STREAM_NAMED("rdf", "Loaded robot model in " << (ros::WallTime::now() - start).toSec() << " seconds");
}

rdf_loader::RDFLoader::RDFLoader(const std::string& urdf_string, const std::string& srdf_string)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(string)");

  auto umodel = std::make_unique<urdf::Model>();
  if (umodel->initString(urdf_string))
  {
    auto smodel = std::make_shared<srdf::Model>();
    if (!smodel->initString(*umodel, srdf_string))
    {
      ROS_ERROR_NAMED("rdf_loader", "Unable to parse SRDF");
    }
    urdf_ = std::move(umodel);
    srdf_ = std::move(smodel);
  }
  else
  {
    ROS_ERROR_NAMED("rdf_loader", "Unable to parse URDF");
  }
}

bool rdf_loader::RDFLoader::isXacroFile(const std::string& path)
{
  std::string lower_path = path;
  std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);

  return lower_path.find(".xacro") != std::string::npos;
}

bool rdf_loader::RDFLoader::loadFileToString(std::string& buffer, const std::string& path)
{
  if (path.empty())
  {
    ROS_ERROR_NAMED("rdf_loader", "Path is empty");
    return false;
  }

  if (!boost::filesystem::exists(path))
  {
    ROS_ERROR_NAMED("rdf_loader", "File does not exist");
    return false;
  }

  std::ifstream stream(path.c_str());
  if (!stream.good())
  {
    ROS_ERROR_NAMED("rdf_loader", "Unable to load path");
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  stream.seekg(0, std::ios::end);
  buffer.reserve(stream.tellg());
  stream.seekg(0, std::ios::beg);
  buffer.assign((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  stream.close();

  return true;
}

bool rdf_loader::RDFLoader::loadXacroFileToString(std::string& buffer, const std::string& path,
                                                  const std::vector<std::string>& xacro_args)
{
  buffer.clear();
  if (path.empty())
  {
    ROS_ERROR_NAMED("rdf_loader", "Path is empty");
    return false;
  }

  if (!boost::filesystem::exists(path))
  {
    ROS_ERROR_NAMED("rdf_loader", "File does not exist");
    return false;
  }

  std::string cmd = "rosrun xacro xacro ";
  for (const std::string& xacro_arg : xacro_args)
    cmd += xacro_arg + " ";
  cmd += path;

  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe)
  {
    ROS_ERROR_NAMED("rdf_loader", "Unable to load path");
    return false;
  }

  char pipe_buffer[128];
  while (!feof(pipe))
  {
    if (fgets(pipe_buffer, 128, pipe) != nullptr)
      buffer += pipe_buffer;
  }
  pclose(pipe);

  return true;
}

bool rdf_loader::RDFLoader::loadXmlFileToString(std::string& buffer, const std::string& path,
                                                const std::vector<std::string>& xacro_args)
{
  if (isXacroFile(path))
  {
    return loadXacroFileToString(buffer, path, xacro_args);
  }

  return loadFileToString(buffer, path);
}

bool rdf_loader::RDFLoader::loadPkgFileToString(std::string& buffer, const std::string& package_name,
                                                const std::string& relative_path,
                                                const std::vector<std::string>& xacro_args)
{
  std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_ERROR_STREAM_NAMED("rdf_loader", "ROS was unable to find the package name '" << package_name << "'");
    return false;
  }

  boost::filesystem::path path(package_path);
  // Use boost to cross-platform combine paths
  path = path / relative_path;

  return loadXmlFileToString(buffer, path.string(), xacro_args);
}
