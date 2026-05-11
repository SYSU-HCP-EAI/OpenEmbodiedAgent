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

#pragma once

#include <boost/python.hpp>
#include <string>

namespace moveit
{
/** \brief Tools for creating python bindings for MoveIt */
namespace py_bindings_tools
{
/** \brief The constructor of this class ensures that ros::init() has
    been called.  Thread safety and multiple initialization is
    properly handled. When the process terminates, ros::shotdown() is
    also called, if needed. */
class ROScppInitializer
{
public:
  ROScppInitializer();
  ROScppInitializer(boost::python::list& argv);
  ROScppInitializer(const std::string& node_name, boost::python::list& argv);
};

/** \brief This function can be used to specify the ROS command line arguments for the internal ROScpp instance;
    Usually this function would also be exposed in the py module that uses ROScppInitializer. */
void roscpp_set_arguments(const std::string& node_name, boost::python::list& argv);

/** \brief Initialize ROScpp with specified command line args */
void roscpp_init(const std::string& node_name, boost::python::list& argv);

/** \brief Initialize ROScpp with specified command line args */
void roscpp_init(boost::python::list& argv);

/** \brief Initialize ROScpp with default command line args */
void roscpp_init();

void roscpp_shutdown();
}  // namespace py_bindings_tools
}  // namespace moveit
