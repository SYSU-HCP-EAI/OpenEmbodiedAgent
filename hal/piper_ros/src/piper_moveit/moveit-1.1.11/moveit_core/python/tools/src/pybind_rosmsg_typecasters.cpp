/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Robert Haschke
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
 *   * The name of Robert Haschke may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Robert Haschke */

#include <moveit/python/pybind_rosmsg_typecasters.h>

namespace py = pybind11;
namespace moveit
{
namespace python
{
py::object createMessage(const std::string& ros_msg_name)
{
  // find delimiting '/' in ros msg name
  std::size_t pos = ros_msg_name.find('/');
  // import module
  py::module m = py::module::import((ros_msg_name.substr(0, pos) + ".msg").c_str());
  // retrieve type instance
  py::object cls = m.attr(ros_msg_name.substr(pos + 1).c_str());
  // create message instance
  return cls();
}

bool convertible(const pybind11::handle& h, const char* ros_msg_name)
{
  try
  {
    PyObject* o = h.attr("_type").ptr();
    return py::cast<std::string>(o) == ros_msg_name;
  }
  catch (const std::exception& e)
  {
    return false;
  }
}
}  // namespace python
}  // namespace moveit
