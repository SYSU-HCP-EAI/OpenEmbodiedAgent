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
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>
#include <map>

namespace moveit
{
namespace py_bindings_tools
{
template <typename T>
std::vector<T> typeFromList(const boost::python::object& values)
{
  boost::python::stl_input_iterator<T> begin(values), end;
  std::vector<T> v;
  v.assign(begin, end);
  return v;
}

template <typename T>
boost::python::list listFromType(const std::vector<T>& v)
{
  boost::python::list l;
  for (std::size_t i = 0; i < v.size(); ++i)
    l.append(v[i]);
  return l;
}

template <typename T>
boost::python::dict dictFromType(const std::map<std::string, T>& v)
{
  boost::python::dict d;
  for (typename std::map<std::string, T>::const_iterator it = v.begin(); it != v.end(); ++it)
    d[it->first] = it->second;
  return d;
}

std::vector<double> doubleFromList(const boost::python::object& values)
{
  return typeFromList<double>(values);
}

std::vector<std::string> stringFromList(const boost::python::object& values)
{
  return typeFromList<std::string>(values);
}

boost::python::list listFromDouble(const std::vector<double>& v)
{
  return listFromType<double>(v);
}

boost::python::list listFromString(const std::vector<std::string>& v)
{
  return listFromType<std::string>(v);
}
}  // namespace py_bindings_tools
}  // namespace moveit
