/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University
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
 *   * Neither the name of Hamburg University nor the names of its
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

/* Author: Michael Görner, Robert Haschke */

#include <moveit/utils/xmlrpc_casts.h>
#include <ros/console.h>
#include <boost/algorithm/string/join.hpp>

namespace moveit
{
namespace core
{
double parseDouble(XmlRpc::XmlRpcValue& v)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(v);
  else if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(v);
  else
    return 0.0;
}

bool isArray(XmlRpc::XmlRpcValue& v, size_t size, const std::string& name, const std::string& description)
{
  if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || (size != 0 && static_cast<size_t>(v.size()) != size))
  {
    if (!name.empty())
      ROS_WARN_STREAM(name << " is not an array[" << size << "] of "
                           << (description.empty() ? "elements" : description.c_str()));
    return false;
  }
  return true;
}

bool isStruct(XmlRpc::XmlRpcValue& v, const std::vector<std::string>& keys, const std::string& name)
{
  if (v.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    if (!name.empty())
      ROS_WARN_STREAM(name << " is not a struct with keys " << boost::join(keys, ","));
    return false;
  }

  std::vector<std::string> missing;
  for (const std::string& key : keys)
    if (!v.hasMember(key))
      missing.push_back(key);

  if (!name.empty() && !missing.empty())
  {
    ROS_WARN_STREAM(name << " is not a struct with keys " << boost::join(keys, ",") << " (misses "
                         << boost::join(missing, ",") << ")");
    return false;
  }

  return missing.empty();
}
}  // namespace core
}  // namespace moveit
