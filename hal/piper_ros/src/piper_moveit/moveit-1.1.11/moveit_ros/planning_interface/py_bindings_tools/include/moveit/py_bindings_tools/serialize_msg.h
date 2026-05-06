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

#include <ros/ros.h>
#include <Python.h>
#include <boost/python.hpp>
#include <string>
#include <stdexcept>
#include <type_traits>

namespace moveit
{
namespace py_bindings_tools
{
/** \brief C++ Wrapper class for Python 3 \c Bytes Object */
class ByteString : public boost::python::object
{
public:
  // constructors for bp::handle and friends
  BOOST_PYTHON_FORWARD_OBJECT_CONSTRUCTORS(ByteString, boost::python::object)
  ByteString() : boost::python::object(boost::python::handle<>(PyBytes_FromString("")))
  {
  }
  explicit ByteString(const char* s) : boost::python::object(boost::python::handle<>(PyBytes_FromString(s)))
  {
  }
  explicit ByteString(const std::string& s)
    : boost::python::object(boost::python::handle<>(PyBytes_FromStringAndSize(s.c_str(), s.size())))
  {
  }
  // bp::list[] returns a proxy which has to be converted to an object first
  template <typename T>
  explicit ByteString(const boost::python::api::proxy<T>& proxy) : boost::python::object(proxy)
  {
  }
  /** \brief Serializes a ROS message into a Python Bytes object
   * The second template parameter ensures that this overload is only chosen with a ROS message argument
   */
  template <typename T, typename std::enable_if<ros::message_traits::IsMessage<T>::value, int>::type = 0>
  explicit ByteString(const T& msg)
    : boost::python::object(
          boost::python::handle<>(PyBytes_FromStringAndSize(nullptr, ros::serialization::serializationLength(msg))))
  {
    ros::serialization::OStream stream_arg(reinterpret_cast<uint8_t*>(PyBytes_AS_STRING(ptr())),
                                           PyBytes_GET_SIZE(ptr()));
    ros::serialization::serialize(stream_arg, msg);
  }

  /** \brief Convert content to a ROS message */
  template <typename T>
  void deserialize(T& msg) const
  {
    static_assert(sizeof(uint8_t) == sizeof(char), "ros/python buffer layout mismatch");
    char* buf = PyBytes_AsString(ptr());
    // buf == NULL on error
    if (!buf)
    {
      throw std::runtime_error("Underlying python object is not a Bytes/String instance");
    }
    // unfortunately no constructor with const uint8_t
    ros::serialization::IStream stream_arg(reinterpret_cast<uint8_t*>(buf), PyBytes_GET_SIZE(ptr()));
    ros::serialization::deserialize(stream_arg, msg);
  }
};

/** \brief Convert a ROS message to a Python Bytestring */
template <typename T>
ByteString serializeMsg(const T& msg)
{
  return ByteString(msg);
}

/** \brief Convert a Python Bytestring to a ROS message */
template <typename T>
void deserializeMsg(const ByteString& data, T& msg)
{
  data.deserialize(msg);
}

}  // namespace py_bindings_tools
}  // namespace moveit

namespace boost
{
namespace python
{
namespace converter
{
// only accept Python 3 Bytes / Python 2 String instance when used as C++ function parameter
template <>
struct object_manager_traits<moveit::py_bindings_tools::ByteString>
#if PY_VERSION_HEX >= 0x03000000
  : pytype_object_manager_traits<&PyBytes_Type, moveit::py_bindings_tools::ByteString>
#else
  : pytype_object_manager_traits<&PyString_Type, moveit::py_bindings_tools::ByteString>
#endif

{
};
}  // namespace converter
}  // namespace python
}  // namespace boost
