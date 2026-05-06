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

#pragma once

#include <pybind11/pybind11.h>
#include <ros/duration.h>
#include <ros/serialization.h>

/** Provide pybind11 type converters for ROS types */

namespace moveit
{
namespace python
{
PYBIND11_EXPORT pybind11::object createMessage(const std::string& ros_msg_name);
PYBIND11_EXPORT bool convertible(const pybind11::handle& h, const char* ros_msg_name);

}  // namespace python
}  // namespace moveit

namespace pybind11
{
namespace detail
{
/// Convert ros::Duration / ros::WallDuration into a float
template <typename T>
struct DurationCaster
{
  // C++ -> Python
  static handle cast(T&& src, return_value_policy /* policy */, handle /* parent */)
  {
    return PyFloat_FromDouble(src.toSec());
  }

  // Python -> C++
  bool load(handle src, bool convert)
  {
    if (hasattr(src, "to_sec"))
    {
      value = T(src.attr("to_sec")().cast<double>());
    }
    else if (convert)
    {
      value = T(src.cast<double>());
    }
    else
      return false;
    return true;
  }

  PYBIND11_TYPE_CASTER(T, _("Duration"));
};

template <>
struct type_caster<ros::Duration> : DurationCaster<ros::Duration>
{
};

template <>
struct type_caster<ros::WallDuration> : DurationCaster<ros::WallDuration>
{
};

/// Convert ROS message types (C++ <-> python)
template <typename T>
struct type_caster<T, enable_if_t<ros::message_traits::IsMessage<T>::value>>
{
  // C++ -> Python
  static handle cast(const T& src, return_value_policy /* policy */, handle /* parent */)
  {
    // serialize src into (python) buffer
    std::size_t size = ros::serialization::serializationLength(src);
    object pbuffer = reinterpret_steal<object>(PyBytes_FromStringAndSize(nullptr, size));
    ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(PyBytes_AsString(pbuffer.ptr())), size);
    ros::serialization::serialize(stream, src);
    // deserialize python type from buffer
    object msg = moveit::python::createMessage(ros::message_traits::DataType<T>::value());
    msg.attr("deserialize")(pbuffer);
    return msg.release();
  }

  // Python -> C++
  bool load(handle src, bool /*convert*/)
  {
    if (!moveit::python::convertible(src, ros::message_traits::DataType<T>::value()))
      return false;
    // serialize src into (python) buffer
    object pstream = module::import("io").attr("BytesIO")();
    src.attr("serialize")(pstream);
    object pbuffer = pstream.attr("getvalue")();
    // deserialize C++ type from buffer
    char* cbuffer = nullptr;
    Py_ssize_t size;
    PyBytes_AsStringAndSize(pbuffer.ptr(), &cbuffer, &size);
    ros::serialization::IStream cstream(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(cbuffer)), size);
    ros::serialization::deserialize(cstream, value);
    return true;
  }

  PYBIND11_TYPE_CASTER(T, _<T>());
};
}  // namespace detail
}  // namespace pybind11
