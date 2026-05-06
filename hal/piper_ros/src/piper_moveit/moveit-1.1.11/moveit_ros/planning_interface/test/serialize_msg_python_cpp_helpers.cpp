/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, RWTH Aachen University.
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
 *   * Neither the name of RWTH Aachen University nor the names of its
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

/* Author: Bjarne von Horn */

#include <Python.h>
#include <boost/python.hpp>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <geometry_msgs/Vector3.h>
#include <cstring>

namespace bp = boost::python;
using moveit::py_bindings_tools::ByteString;

// Helper class to be exposed to Python
class ByteStringTestHelper
{
  // Helper to test whether a vector of unsigned chars has the same content as a bytes Object
  bool doCompare(const std::vector<unsigned char>& data, PyObject* obj)
  {
    const char* py_data = PyBytes_AsString(obj);
    if (!py_data)
      return false;
    Py_ssize_t size = PyBytes_GET_SIZE(obj);
    if (size < 0 || std::vector<unsigned char>::size_type(size) != data.size())
      return false;
    return std::memcmp(py_data, &data[0], size) == 0;
  }

public:
  bool compareEmbeddedZeros(const ByteString& s)
  {
    const std::vector<unsigned char> testdata{ 0xff, 0xef, 0x00, 0x10 };
    return doCompare(testdata, s.ptr());
  }
  bool compareTuple(const bp::tuple& t)
  {
    const ByteString s(t[0]);
    const std::vector<unsigned char> testdata{ 'm', 'n', 'o' };
    return doCompare(testdata, s.ptr());
  }

  ByteString getBytesPChar()
  {
    return ByteString("abcdef");
  }
  ByteString getBytesStdString()
  {
    std::string s;
    s.push_back('\xff');
    s.push_back('\xfe');
    s.push_back('\x10');
    s.push_back('\x00');
    s.push_back('\x00');
    return ByteString(s);
  }
  ByteString getDefaultBytes()
  {
    return ByteString();
  }
  bp::tuple getTuple()
  {
    return bp::make_tuple(ByteString("abcdef"));
  }
  ByteString getVector()
  {
    geometry_msgs::Vector3 v;
    v.x = 1.0;
    v.y = -2.0;
    v.z = 0.25;
    return ByteString(v);
  }
  bool compareVector(const ByteString& s)
  {
    geometry_msgs::Vector3 v;
    s.deserialize(v);
    return v.x == 1.0 && v.y == -2.0 && v.z == 0.25;
  }
  bool compareVectorTuple(const bp::tuple& t)
  {
    const ByteString s(t[0]);
    return compareVector(s);
  }

  static void setup()
  {
    bp::class_<ByteStringTestHelper> cls("ByteStringTestHelper");
    cls.def("compareEmbeddedZeros", &ByteStringTestHelper::compareEmbeddedZeros);
    cls.def("compareTuple", &ByteStringTestHelper::compareTuple);
    cls.def("compareVectorTuple", &ByteStringTestHelper::compareVectorTuple);
    cls.def("getBytesPChar", &ByteStringTestHelper::getBytesPChar);
    cls.def("getBytesStdString", &ByteStringTestHelper::getBytesStdString);
    cls.def("getDefaultBytes", &ByteStringTestHelper::getDefaultBytes);
    cls.def("getTuple", &ByteStringTestHelper::getTuple);
    cls.def("getVector", &ByteStringTestHelper::getVector);
    cls.def("compareVector", &ByteStringTestHelper::compareVector);
  }
};

BOOST_PYTHON_MODULE(_moveit_planning_interface_test_serialize_msg_cpp_helper)
{
  ByteStringTestHelper::setup();
}
