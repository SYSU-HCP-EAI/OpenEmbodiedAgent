/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Peter Mitrano
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
 *   * The name of Peter Mitrano may not be used to endorse or promote
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

/* Author: Peter Mitrano */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/transforms/transforms.h>

namespace py = pybind11;
using namespace moveit::core;

void def_transforms_bindings(py::module& m)
{
  m.doc() = "Provides an implementation of a snapshot of a transform tree that can be easily queried for transforming "
            "different quantities. Transforms are maintained as a list of transforms to a particular frame. All stored "
            "transforms are considered fixed.";
  py::class_<Transforms, TransformsPtr>(m, "Transforms")
      .def(py::init<const std::string&>())
      .def("canTransform", &Transforms::canTransform)
      .def("getTargetFrame", &Transforms::getTargetFrame)
      .def("getTransform", &Transforms::getTransform)
      .def("isFixedFrame", &Transforms::isFixedFrame)
      .def("getAllTransforms", &Transforms::getAllTransforms)
      .def("setTransform", py::overload_cast<const Eigen::Isometry3d&, const std::string&>(&Transforms::setTransform))
      .def("setTransform", py::overload_cast<const geometry_msgs::TransformStamped&>(&Transforms::setTransform))
      .def("setTransforms", &Transforms::setTransforms)
      .def("setAllTransforms", &Transforms::setAllTransforms)
      .def("transformVector3", &Transforms::transformVector3)
      .def("transformQuaternion", &Transforms::transformQuaternion)
      .def("transformRotationMatrix", &Transforms::transformRotationMatrix)
      .def("transformPose", &Transforms::transformPose)
      //
      ;
}
