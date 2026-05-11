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
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

namespace py = pybind11;

using namespace kinematic_constraints;

void def_kinematic_constraints_bindings(py::module& m)
{
  m.doc() = "Class for joint, position, visibility, and other constraints";

  py::class_<ConstraintEvaluationResult, std::shared_ptr<ConstraintEvaluationResult>>(m, "ConstraintEvaluationResult")
      .def(py::init<>())
      .def_readwrite("satisfied", &ConstraintEvaluationResult::satisfied)
      .def_readwrite("distance", &ConstraintEvaluationResult::distance)
      //
      ;

  py::class_<KinematicConstraintSet, KinematicConstraintSetPtr>(m, "KinematicConstraintSet")
      .def(py::init<robot_model::RobotModelConstPtr>(), py::arg("robot_model"))
      .def("add", py::overload_cast<const moveit_msgs::Constraints&, const moveit::core::Transforms&>(
                      &KinematicConstraintSet::add))
      .def("decide",
           py::overload_cast<const moveit::core::RobotState&, bool>(&KinematicConstraintSet::decide, py::const_),
           py::arg("state"), py::arg("verbose") = false)
      //
      ;

  m.def("constructGoalConstraints",
        py::overload_cast<const std::string&, const geometry_msgs::PoseStamped&, double, double>(
            &constructGoalConstraints),
        py::arg("link_name"), py::arg("pose"), py::arg("tolerance_pos") = 1e-3, py::arg("tolerance_angle") = 1e-2)
      //
      ;
}
