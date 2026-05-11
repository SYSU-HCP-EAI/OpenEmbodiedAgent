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
#include <urdf_parser/urdf_parser.h>
#include <moveit/robot_model/robot_model.h>
#include <srdfdom/model.h>

namespace py = pybind11;

// Each of these functions defines the bindings for a namespace/subfolder within moveit_core
void def_collision_detection_bindings(py::module& contact);

void def_robot_model_bindings(py::module& m);

void def_robot_state_bindings(py::module& m);

void def_transforms_bindings(py::module& m);

void def_planning_scene_bindings(py::module& m);

void def_kinematic_constraints_bindings(py::module& m);

auto load_robot_model(const std::string& urdf_path, const std::string& srdf_path)
{
  auto urdf_model = urdf::parseURDFFile(urdf_path);
  auto srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initFile(*urdf_model, srdf_path);
  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  return robot_model;
}

PYBIND11_MODULE(pymoveit_core, m)
{
  auto collision_detection_m = m.def_submodule("collision_detection");
  auto planning_scene_m = m.def_submodule("planning_scene");
  auto robot_model_m = m.def_submodule("robot_model");
  auto robot_state_m = m.def_submodule("robot_state");
  auto kinematic_constraints_m = m.def_submodule("kinematic_constraints");
  auto transforms_m = m.def_submodule("transforms");

  def_collision_detection_bindings(collision_detection_m);
  def_robot_model_bindings(robot_model_m);
  def_robot_state_bindings(robot_state_m);
  def_planning_scene_bindings(planning_scene_m);
  def_transforms_bindings(transforms_m);
  def_kinematic_constraints_bindings(kinematic_constraints_m);

  // convenience function
  m.def("load_robot_model", &load_robot_model);
}
