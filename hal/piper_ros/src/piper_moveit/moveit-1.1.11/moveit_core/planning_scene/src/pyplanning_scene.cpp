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

#include <memory>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/pybind_rosmsg_typecasters.h>
#include <moveit/planning_scene/planning_scene.h>

namespace py = pybind11;
using namespace planning_scene;

void def_planning_scene_bindings(py::module& m)
{
  m.doc() = "The planning scene represents the state of the world and the robot, "
            "and can be used for collision checking";

  py::class_<PlanningScene, PlanningScenePtr>(m, "PlanningScene")
      .def(py::init<const moveit::core::RobotModelConstPtr&, const collision_detection::WorldPtr&>(),
           py::arg("robot_model"), py::arg("world") = std::make_shared<collision_detection::World>())
      .def("checkSelfCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &PlanningScene::checkSelfCollision, py::const_))
      .def("checkSelfCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             const robot_state::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &PlanningScene::checkSelfCollision, py::const_))
      .def("checkCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&>(
               &PlanningScene::checkCollision, py::const_))
      .def("checkCollision",
           py::overload_cast<const collision_detection::CollisionRequest&, collision_detection::CollisionResult&,
                             const robot_state::RobotState&, const collision_detection::AllowedCollisionMatrix&>(
               &PlanningScene::checkCollision, py::const_))
      .def("getCurrentStateNonConst", &PlanningScene::getCurrentStateNonConst)
      .def("getCurrentState", &PlanningScene::getCurrentState)
      .def("getAllowedCollisionMatrix", &PlanningScene::getAllowedCollisionMatrix)
      .def("isStateConstrained",
           py::overload_cast<const moveit_msgs::RobotState&, const kinematic_constraints::KinematicConstraintSet&, bool>(
               &PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def(
          "isStateConstrained",
          py::overload_cast<const moveit::core::RobotState&, const kinematic_constraints::KinematicConstraintSet&, bool>(
              &PlanningScene::isStateConstrained, py::const_),
          py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained",
           py::overload_cast<const moveit_msgs::RobotState&, const moveit_msgs::Constraints&, bool>(
               &PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("isStateConstrained",
           py::overload_cast<const moveit::core::RobotState&, const moveit_msgs::Constraints&, bool>(
               &PlanningScene::isStateConstrained, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("verbose") = false)
      .def("getTransforms", py::overload_cast<>(&PlanningScene::getTransforms, py::const_),
           py::return_value_policy::reference)
      //      .def("setStateFeasibilityPredicate", &PlanningScene::setStateFeasibilityPredicate)
      .def("isStateValid",
           py::overload_cast<const moveit_msgs::RobotState&, const std::string&, bool>(&PlanningScene::isStateValid,
                                                                                       py::const_),
           py::arg("state"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit::core::RobotState&, const std::string&, bool>(&PlanningScene::isStateValid,
                                                                                        py::const_),
           py::arg("state"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit_msgs::RobotState&, const moveit_msgs::Constraints&, const std::string&, bool>(
               &PlanningScene::isStateValid, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit::core::RobotState&, const moveit_msgs::Constraints&, const std::string&, bool>(
               &PlanningScene::isStateValid, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("group"), py::arg("verbose") = false)
      .def("isStateValid",
           py::overload_cast<const moveit::core::RobotState&, const kinematic_constraints::KinematicConstraintSet&,
                             const std::string&, bool>(&PlanningScene::isStateValid, py::const_),
           py::arg("state"), py::arg("constr"), py::arg("group"), py::arg("verbose") = false)
      .def("setCurrentState", py::overload_cast<const moveit_msgs::RobotState&>(&PlanningScene::setCurrentState))
      .def("setCurrentState", py::overload_cast<const robot_state::RobotState&>(&PlanningScene::setCurrentState))
      //
      ;
}
