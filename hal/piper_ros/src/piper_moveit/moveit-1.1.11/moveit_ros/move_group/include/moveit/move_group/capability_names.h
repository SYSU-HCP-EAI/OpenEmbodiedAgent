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

#include <string>

namespace move_group
{
static const std::string PLANNER_SERVICE_NAME =
    "plan_kinematic_path";  // name of the advertised service (within the ~ namespace)
static const std::string EXECUTE_ACTION_NAME = "execute_trajectory";  // name of 'execute' action
static const std::string QUERY_PLANNERS_SERVICE_NAME =
    "query_planner_interface";  // name of the advertised query planners service
static const std::string GET_PLANNER_PARAMS_SERVICE_NAME =
    "get_planner_params";  // service name to retrieve planner parameters
static const std::string SET_PLANNER_PARAMS_SERVICE_NAME =
    "set_planner_params";                                 // service name to set planner parameters
static const std::string MOVE_ACTION = "move_group";      // name of 'move' action
static const std::string IK_SERVICE_NAME = "compute_ik";  // name of ik service
static const std::string FK_SERVICE_NAME = "compute_fk";  // name of fk service
static const std::string STATE_VALIDITY_SERVICE_NAME =
    "check_state_validity";  // name of the service that validates states
static const std::string CARTESIAN_PATH_SERVICE_NAME =
    "compute_cartesian_path";  // name of the service that computes cartesian paths
static const std::string GET_PLANNING_SCENE_SERVICE_NAME =
    "get_planning_scene";  // name of the service that can be used to query the planning scene
static const std::string APPLY_PLANNING_SCENE_SERVICE_NAME =
    "apply_planning_scene";  // name of the service that applies a given planning scene
static const std::string CLEAR_OCTOMAP_SERVICE_NAME =
    "clear_octomap";  // name of the service that can be used to clear the octomap
}  // namespace move_group
