/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#include <ros/ros.h>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <moveit/utils/xmlrpc_casts.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <map>

using namespace moveit::core;

const std::string LOGNAME("SimpleControllerManager");

namespace moveit_simple_controller_manager
{
class MoveItSimpleControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItSimpleControllerManager() : node_handle_("~")
  {
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (!isArray(controller_list))
    {
      ROS_ERROR_NAMED(LOGNAME, "Parameter controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      if (!isStruct(controller_list[i], { "name", "joints", "action_ns", "type" }))
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "name, joints, action_ns, and type must be specifed for each controller");
        continue;
      }

      try
      {
        const std::string name = std::string(controller_list[i]["name"]);
        const std::string action_ns = std::string(controller_list[i]["action_ns"]);
        const std::string type = std::string(controller_list[i]["type"]);

        if (!isArray(controller_list[i]["joints"]))
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME,
                                 "The list of joints for controller " << name << " is not specified as an array");
          continue;
        }

        ActionBasedControllerHandleBasePtr new_handle;
        if (type == "GripperCommand")
        {
          const double max_effort =
              controller_list[i].hasMember("max_effort") ? double(controller_list[i]["max_effort"]) : 0.0;

          new_handle = std::make_shared<GripperControllerHandle>(name, action_ns, max_effort);
          if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
          {
            if (controller_list[i].hasMember("parallel"))
            {
              if (controller_list[i]["joints"].size() != 2)
              {
                ROS_ERROR_STREAM_NAMED(LOGNAME, "Parallel Gripper requires exactly two joints");
                continue;
              }
              static_cast<GripperControllerHandle*>(new_handle.get())
                  ->setParallelJawGripper(controller_list[i]["joints"][0], controller_list[i]["joints"][1]);
            }
            else
            {
              if (controller_list[i].hasMember("command_joint"))
                static_cast<GripperControllerHandle*>(new_handle.get())
                    ->setCommandJoint(controller_list[i]["command_joint"]);
              else
                static_cast<GripperControllerHandle*>(new_handle.get())->setCommandJoint(controller_list[i]["joints"][0]);
            }

            if (controller_list[i].hasMember("allow_failure"))
              static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(true);

            ROS_INFO_STREAM_NAMED(LOGNAME, "Added GripperCommand controller for " << name);
            controllers_[name] = new_handle;
          }
        }
        else if (type == "FollowJointTrajectory")
        {
          auto h = new FollowJointTrajectoryControllerHandle(name, action_ns);
          new_handle.reset(h);
          if (h->isConnected())
          {
            ROS_INFO_STREAM_NAMED(LOGNAME, "Added FollowJointTrajectory controller for " << name);
            controllers_[name] = new_handle;
          }
        }
        else
        {
          ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown controller type: " << type.c_str());
          continue;
        }
        if (!controllers_[name])
        {
          controllers_.erase(name);
          continue;
        }

        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        state.default_ = controller_list[i].hasMember("default") ? (bool)controller_list[i]["default"] : false;
        state.active_ = true;

        controller_states_[name] = state;

        /* add list of joints, used by controller manager and MoveIt */
        for (int j = 0; j < controller_list[i]["joints"].size(); ++j)
          new_handle->addJoint(std::string(controller_list[i]["joints"][j]));

        new_handle->configure(controller_list[i]);
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Caught unknown exception while parsing controller information");
      }
    }
  }

  ~MoveItSimpleControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      ROS_FATAL_STREAM_NAMED(LOGNAME, "No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    for (std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM_NAMED(LOGNAME, "Returned " << names.size() << " controllers in list");
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal
   * with it anyways!
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN_NAMED(LOGNAME,
                     "The joints for controller '%s' are not known. Perhaps the controller configuration is "
                     "not loaded on the param server?",
                     name.c_str());
      joints.clear();
    }
  }

  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    return controller_states_[name];
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& /* activate */,
                         const std::vector<std::string>& /* deactivate */) override
  {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
  std::map<std::string, moveit_controller_manager::MoveItControllerManager::ControllerState> controller_states_;
};

}  // end namespace moveit_simple_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_simple_controller_manager::MoveItSimpleControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
