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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/py_bindings_tools/gil_releaser.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <memory>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

using moveit::py_bindings_tools::GILReleaser;

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public MoveGroupInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if
  // needed
  MoveGroupInterfaceWrapper(const std::string& group_name, const std::string& robot_description,
                            const std::string& ns = "", double wait_for_servers = 5.0)
    : py_bindings_tools::ROScppInitializer()
    , MoveGroupInterface(Options(group_name, robot_description, ros::NodeHandle(ns)),
                         std::shared_ptr<tf2_ros::Buffer>(), ros::WallDuration(wait_for_servers))
  {
  }

  bool setJointValueTargetPerJointPythonList(const std::string& joint, bp::list& values)
  {
    return setJointValueTarget(joint, py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonIterable(bp::object& values)
  {
    return setJointValueTarget(py_bindings_tools::doubleFromList(values));
  }

  bool setJointValueTargetPythonDict(bp::dict& values)
  {
    bp::list k = values.keys();
    int l = bp::len(k);
    std::map<std::string, double> v;
    for (int i = 0; i < l; ++i)
      v[bp::extract<std::string>(k[i])] = bp::extract<double>(values[k[i]]);
    return setJointValueTarget(v);
  }

  bool setJointValueTargetFromPosePython(const py_bindings_tools::ByteString& pose_str, const std::string& eef,
                                         bool approx)
  {
    geometry_msgs::Pose pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromPoseStampedPython(const py_bindings_tools::ByteString& pose_str, const std::string& eef,
                                                bool approx)
  {
    geometry_msgs::PoseStamped pose_msg;
    py_bindings_tools::deserializeMsg(pose_str, pose_msg);
    return approx ? setApproximateJointValueTarget(pose_msg, eef) : setJointValueTarget(pose_msg, eef);
  }

  bool setJointValueTargetFromJointStatePython(const py_bindings_tools::ByteString& js_str)
  {
    sensor_msgs::JointState js_msg;
    py_bindings_tools::deserializeMsg(js_str, js_msg);
    return setJointValueTarget(js_msg);
  }

  bp::list getJointValueTargetPythonList()
  {
    std::vector<double> values;
    MoveGroupInterface::getJointValueTarget(values);
    bp::list l;
    for (const double value : values)
      l.append(value);
    return l;
  }

  py_bindings_tools::ByteString getJointValueTarget()
  {
    moveit_msgs::RobotState msg;
    const moveit::core::RobotState state = moveit::planning_interface::MoveGroupInterface::getTargetRobotState();
    moveit::core::robotStateToRobotStateMsg(state, msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  void rememberJointValuesFromPythonList(const std::string& string, bp::list& values)
  {
    rememberJointValues(string, py_bindings_tools::doubleFromList(values));
  }

  const char* getPlanningFrameCStr() const
  {
    return getPlanningFrame().c_str();
  }

  py_bindings_tools::ByteString getInterfaceDescriptionPython()
  {
    moveit_msgs::PlannerInterfaceDescription msg;
    getInterfaceDescription(msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  bp::list getActiveJointsList() const
  {
    return py_bindings_tools::listFromString(getActiveJoints());
  }

  bp::list getJointsList() const
  {
    return py_bindings_tools::listFromString(getJoints());
  }

  bp::list getCurrentJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getCurrentJointValues());
  }

  bp::list getRandomJointValuesList()
  {
    return py_bindings_tools::listFromDouble(getRandomJointValues());
  }

  bp::dict getRememberedJointValuesPython() const
  {
    const std::map<std::string, std::vector<double>>& rv = getRememberedJointValues();
    bp::dict d;
    for (const std::pair<const std::string, std::vector<double>>& it : rv)
      d[it.first] = py_bindings_tools::listFromDouble(it.second);
    return d;
  }

  bp::list convertPoseToList(const geometry_msgs::Pose& pose) const
  {
    std::vector<double> v(7);
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.x;
    v[4] = pose.orientation.y;
    v[5] = pose.orientation.z;
    v[6] = pose.orientation.w;
    return moveit::py_bindings_tools::listFromDouble(v);
  }

  bp::list convertTransformToList(const geometry_msgs::Transform& tr) const
  {
    std::vector<double> v(7);
    v[0] = tr.translation.x;
    v[1] = tr.translation.y;
    v[2] = tr.translation.z;
    v[3] = tr.rotation.x;
    v[4] = tr.rotation.y;
    v[5] = tr.rotation.z;
    v[6] = tr.rotation.w;
    return py_bindings_tools::listFromDouble(v);
  }

  void convertListToTransform(const bp::list& l, geometry_msgs::Transform& tr) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    tr.translation.x = v[0];
    tr.translation.y = v[1];
    tr.translation.z = v[2];
    tr.rotation.x = v[3];
    tr.rotation.y = v[4];
    tr.rotation.z = v[5];
    tr.rotation.w = v[6];
  }

  void convertListToPose(const bp::list& l, geometry_msgs::Pose& p) const
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    p.position.x = v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    p.orientation.x = v[3];
    p.orientation.y = v[4];
    p.orientation.z = v[5];
    p.orientation.w = v[6];
  }

  bp::list getCurrentRPYPython(const std::string& end_effector_link = "")
  {
    return py_bindings_tools::listFromDouble(getCurrentRPY(end_effector_link));
  }

  bp::list getCurrentPosePython(const std::string& end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getCurrentPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getRandomPosePython(const std::string& end_effector_link = "")
  {
    geometry_msgs::PoseStamped pose = getRandomPose(end_effector_link);
    return convertPoseToList(pose.pose);
  }

  bp::list getKnownConstraintsList() const
  {
    return py_bindings_tools::listFromString(getKnownConstraints());
  }

  bool placePose(const std::string& object_name, const bp::list& pose, bool plan_only = false)
  {
    geometry_msgs::PoseStamped msg;
    convertListToPose(pose, msg.pose);
    msg.header.frame_id = getPoseReferenceFrame();
    msg.header.stamp = ros::Time::now();
    GILReleaser gr;
    return place(object_name, msg, plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool placePoses(const std::string& object_name, const bp::list& poses_list, bool plan_only = false)
  {
    int l = bp::len(poses_list);
    std::vector<geometry_msgs::PoseStamped> poses(l);
    for (int i = 0; i < l; ++i)
      py_bindings_tools::deserializeMsg(py_bindings_tools::ByteString(poses_list[i]), poses[i]);
    GILReleaser gr;
    return place(object_name, poses, plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool placeLocation(const std::string& object_name, const py_bindings_tools::ByteString& location_str,
                     bool plan_only = false)
  {
    std::vector<moveit_msgs::PlaceLocation> locations(1);
    py_bindings_tools::deserializeMsg(location_str, locations[0]);
    GILReleaser gr;
    return place(object_name, std::move(locations), plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool placeLocations(const std::string& object_name, const bp::list& location_list, bool plan_only = false)
  {
    int l = bp::len(location_list);
    std::vector<moveit_msgs::PlaceLocation> locations(l);
    for (int i = 0; i < l; ++i)
      py_bindings_tools::deserializeMsg(py_bindings_tools::ByteString(location_list[i]), locations[i]);
    GILReleaser gr;
    return place(object_name, std::move(locations), plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool placeAnywhere(const std::string& object_name, bool plan_only = false)
  {
    GILReleaser gr;
    return place(object_name, plan_only) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void convertListToArrayOfPoses(const bp::list& poses, std::vector<geometry_msgs::Pose>& msg)
  {
    int l = bp::len(poses);
    for (int i = 0; i < l; ++i)
    {
      const bp::list& pose = bp::extract<bp::list>(poses[i]);
      std::vector<double> v = py_bindings_tools::doubleFromList(pose);
      if (v.size() == 6 || v.size() == 7)
      {
        Eigen::Isometry3d p;
        if (v.size() == 6)
        {
          tf2::Quaternion tq;
          tq.setRPY(v[3], v[4], v[5]);
          Eigen::Quaterniond eq;
          tf2::convert(tq, eq);
          p = Eigen::Isometry3d(eq);
        }
        else
          p = Eigen::Isometry3d(Eigen::Quaterniond(v[6], v[3], v[4], v[5]));
        p.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
        geometry_msgs::Pose pm = tf2::toMsg(p);
        msg.push_back(pm);
      }
      else
        ROS_WARN("Incorrect number of values for a pose: %u", (unsigned int)v.size());
    }
  }

  bp::dict getCurrentStateBoundedPython()
  {
    moveit::core::RobotStatePtr current = getCurrentState();
    current->enforceBounds();
    moveit_msgs::RobotState rsmv;
    moveit::core::robotStateToRobotStateMsg(*current, rsmv);
    bp::dict output;
    for (size_t x = 0; x < rsmv.joint_state.name.size(); ++x)
      output[rsmv.joint_state.name[x]] = rsmv.joint_state.position[x];
    return output;
  }

  py_bindings_tools::ByteString getCurrentStatePython()
  {
    moveit::core::RobotStatePtr current_state = getCurrentState();
    moveit_msgs::RobotState state_message;
    moveit::core::robotStateToRobotStateMsg(*current_state, state_message);
    return py_bindings_tools::serializeMsg(state_message);
  }

  void setStartStatePython(const py_bindings_tools::ByteString& msg_str)
  {
    moveit_msgs::RobotState msg;
    py_bindings_tools::deserializeMsg(msg_str, msg);
    setStartState(msg);
  }

  bool setPoseTargetsPython(bp::list& poses, const std::string& end_effector_link = "")
  {
    std::vector<geometry_msgs::Pose> msg;
    convertListToArrayOfPoses(poses, msg);
    return setPoseTargets(msg, end_effector_link);
  }
  py_bindings_tools::ByteString getPoseTargetPython(const std::string& end_effector_link)
  {
    geometry_msgs::PoseStamped pose = moveit::planning_interface::MoveGroupInterface::getPoseTarget(end_effector_link);
    return py_bindings_tools::serializeMsg(pose);
  }

  bool setPoseTargetPython(bp::list& pose, const std::string& end_effector_link = "")
  {
    std::vector<double> v = py_bindings_tools::doubleFromList(pose);
    geometry_msgs::Pose msg;
    if (v.size() == 6)
    {
      tf2::Quaternion q;
      q.setRPY(v[3], v[4], v[5]);
      tf2::convert(q, msg.orientation);
    }
    else if (v.size() == 7)
    {
      msg.orientation.x = v[3];
      msg.orientation.y = v[4];
      msg.orientation.z = v[5];
      msg.orientation.w = v[6];
    }
    else
    {
      ROS_ERROR("Pose description expected to consist of either 6 or 7 values");
      return false;
    }
    msg.position.x = v[0];
    msg.position.y = v[1];
    msg.position.z = v[2];
    return setPoseTarget(msg, end_effector_link);
  }

  const char* getEndEffectorLinkCStr() const
  {
    return getEndEffectorLink().c_str();
  }

  const char* getPoseReferenceFrameCStr() const
  {
    return getPoseReferenceFrame().c_str();
  }

  const char* getNameCStr() const
  {
    return getName().c_str();
  }

  const char* getPlannerIdCStr() const
  {
    return getPlannerId().c_str();
  }

  const char* getPlanningPipelineIdCStr() const
  {
    return getPlanningPipelineId().c_str();
  }

  bp::dict getNamedTargetValuesPython(const std::string& name)
  {
    bp::dict output;
    std::map<std::string, double> positions = getNamedTargetValues(name);
    std::map<std::string, double>::iterator iterator;

    for (iterator = positions.begin(); iterator != positions.end(); ++iterator)
      output[iterator->first] = iterator->second;
    return output;
  }

  bp::list getNamedTargetsPython()
  {
    return py_bindings_tools::listFromString(getNamedTargets());
  }

  bool movePython()
  {
    GILReleaser gr;
    return move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool asyncMovePython()
  {
    return asyncMove() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool attachObjectPython(const std::string& object_name, const std::string& link_name, const bp::list& touch_links)
  {
    return attachObject(object_name, link_name, py_bindings_tools::stringFromList(touch_links));
  }

  bool executePython(const py_bindings_tools::ByteString& plan_str)
  {
    MoveGroupInterface::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    GILReleaser gr;
    return execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool asyncExecutePython(const py_bindings_tools::ByteString& plan_str)
  {
    MoveGroupInterface::Plan plan;
    py_bindings_tools::deserializeMsg(plan_str, plan.trajectory_);
    return asyncExecute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bp::tuple planPython()
  {
    MoveGroupInterface::Plan plan;
    moveit_msgs::MoveItErrorCodes res;
    {
      GILReleaser gr;
      res = MoveGroupInterface::plan(plan);
    }
    return bp::make_tuple(py_bindings_tools::serializeMsg(res), py_bindings_tools::serializeMsg(plan.trajectory_),
                          plan.planning_time_);
  }

  py_bindings_tools::ByteString constructMotionPlanRequestPython()
  {
    moveit_msgs::MotionPlanRequest request;
    constructMotionPlanRequest(request);
    return py_bindings_tools::serializeMsg(request);
  }

  bp::tuple computeCartesianPathPython(const bp::list& waypoints, double eef_step, double jump_threshold,
                                       bool avoid_collisions)
  {
    moveit_msgs::Constraints path_constraints_tmp;
    return doComputeCartesianPathPython(waypoints, eef_step, jump_threshold, avoid_collisions, path_constraints_tmp);
  }

  bp::tuple computeCartesianPathConstrainedPython(const bp::list& waypoints, double eef_step, double jump_threshold,
                                                  bool avoid_collisions,
                                                  const py_bindings_tools::ByteString& path_constraints_str)
  {
    moveit_msgs::Constraints path_constraints;
    py_bindings_tools::deserializeMsg(path_constraints_str, path_constraints);
    return doComputeCartesianPathPython(waypoints, eef_step, jump_threshold, avoid_collisions, path_constraints);
  }

  bp::tuple doComputeCartesianPathPython(const bp::list& waypoints, double eef_step, double jump_threshold,
                                         bool avoid_collisions, const moveit_msgs::Constraints& path_constraints)
  {
    std::vector<geometry_msgs::Pose> poses;
    convertListToArrayOfPoses(waypoints, poses);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction;
    {
      GILReleaser gr;
      fraction = computeCartesianPath(poses, eef_step, jump_threshold, trajectory, path_constraints, avoid_collisions);
    }
    return bp::make_tuple(py_bindings_tools::serializeMsg(trajectory), fraction);
  }

  int pickGrasp(const std::string& object, const py_bindings_tools::ByteString& grasp_str, bool plan_only = false)
  {
    moveit_msgs::Grasp grasp;
    py_bindings_tools::deserializeMsg(grasp_str, grasp);
    GILReleaser gr;
    return pick(object, grasp, plan_only).val;
  }

  int pickGrasps(const std::string& object, const bp::list& grasp_list, bool plan_only = false)
  {
    int l = bp::len(grasp_list);
    std::vector<moveit_msgs::Grasp> grasps(l);
    for (int i = 0; i < l; ++i)
      py_bindings_tools::deserializeMsg(py_bindings_tools::ByteString(grasp_list[i]), grasps[i]);
    GILReleaser gr;
    return pick(object, std::move(grasps), plan_only).val;
  }

  void setPathConstraintsFromMsg(const py_bindings_tools::ByteString& constraints_str)
  {
    moveit_msgs::Constraints constraints_msg;
    py_bindings_tools::deserializeMsg(constraints_str, constraints_msg);
    setPathConstraints(constraints_msg);
  }

  py_bindings_tools::ByteString getPathConstraintsPython()
  {
    moveit_msgs::Constraints constraints_msg(getPathConstraints());
    return py_bindings_tools::serializeMsg(constraints_msg);
  }

  void setTrajectoryConstraintsFromMsg(const py_bindings_tools::ByteString& constraints_str)
  {
    moveit_msgs::TrajectoryConstraints constraints_msg;
    py_bindings_tools::deserializeMsg(constraints_str, constraints_msg);
    setTrajectoryConstraints(constraints_msg);
  }

  py_bindings_tools::ByteString getTrajectoryConstraintsPython()
  {
    moveit_msgs::TrajectoryConstraints constraints_msg(getTrajectoryConstraints());
    return py_bindings_tools::serializeMsg(constraints_msg);
  }

  py_bindings_tools::ByteString retimeTrajectory(const py_bindings_tools::ByteString& ref_state_str,
                                                 const py_bindings_tools::ByteString& traj_str,
                                                 double velocity_scaling_factor, double acceleration_scaling_factor,
                                                 const std::string& algorithm)
  {
    // Convert reference state message to object
    moveit_msgs::RobotState ref_state_msg;
    py_bindings_tools::deserializeMsg(ref_state_str, ref_state_msg);
    moveit::core::RobotState ref_state_obj(getRobotModel());
    if (moveit::core::robotStateMsgToRobotState(ref_state_msg, ref_state_obj, true))
    {
      // Convert trajectory message to object
      moveit_msgs::RobotTrajectory traj_msg;
      py_bindings_tools::deserializeMsg(traj_str, traj_msg);
      bool algorithm_found = true;
      {
        GILReleaser gr;
        robot_trajectory::RobotTrajectory traj_obj(getRobotModel(), getName());
        traj_obj.setRobotTrajectoryMsg(ref_state_obj, traj_msg);

        // Do the actual retiming
        if (algorithm == "iterative_time_parameterization")
        {
          trajectory_processing::IterativeParabolicTimeParameterization time_param;
          time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (algorithm == "iterative_spline_parameterization")
        {
          trajectory_processing::IterativeSplineParameterization time_param;
          time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (algorithm == "time_optimal_trajectory_generation")
        {
          trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
          time_param.computeTimeStamps(traj_obj, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("move_group_py", "Unknown time parameterization algorithm: " << algorithm);
          algorithm_found = false;
          traj_msg = moveit_msgs::RobotTrajectory();
        }

        if (algorithm_found)
          // Convert the retimed trajectory back into a message
          traj_obj.getRobotTrajectoryMsg(traj_msg);
      }
      return py_bindings_tools::serializeMsg(traj_msg);
    }
    else
    {
      ROS_ERROR("Unable to convert RobotState message to RobotState instance.");
      return py_bindings_tools::ByteString("");
    }
  }

  Eigen::MatrixXd getJacobianMatrixPython(const bp::list& joint_values, const bp::object& reference_point = bp::object())
  {
    const std::vector<double> v = py_bindings_tools::doubleFromList(joint_values);
    std::vector<double> ref;
    if (reference_point.is_none())
      ref = { 0.0, 0.0, 0.0 };
    else
      ref = py_bindings_tools::doubleFromList(reference_point);
    if (ref.size() != 3)
      throw std::invalid_argument("reference point needs to have 3 elements, got " + std::to_string(ref.size()));

    moveit::core::RobotState state(getRobotModel());
    state.setToDefaultValues();
    auto group = state.getJointModelGroup(getName());
    state.setJointGroupPositions(group, v);
    return state.getJacobian(group, Eigen::Map<Eigen::Vector3d>(&ref[0]));
  }

  py_bindings_tools::ByteString enforceBoundsPython(const py_bindings_tools::ByteString& msg_str)
  {
    moveit_msgs::RobotState state_msg;
    py_bindings_tools::deserializeMsg(msg_str, state_msg);
    moveit::core::RobotState state(getRobotModel());
    if (moveit::core::robotStateMsgToRobotState(state_msg, state, true))
    {
      state.enforceBounds();
      moveit::core::robotStateToRobotStateMsg(state, state_msg);
      return py_bindings_tools::serializeMsg(state_msg);
    }
    else
    {
      ROS_ERROR("Unable to convert RobotState message to RobotState instance.");
      return py_bindings_tools::ByteString("");
    }
  }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getJacobianMatrixOverloads, getJacobianMatrixPython, 1, 2)

static void wrap_move_group_interface()
{
  eigenpy::enableEigenPy();

  bp::class_<MoveGroupInterfaceWrapper, boost::noncopyable> move_group_interface_class(
      "MoveGroupInterface", bp::init<std::string, std::string, bp::optional<std::string, double>>());

  move_group_interface_class.def("async_move", &MoveGroupInterfaceWrapper::asyncMovePython);
  move_group_interface_class.def("move", &MoveGroupInterfaceWrapper::movePython);
  move_group_interface_class.def("execute", &MoveGroupInterfaceWrapper::executePython);
  move_group_interface_class.def("async_execute", &MoveGroupInterfaceWrapper::asyncExecutePython);
  moveit::core::MoveItErrorCode (MoveGroupInterfaceWrapper::*pick_1)(const std::string&, bool) =
      &MoveGroupInterfaceWrapper::pick;
  move_group_interface_class.def("pick", pick_1);
  move_group_interface_class.def("pick", &MoveGroupInterfaceWrapper::pickGrasp);
  move_group_interface_class.def("pick", &MoveGroupInterfaceWrapper::pickGrasps);
  move_group_interface_class.def("place", &MoveGroupInterfaceWrapper::placePose);
  move_group_interface_class.def("place_poses_list", &MoveGroupInterfaceWrapper::placePoses);
  move_group_interface_class.def("place", &MoveGroupInterfaceWrapper::placeLocation);
  move_group_interface_class.def("place_locations_list", &MoveGroupInterfaceWrapper::placeLocations);
  move_group_interface_class.def("place", &MoveGroupInterfaceWrapper::placeAnywhere);
  move_group_interface_class.def("stop", &MoveGroupInterfaceWrapper::stop);

  move_group_interface_class.def("get_name", &MoveGroupInterfaceWrapper::getNameCStr);
  move_group_interface_class.def("get_planning_frame", &MoveGroupInterfaceWrapper::getPlanningFrameCStr);
  move_group_interface_class.def("get_interface_description", &MoveGroupInterfaceWrapper::getInterfaceDescriptionPython);

  move_group_interface_class.def("get_active_joints", &MoveGroupInterfaceWrapper::getActiveJointsList);
  move_group_interface_class.def("get_joints", &MoveGroupInterfaceWrapper::getJointsList);
  move_group_interface_class.def("get_variable_count", &MoveGroupInterfaceWrapper::getVariableCount);
  move_group_interface_class.def("allow_looking", &MoveGroupInterfaceWrapper::allowLooking);
  move_group_interface_class.def("allow_replanning", &MoveGroupInterfaceWrapper::allowReplanning);

  move_group_interface_class.def("set_pose_reference_frame", &MoveGroupInterfaceWrapper::setPoseReferenceFrame);

  move_group_interface_class.def("set_pose_reference_frame", &MoveGroupInterfaceWrapper::setPoseReferenceFrame);
  move_group_interface_class.def("set_end_effector_link", &MoveGroupInterfaceWrapper::setEndEffectorLink);
  move_group_interface_class.def("get_end_effector_link", &MoveGroupInterfaceWrapper::getEndEffectorLinkCStr);
  move_group_interface_class.def("get_pose_reference_frame", &MoveGroupInterfaceWrapper::getPoseReferenceFrameCStr);

  move_group_interface_class.def("set_pose_target", &MoveGroupInterfaceWrapper::setPoseTargetPython);

  move_group_interface_class.def("set_pose_targets", &MoveGroupInterfaceWrapper::setPoseTargetsPython);

  move_group_interface_class.def("set_position_target", &MoveGroupInterfaceWrapper::setPositionTarget);
  move_group_interface_class.def("set_rpy_target", &MoveGroupInterfaceWrapper::setRPYTarget);
  move_group_interface_class.def("set_orientation_target", &MoveGroupInterfaceWrapper::setOrientationTarget);

  move_group_interface_class.def("get_current_pose", &MoveGroupInterfaceWrapper::getCurrentPosePython);
  move_group_interface_class.def("get_current_rpy", &MoveGroupInterfaceWrapper::getCurrentRPYPython);

  move_group_interface_class.def("get_random_pose", &MoveGroupInterfaceWrapper::getRandomPosePython);

  move_group_interface_class.def("clear_pose_target", &MoveGroupInterfaceWrapper::clearPoseTarget);
  move_group_interface_class.def("clear_pose_targets", &MoveGroupInterfaceWrapper::clearPoseTargets);

  move_group_interface_class.def("set_joint_value_target",
                                 &MoveGroupInterfaceWrapper::setJointValueTargetPythonIterable);
  move_group_interface_class.def("set_joint_value_target", &MoveGroupInterfaceWrapper::setJointValueTargetPythonDict);

  move_group_interface_class.def("set_joint_value_target",
                                 &MoveGroupInterfaceWrapper::setJointValueTargetPerJointPythonList);
  bool (MoveGroupInterfaceWrapper::*set_joint_value_target_4)(const std::string&, double) =
      &MoveGroupInterfaceWrapper::setJointValueTarget;
  move_group_interface_class.def("set_joint_value_target", set_joint_value_target_4);

  move_group_interface_class.def("set_joint_value_target_from_pose",
                                 &MoveGroupInterfaceWrapper::setJointValueTargetFromPosePython);
  move_group_interface_class.def("set_joint_value_target_from_pose_stamped",
                                 &MoveGroupInterfaceWrapper::setJointValueTargetFromPoseStampedPython);
  move_group_interface_class.def("set_joint_value_target_from_joint_state_message",
                                 &MoveGroupInterfaceWrapper::setJointValueTargetFromJointStatePython);

  move_group_interface_class.def("get_joint_value_target", &MoveGroupInterfaceWrapper::getJointValueTargetPythonList);

  move_group_interface_class.def("set_named_target", &MoveGroupInterfaceWrapper::setNamedTarget);
  move_group_interface_class.def("set_random_target", &MoveGroupInterfaceWrapper::setRandomTarget);

  void (MoveGroupInterfaceWrapper::*remember_joint_values_2)(const std::string&) =
      &MoveGroupInterfaceWrapper::rememberJointValues;
  move_group_interface_class.def("remember_joint_values", remember_joint_values_2);

  move_group_interface_class.def("remember_joint_values", &MoveGroupInterfaceWrapper::rememberJointValuesFromPythonList);

  move_group_interface_class.def("start_state_monitor", &MoveGroupInterfaceWrapper::startStateMonitor);
  move_group_interface_class.def("get_current_joint_values", &MoveGroupInterfaceWrapper::getCurrentJointValuesList);
  move_group_interface_class.def("get_random_joint_values", &MoveGroupInterfaceWrapper::getRandomJointValuesList);
  move_group_interface_class.def("get_remembered_joint_values",
                                 &MoveGroupInterfaceWrapper::getRememberedJointValuesPython);

  move_group_interface_class.def("forget_joint_values", &MoveGroupInterfaceWrapper::forgetJointValues);

  move_group_interface_class.def("get_goal_joint_tolerance", &MoveGroupInterfaceWrapper::getGoalJointTolerance);
  move_group_interface_class.def("get_goal_position_tolerance", &MoveGroupInterfaceWrapper::getGoalPositionTolerance);
  move_group_interface_class.def("get_goal_orientation_tolerance",
                                 &MoveGroupInterfaceWrapper::getGoalOrientationTolerance);

  move_group_interface_class.def("set_goal_joint_tolerance", &MoveGroupInterfaceWrapper::setGoalJointTolerance);
  move_group_interface_class.def("set_goal_position_tolerance", &MoveGroupInterfaceWrapper::setGoalPositionTolerance);
  move_group_interface_class.def("set_goal_orientation_tolerance",
                                 &MoveGroupInterfaceWrapper::setGoalOrientationTolerance);
  move_group_interface_class.def("set_goal_tolerance", &MoveGroupInterfaceWrapper::setGoalTolerance);

  move_group_interface_class.def("set_start_state_to_current_state",
                                 &MoveGroupInterfaceWrapper::setStartStateToCurrentState);
  move_group_interface_class.def("set_start_state", &MoveGroupInterfaceWrapper::setStartStatePython);

  bool (MoveGroupInterfaceWrapper::*set_path_constraints_1)(const std::string&) =
      &MoveGroupInterfaceWrapper::setPathConstraints;
  move_group_interface_class.def("set_path_constraints", set_path_constraints_1);
  move_group_interface_class.def("set_path_constraints_from_msg", &MoveGroupInterfaceWrapper::setPathConstraintsFromMsg);
  move_group_interface_class.def("get_path_constraints", &MoveGroupInterfaceWrapper::getPathConstraintsPython);
  move_group_interface_class.def("clear_path_constraints", &MoveGroupInterfaceWrapper::clearPathConstraints);

  move_group_interface_class.def("set_trajectory_constraints_from_msg",
                                 &MoveGroupInterfaceWrapper::setTrajectoryConstraintsFromMsg);
  move_group_interface_class.def("get_trajectory_constraints",
                                 &MoveGroupInterfaceWrapper::getTrajectoryConstraintsPython);
  move_group_interface_class.def("clear_trajectory_constraints", &MoveGroupInterfaceWrapper::clearTrajectoryConstraints);
  move_group_interface_class.def("get_known_constraints", &MoveGroupInterfaceWrapper::getKnownConstraintsList);
  move_group_interface_class.def("set_constraints_database", &MoveGroupInterfaceWrapper::setConstraintsDatabase);
  move_group_interface_class.def("set_workspace", &MoveGroupInterfaceWrapper::setWorkspace);
  move_group_interface_class.def("set_planning_time", &MoveGroupInterfaceWrapper::setPlanningTime);
  move_group_interface_class.def("get_planning_time", &MoveGroupInterfaceWrapper::getPlanningTime);
  move_group_interface_class.def("set_max_velocity_scaling_factor",
                                 &MoveGroupInterfaceWrapper::setMaxVelocityScalingFactor);
  move_group_interface_class.def("set_max_acceleration_scaling_factor",
                                 &MoveGroupInterfaceWrapper::setMaxAccelerationScalingFactor);
  move_group_interface_class.def("limit_max_cartesian_link_speed",
                                 &MoveGroupInterfaceWrapper::limitMaxCartesianLinkSpeed);
  move_group_interface_class.def("clear_max_cartesian_link_speed",
                                 &MoveGroupInterfaceWrapper::clearMaxCartesianLinkSpeed);
  move_group_interface_class.def("set_planner_id", &MoveGroupInterfaceWrapper::setPlannerId);
  move_group_interface_class.def("get_planner_id", &MoveGroupInterfaceWrapper::getPlannerIdCStr);
  move_group_interface_class.def("set_planning_pipeline_id", &MoveGroupInterfaceWrapper::setPlanningPipelineId);
  move_group_interface_class.def("get_planning_pipeline_id", &MoveGroupInterfaceWrapper::getPlanningPipelineIdCStr);
  move_group_interface_class.def("set_num_planning_attempts", &MoveGroupInterfaceWrapper::setNumPlanningAttempts);
  move_group_interface_class.def("plan", &MoveGroupInterfaceWrapper::planPython);
  move_group_interface_class.def("construct_motion_plan_request",
                                 &MoveGroupInterfaceWrapper::constructMotionPlanRequestPython);
  move_group_interface_class.def("compute_cartesian_path", &MoveGroupInterfaceWrapper::computeCartesianPathPython);
  move_group_interface_class.def("compute_cartesian_path",
                                 &MoveGroupInterfaceWrapper::computeCartesianPathConstrainedPython);
  move_group_interface_class.def("set_support_surface_name", &MoveGroupInterfaceWrapper::setSupportSurfaceName);
  move_group_interface_class.def("attach_object", &MoveGroupInterfaceWrapper::attachObjectPython);
  move_group_interface_class.def("detach_object", &MoveGroupInterfaceWrapper::detachObject);
  move_group_interface_class.def("retime_trajectory", &MoveGroupInterfaceWrapper::retimeTrajectory);
  move_group_interface_class.def("get_named_targets", &MoveGroupInterfaceWrapper::getNamedTargetsPython);
  move_group_interface_class.def("get_named_target_values", &MoveGroupInterfaceWrapper::getNamedTargetValuesPython);
  move_group_interface_class.def("get_current_state_bounded", &MoveGroupInterfaceWrapper::getCurrentStateBoundedPython);
  move_group_interface_class.def("get_current_state", &MoveGroupInterfaceWrapper::getCurrentStatePython);
  move_group_interface_class.def("get_jacobian_matrix", &MoveGroupInterfaceWrapper::getJacobianMatrixPython,
                                 getJacobianMatrixOverloads());
  move_group_interface_class.def("enforce_bounds", &MoveGroupInterfaceWrapper::enforceBoundsPython);
}
}  // namespace planning_interface
}  // namespace moveit

BOOST_PYTHON_MODULE(_moveit_move_group_interface)
{
  using namespace moveit::planning_interface;
  wrap_move_group_interface();
}

/** @endcond */
