/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/utils/xmlrpc_casts.h>
#include <moveit/utils/message_checks.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace moveit::core;

namespace kinematic_constraints
{
const std::string LOGNAME = "kinematic_constraint_utils";

moveit_msgs::Constraints mergeConstraints(const moveit_msgs::Constraints& first, const moveit_msgs::Constraints& second)
{
  moveit_msgs::Constraints r;

  // add all joint constraints that are in first but not in second
  // and merge joint constraints that are for the same joint
  for (const moveit_msgs::JointConstraint& jc_first : first.joint_constraints)
  {
    bool add = true;
    for (const moveit_msgs::JointConstraint& jc_second : second.joint_constraints)
      if (jc_second.joint_name == jc_first.joint_name)
      {
        add = false;
        // now we merge
        moveit_msgs::JointConstraint m;
        const moveit_msgs::JointConstraint& a = jc_first;
        const moveit_msgs::JointConstraint& b = jc_second;
        double low = std::max(a.position - a.tolerance_below, b.position - b.tolerance_below);
        double high = std::min(a.position + a.tolerance_above, b.position + b.tolerance_above);
        if (low > high)
          ROS_ERROR_NAMED("kinematic_constraints",
                          "Attempted to merge incompatible constraints for joint '%s'. Discarding constraint.",
                          a.joint_name.c_str());
        else
        {
          m.joint_name = a.joint_name;
          m.position =
              std::max(low, std::min((a.position * a.weight + b.position * b.weight) / (a.weight + b.weight), high));
          m.weight = (a.weight + b.weight) / 2.0;
          m.tolerance_above = std::max(0.0, high - m.position);
          m.tolerance_below = std::max(0.0, m.position - low);
          r.joint_constraints.push_back(m);
        }
        break;
      }
    if (add)
      r.joint_constraints.push_back(jc_first);
  }

  // add all joint constraints that are in second but not in first
  for (const moveit_msgs::JointConstraint& jc_second : second.joint_constraints)
  {
    bool add = true;
    for (const moveit_msgs::JointConstraint& jc_first : first.joint_constraints)
      if (jc_second.joint_name == jc_first.joint_name)
      {
        add = false;
        break;
      }
    if (add)
      r.joint_constraints.push_back(jc_second);
  }

  // merge rest of constraints
  r.position_constraints = first.position_constraints;
  for (const moveit_msgs::PositionConstraint& position_constraint : second.position_constraints)
    r.position_constraints.push_back(position_constraint);

  r.orientation_constraints = first.orientation_constraints;
  for (const moveit_msgs::OrientationConstraint& orientation_constraint : second.orientation_constraints)
    r.orientation_constraints.push_back(orientation_constraint);

  r.visibility_constraints = first.visibility_constraints;
  for (const moveit_msgs::VisibilityConstraint& visibility_constraint : second.visibility_constraints)
    r.visibility_constraints.push_back(visibility_constraint);

  return r;
}

bool isEmpty(const moveit_msgs::Constraints& constr)
{
  return moveit::core::isEmpty(constr);
}

std::size_t countIndividualConstraints(const moveit_msgs::Constraints& constr)
{
  return constr.position_constraints.size() + constr.orientation_constraints.size() +
         constr.visibility_constraints.size() + constr.joint_constraints.size();
}

moveit_msgs::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                  const moveit::core::JointModelGroup* jmg, double tolerance)
{
  return constructGoalConstraints(state, jmg, tolerance, tolerance);
}

moveit_msgs::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                  const moveit::core::JointModelGroup* jmg, double tolerance_below,
                                                  double tolerance_above)
{
  moveit_msgs::Constraints goal;
  std::vector<double> vals;
  state.copyJointGroupPositions(jmg, vals);
  goal.joint_constraints.resize(vals.size());
  for (std::size_t i = 0; i < vals.size(); ++i)
  {
    goal.joint_constraints[i].joint_name = jmg->getVariableNames()[i];
    goal.joint_constraints[i].position = vals[i];
    goal.joint_constraints[i].tolerance_above = tolerance_above;
    goal.joint_constraints[i].tolerance_below = tolerance_below;
    goal.joint_constraints[i].weight = 1.0;
  }

  return goal;
}

moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name, const geometry_msgs::PoseStamped& pose,
                                                  double tolerance_pos, double tolerance_angle)
{
  moveit_msgs::Constraints goal;

  goal.position_constraints.resize(1);
  moveit_msgs::PositionConstraint& pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  shape_msgs::SolidPrimitive& bv = pcm.constraint_region.primitives[0];
  bv.type = shape_msgs::SolidPrimitive::SPHERE;
  bv.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>());
  bv.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = tolerance_pos;

  pcm.header = pose.header;
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position = pose.pose.position;

  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  goal.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint& ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = pose.header;
  ocm.orientation = pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = tolerance_angle;
  ocm.absolute_y_axis_tolerance = tolerance_angle;
  ocm.absolute_z_axis_tolerance = tolerance_angle;
  ocm.weight = 1.0;

  return goal;
}

moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name, const geometry_msgs::PoseStamped& pose,
                                                  const std::vector<double>& tolerance_pos,
                                                  const std::vector<double>& tolerance_angle)
{
  moveit_msgs::Constraints goal = constructGoalConstraints(link_name, pose);
  if (tolerance_pos.size() == 3)
  {
    shape_msgs::SolidPrimitive& bv = goal.position_constraints[0].constraint_region.primitives[0];
    bv.type = shape_msgs::SolidPrimitive::BOX;
    bv.dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>());
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_X] = tolerance_pos[0];
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tolerance_pos[1];
    bv.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tolerance_pos[2];
  }
  if (tolerance_angle.size() == 3)
  {
    moveit_msgs::OrientationConstraint& ocm = goal.orientation_constraints[0];
    ocm.absolute_x_axis_tolerance = tolerance_angle[0];
    ocm.absolute_y_axis_tolerance = tolerance_angle[1];
    ocm.absolute_z_axis_tolerance = tolerance_angle[2];
  }
  return goal;
}

moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::QuaternionStamped& quat, double tolerance)
{
  moveit_msgs::Constraints goal;
  goal.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint& ocm = goal.orientation_constraints[0];
  ocm.link_name = link_name;
  ocm.header = quat.header;
  ocm.orientation = quat.quaternion;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;
  ocm.weight = 1.0;
  return goal;
}

moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::PointStamped& goal_point, double tolerance)
{
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  return constructGoalConstraints(link_name, p, goal_point, tolerance);
}

moveit_msgs::Constraints constructGoalConstraints(const std::string& link_name,
                                                  const geometry_msgs::Point& reference_point,
                                                  const geometry_msgs::PointStamped& goal_point, double tolerance)
{
  moveit_msgs::Constraints goal;
  goal.position_constraints.resize(1);
  moveit_msgs::PositionConstraint& pcm = goal.position_constraints[0];
  pcm.link_name = link_name;
  pcm.target_point_offset.x = reference_point.x;
  pcm.target_point_offset.y = reference_point.y;
  pcm.target_point_offset.z = reference_point.z;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>());
  pcm.constraint_region.primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = tolerance;

  pcm.header = goal_point.header;
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position = goal_point.point;

  // orientation of constraint region does not affect anything, since it is a sphere
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;

  return goal;
}

static bool constructPoseStamped(XmlRpc::XmlRpcValue::iterator& it, geometry_msgs::PoseStamped& pose)
{
  if (!isStruct(it->second, { "frame_id", "position", "orientation" }, it->first))
    return false;
  pose.header.frame_id = static_cast<std::string>(it->second["frame_id"]);

  if (!isArray(it->second["orientation"], 3, "orientation", "RPY values"))
    return false;
  auto& rpy = it->second["orientation"];
  tf2::Quaternion q;
  q.setRPY(parseDouble(rpy[0]), parseDouble(rpy[1]), parseDouble(rpy[2]));
  pose.pose.orientation = toMsg(q);

  if (!isArray(it->second["position"], 3, "position", "xyz position"))
    return false;
  pose.pose.position.x = parseDouble(it->second["position"][0]);
  pose.pose.position.y = parseDouble(it->second["position"][1]);
  pose.pose.position.z = parseDouble(it->second["position"][2]);

  return true;
}

static bool constructConstraint(XmlRpc::XmlRpcValue& params, moveit_msgs::JointConstraint& constraint)
{
  for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
  {
    if (it->first == "type")
      continue;
    else if (it->first == "joint_name")
      constraint.joint_name = static_cast<std::string>(it->second);
    else if (it->first == "weight")
      constraint.weight = parseDouble(it->second);
    else if (it->first == "position")
    {
      constraint.position = parseDouble(it->second);
    }
    else if (it->first == "tolerance")
    {
      constraint.tolerance_below = parseDouble(it->second);
      constraint.tolerance_above = parseDouble(it->second);
    }
    else if (it->first == "tolerances")
    {
      if (!isArray(it->second, 2, it->first, "lower/upper tolerances"))
        return false;

      constraint.tolerance_below = parseDouble(it->second[0]);
      constraint.tolerance_above = parseDouble(it->second[1]);
    }
    else if (it->first == "bounds")
    {
      if (!isArray(it->second, 2, it->first, "lower/upper bound"))
        return false;

      const double lower_bound = parseDouble(it->second[0]);
      const double upper_bound = parseDouble(it->second[1]);

      constraint.position = (lower_bound + upper_bound) / 2;
      constraint.tolerance_below = constraint.position - lower_bound;
      constraint.tolerance_above = upper_bound - constraint.position;
    }
    else
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "joint constraint contains unknown entity '" << it->first << "'");
    }
  }
  return true;
}

static bool constructConstraint(XmlRpc::XmlRpcValue& params, moveit_msgs::PositionConstraint& constraint)
{
  for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
  {
    if (it->first == "type")
      continue;
    else if (it->first == "frame_id")
      constraint.header.frame_id = static_cast<std::string>(it->second);
    else if (it->first == "weight")
      constraint.weight = parseDouble(it->second);
    else if (it->first == "link_name")
      constraint.link_name = static_cast<std::string>(it->second);
    else if (it->first == "target_offset")
    {
      if (!isArray(it->second, 3, it->first, "x/y/z position"))
        return false;

      constraint.target_point_offset.x = parseDouble(it->second[0]);
      constraint.target_point_offset.y = parseDouble(it->second[1]);
      constraint.target_point_offset.z = parseDouble(it->second[2]);
    }
    else if (it->first == "region")
    {
      if (!isStruct(it->second, { "x", "y", "z" }, "region"))
        return false;

      constraint.constraint_region.primitive_poses.emplace_back();
      constraint.constraint_region.primitives.emplace_back();

      geometry_msgs::Pose& region_pose = constraint.constraint_region.primitive_poses.back();
      shape_msgs::SolidPrimitive& region_primitive = constraint.constraint_region.primitives.back();

      region_primitive.type = shape_msgs::SolidPrimitive::BOX;
      region_primitive.dimensions.resize(3);

      std::function<void(XmlRpc::XmlRpcValue&, double&, double&)> parse_dimension =
          [](XmlRpc::XmlRpcValue& it, double& center, double& dimension) {
            center = (parseDouble(it[0]) + parseDouble(it[1])) / 2;
            dimension = parseDouble(it[1]) - parseDouble(it[0]);
          };

      parse_dimension(it->second["x"], region_pose.position.x,
                      region_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X]);
      parse_dimension(it->second["y"], region_pose.position.y,
                      region_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
      parse_dimension(it->second["z"], region_pose.position.z,
                      region_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

      region_pose.orientation.w = 1.0;
    }
    else
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "position constraint contains unknown entity '" << it->first << "'");
    }
  }
  return true;
}

static bool constructConstraint(XmlRpc::XmlRpcValue& params, moveit_msgs::OrientationConstraint& constraint)
{
  for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
  {
    if (it->first == "type")
      continue;
    else if (it->first == "frame_id")
      constraint.header.frame_id = static_cast<std::string>(it->second);
    else if (it->first == "weight")
      constraint.weight = parseDouble(it->second);
    else if (it->first == "link_name")
      constraint.link_name = static_cast<std::string>(it->second);
    else if (it->first == "orientation")
    {
      if (!isArray(it->second, 3, it->first, "RPY values"))
        return false;

      tf2::Quaternion q;
      q.setRPY(parseDouble(it->second[0]), parseDouble(it->second[1]), parseDouble(it->second[2]));
      constraint.orientation = toMsg(q);
    }
    else if (it->first == "tolerances")
    {
      if (!isArray(it->second, 3, it->first, "xyz tolerances"))
        return false;

      constraint.absolute_x_axis_tolerance = parseDouble(it->second[0]);
      constraint.absolute_y_axis_tolerance = parseDouble(it->second[1]);
      constraint.absolute_z_axis_tolerance = parseDouble(it->second[2]);
    }
    else
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "orientation constraint contains unknown entity '" << it->first << "'");
    }
  }
  return true;
}

static bool constructConstraint(XmlRpc::XmlRpcValue& params, moveit_msgs::VisibilityConstraint& constraint)
{
  for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
  {
    if (it->first == "type")
      continue;
    else if (it->first == "weight")
      constraint.weight = parseDouble(it->second);
    else if (it->first == "target_radius")
      constraint.target_radius = parseDouble(it->second);
    else if (it->first == "target_pose")
    {
      if (!constructPoseStamped(it, constraint.target_pose))
        return false;
    }
    else if (it->first == "cone_sides")
      constraint.cone_sides = static_cast<int>(it->second);
    else if (it->first == "sensor_pose")
    {
      if (!constructPoseStamped(it, constraint.sensor_pose))
        return false;
    }
    else if (it->first == "max_view_angle")
      constraint.max_view_angle = parseDouble(it->second);
    else if (it->first == "max_range_angle")
      constraint.max_range_angle = parseDouble(it->second);
    else
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, "orientation constraint contains unknown entity '" << it->first << "'");
    }
  }

  constraint.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_X;

  return true;
}

static bool collectConstraints(XmlRpc::XmlRpcValue& params, moveit_msgs::Constraints& constraints)
{
  if (params.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED(LOGNAME, "expected constraints as array");
    return false;
  }

  for (int i = 0; i < params.size(); ++i)  // NOLINT(modernize-loop-convert)
  {
    if (!params[i].hasMember("type"))
    {
      ROS_ERROR_NAMED(LOGNAME, "constraint parameter does not specify its type");
    }
    else if (params[i]["type"] == "joint")
    {
      constraints.joint_constraints.emplace_back();
      if (!constructConstraint(params[i], constraints.joint_constraints.back()))
        return false;
    }
    else if (params[i]["type"] == "position")
    {
      constraints.position_constraints.emplace_back();
      if (!constructConstraint(params[i], constraints.position_constraints.back()))
        return false;
    }
    else if (params[i]["type"] == "orientation")
    {
      constraints.orientation_constraints.emplace_back();
      if (!constructConstraint(params[i], constraints.orientation_constraints.back()))
        return false;
    }
    else if (params[i]["type"] == "visibility")
    {
      constraints.visibility_constraints.emplace_back();
      if (!constructConstraint(params[i], constraints.visibility_constraints.back()))
        return false;
    }
  }

  return true;
}

bool constructConstraints(XmlRpc::XmlRpcValue& params, moveit_msgs::Constraints& constraints)
{
  if (!isStruct(params, { "name", "constraints" }, "Parameter"))
    return false;

  constraints.name = static_cast<std::string>(params["name"]);
  return collectConstraints(params["constraints"], constraints);
}
}  // namespace kinematic_constraints

bool kinematic_constraints::resolveConstraintFrames(const moveit::core::RobotState& state,
                                                    moveit_msgs::Constraints& constraints)
{
  for (auto& c : constraints.position_constraints)
  {
    bool frame_found;
    const moveit::core::LinkModel* robot_link;
    const Eigen::Isometry3d& transform = state.getFrameInfo(c.link_name, robot_link, frame_found);
    if (!frame_found)
      return false;

    // If the frame of the constraint is not part of the robot link model (but an attached body or subframe),
    // the constraint needs to be expressed in the frame of a robot link.
    if (c.link_name != robot_link->getName())
    {
      Eigen::Isometry3d robot_link_to_link_name = state.getGlobalLinkTransform(robot_link).inverse() * transform;
      Eigen::Vector3d offset_link_name(c.target_point_offset.x, c.target_point_offset.y, c.target_point_offset.z);
      Eigen::Vector3d offset_robot_link = robot_link_to_link_name * offset_link_name;

      c.link_name = robot_link->getName();
      tf2::toMsg(offset_robot_link, c.target_point_offset);
    }
  }

  for (auto& c : constraints.orientation_constraints)
  {
    bool frame_found;
    const moveit::core::LinkModel* robot_link;
    // getFrameInfo() returns a valid isometry by contract
    const Eigen::Isometry3d& transform = state.getFrameInfo(c.link_name, robot_link, frame_found);
    if (!frame_found)
      return false;

    // If the frame of the constraint is not part of the robot link model (but an attached body or subframe),
    // the constraint needs to be expressed in the frame of a robot link.
    if (c.link_name != robot_link->getName())
    {
      c.link_name = robot_link->getName();
      Eigen::Quaterniond link_name_to_robot_link(transform.linear().transpose() *
                                                 state.getGlobalLinkTransform(robot_link).linear());
      Eigen::Quaterniond quat_target;
      tf2::fromMsg(c.orientation, quat_target);
      c.orientation = tf2::toMsg(quat_target * link_name_to_robot_link);
    }
  }
  return true;
}
