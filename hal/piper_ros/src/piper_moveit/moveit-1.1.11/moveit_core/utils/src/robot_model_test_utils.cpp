/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bryce Willey.
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
 *   * Neither the name of MoveIt nor the names of its
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

/* Author: Bryce Willey */

#include <ros/ros.h>
#include <boost/algorithm/string_regex.hpp>
#include <boost/math/constants/constants.hpp>
#include <geometry_msgs/Pose.h>

#include <urdf_parser/urdf_parser.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <pluginlib/class_loader.hpp>
#include <ros/package.h>

namespace moveit
{
namespace core
{
const std::string LOGNAME = "robot_model_builder";

moveit::core::RobotModelPtr loadTestingRobotModel(const std::string& robot_name)
{
  urdf::ModelInterfaceSharedPtr urdf = loadModelInterface(robot_name);
  srdf::ModelSharedPtr srdf = loadSRDFModel(robot_name);
  moveit::core::RobotModelPtr robot_model(new moveit::core::RobotModel(urdf, srdf));
  return robot_model;
}

urdf::ModelInterfaceSharedPtr loadModelInterface(const std::string& robot_name)
{
  std::string urdf_path;
  if (robot_name == "pr2")
  {
    urdf_path = ros::package::getPath("moveit_resources_pr2_description") + "/urdf/robot.xml";
  }
  else
  {
    urdf_path =
        ros::package::getPath("moveit_resources_" + robot_name + "_description") + "/urdf/" + robot_name + ".urdf";
  }
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_path);
  if (urdf_model == nullptr)
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot find URDF for %s. Make sure moveit_resources/your robot description is installed",
                    robot_name.c_str());
  }
  return urdf_model;
}

srdf::ModelSharedPtr loadSRDFModel(const std::string& robot_name)
{
  urdf::ModelInterfaceSharedPtr urdf_model = loadModelInterface(robot_name);
  srdf::ModelSharedPtr srdf_model(new srdf::Model());
  std::string srdf_path;
  if (robot_name == "pr2")
  {
    srdf_path = ros::package::getPath("moveit_resources_pr2_description") + "/srdf/robot.xml";
  }
  else
  {
    srdf_path =
        ros::package::getPath("moveit_resources_" + robot_name + "_moveit_config") + "/config/" + robot_name + ".srdf";
  }
  srdf_model->initFile(*urdf_model, srdf_path);
  return srdf_model;
}

void loadIKPluginForGroup(JointModelGroup* jmg, const std::string& base_link, const std::string& tip_link,
                          std::string plugin, double timeout)
{
  using LoaderType = pluginlib::ClassLoader<kinematics::KinematicsBase>;
  static std::weak_ptr<LoaderType> cached_loader;
  std::shared_ptr<LoaderType> loader = cached_loader.lock();
  if (!loader)
  {
    loader = std::make_shared<LoaderType>("moveit_core", "kinematics::KinematicsBase");
    cached_loader = loader;
  }

  // translate short to long names
  if (plugin == "KDL")
    plugin = "kdl_kinematics_plugin/KDLKinematicsPlugin";

  jmg->setSolverAllocators(
      [=](const JointModelGroup* jmg) -> kinematics::KinematicsBasePtr {
        kinematics::KinematicsBasePtr result = loader->createUniqueInstance(plugin);
        result->initialize(jmg->getParentModel(), jmg->getName(), base_link, { tip_link }, 0.0);
        result->setDefaultTimeout(timeout);
        return result;
      },
      SolverAllocatorMapFn());
}

RobotModelBuilder::RobotModelBuilder(const std::string& name, const std::string& base_link_name)
  : urdf_model_(new urdf::ModelInterface()), srdf_writer_(new srdf::SRDFWriter())
{
  urdf_model_->clear();
  urdf_model_->name_ = name;

  urdf::LinkSharedPtr base_link(new urdf::Link);
  base_link->name = base_link_name;
  urdf_model_->links_.insert(std::make_pair(base_link_name, base_link));

  srdf_writer_->robot_name_ = name;
}

void RobotModelBuilder::addChain(const std::string& section, const std::string& type,
                                 const std::vector<geometry_msgs::Pose>& joint_origins, urdf::Vector3 joint_axis)
{
  std::vector<std::string> link_names;
  boost::split_regex(link_names, section, boost::regex("->"));
  if (link_names.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "No links specified (empty section?)");
    is_valid_ = false;
    return;
  }
  // First link should already be added.
  if (!urdf_model_->getLink(link_names[0]))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link %s not present in builder yet!", link_names[0].c_str());
    is_valid_ = false;
    return;
  }

  if (!joint_origins.empty() && link_names.size() - 1 != joint_origins.size())
  {
    ROS_ERROR_NAMED(LOGNAME, "There should be one more link (%zu) than there are joint origins (%zu)",
                    link_names.size(), joint_origins.size());
    is_valid_ = false;
    return;
  }

  // Iterate through each link.
  for (size_t i = 1; i < link_names.size(); ++i)
  {
    // These links shouldn't be present already.
    if (urdf_model_->getLink(link_names[i]))
    {
      ROS_ERROR_NAMED(LOGNAME, "Link %s is already specified", link_names[i].c_str());
      is_valid_ = false;
      return;
    }
    urdf::LinkSharedPtr link(new urdf::Link);
    link->name = link_names[i];
    urdf_model_->links_.insert(std::make_pair(link_names[i], link));
    urdf::JointSharedPtr joint(new urdf::Joint);
    joint->name = link_names[i - 1] + "-" + link_names[i] + "-joint";
    // Default to Identity transform for origins.
    joint->parent_to_joint_origin_transform.clear();
    if (!joint_origins.empty())
    {
      geometry_msgs::Pose o = joint_origins[i - 1];
      joint->parent_to_joint_origin_transform.position = urdf::Vector3(o.position.x, o.position.y, o.position.z);
      joint->parent_to_joint_origin_transform.rotation =
          urdf::Rotation(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w);
    }

    joint->parent_link_name = link_names[i - 1];
    joint->child_link_name = link_names[i];
    if (type == "planar")
      joint->type = urdf::Joint::PLANAR;
    else if (type == "floating")
      joint->type = urdf::Joint::FLOATING;
    else if (type == "revolute")
      joint->type = urdf::Joint::REVOLUTE;
    else if (type == "continuous")
      joint->type = urdf::Joint::CONTINUOUS;
    else if (type == "prismatic")
      joint->type = urdf::Joint::PRISMATIC;
    else if (type == "fixed")
      joint->type = urdf::Joint::FIXED;
    else
    {
      ROS_ERROR_NAMED(LOGNAME, "No such joint type as %s", type.c_str());
      is_valid_ = false;
      return;
    }

    joint->axis = joint_axis;
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC)
    {
      urdf::JointLimitsSharedPtr limits(new urdf::JointLimits);
      limits->lower = -boost::math::constants::pi<double>();
      limits->upper = boost::math::constants::pi<double>();

      joint->limits = limits;
    }
    urdf_model_->joints_.insert(std::make_pair(joint->name, joint));
  }
}

void RobotModelBuilder::addInertial(const std::string& link_name, double mass, geometry_msgs::Pose origin, double ixx,
                                    double ixy, double ixz, double iyy, double iyz, double izz)
{
  if (!urdf_model_->getLink(link_name))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link %s not present in builder yet!", link_name.c_str());
    is_valid_ = false;
    return;
  }

  urdf::InertialSharedPtr inertial(new urdf::Inertial);
  inertial->origin.position = urdf::Vector3(origin.position.x, origin.position.y, origin.position.z);
  inertial->origin.rotation =
      urdf::Rotation(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);
  inertial->mass = mass;
  inertial->ixx = ixx;
  inertial->ixy = ixy;
  inertial->ixz = ixz;
  inertial->iyy = iyy;
  inertial->iyz = iyz;
  inertial->izz = izz;

  urdf::LinkSharedPtr link;
  urdf_model_->getLink(link_name, link);
  link->inertial = inertial;
}

void RobotModelBuilder::addVisualBox(const std::string& link_name, const std::vector<double>& size,
                                     geometry_msgs::Pose origin)
{
  urdf::VisualSharedPtr vis(new urdf::Visual);
  urdf::BoxSharedPtr geometry(new urdf::Box);
  geometry->dim = urdf::Vector3(size[0], size[1], size[2]);
  vis->geometry = geometry;
  addLinkVisual(link_name, vis, origin);
}

void RobotModelBuilder::addCollisionBox(const std::string& link_name, const std::vector<double>& dims,
                                        geometry_msgs::Pose origin)
{
  if (dims.size() != 3)
  {
    ROS_ERROR("There can only be 3 dimensions of a box (given %zu!)", dims.size());
    is_valid_ = false;
    return;
  }
  urdf::CollisionSharedPtr coll(new urdf::Collision);
  urdf::BoxSharedPtr geometry(new urdf::Box);
  geometry->dim = urdf::Vector3(dims[0], dims[1], dims[2]);
  coll->geometry = geometry;
  addLinkCollision(link_name, coll, origin);
}

void RobotModelBuilder::addCollisionMesh(const std::string& link_name, const std::string& filename,
                                         geometry_msgs::Pose origin)
{
  urdf::CollisionSharedPtr coll(new urdf::Collision);
  urdf::MeshSharedPtr geometry(new urdf::Mesh);
  geometry->filename = filename;
  coll->geometry = geometry;
  addLinkCollision(link_name, coll, origin);
}

void RobotModelBuilder::addLinkCollision(const std::string& link_name, const urdf::CollisionSharedPtr& collision,
                                         geometry_msgs::Pose origin)
{
  if (!urdf_model_->getLink(link_name))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link %s not present in builder yet!", link_name.c_str());
    is_valid_ = false;
    return;
  }
  collision->origin.position = urdf::Vector3(origin.position.x, origin.position.y, origin.position.z);
  collision->origin.rotation =
      urdf::Rotation(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);

  urdf::LinkSharedPtr link;
  urdf_model_->getLink(link_name, link);
  link->collision_array.push_back(collision);
}

void RobotModelBuilder::addLinkVisual(const std::string& link_name, const urdf::VisualSharedPtr& vis,
                                      geometry_msgs::Pose origin)
{
  if (!urdf_model_->getLink(link_name))
  {
    ROS_ERROR_NAMED(LOGNAME, "Link %s not present in builder yet!", link_name.c_str());
    is_valid_ = false;
    return;
  }
  vis->origin.position = urdf::Vector3(origin.position.x, origin.position.y, origin.position.z);
  vis->origin.rotation =
      urdf::Rotation(origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);

  urdf::LinkSharedPtr link;
  urdf_model_->getLink(link_name, link);
  if (!link->visual_array.empty())
  {
    link->visual_array.push_back(vis);
  }
  else if (link->visual)
  {
    link->visual_array.push_back(link->visual);
    link->visual.reset();
    link->visual_array.push_back(vis);
  }
  else
  {
    link->visual = vis;
  }
}

void RobotModelBuilder::addVirtualJoint(const std::string& parent_frame, const std::string& child_link,
                                        const std::string& type, const std::string& name)
{
  srdf::Model::VirtualJoint new_virtual_joint;
  if (name.empty())
    new_virtual_joint.name_ = parent_frame + "-" + child_link + "-virtual_joint";
  else
    new_virtual_joint.name_ = name;
  new_virtual_joint.type_ = type;
  new_virtual_joint.parent_frame_ = parent_frame;
  new_virtual_joint.child_link_ = child_link;
  srdf_writer_->virtual_joints_.push_back(new_virtual_joint);
}

void RobotModelBuilder::addGroupChain(const std::string& base_link, const std::string& tip_link, const std::string& name)
{
  srdf::Model::Group new_group;
  if (name.empty())
    new_group.name_ = base_link + "-" + tip_link + "-chain-group";
  else
    new_group.name_ = name;
  new_group.chains_.push_back(std::make_pair(base_link, tip_link));
  srdf_writer_->groups_.push_back(new_group);
}

void RobotModelBuilder::addGroup(const std::vector<std::string>& links, const std::vector<std::string>& joints,
                                 const std::string& name)
{
  srdf::Model::Group new_group;
  new_group.name_ = name;
  new_group.links_ = links;
  new_group.joints_ = joints;
  srdf_writer_->groups_.push_back(new_group);
}

void RobotModelBuilder::addEndEffector(const std::string& name, const std::string& parent_link,
                                       const std::string& parent_group, const std::string& component_group)
{
  srdf::Model::EndEffector eef;
  eef.name_ = name;
  eef.parent_link_ = parent_link;
  eef.parent_group_ = parent_group;
  eef.component_group_ = component_group;
  srdf_writer_->end_effectors_.push_back(eef);
}

bool RobotModelBuilder::isValid()
{
  return is_valid_;
}

moveit::core::RobotModelPtr RobotModelBuilder::build()
{
  moveit::core::RobotModelPtr robot_model;
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  try
  {
    urdf_model_->initTree(parent_link_tree);
  }
  catch (urdf::ParseError& e)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to build tree: %s", e.what());
    return robot_model;
  }

  // find the root link
  try
  {
    urdf_model_->initRoot(parent_link_tree);
  }
  catch (urdf::ParseError& e)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to find root link: %s", e.what());
    return robot_model;
  }
  srdf_writer_->updateSRDFModel(*urdf_model_);
  robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_writer_->srdf_model_);
  return robot_model;
}
}  // namespace core
}  // namespace moveit
