/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include "pilz_industrial_motion_planner_testutils/xml_testdata_loader.h"

#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/optional.hpp>

#include "pilz_industrial_motion_planner_testutils/default_values.h"
#include "pilz_industrial_motion_planner_testutils/exception_types.h"
#include "pilz_industrial_motion_planner_testutils/xml_constants.h"

namespace pt = boost::property_tree;
namespace pilz_industrial_motion_planner_testutils
{
class CmdReader
{
public:
  CmdReader(const pt::ptree::value_type& node) : cmd_node_(node)
  {
  }

public:
  std::string getPlanningGroup() const;
  std::string getTargetLink() const;
  std::string getStartPoseName() const;
  std::string getEndPoseName() const;

  double getVelocityScale() const;
  double getAccelerationScale() const;

  CmdReader& setDefaultVelocityScale(double scale);
  CmdReader& setDefaultAccelerationScale(double scale);

private:
  const pt::ptree::value_type& cmd_node_;

  double default_velocity_scale_{ DEFAULT_VEL };
  double default_acceleration_scale_{ DEFAULT_ACC };
};

inline std::string CmdReader::getPlanningGroup() const
{
  return cmd_node_.second.get<std::string>(PLANNING_GROUP_STR);
}

inline std::string CmdReader::getTargetLink() const
{
  return cmd_node_.second.get<std::string>(TARGET_LINK_STR);
}

inline std::string CmdReader::getStartPoseName() const
{
  return cmd_node_.second.get<std::string>(START_POS_STR);
}

inline std::string CmdReader::getEndPoseName() const
{
  return cmd_node_.second.get<std::string>(END_POS_STR);
}

inline double CmdReader::getVelocityScale() const
{
  return cmd_node_.second.get<double>(VEL_STR, default_velocity_scale_);
}

inline double CmdReader::getAccelerationScale() const
{
  return cmd_node_.second.get<double>(ACC_STR, default_acceleration_scale_);
}

inline CmdReader& CmdReader::setDefaultVelocityScale(double scale)
{
  default_velocity_scale_ = scale;
  return *this;
}

inline CmdReader& CmdReader::setDefaultAccelerationScale(double scale)
{
  default_acceleration_scale_ = scale;
  return *this;
}

template <class CmdType>
class CmdGetterAdapter : public XmlTestdataLoader::AbstractCmdGetterAdapter
{
public:
  using FuncType = std::function<CmdType(const std::string&)>;

  CmdGetterAdapter(FuncType func) : AbstractCmdGetterAdapter(), func_(func)
  {
  }

public:
  CmdVariant getCmd(const std::string& cmd_name) const override
  {
    return CmdVariant(func_(cmd_name));
  }

private:
  FuncType func_;
};

XmlTestdataLoader::XmlTestdataLoader(const std::string& path_filename) : TestdataLoader(), path_filename_(path_filename)
{
  // Parse the XML into the property tree.
  pt::read_xml(path_filename_, tree_, pt::xml_parser::no_comments);

  using std::placeholders::_1;
  cmd_getter_funcs_["ptp"] = AbstractCmdGetterUPtr(
      new CmdGetterAdapter<PtpJoint>([this](const std::string& cmd) { return getPtpJoint(cmd); }));
  cmd_getter_funcs_["ptp_joint_cart"] = AbstractCmdGetterUPtr(
      new CmdGetterAdapter<PtpJointCart>([this](const std::string& cmd) { return getPtpJointCart(cmd); }));
  cmd_getter_funcs_["ptp_cart_cart"] =
      AbstractCmdGetterUPtr(new CmdGetterAdapter<PtpCart>([this](const std::string& cmd) { return getPtpCart(cmd); }));

  cmd_getter_funcs_["lin"] = AbstractCmdGetterUPtr(
      new CmdGetterAdapter<LinJoint>([this](const std::string& cmd) { return getLinJoint(cmd); }));
  cmd_getter_funcs_["lin_cart"] =
      AbstractCmdGetterUPtr(new CmdGetterAdapter<LinCart>([this](const std::string& cmd) { return getLinCart(cmd); }));

  cmd_getter_funcs_["circ_center_cart"] = AbstractCmdGetterUPtr(
      new CmdGetterAdapter<CircCenterCart>([this](const std::string& cmd) { return getCircCartCenterCart(cmd); }));
  cmd_getter_funcs_["circ_interim_cart"] = AbstractCmdGetterUPtr(
      new CmdGetterAdapter<CircInterimCart>([this](const std::string& cmd) { return getCircCartInterimCart(cmd); }));
  cmd_getter_funcs_["circ_joint_interim_cart"] = AbstractCmdGetterUPtr(new CmdGetterAdapter<CircJointInterimCart>(
      [this](const std::string& cmd) { return getCircJointInterimCart(cmd); }));

  cmd_getter_funcs_["gripper"] =
      AbstractCmdGetterUPtr(new CmdGetterAdapter<Gripper>([this](const std::string& cmd) { return getGripper(cmd); }));
}

XmlTestdataLoader::XmlTestdataLoader(const std::string& path_filename,
                                     const moveit::core::RobotModelConstPtr& robot_model)
  : XmlTestdataLoader(path_filename)
{
  setRobotModel(robot_model);
}

XmlTestdataLoader::~XmlTestdataLoader()
{
}

const pt::ptree::value_type& XmlTestdataLoader::findNodeWithName(const boost::property_tree::ptree& tree,
                                                                 const std::string& name, const std::string& key,
                                                                 const std::string& path) const
{
  std::string path_str{ (path.empty() ? NAME_PATH_STR : path) };

  // Search for node with given name.
  for (const pt::ptree::value_type& val : tree)
  {
    // Ignore attributes which are always the first element of a tree.
    if (val.first == XML_ATTR_STR)
    {
      continue;
    }

    if (val.first != key)
    {
      continue;
    }

    const auto& node{ val.second.get_child(path_str, empty_tree_) };
    if (node == empty_tree_)
    {
      break;
    }
    if (node.data() == name)
    {
      return val;
    }
  }

  std::string msg;
  msg.append("Node of type \"")
      .append(key)
      .append("\" with ")
      .append(path_str)
      .append("=\"")
      .append(name)
      .append("\" "
              "not "
              "foun"
              "d.");
  throw TestDataLoaderReadingException(msg);
}

JointConfiguration XmlTestdataLoader::getJoints(const std::string& pos_name, const std::string& group_name) const
{
  // Search for node with given name.
  const auto& poses_tree{ tree_.get_child(POSES_PATH_STR, empty_tree_) };
  if (poses_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No poses found.");
  }
  return getJoints(findNodeWithName(poses_tree, pos_name, POSE_STR).second, group_name);
}

JointConfiguration XmlTestdataLoader::getJoints(const boost::property_tree::ptree& joints_tree,
                                                const std::string& group_name) const
{
  // Search joints node with given group_name.
  if (joints_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No joints found.");
  }
  const auto& joint_node{ findNodeWithName(joints_tree, group_name, JOINT_STR, GROUP_NAME_PATH_STR) };

  std::vector<std::string> strs;
  boost::split(strs, joint_node.second.data(), boost::is_any_of(" "));
  return JointConfiguration(group_name, strVec2doubleVec(strs), robot_model_);
}

CartesianConfiguration XmlTestdataLoader::getPose(const std::string& pos_name, const std::string& group_name) const
{
  const auto& all_poses_tree{ tree_.get_child(POSES_PATH_STR, empty_tree_) };
  if (all_poses_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No poses found.");
  }
  const auto& pose_tree{ findNodeWithName(all_poses_tree, pos_name, POSE_STR).second };
  const auto& xyz_quat_tree{ findNodeWithName(pose_tree, group_name, XYZ_QUAT_STR, GROUP_NAME_PATH_STR).second };
  const boost::property_tree::ptree& link_name_attr{ xyz_quat_tree.get_child(LINK_NAME_PATH_STR, empty_tree_) };
  if (link_name_attr == empty_tree_)
  {
    throw TestDataLoaderReadingException("No link name found.");
  }

  // Get rid of things like "\n", etc.
  std::string data{ xyz_quat_tree.data() };
  boost::trim(data);

  std::vector<std::string> pos_ori_str;
  boost::split(pos_ori_str, data, boost::is_any_of(" "));
  CartesianConfiguration cart_config{ CartesianConfiguration(group_name, link_name_attr.data(),
                                                             strVec2doubleVec(pos_ori_str), robot_model_) };

  const auto& seeds_tree{ xyz_quat_tree.get_child(SEED_STR, empty_tree_) };
  if (seeds_tree != empty_tree_)
  {
    cart_config.setSeed(getJoints(seeds_tree, group_name));
  }
  return cart_config;
}

PtpJoint XmlTestdataLoader::getPtpJoint(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, PTPS_PATH_STR, PTP_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  PtpJoint cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getJoints(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

PtpJointCart XmlTestdataLoader::getPtpJointCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, PTPS_PATH_STR, PTP_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  PtpJointCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getPose(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

PtpCart XmlTestdataLoader::getPtpCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, PTPS_PATH_STR, PTP_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  PtpCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getPose(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getPose(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

LinJoint XmlTestdataLoader::getLinJoint(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, LINS_PATH_STR, LIN_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  LinJoint cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getJoints(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

LinCart XmlTestdataLoader::getLinCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, LINS_PATH_STR, LIN_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  LinCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getPose(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getPose(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

LinJointCart XmlTestdataLoader::getLinJointCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, LINS_PATH_STR, LIN_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  LinJointCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getPose(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

const pt::ptree::value_type& XmlTestdataLoader::findCmd(const std::string& cmd_name, const std::string& cmd_path,
                                                        const std::string& cmd_key) const
{
  // Search for node with given name.
  const boost::property_tree::ptree& cmds_tree{ tree_.get_child(cmd_path, empty_tree_) };
  if (cmds_tree == empty_tree_)
  {
    throw TestDataLoaderReadingException("No list of commands of type \"" + cmd_key + "\" found");
  }

  return findNodeWithName(cmds_tree, cmd_name, cmd_key);
}

CartesianCenter XmlTestdataLoader::getCartesianCenter(const std::string& cmd_name,
                                                      const std::string& planning_group) const
{
  const pt::ptree::value_type& cmd_node{ findCmd(cmd_name, CIRCS_PATH_STR, CIRC_STR) };
  std::string aux_pos_name;
  try
  {
    aux_pos_name = cmd_node.second.get<std::string>(CENTER_POS_STR);
  }
  catch (...)
  {
    throw TestDataLoaderReadingException("Did not find center of circ");
  }

  CartesianCenter aux;
  aux.setConfiguration(getPose(aux_pos_name, planning_group));
  return aux;
}

CartesianInterim XmlTestdataLoader::getCartesianInterim(const std::string& cmd_name,
                                                        const std::string& planning_group) const
{
  const pt::ptree::value_type& cmd_node{ findCmd(cmd_name, CIRCS_PATH_STR, CIRC_STR) };
  std::string aux_pos_name;
  try
  {
    aux_pos_name = cmd_node.second.get<std::string>(INTERMEDIATE_POS_STR);
  }
  catch (...)
  {
    throw TestDataLoaderReadingException("Did not find interim of circ");
  }

  CartesianInterim aux;
  aux.setConfiguration(getPose(aux_pos_name, planning_group));
  return aux;
}

CircCenterCart XmlTestdataLoader::getCircCartCenterCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, CIRCS_PATH_STR, CIRC_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  CircCenterCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getPose(cmd_reader.getStartPoseName(), planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianCenter(cmd_name, planning_group));
  cmd.setGoalConfiguration(getPose(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

CircInterimCart XmlTestdataLoader::getCircCartInterimCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, CIRCS_PATH_STR, CIRC_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  CircInterimCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getPose(cmd_reader.getStartPoseName(), planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianInterim(cmd_name, planning_group));
  cmd.setGoalConfiguration(getPose(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

CircJointInterimCart XmlTestdataLoader::getCircJointInterimCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, CIRCS_PATH_STR, CIRC_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  CircJointInterimCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianInterim(cmd_name, planning_group));
  cmd.setGoalConfiguration(getJoints(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

CircJointCenterCart XmlTestdataLoader::getCircJointCenterCart(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, CIRCS_PATH_STR, CIRC_STR) };
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  CircJointCenterCart cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setAuxiliaryConfiguration(getCartesianCenter(cmd_name, planning_group));
  cmd.setGoalConfiguration(getJoints(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

Sequence XmlTestdataLoader::getSequence(const std::string& cmd_name) const
{
  Sequence seq;

  // Find sequence with given name and loop over all its cmds
  const auto& sequence_cmd_tree{ findCmd(cmd_name, SEQUENCE_PATH_STR, BLEND_STR).second };
  for (const pt::ptree::value_type& seq_cmd : sequence_cmd_tree)
  {
    // Ignore attributes which are always the first element of a tree.
    if (seq_cmd.first == XML_ATTR_STR)
    {
      continue;
    }

    // Get name of blend cmd.
    const boost::property_tree::ptree& cmd_name_attr = seq_cmd.second.get_child(NAME_PATH_STR, empty_tree_);
    if (cmd_name_attr == empty_tree_)
    {
      throw TestDataLoaderReadingException("Did not find name of sequence cmd");
    }

    const std::string& cmd_name{ cmd_name_attr.data() };

    // Get type of blend cmd
    const boost::property_tree::ptree& type_name_attr{ seq_cmd.second.get_child(CMD_TYPE_PATH_STR, empty_tree_) };
    if (type_name_attr == empty_tree_)
    {
      throw TestDataLoaderReadingException("Did not find type of sequence cmd \"" + cmd_name + "\"");
    }
    const std::string& cmd_type{ type_name_attr.data() };

    // Get blend radius of blend cmd.
    double blend_radius{ seq_cmd.second.get<double>(BLEND_RADIUS_PATH_STR, DEFAULT_BLEND_RADIUS) };

    // Read current command from test data + Add command to sequence
    seq.add(cmd_getter_funcs_.at(cmd_type)->getCmd(cmd_name), blend_radius);
  }

  return seq;
}

Gripper XmlTestdataLoader::getGripper(const std::string& cmd_name) const
{
  CmdReader cmd_reader{ findCmd(cmd_name, GRIPPERS_PATH_STR, GRIPPER_STR) };
  cmd_reader.setDefaultVelocityScale(DEFAULT_VEL_GRIPPER);
  cmd_reader.setDefaultAccelerationScale(DEFAULT_ACC_GRIPPER);
  std::string planning_group{ cmd_reader.getPlanningGroup() };

  Gripper cmd;
  cmd.setPlanningGroup(planning_group);
  cmd.setVelocityScale(cmd_reader.getVelocityScale());
  cmd.setAccelerationScale(cmd_reader.getAccelerationScale());

  cmd.setStartConfiguration(getJoints(cmd_reader.getStartPoseName(), planning_group));
  cmd.setGoalConfiguration(getJoints(cmd_reader.getEndPoseName(), planning_group));

  return cmd;
}

}  // namespace pilz_industrial_motion_planner_testutils
