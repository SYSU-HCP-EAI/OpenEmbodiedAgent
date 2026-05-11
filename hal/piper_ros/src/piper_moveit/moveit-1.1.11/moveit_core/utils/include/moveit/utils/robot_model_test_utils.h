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
/** \brief convenience functions and classes used for making simple robot models for testing. */

#pragma once

#include <srdfdom/srdf_writer.h>
#include <urdf/model.h>
#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/Pose.h>

namespace moveit
{
namespace core
{
/** \brief Loads a robot from moveit_resources.
 * \param[in] robot_name The name of the robot in moveit_resources to load.
 *            This should be the prefix to many of the robot packages.
 *            For example, "pr2", "panda", or "fanuc".
 * \returns a RobotModel constructed from robot_name's URDF and SRDF.
 */
moveit::core::RobotModelPtr loadTestingRobotModel(const std::string& robot_name);

/** \brief Loads a URDF Model Interface from moveit_resources.
 * \param[in] robot_name The name of the robot in moveit_resources to load.
 *            This should be the prefix to many of the robot packages.
 *            For example, "pr2", "panda", or "fanuc".
 * \returns a ModelInterface constructed from robot_name's URDF.
 */
urdf::ModelInterfaceSharedPtr loadModelInterface(const std::string& robot_name);

/** \brief Loads an SRDF Model from moveit_resources.
 * \param[in] robot_name The name of the robot in moveit_resources to load.
 *            This should be the prefix to many of the robot packages.
 *            For example, "pr2", "panda", or "fanuc".
 * \returns an SRDF Model constructed from robot_name's URDF and SRDF.
 */
srdf::ModelSharedPtr loadSRDFModel(const std::string& robot_name);

/** \brief Load an IK solver plugin for the given joint model group
 * \param[in] jmg joint model group to load the plugin for
 * \param[in] base_link base link of chain
 * \param[in] tip_link tip link of chain
 * \param[in] plugin name of the plugin ("KDL", or full name)
 * \param[in] timeout default solver timeout
 */
void loadIKPluginForGroup(JointModelGroup* jmg, const std::string& base_link, const std::string& tip_link,
                          std::string plugin = "KDL", double timeout = 0.1);

/** \brief Easily build different robot models for testing.
 *  Essentially a programmer-friendly light wrapper around URDF and SRDF.
 *  Best shown by an example:
 *  \code{.cpp}
 *  RobotModelBuilder builder("my_robot", "base_link");
 *  builder.addChain("a->b->c", "continuous");
 *  builder.addGroup({"a", "b"}, {}, "example_group");
 *  ASSERT_TRUE(builder.isValid());
 *  RobotModelPtr model = builder.build();
 *  \endcode
 */
class RobotModelBuilder
{
public:
  /** \brief Constructor, takes the names of the robot and the base link.
   * \param[in] name The name of the robot, i.e. the 'name' attribute of the robot tag in URDF
   * \param[in] base_link_name The name of the root link of the robot. All other links should be descendants of this
   */
  RobotModelBuilder(const std::string& name, const std::string& base_link_name);

  /** \name URDF Functions
      \{ */

  /** \brief Adds a chain of links and joints to the builder.
   *  The joint names are generated automatically as "<parent>-<child>-joint".
   * \param[in] section A list of link names separated by "->". The first link should already be added to the build by
   * the time this function is called
   * \param[in] type The type of the joints connecting all of the given links, e.g. "revolute" or "continuous". All of
   * the joints will be given this type. To add multiple types of joints, call this method multiple times
   * \param[in] joint_origins The "parent to joint" origins for the joints connecting the links. If not used, all
   * origins will default to the identity transform
   * \param[in] joint_axis The joint axis specified in the joint frame defaults to (1,0,0)
   */
  void addChain(const std::string& section, const std::string& type,
                const std::vector<geometry_msgs::Pose>& joint_origins = {},
                urdf::Vector3 joint_axis = urdf::Vector3(1.0, 0.0, 0.0));

  /** \brief Adds a collision mesh to a specific link.
   *  \param[in] link_name The name of the link to which the mesh will be added. Must already be in the builder
   *  \param[in] filename The path to the mesh file, e.g.
   * "package://moveit_resources_pr2_description/urdf/meshes/base_v0/base_L.stl"
   *  \param[in] origin The origin pose of this collision mesh relative to the link origin
   */
  void addCollisionMesh(const std::string& link_name, const std::string& filename, geometry_msgs::Pose origin);
  /** \brief Adds a collision box to a specific link.
   *  \param[in] link_name The name of the link to which the box will be added. Must already be in the builder.
   *  \param[in] size The dimensions of the box
   *  \param[in] origin The origin pose of this collision box relative to the link origin
   */
  void addCollisionBox(const std::string& link_name, const std::vector<double>& dims, geometry_msgs::Pose origin);

  /** \brief Adds a visual box to a specific link.
   *  \param[in] link_name The name of the link to which the box will be added. Must already be in the builder.
   *  \param[in] size The dimensions of the box
   *  \param[in] origin The origin pose of this visual box relative to the link origin
   */
  void addVisualBox(const std::string& link_name, const std::vector<double>& size, geometry_msgs::Pose origin);

  /**
   * Adds an inertial component to a link.
   * \param[in] link_name The name of the link for this inertial information
   * \param[in] mass The mass of the link
   * \param[in] origin The origin center pose of the center of mass of this link
   */
  void addInertial(const std::string& link_name, double mass, geometry_msgs::Pose origin, double ixx, double ixy,
                   double ixz, double iyy, double iyz, double izz);

  /** \} */

  /** \name SRDF functions
      \{ */

  /** \brief Adds a virtual joint to the SRDF.
   *  \param[in] parent_frame The parent, e.g. "odom"
   *  \param[in] child_link The child link of this virtual joint, usually the base link
   *  \param[in] type The type of joint, can be "fixed", "floating", or "planar"
   *  \param[in] name The name of the virtual joint, if not given it's automatically made to be
   * "<parent_frame>-<child>-virtual-joint"
   */
  void addVirtualJoint(const std::string& parent_frame, const std::string& child_link, const std::string& type,
                       const std::string& name = "");

  /** \brief Adds a new group using a chain of links. The group is the parent joint of each link in the chain.
   *  \param[in] base_link The starting link of the chain
   *  \param[in] tip_link The ending link of the chain.
   *  \param[in] name The name of the group, if not given it's set as "<base>-<tip>-chain-group"
   */
  void addGroupChain(const std::string& base_link, const std::string& tip_link, const std::string& name = "");

  /** \brief Adds a new group using a list of links and a list of joints.
   *  \param[in] links The links (really their parent joints) to include in the group
   *  \param[in] joints The joints to include in the group
   *  \param[in] name The name of the group, required
   */
  void addGroup(const std::vector<std::string>& links, const std::vector<std::string>& joints, const std::string& name);

  void addEndEffector(const std::string& name, const std::string& parent_link, const std::string& parent_group = "",
                      const std::string& component_group = "");

  /** \} */

  /** \brief Returns true if the building process so far has been valid. */
  bool isValid();

  /** \brief Builds and returns the robot model added to the builder.
   */
  moveit::core::RobotModelPtr build();

private:
  /** \brief Adds different collision geometries to a link. */
  void addLinkCollision(const std::string& link_name, const urdf::CollisionSharedPtr& coll, geometry_msgs::Pose origin);

  /** \brief Adds different visual geometries to a link. */
  void addLinkVisual(const std::string& link_name, const urdf::VisualSharedPtr& vis, geometry_msgs::Pose origin);

  /// The URDF model, holds all of the URDF components of the robot added so far.
  urdf::ModelInterfaceSharedPtr urdf_model_;
  /// The SRDF model, holds all of the SRDF components of the robot added so far.
  srdf::SRDFWriterPtr srdf_writer_;

  /// Whether the current builder state is valid. If any 'add' method fails, this becomes false.
  bool is_valid_ = true;
};
}  // namespace core
}  // namespace moveit
