/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2013, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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

/* Author: Ioan Sucan, Adam Leeper */

#include "moveit/robot_interaction/robot_interaction.h"
#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_interaction/kinematic_options_map.h>
#include <moveit/transforms/transforms.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_interaction
{
static const float END_EFFECTOR_UNREACHABLE_COLOR[4] = { 1.0, 0.3, 0.3, 1.0 };
static const float END_EFFECTOR_REACHABLE_COLOR[4] = { 0.2, 1.0, 0.2, 1.0 };

const std::string RobotInteraction::INTERACTIVE_MARKER_TOPIC = "robot_interaction_interactive_marker_topic";

RobotInteraction::RobotInteraction(const moveit::core::RobotModelConstPtr& robot_model, const std::string& ns)
  : robot_model_(robot_model), kinematic_options_map_(new KinematicOptionsMap)
{
  topic_ = ns.empty() ? INTERACTIVE_MARKER_TOPIC : ns + "/" + INTERACTIVE_MARKER_TOPIC;
  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(topic_);

  // spin a thread that will process feedback events
  run_processing_thread_ = true;
  processing_thread_ = std::make_unique<boost::thread>([this] { processingThread(); });
}

RobotInteraction::~RobotInteraction()
{
  run_processing_thread_ = false;
  new_feedback_condition_.notify_all();
  processing_thread_->join();

  clear();
  delete int_marker_server_;
}

void RobotInteraction::decideActiveComponents(const std::string& group)
{
  decideActiveComponents(group, InteractionStyle::SIX_DOF);
}

void RobotInteraction::decideActiveComponents(const std::string& group, InteractionStyle::InteractionStyle style)
{
  decideActiveEndEffectors(group, style);
  decideActiveJoints(group);
  if (!group.empty() && active_eef_.empty() && active_vj_.empty() && active_generic_.empty())
    ROS_INFO_NAMED("robot_interaction",
                   "No active joints or end effectors found for group '%s'. "
                   "Make sure that kinematics.yaml is loaded in this node's namespace.",
                   group.c_str());
}

void RobotInteraction::addActiveComponent(const InteractiveMarkerConstructorFn& construct,
                                          const ProcessFeedbackFn& process, const InteractiveMarkerUpdateFn& update,
                                          const std::string& name)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  GenericInteraction g;
  g.construct_marker = construct;
  g.update_pose = update;
  g.process_feedback = process;
  // compute the suffix that will be added to the generated markers
  g.marker_name_suffix = "_" + name + "_" + boost::lexical_cast<std::string>(active_generic_.size());
  active_generic_.push_back(g);
}

static const double DEFAULT_SCALE = 0.25;
double RobotInteraction::computeLinkMarkerSize(const std::string& link)
{
  const moveit::core::LinkModel* lm = robot_model_->getLinkModel(link);
  double size = 0;

  while (lm)
  {
    Eigen::Vector3d ext = lm->getShapeExtentsAtOrigin();
    // drop largest extension and take norm of two remaining
    Eigen::MatrixXd::Index max_index;
    ext.maxCoeff(&max_index);
    ext[max_index] = 0;
    size = 1.01 * ext.norm();
    if (size > 0)
      break;  // break, if a non-empty shape was found

    // process kinematic chain upwards (but only following fixed joints)
    // to find a link with some non-empty shape (to ignore virtual links)
    if (lm->getParentJointModel()->getType() == moveit::core::JointModel::FIXED)
      lm = lm->getParentLinkModel();
    else
      lm = nullptr;
  }
  if (!lm)
    return DEFAULT_SCALE;  // no link with non-zero shape extends found

  // the marker sphere will be half the size, so double the size here
  return 2. * size;
}

double RobotInteraction::computeGroupMarkerSize(const std::string& group)
{
  if (group.empty())
    return DEFAULT_SCALE;
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
  if (!jmg)
    return 0.0;

  const std::vector<std::string>& links = jmg->getLinkModelNames();
  if (links.empty())
    return DEFAULT_SCALE;

  // compute the aabb of the links that make up the group
  double size = 0;
  for (const std::string& link : links)
  {
    const moveit::core::LinkModel* lm = robot_model_->getLinkModel(link);
    if (!lm)
      continue;
    Eigen::Vector3d ext = lm->getShapeExtentsAtOrigin();

    // drop largest extension and take norm of two remaining
    Eigen::MatrixXd::Index max_index;
    ext.maxCoeff(&max_index);
    ext[max_index] = 0;
    size = std::max(size, 1.01 * ext.norm());
  }

  // if size is zero, all links have empty shapes and are placed at same position
  // in this case, fall back to link marker size
  if (size == 0)
    return computeLinkMarkerSize(links[0]);

  // the marker sphere will be half the size, so double the size here
  return 2. * size;
}

void RobotInteraction::decideActiveJoints(const std::string& group)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_vj_.clear();

  if (group.empty())
    return;

  ROS_DEBUG_NAMED("robot_interaction", "Deciding active joints for group '%s'", group.c_str());

  const srdf::ModelConstSharedPtr& srdf = robot_model_->getSRDF();
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);

  if (!jmg || !srdf)
    return;

  std::set<std::string> used;
  if (jmg->hasJointModel(robot_model_->getRootJointName()))
  {
    moveit::core::RobotState default_state(robot_model_);
    default_state.setToDefaultValues();
    std::vector<double> aabb;
    default_state.computeAABB(aabb);

    const std::vector<srdf::Model::VirtualJoint>& vj = srdf->getVirtualJoints();
    for (const srdf::Model::VirtualJoint& joint : vj)
      if (joint.name_ == robot_model_->getRootJointName())
      {
        if (joint.type_ == "planar" || joint.type_ == "floating")
        {
          JointInteraction v;
          v.connecting_link = joint.child_link_;
          v.parent_frame = joint.parent_frame_;
          if (!v.parent_frame.empty() && v.parent_frame[0] == '/')
            v.parent_frame = v.parent_frame.substr(1);
          v.joint_name = joint.name_;
          if (joint.type_ == "planar")
            v.dof = 3;
          else
            v.dof = 6;
          // take the max of the X, Y, Z extent
          v.size = std::max(std::max(aabb[1] - aabb[0], aabb[3] - aabb[2]), aabb[5] - aabb[4]);
          active_vj_.push_back(v);
          used.insert(v.joint_name);
        }
      }
  }

  const std::vector<const moveit::core::JointModel*>& joints = jmg->getJointModels();
  for (const moveit::core::JointModel* joint : joints)
  {
    if ((joint->getType() == moveit::core::JointModel::PLANAR ||
         joint->getType() == moveit::core::JointModel::FLOATING) &&
        used.find(joint->getName()) == used.end())
    {
      JointInteraction v;
      v.connecting_link = joint->getChildLinkModel()->getName();
      if (joint->getParentLinkModel())
        v.parent_frame = joint->getParentLinkModel()->getName();
      v.joint_name = joint->getName();
      if (joint->getType() == moveit::core::JointModel::PLANAR)
        v.dof = 3;
      else
        v.dof = 6;
      // take the max of the X, Y, Z extent
      v.size = computeGroupMarkerSize(group);
      active_vj_.push_back(v);
    }
  }
}

void RobotInteraction::decideActiveEndEffectors(const std::string& group)
{
  decideActiveEndEffectors(group, InteractionStyle::SIX_DOF);
}

void RobotInteraction::decideActiveEndEffectors(const std::string& group, InteractionStyle::InteractionStyle style)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_eef_.clear();

  if (group.empty())
    return;

  ROS_DEBUG_NAMED("robot_interaction", "Deciding active end-effectors for group '%s'", group.c_str());

  const srdf::ModelConstSharedPtr& srdf = robot_model_->getSRDF();
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);

  if (!jmg || !srdf)
  {
    ROS_WARN_NAMED("robot_interaction", "Unable to decide active end effector: no joint model group or no srdf model");
    return;
  }

  const std::vector<srdf::Model::EndEffector>& eefs = srdf->getEndEffectors();
  const std::pair<moveit::core::JointModelGroup::KinematicsSolver, moveit::core::JointModelGroup::KinematicsSolverMap>&
      smap = jmg->getGroupKinematics();

  auto add_active_end_effectors_for_single_group = [&](const moveit::core::JointModelGroup* single_group) {
    bool found_eef{ false };
    for (const srdf::Model::EndEffector& eef : eefs)
    {
      if (single_group->hasLinkModel(eef.parent_link_) &&
          (eef.parent_group_.empty() || single_group->getName() == eef.parent_group_) &&
          single_group->canSetStateFromIK(eef.parent_link_))
      {
        // We found an end-effector whose parent is the group.
        EndEffectorInteraction ee;
        ee.parent_group = single_group->getName();
        ee.parent_link = eef.parent_link_;
        ee.eef_group = eef.component_group_;
        ee.interaction = style;
        active_eef_.push_back(ee);
        found_eef = true;
      }
    }

    // No end effectors found. Use last link in group as the "end effector".
    if (!found_eef && !single_group->getLinkModelNames().empty())
    {
      std::string last_link{ single_group->getLinkModelNames().back() };

      if (single_group->canSetStateFromIK(last_link))
      {
        EndEffectorInteraction ee;
        ee.parent_group = single_group->getName();
        ee.parent_link = last_link;
        ee.eef_group = single_group->getName();
        ee.interaction = style;
        active_eef_.push_back(ee);
      }
    }
  };

  // if we have an IK solver for the selected group, we check if there are any end effectors attached to this group
  if (smap.first)
  {
    add_active_end_effectors_for_single_group(jmg);
  }
  // if the group contains subgroups with IK, add markers for them individually
  else if (!smap.second.empty())
  {
    for (const std::pair<const moveit::core::JointModelGroup* const, moveit::core::JointModelGroup::KinematicsSolver>&
             it : smap.second)
      add_active_end_effectors_for_single_group(it.first);
  }

  // lastly determine automatic marker sizes
  for (EndEffectorInteraction& eef : active_eef_)
  {
    // if we have a separate group for the eef, we compute the scale based on it;
    // otherwise, we use the size of the parent_link
    eef.size = (eef.eef_group == eef.parent_group) ? computeLinkMarkerSize(eef.parent_link) :
                                                     computeGroupMarkerSize(eef.eef_group);
    ROS_DEBUG_NAMED("robot_interaction", "Found active end-effector '%s', of scale %lf", eef.eef_group.c_str(),
                    eef.size);
  }
  // if there is only a single end effector marker, we can safely use a larger marker
  if (active_eef_.size() == 1)
    active_eef_[0].size *= 1.5;
}

void RobotInteraction::clear()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  active_eef_.clear();
  active_vj_.clear();
  active_generic_.clear();
  clearInteractiveMarkersUnsafe();
  publishInteractiveMarkers();
}

void RobotInteraction::clearInteractiveMarkers()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  clearInteractiveMarkersUnsafe();
}

void RobotInteraction::clearInteractiveMarkersUnsafe()
{
  handlers_.clear();
  shown_markers_.clear();
  int_marker_move_subscribers_.clear();
  int_marker_move_topics_.clear();
  int_marker_names_.clear();
  int_marker_server_->clear();
}

void RobotInteraction::addEndEffectorMarkers(const InteractionHandlerPtr& handler, const EndEffectorInteraction& eef,
                                             visualization_msgs::InteractiveMarker& im, bool position, bool orientation)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  addEndEffectorMarkers(handler, eef, pose, im, position, orientation);
}

void RobotInteraction::addEndEffectorMarkers(const InteractionHandlerPtr& handler, const EndEffectorInteraction& eef,
                                             const geometry_msgs::Pose& im_to_eef,
                                             visualization_msgs::InteractiveMarker& im, bool position, bool orientation)
{
  if (eef.parent_group == eef.eef_group || !robot_model_->hasLinkModel(eef.parent_link))
    return;

  visualization_msgs::InteractiveMarkerControl m_control;
  m_control.always_visible = false;
  if (position && orientation)
    m_control.interaction_mode = m_control.MOVE_ROTATE_3D;
  else if (orientation)
    m_control.interaction_mode = m_control.ROTATE_3D;
  else
    m_control.interaction_mode = m_control.MOVE_3D;

  std_msgs::ColorRGBA marker_color;
  const float* color = handler->inError(eef) ? END_EFFECTOR_UNREACHABLE_COLOR : END_EFFECTOR_REACHABLE_COLOR;
  marker_color.r = color[0];
  marker_color.g = color[1];
  marker_color.b = color[2];
  marker_color.a = color[3];

  moveit::core::RobotStateConstPtr rstate = handler->getState();
  const std::vector<std::string>& link_names = rstate->getJointModelGroup(eef.eef_group)->getLinkModelNames();
  visualization_msgs::MarkerArray marker_array;
  rstate->getRobotMarkers(marker_array, link_names, marker_color, eef.eef_group, ros::Duration());
  tf2::Transform tf_root_to_link;
  tf2::fromMsg(tf2::toMsg(rstate->getGlobalLinkTransform(eef.parent_link)), tf_root_to_link);
  // Release the ptr count on the kinematic state
  rstate.reset();

  for (visualization_msgs::Marker& marker : marker_array.markers)
  {
    marker.header = im.header;
    marker.mesh_use_embedded_materials = !marker.mesh_resource.empty();
    // - - - - - - Do some math for the offset - - - - - -
    tf2::Transform tf_root_to_im, tf_root_to_mesh, tf_im_to_eef;
    tf2::fromMsg(im.pose, tf_root_to_im);
    tf2::fromMsg(marker.pose, tf_root_to_mesh);
    tf2::fromMsg(im_to_eef, tf_im_to_eef);
    tf2::Transform tf_eef_to_mesh = tf_root_to_link.inverse() * tf_root_to_mesh;
    tf2::Transform tf_im_to_mesh = tf_im_to_eef * tf_eef_to_mesh;
    tf2::Transform tf_root_to_mesh_new = tf_root_to_im * tf_im_to_mesh;
    tf2::toMsg(tf_root_to_mesh_new, marker.pose);
    // - - - - - - - - - - - - - - - - - - - - - - - - - -
    m_control.markers.push_back(marker);
  }

  im.controls.push_back(m_control);
}

static inline std::string getMarkerName(const InteractionHandlerPtr& handler, const EndEffectorInteraction& eef)
{
  return "EE:" + handler->getName() + "_" + eef.parent_link;
}

static inline std::string getMarkerName(const InteractionHandlerPtr& handler, const JointInteraction& vj)
{
  return "JJ:" + handler->getName() + "_" + vj.connecting_link;
}

static inline std::string getMarkerName(const InteractionHandlerPtr& handler, const GenericInteraction& g)
{
  return "GG:" + handler->getName() + "_" + g.marker_name_suffix;
}

void RobotInteraction::addInteractiveMarkers(const InteractionHandlerPtr& handler, const double marker_scale)
{
  // If scale is left at default size of 0, scale will be based on end effector link size. a good value is between 0-1
  std::vector<visualization_msgs::InteractiveMarker> ims;
  {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
    moveit::core::RobotStateConstPtr s = handler->getState();

    for (std::size_t i = 0; i < active_generic_.size(); ++i)
    {
      visualization_msgs::InteractiveMarker im;
      if (active_generic_[i].construct_marker(*s, im))
      {
        im.name = getMarkerName(handler, active_generic_[i]);
        shown_markers_[im.name] = i;
        ims.push_back(im);
        ROS_DEBUG_NAMED("robot_interaction", "Publishing interactive marker %s (size = %lf)", im.name.c_str(), im.scale);
      }
    }

    for (std::size_t i = 0; i < active_eef_.size(); ++i)
    {
      geometry_msgs::PoseStamped pose;
      geometry_msgs::Pose control_to_eef_tf;
      pose.header.frame_id = robot_model_->getModelFrame();
      pose.header.stamp = ros::Time::now();
      computeMarkerPose(handler, active_eef_[i], *s, pose.pose, control_to_eef_tf);

      std::string marker_name = getMarkerName(handler, active_eef_[i]);
      shown_markers_[marker_name] = i;

      // Determine interactive maker size
      double mscale = marker_scale < std::numeric_limits<double>::epsilon() ? active_eef_[i].size : marker_scale;

      visualization_msgs::InteractiveMarker im = makeEmptyInteractiveMarker(marker_name, pose, mscale);
      if (handler && handler->getControlsVisible())
      {
        if (active_eef_[i].interaction & InteractionStyle::POSITION_ARROWS)
          addPositionControl(im, active_eef_[i].interaction & InteractionStyle::FIXED);
        if (active_eef_[i].interaction & InteractionStyle::ORIENTATION_CIRCLES)
          addOrientationControl(im, active_eef_[i].interaction & InteractionStyle::FIXED);
        if (active_eef_[i].interaction & (InteractionStyle::POSITION_SPHERE | InteractionStyle::ORIENTATION_SPHERE))
        {
          std_msgs::ColorRGBA color;
          color.r = 0;
          color.g = 1;
          color.b = 1;
          color.a = 0.5;
          addViewPlaneControl(im, mscale * 0.25, color, active_eef_[i].interaction & InteractionStyle::POSITION_SPHERE,
                              active_eef_[i].interaction & InteractionStyle::ORIENTATION_SPHERE);
        }
      }
      if (handler && handler->getMeshesVisible() &&
          (active_eef_[i].interaction & (InteractionStyle::POSITION_EEF | InteractionStyle::ORIENTATION_EEF)))
        addEndEffectorMarkers(handler, active_eef_[i], control_to_eef_tf, im,
                              active_eef_[i].interaction & InteractionStyle::POSITION_EEF,
                              active_eef_[i].interaction & InteractionStyle::ORIENTATION_EEF);
      ims.push_back(im);
      registerMoveInteractiveMarkerTopic(marker_name, handler->getName() + "_" + active_eef_[i].parent_link);
      ROS_DEBUG_NAMED("robot_interaction", "Publishing interactive marker %s (size = %lf)", marker_name.c_str(), mscale);
    }
    for (std::size_t i = 0; i < active_vj_.size(); ++i)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = robot_model_->getModelFrame();
      pose.header.stamp = ros::Time::now();
      pose.pose = tf2::toMsg(s->getGlobalLinkTransform(active_vj_[i].connecting_link));
      std::string marker_name = getMarkerName(handler, active_vj_[i]);
      shown_markers_[marker_name] = i;

      // Determine interactive maker size
      double mscale = marker_scale < std::numeric_limits<double>::epsilon() ? active_vj_[i].size : marker_scale;

      visualization_msgs::InteractiveMarker im = makeEmptyInteractiveMarker(marker_name, pose, mscale);
      if (handler && handler->getControlsVisible())
      {
        if (active_vj_[i].dof == 3)  // planar joint
          addPlanarXYControl(im, false);
        else
          add6DOFControl(im, false);
      }
      ims.push_back(im);
      registerMoveInteractiveMarkerTopic(marker_name, handler->getName() + "_" + active_vj_[i].connecting_link);
      ROS_DEBUG_NAMED("robot_interaction", "Publishing interactive marker %s (size = %lf)", marker_name.c_str(), mscale);
    }
    handlers_[handler->getName()] = handler;
  }

  // we do this while marker_access_lock_ is unlocked because the interactive marker server locks
  // for most function calls, and maintains that lock while the feedback callback is running
  // that can cause a deadlock if we were to run the loop below while marker_access_lock_ is locked
  for (const visualization_msgs::InteractiveMarker& im : ims)
  {
    int_marker_server_->insert(im);
    int_marker_server_->setCallback(im.name,
                                    [this](const auto& feedback) { processInteractiveMarkerFeedback(feedback); });

    // Add menu handler to all markers that this interaction handler creates.
    if (std::shared_ptr<interactive_markers::MenuHandler> mh = handler->getMenuHandler())
      mh->apply(*int_marker_server_, im.name);
  }
}

void RobotInteraction::registerMoveInteractiveMarkerTopic(const std::string& marker_name, const std::string& name)
{
  std::stringstream ss;
  ss << "/rviz/moveit/move_marker/";
  ss << name;
  int_marker_move_topics_.push_back(ss.str());
  int_marker_names_.push_back(marker_name);
}

void RobotInteraction::toggleMoveInteractiveMarkerTopic(bool enable)
{
  if (enable)
  {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
    if (int_marker_move_subscribers_.empty())
    {
      ros::NodeHandle nh;
      for (size_t i = 0; i < int_marker_move_topics_.size(); i++)
      {
        std::string topic_name = int_marker_move_topics_[i];
        std::string marker_name = int_marker_names_[i];
        int_marker_move_subscribers_.push_back(nh.subscribe<geometry_msgs::PoseStamped>(
            topic_name, 1, [this, marker_name](const geometry_msgs::PoseStampedConstPtr& pose) {
              moveInteractiveMarker(marker_name, *pose);
            }));
      }
    }
  }
  else
  {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
    int_marker_move_subscribers_.clear();
  }
}

void RobotInteraction::computeMarkerPose(const InteractionHandlerPtr& handler, const EndEffectorInteraction& eef,
                                         const moveit::core::RobotState& robot_state, geometry_msgs::Pose& pose,
                                         geometry_msgs::Pose& control_to_eef_tf) const
{
  // Need to allow for control pose offsets
  tf2::Transform tf_root_to_link, tf_root_to_control;
  tf2::fromMsg(tf2::toMsg(robot_state.getGlobalLinkTransform(eef.parent_link)), tf_root_to_link);

  geometry_msgs::Pose msg_link_to_control;
  if (handler->getPoseOffset(eef, msg_link_to_control))
  {
    tf2::Transform tf_link_to_control;
    tf2::fromMsg(msg_link_to_control, tf_link_to_control);

    tf_root_to_control = tf_root_to_link * tf_link_to_control;
    tf2::toMsg(tf_link_to_control.inverse(), control_to_eef_tf);
  }
  else
  {
    tf_root_to_control = tf_root_to_link;
    control_to_eef_tf.orientation.x = 0.0;
    control_to_eef_tf.orientation.y = 0.0;
    control_to_eef_tf.orientation.z = 0.0;
    control_to_eef_tf.orientation.w = 1.0;
  }

  tf2::toMsg(tf_root_to_control, pose);
}

void RobotInteraction::updateInteractiveMarkers(const InteractionHandlerPtr& handler)
{
  std::string root_link;
  std::map<std::string, geometry_msgs::Pose> pose_updates;
  {
    boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

    moveit::core::RobotStateConstPtr s = handler->getState();
    root_link = s->getRobotModel()->getModelFrame();

    for (const EndEffectorInteraction& eef : active_eef_)
    {
      std::string marker_name = getMarkerName(handler, eef);
      geometry_msgs::Pose control_to_eef_tf;
      computeMarkerPose(handler, eef, *s, pose_updates[marker_name], control_to_eef_tf);
    }

    for (JointInteraction& vj : active_vj_)
    {
      std::string marker_name = getMarkerName(handler, vj);
      pose_updates[marker_name] = tf2::toMsg(s->getGlobalLinkTransform(vj.connecting_link));
    }

    for (GenericInteraction& gi : active_generic_)
    {
      std::string marker_name = getMarkerName(handler, gi);
      geometry_msgs::Pose pose;
      if (gi.update_pose && gi.update_pose(*s, pose))
        pose_updates[marker_name] = pose;
    }
  }

  std_msgs::Header header;
  header.frame_id = root_link;  // marker poses are give w.r.t. root frame
  for (std::map<std::string, geometry_msgs::Pose>::const_iterator it = pose_updates.begin(); it != pose_updates.end();
       ++it)
    int_marker_server_->setPose(it->first, it->second, header);
  int_marker_server_->applyChanges();
}

void RobotInteraction::publishInteractiveMarkers()
{
  // the server locks internally, so we need not worry about locking
  int_marker_server_->applyChanges();
}

bool RobotInteraction::showingMarkers(const InteractionHandlerPtr& handler)
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

  for (const EndEffectorInteraction& eef : active_eef_)
    if (shown_markers_.find(getMarkerName(handler, eef)) == shown_markers_.end())
      return false;
  for (const JointInteraction& vj : active_vj_)
    if (shown_markers_.find(getMarkerName(handler, vj)) == shown_markers_.end())
      return false;
  for (const GenericInteraction& gi : active_generic_)
    if (shown_markers_.find(getMarkerName(handler, gi)) == shown_markers_.end())
      return false;
  return true;
}

void RobotInteraction::moveInteractiveMarker(const std::string& name, const geometry_msgs::PoseStamped& msg)
{
  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(name);
  if (it != shown_markers_.end())
  {
    auto feedback = boost::make_shared<visualization_msgs::InteractiveMarkerFeedback>();
    feedback->header = msg.header;
    feedback->marker_name = name;
    feedback->pose = msg.pose;
    feedback->event_type = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
    processInteractiveMarkerFeedback(feedback);
    {
      boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
      int_marker_server_->setPose(name, msg.pose, msg.header);  // move the interactive marker
      int_marker_server_->applyChanges();
    }
  }
}

void RobotInteraction::processInteractiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // perform some validity checks
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);
  std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
  if (it == shown_markers_.end())
  {
    ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class)", feedback->marker_name.c_str());
    return;
  }

  std::size_t u = feedback->marker_name.find_first_of('_');
  if (u == std::string::npos || u < 4)
  {
    ROS_ERROR("Invalid marker name: '%s'", feedback->marker_name.c_str());
    return;
  }

  feedback_map_[feedback->marker_name] = feedback;
  new_feedback_condition_.notify_all();
}

void RobotInteraction::processingThread()
{
  boost::unique_lock<boost::mutex> ulock(marker_access_lock_);

  while (run_processing_thread_ && ros::ok())
  {
    while (feedback_map_.empty() && run_processing_thread_ && ros::ok())
      new_feedback_condition_.wait(ulock);

    while (!feedback_map_.empty() && ros::ok())
    {
      visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback = feedback_map_.begin()->second;
      feedback_map_.erase(feedback_map_.begin());
      ROS_DEBUG_NAMED("robot_interaction", "Processing feedback from map for marker [%s]",
                      feedback->marker_name.c_str());

      std::map<std::string, std::size_t>::const_iterator it = shown_markers_.find(feedback->marker_name);
      if (it == shown_markers_.end())
      {
        ROS_ERROR("Unknown marker name: '%s' (not published by RobotInteraction class) "
                  "(should never have ended up in the feedback_map!)",
                  feedback->marker_name.c_str());
        continue;
      }
      std::size_t u = feedback->marker_name.find_first_of('_');
      if (u == std::string::npos || u < 4)
      {
        ROS_ERROR("Invalid marker name: '%s' (should never have ended up in the feedback_map!)",
                  feedback->marker_name.c_str());
        continue;
      }
      std::string marker_class = feedback->marker_name.substr(0, 2);
      std::string handler_name = feedback->marker_name.substr(3, u - 3);  // skip the ":"
      std::map<std::string, InteractionHandlerPtr>::const_iterator jt = handlers_.find(handler_name);
      if (jt == handlers_.end())
      {
        ROS_ERROR("Interactive Marker Handler '%s' is not known.", handler_name.c_str());
        continue;
      }

      // we put this in a try-catch because user specified callbacks may be triggered
      try
      {
        if (marker_class == "EE")
        {
          // make a copy of the data, so we do not lose it while we are unlocked
          EndEffectorInteraction eef = active_eef_[it->second];
          InteractionHandlerPtr ih = jt->second;
          marker_access_lock_.unlock();
          try
          {
            ih->handleEndEffector(eef, feedback);
          }
          catch (std::exception& ex)
          {
            ROS_ERROR("Exception caught while handling end-effector update: %s", ex.what());
          }
          marker_access_lock_.lock();
        }
        else if (marker_class == "JJ")
        {
          // make a copy of the data, so we do not lose it while we are unlocked
          JointInteraction vj = active_vj_[it->second];
          InteractionHandlerPtr ih = jt->second;
          marker_access_lock_.unlock();
          try
          {
            ih->handleJoint(vj, feedback);
          }
          catch (std::exception& ex)
          {
            ROS_ERROR("Exception caught while handling joint update: %s", ex.what());
          }
          marker_access_lock_.lock();
        }
        else if (marker_class == "GG")
        {
          InteractionHandlerPtr ih = jt->second;
          GenericInteraction g = active_generic_[it->second];
          marker_access_lock_.unlock();
          try
          {
            ih->handleGeneric(g, feedback);
          }
          catch (std::exception& ex)
          {
            ROS_ERROR("Exception caught while handling joint update: %s", ex.what());
          }
          marker_access_lock_.lock();
        }
        else
          ROS_ERROR("Unknown marker class ('%s') for marker '%s'", marker_class.c_str(), feedback->marker_name.c_str());
      }
      catch (std::exception& ex)
      {
        ROS_ERROR("Exception caught while processing event: %s", ex.what());
      }
    }
  }
}
}  // namespace robot_interaction
