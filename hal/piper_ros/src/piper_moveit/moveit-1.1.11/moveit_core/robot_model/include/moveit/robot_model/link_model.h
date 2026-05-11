/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#include <vector>
#include <utility>
#include <map>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <moveit/macros/class_forward.h>
#include <geometric_shapes/check_isometry.h>

namespace shapes
{
MOVEIT_CLASS_FORWARD(Shape);  // Defines ShapePtr, ConstPtr, WeakPtr... etc
}

namespace moveit
{
namespace core
{
class JointModel;
class LinkModel;

/** \brief Map of names to instances for LinkModel */
typedef std::map<std::string, LinkModel*> LinkModelMap;

/** \brief Map of names to const instances for LinkModel */
using LinkModelMapConst = std::map<std::string, const LinkModel*>;

/** \brief Map from link model instances to Eigen transforms */
using LinkTransformMap = std::map<const LinkModel*, Eigen::Isometry3d, std::less<const LinkModel*>,
                                  Eigen::aligned_allocator<std::pair<const LinkModel* const, Eigen::Isometry3d> > >;

/** \brief A link from the robot. Contains the constant transform applied to the link and its geometry */
class LinkModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LinkModel(const std::string& name);
  ~LinkModel();

  /** \brief The name of this link */
  const std::string& getName() const
  {
    return name_;
  }

  /** \brief The index of this joint when traversing the kinematic tree in depth first fashion */
  int getLinkIndex() const
  {
    return link_index_;
  }

  void setLinkIndex(int index)
  {
    link_index_ = index;
  }

  int getFirstCollisionBodyTransformIndex() const
  {
    return first_collision_body_transform_index_;
  }

  void setFirstCollisionBodyTransformIndex(int index)
  {
    first_collision_body_transform_index_ = index;
  }

  /** \brief Get the joint model whose child this link is. There will always be a parent joint */
  const JointModel* getParentJointModel() const
  {
    return parent_joint_model_;
  }

  void setParentJointModel(const JointModel* joint);

  /** \brief Get the link model whose child this link is (through some joint). There may not always be a parent link
   * (NULL is returned for the root link) */
  const LinkModel* getParentLinkModel() const
  {
    return parent_link_model_;
  }

  void setParentLinkModel(const LinkModel* link)
  {
    parent_link_model_ = link;
  }

  /** \brief A link may have 0 or more child joints. From those joints there will certainly be other descendant links */
  const std::vector<const JointModel*>& getChildJointModels() const
  {
    return child_joint_models_;
  }

  void addChildJointModel(const JointModel* joint)
  {
    child_joint_models_.push_back(joint);
  }

  /** \brief When transforms are computed for this link,
      they are usually applied to the link's origin. The
      joint origin transform acts as an offset -- it is
      pre-applied before any other transform. The
      transform is guaranteed to be a valid isometry. */
  const Eigen::Isometry3d& getJointOriginTransform() const
  {
    return joint_origin_transform_;
  }

  bool jointOriginTransformIsIdentity() const
  {
    return joint_origin_transform_is_identity_;
  }

  bool parentJointIsFixed() const
  {
    return is_parent_joint_fixed_;
  }

  void setJointOriginTransform(const Eigen::Isometry3d& transform);

  /** \brief In addition to the link transform, the geometry
      of a link that is used for collision checking may have
      a different offset itself, with respect to the origin.
      The transform is guaranteed to be a valid isometry. */
  const EigenSTL::vector_Isometry3d& getCollisionOriginTransforms() const
  {
    return collision_origin_transform_;
  }

  /** \brief Return flags for each transform specifying whether they are identity or not */
  const std::vector<int>& areCollisionOriginTransformsIdentity() const
  {
    return collision_origin_transform_is_identity_;
  }

  /** \brief Get shape associated to the collision geometry for this link */
  const std::vector<shapes::ShapeConstPtr>& getShapes() const
  {
    return shapes_;
  }

  void setGeometry(const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& origins);

  /** \brief Get the extents of the link's geometry (dimensions of axis-aligned bounding box around all shapes that make
     up the
      link, when the link is positioned at origin -- only collision origin transforms are considered) */
  const Eigen::Vector3d& getShapeExtentsAtOrigin() const
  {
    return shape_extents_;
  }

  /** \brief Get the offset of the center of the bounding box of this link when the link is positioned at origin. */
  const Eigen::Vector3d& getCenteredBoundingBoxOffset() const
  {
    return centered_bounding_box_offset_;
  }

  /** \brief Get the set of links that are attached to this one via fixed transforms. The returned transforms are
   * guaranteed to be valid isometries. */
  const LinkTransformMap& getAssociatedFixedTransforms() const
  {
    return associated_fixed_transforms_;
  }

  /** \brief Remember that \e link_model is attached to this link using a fixed transform */
  void addAssociatedFixedTransform(const LinkModel* link_model, const Eigen::Isometry3d& transform)
  {
    ASSERT_ISOMETRY(transform);  // unsanitized input, could contain a non-isometry
    associated_fixed_transforms_[link_model] = transform;
  }

  /** \brief Get the filename of the mesh resource used for visual display of this link */
  const std::string& getVisualMeshFilename() const
  {
    return visual_mesh_filename_;
  }

  /** \brief Get the scale of the mesh resource for this link */
  const Eigen::Vector3d& getVisualMeshScale() const
  {
    return visual_mesh_scale_;
  }

  /** \brief Get the transform for the visual mesh origin */
  const Eigen::Isometry3d& getVisualMeshOrigin() const
  {
    return visual_mesh_origin_;
  }

  void setVisualMesh(const std::string& visual_mesh, const Eigen::Isometry3d& origin, const Eigen::Vector3d& scale);

private:
  /** \brief Name of the link */
  std::string name_;

  /** \brief JointModel that connects this link to the parent link */
  const JointModel* parent_joint_model_;

  /** \brief The parent link model (NULL for the root link) */
  const LinkModel* parent_link_model_;

  /** \brief List of directly descending joints (each connects to a child link) */
  std::vector<const JointModel*> child_joint_models_;

  /** \brief True if the parent joint of this link is fixed */
  bool is_parent_joint_fixed_;

  /** \brief True of the joint origin transform is identity */
  bool joint_origin_transform_is_identity_;

  /** \brief The constant transform applied to the link (local) */
  Eigen::Isometry3d joint_origin_transform_;

  /** \brief The constant transform applied to the collision geometry of the link (local) */
  EigenSTL::vector_Isometry3d collision_origin_transform_;

  /** \brief Flag indicating if the constant transform applied to the collision geometry of the link (local) is
   * identity; use int instead of bool to avoid bit operations */
  std::vector<int> collision_origin_transform_is_identity_;

  /** \brief The set of links that are attached to this one via fixed transforms */
  LinkTransformMap associated_fixed_transforms_;

  /** \brief The collision geometry of the link */
  std::vector<shapes::ShapeConstPtr> shapes_;

  /** \brief The extents of shape (dimensions of axis aligned bounding box when shape is at origin). */
  Eigen::Vector3d shape_extents_;

  /** \brief Center of the axis aligned bounding box with size shape_extents_ (zero if symmetric along all axes). */
  Eigen::Vector3d centered_bounding_box_offset_;

  /** \brief Filename associated with the visual geometry mesh of this link. If empty, no mesh was used. */
  std::string visual_mesh_filename_;

  /** \brief The additional origin transform for the mesh */
  Eigen::Isometry3d visual_mesh_origin_;

  /** \brief Scale factor associated with the visual geometry mesh of this link. */
  Eigen::Vector3d visual_mesh_scale_;

  /** \brief Index of the transform for the first shape that makes up the geometry of this link in the full robot state
   */
  int first_collision_body_transform_index_;

  /** \brief Index of the transform for this link in the full robot frame */
  int link_index_;
};
}  // namespace core
}  // namespace moveit
