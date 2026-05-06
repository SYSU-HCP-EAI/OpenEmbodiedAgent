/*********************************************************************
 * Software License Agreement (BSD-2-Clause)
 *
 * Copyright (c) 2017, Southwest Research Institute
 * Copyright (c) 2013, John Schulman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: John Schulman, Levi Armstrong */

#pragma once

#include <btBulletCollisionCommon.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>
#include <moveit/collision_detection_bullet/bullet_integration/contact_checker_common.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/macros/declare_ptr.h>
#include <moveit/macros/class_forward.h>

namespace collision_detection_bullet
{
#define METERS

const btScalar BULLET_MARGIN = 0.0f;
const btScalar BULLET_SUPPORT_FUNC_TOLERANCE = 0.01f METERS;
const btScalar BULLET_LENGTH_TOLERANCE = 0.001f METERS;
const btScalar BULLET_EPSILON = 1e-3f;                   // numerical precision limit
const btScalar BULLET_DEFAULT_CONTACT_DISTANCE = 0.00f;  // All pairs closer than this distance get reported
const bool BULLET_COMPOUND_USE_DYNAMIC_AABB = true;

MOVEIT_CLASS_FORWARD(CollisionObjectWrapper);

/** \brief Allowed = true */
inline bool acmCheck(const std::string& body_1, const std::string& body_2,
                     const collision_detection::AllowedCollisionMatrix* acm)
{
  collision_detection::AllowedCollision::Type allowed_type;

  if (acm != nullptr)
  {
    if (acm->getAllowedCollision(body_1, body_2, allowed_type))
    {
      if (allowed_type == collision_detection::AllowedCollision::Type::NEVER)
      {
        ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Not allowed entry in ACM found, collision check between "
                                                                 << body_1 << " and " << body_2);
        return false;
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Entry in ACM found, skipping collision check as allowed "
                                                                 << body_1 << " and " << body_2);
        return true;
      }
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                             "No entry in ACM found, collision check between " << body_1 << " and " << body_2);
      return false;
    }
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                           "No ACM, collision check between " << body_1 << " and " << body_2);
    return false;
  }
}

/** \brief Converts eigen vector to bullet vector */
inline btVector3 convertEigenToBt(const Eigen::Vector3d& v)
{
  return btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2]));
}

/** \brief Converts bullet vector to eigen vector */
inline Eigen::Vector3d convertBtToEigen(const btVector3& v)
{
  return Eigen::Vector3d(static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z()));
}

/** \brief Converts eigen quaternion to bullet quaternion */
inline btQuaternion convertEigenToBt(const Eigen::Quaterniond& q)
{
  return btQuaternion(static_cast<btScalar>(q.x()), static_cast<btScalar>(q.y()), static_cast<btScalar>(q.z()),
                      static_cast<btScalar>(q.w()));
}

/** \brief Converts eigen matrix to bullet matrix */
inline btMatrix3x3 convertEigenToBt(const Eigen::Matrix3d& r)
{
  return btMatrix3x3(static_cast<btScalar>(r(0, 0)), static_cast<btScalar>(r(0, 1)), static_cast<btScalar>(r(0, 2)),
                     static_cast<btScalar>(r(1, 0)), static_cast<btScalar>(r(1, 1)), static_cast<btScalar>(r(1, 2)),
                     static_cast<btScalar>(r(2, 0)), static_cast<btScalar>(r(2, 1)), static_cast<btScalar>(r(2, 2)));
}

/** \brief Converts bullet transform to eigen transform */
inline btTransform convertEigenToBt(const Eigen::Isometry3d& t)
{
  const Eigen::Matrix3d& rot = t.matrix().block<3, 3>(0, 0);
  const Eigen::Vector3d& tran = t.translation();

  btMatrix3x3 mat = convertEigenToBt(rot);
  btVector3 translation = convertEigenToBt(tran);

  return btTransform(mat, translation);
}

/** @brief Tesseract bullet collision object.
 *
 *  A wrapper around bullet's collision object which contains specific information related to bullet. One of the main
 *  differences is that a bullet collision object has a single world transformation and all shapes have transformation
 *  relative to this world transform. The default collision object category is active and active objects are checked
 *  against active objects and static objects whereas static objects are only checked against active ones. */
class CollisionObjectWrapper : public btCollisionObject
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Standard constructor
   *
   *  \param shape_poses Assumes all poses are in a single global frame */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types, bool active = true);

  /** \brief Constructor for attached robot objects
   *
   *  \param shape_poses These poses are in the global (planning) frame */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types,
                         const std::set<std::string>& touch_links);

  /** \brief Bitfield specifies to which group the object belongs */
  short int m_collisionFilterGroup;

  /** \brief Bitfield specifies against which other groups the object is collision checked */
  short int m_collisionFilterMask;

  /** \brief Indicates if the collision object is used for a collision check */
  bool m_enabled{ true };

  /** \brief The robot links the collision objects is allowed to touch */
  std::set<std::string> m_touch_links;

  /** @brief Get the collision object name */
  const std::string& getName() const
  {
    return m_name;
  }

  /** @brief Get a user defined type */
  const collision_detection::BodyType& getTypeID() const
  {
    return m_type_id;
  }

  /** \brief Check if two CollisionObjectWrapper objects point to the same source object
   *  \return True if same objects, false otherwise */
  bool sameObject(const CollisionObjectWrapper& other) const
  {
    return m_name == other.m_name && m_type_id == other.m_type_id && m_shapes.size() == other.m_shapes.size() &&
           m_shape_poses.size() == other.m_shape_poses.size() &&
           std::equal(m_shapes.begin(), m_shapes.end(), other.m_shapes.begin()) &&
           std::equal(m_shape_poses.begin(), m_shape_poses.end(), other.m_shape_poses.begin(),
                      [](const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2) { return t1.isApprox(t2); });
  }

  /** @brief Get the collision objects axis aligned bounding box
   *  @param aabb_min The minimum point
   *  @param aabb_max The maximum point */
  void getAABB(btVector3& aabb_min, btVector3& aabb_max) const
  {
    getCollisionShape()->getAabb(getWorldTransform(), aabb_min, aabb_max);
    const btScalar& distance = getContactProcessingThreshold();
    // note that bullet expands each AABB by 4 cm
    btVector3 contact_threshold(distance, distance, distance);
    aabb_min -= contact_threshold;
    aabb_max += contact_threshold;
  }

  /** @brief Clones the collision objects but not the collision shape wich is const.
   *  @return Shared Pointer to the cloned collision object */
  std::shared_ptr<CollisionObjectWrapper> clone()
  {
    std::shared_ptr<CollisionObjectWrapper> clone_cow(
        new CollisionObjectWrapper(m_name, m_type_id, m_shapes, m_shape_poses, m_collision_object_types, m_data));
    clone_cow->setCollisionShape(getCollisionShape());
    clone_cow->setWorldTransform(getWorldTransform());
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    clone_cow->setBroadphaseHandle(nullptr);
    clone_cow->m_touch_links = m_touch_links;
    clone_cow->setContactProcessingThreshold(this->getContactProcessingThreshold());
    return clone_cow;
  }

  /** \brief Manage memory of a raw pointer shape */
  template <class T>
  void manage(T* t)
  {
    m_data.push_back(std::shared_ptr<T>(t));
  }

  /** \brief Manage memory of a shared pointer shape */
  template <class T>
  void manage(std::shared_ptr<T> t)
  {
    m_data.push_back(t);
  }

protected:
  /** @brief Special constructor used by the clone method */
  CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const AlignedVector<Eigen::Isometry3d>& shape_poses,
                         const std::vector<CollisionObjectType>& collision_object_types,
                         const std::vector<std::shared_ptr<void>>& data);

  /** \brief The name of the object, must be unique. */
  std::string m_name;

  collision_detection::BodyType m_type_id;

  /** @brief The shapes that define the collison object */
  std::vector<shapes::ShapeConstPtr> m_shapes;

  /** @brief The poses of the shapes, must be same length as m_shapes */
  AlignedVector<Eigen::Isometry3d> m_shape_poses;

  /** @brief The shape collision object type to be used. */
  std::vector<CollisionObjectType> m_collision_object_types;

  /** @brief Manages the collision shape pointer so they get destroyed */
  std::vector<std::shared_ptr<void>> m_data;
};

/** @brief Casted collision shape used for checking if an object is collision free between two discrete poses
 *
 *  The cast is not explicitely computed but implicitely represented through the single shape and the transformation
 *  between the first and second pose. */
struct CastHullShape : public btConvexShape
{
public:
  /** \brief The casted shape */
  btConvexShape* m_shape;

  /** \brief Transformation from the first pose to the second pose */
  btTransform m_shape_transform;

  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_shape_transform(t01)
  {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
  }

  void updateCastTransform(const btTransform& cast_transform)
  {
    m_shape_transform = cast_transform;
  }

  /** \brief From both shape poses computes the support vertex and returns the larger one. */
  btVector3 localGetSupportingVertex(const btVector3& vec) const override
  {
    btVector3 support_vector_0 = m_shape->localGetSupportingVertex(vec);
    btVector3 support_vector_1 =
        m_shape_transform * m_shape->localGetSupportingVertex(vec * m_shape_transform.getBasis());
    return (vec.dot(support_vector_0) > vec.dot(support_vector_1)) ? support_vector_0 : support_vector_1;
  }

  void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* /*vectors*/,
                                                         btVector3* /*supportVerticesOut*/,
                                                         int /*numVectors*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  /** \brief Shape specific fast recalculation of the AABB at a certain pose
   *
   *  The AABB is not recalculated from scratch but updated depending on the given transformation. */
  void getAabb(const btTransform& transform_world, btVector3& aabbMin, btVector3& aabbMax) const override
  {
    m_shape->getAabb(transform_world, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(transform_world * m_shape_transform, min1, max1);
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  void getAabbSlow(const btTransform& /*t*/, btVector3& /*aabbMin*/, btVector3& /*aabbMax*/) const override
  {
    throw std::runtime_error("shouldn't happen");
  }

  void setLocalScaling(const btVector3& /*scaling*/) override
  {
  }

  const btVector3& getLocalScaling() const override
  {
    static btVector3 out(1, 1, 1);
    return out;
  }

  void setMargin(btScalar /*margin*/) override
  {
  }

  btScalar getMargin() const override
  {
    return 0;
  }

  int getNumPreferredPenetrationDirections() const override
  {
    return 0;
  }

  void getPreferredPenetrationDirection(int /*index*/, btVector3& /*penetrationVector*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  void calculateLocalInertia(btScalar /*mass*/, btVector3& /*inertia*/) const override
  {
    throw std::runtime_error("not implemented");
  }

  const char* getName() const override
  {
    return "CastHull";
  }

  btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const override
  {
    return localGetSupportingVertex(v);
  }
};

/** \brief Computes the local supporting vertex of a convex shape.
 *
 *  If multiple vertices with equal support products exists, their average is calculated and returned.
 *
 *  \param shape The convex shape to check
 *  \param localNormal The support direction to search for in shape local coordinates
 *  \param outsupport The value of the calculated support mapping
 *  \param outpt The computed support point */
inline void getAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport,
                              btVector3& outpt)
{
  btVector3 pt_sum(0, 0, 0);
  float pt_count = 0;
  float max_support = -1000;

  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape)
  {
    int n_pts = pshape->getNumVertices();

    for (int i = 0; i < n_pts; ++i)
    {
      btVector3 pt;
      pshape->getVertex(i, pt);

      float sup = pt.dot(localNormal);
      if (sup > max_support + BULLET_EPSILON)
      {
        pt_count = 1;
        pt_sum = pt;
        max_support = sup;
      }
      else if (sup < max_support - BULLET_EPSILON)
      {
      }
      else
      {
        pt_count += 1;
        pt_sum += pt;
      }
    }
    outsupport = max_support;
    outpt = pt_sum / pt_count;
  }
  else
  {
    outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
    outsupport = localNormal.dot(outpt);
  }
}

/** \brief Converts a bullet contact result to MoveIt format and adds it to the result data structure */
inline btScalar addDiscreteSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,
                                        const btCollisionObjectWrapper* colObj1Wrap, ContactTestData& collisions)
{
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject()) != nullptr);
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject()) != nullptr);
  const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

  std::pair<std::string, std::string> pc = getObjectPairKey(cd0->getName(), cd1->getName());

  const auto& it = collisions.res.contacts.find(pc);
  bool found = (it != collisions.res.contacts.end());

  collision_detection::Contact contact;
  contact.body_name_1 = cd0->getName();
  contact.body_name_2 = cd1->getName();
  contact.depth = static_cast<double>(cp.m_distance1);
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);
  contact.pos = convertBtToEigen(cp.m_positionWorldOnA);
  contact.nearest_points[0] = contact.pos;
  contact.nearest_points[1] = convertBtToEigen(cp.m_positionWorldOnB);

  contact.body_type_1 = cd0->getTypeID();
  contact.body_type_2 = cd1->getTypeID();

  if (!processResult(collisions, contact, pc, found))
  {
    return 0;
  }

  return 1;
}

inline btScalar addCastSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*index0*/,
                                    const btCollisionObjectWrapper* colObj1Wrap, int /*index1*/,
                                    ContactTestData& collisions)
{
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject()) != nullptr);
  assert(dynamic_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject()) != nullptr);

  const CollisionObjectWrapper* cd0 = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
  const CollisionObjectWrapper* cd1 = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());

  std::pair<std::string, std::string> pc = getObjectPairKey(cd0->getName(), cd1->getName());

  const auto& it = collisions.res.contacts.find(pc);
  bool found = (it != collisions.res.contacts.end());

  collision_detection::Contact contact;
  contact.body_name_1 = cd0->getName();
  contact.body_name_2 = cd1->getName();
  contact.depth = static_cast<double>(cp.m_distance1);
  contact.normal = convertBtToEigen(-1 * cp.m_normalWorldOnB);
  contact.pos = convertBtToEigen(cp.m_positionWorldOnA);

  contact.body_type_1 = cd0->getTypeID();
  contact.body_type_2 = cd1->getTypeID();

  collision_detection::Contact* col = processResult(collisions, contact, pc, found);

  if (!col)
  {
    return 0;
  }

  assert(!((cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter) &&
           (cd1->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)));

  bool cast_shape_is_first = cd0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter;

  btVector3 normal_world_from_cast = (cast_shape_is_first ? -1 : 1) * cp.m_normalWorldOnB;
  const btCollisionObjectWrapper* first_col_obj_wrap = (cast_shape_is_first ? colObj0Wrap : colObj1Wrap);

  // we want the contact information of the non-casted object come first, therefore swap values
  if (cast_shape_is_first)
  {
    std::swap(col->nearest_points[0], col->nearest_points[1]);
    contact.pos = convertBtToEigen(cp.m_positionWorldOnB);
    std::swap(col->body_name_1, col->body_name_2);
    std::swap(col->body_type_1, col->body_type_2);
    col->normal *= -1;
  }

  btTransform tf_world0, tf_world1;
  const CastHullShape* shape = static_cast<const CastHullShape*>(first_col_obj_wrap->getCollisionShape());

  tf_world0 = first_col_obj_wrap->getWorldTransform();
  tf_world1 = first_col_obj_wrap->getWorldTransform() * shape->m_shape_transform;

  // transform normals into pose local coordinate systems
  btVector3 normal_local0 = normal_world_from_cast * tf_world0.getBasis();
  btVector3 normal_local1 = normal_world_from_cast * tf_world1.getBasis();

  btVector3 pt_local0;
  float localsup0;
  getAverageSupport(shape->m_shape, normal_local0, localsup0, pt_local0);
  btVector3 pt_world0 = tf_world0 * pt_local0;
  btVector3 pt_local1;
  float localsup1;
  getAverageSupport(shape->m_shape, normal_local1, localsup1, pt_local1);
  btVector3 pt_world1 = tf_world1 * pt_local1;

  float sup0 = normal_world_from_cast.dot(pt_world0);
  float sup1 = normal_world_from_cast.dot(pt_world1);

  // TODO: this section is potentially problematic. think hard about the math
  if (sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    col->percent_interpolation = 0;
  }
  else if (sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE)
  {
    col->percent_interpolation = 1;
  }
  else
  {
    const btVector3& pt_on_cast = cast_shape_is_first ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
    float l0c = (pt_on_cast - pt_world0).length();
    float l1c = (pt_on_cast - pt_world1).length();

    if (l0c + l1c < BULLET_LENGTH_TOLERANCE)
    {
      col->percent_interpolation = .5;
    }
    else
    {
      col->percent_interpolation = static_cast<double>(l0c / (l0c + l1c));
    }
  }

  return 1;
}

/** \brief Checks if the collision pair is kinematic vs kinematic objects */
inline bool isOnlyKinematic(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1)
{
  return cow0->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter &&
         cow1->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter;
}

/** @brief Callback structure for both discrete and continuous broadphase collision pair
 *
 *  /e needsCollision is the callback executed before a narrowphase check is executed.
 *  /e addSingleResult is the callback executed after the narrowphase check delivers a result. */
struct BroadphaseContactResultCallback
{
  ContactTestData& collisions_;
  double contact_distance_;
  const collision_detection::AllowedCollisionMatrix* acm_{ nullptr };

  /** \brief Indicates if the callback is used for only self-collision checking */
  bool self_;

  /** \brief Indicates if the callback is used for casted collisions */
  bool cast_{ false };

  BroadphaseContactResultCallback(ContactTestData& collisions, double contact_distance,
                                  const collision_detection::AllowedCollisionMatrix* acm, bool self, bool cast = false)
    : collisions_(collisions), contact_distance_(contact_distance), acm_(acm), self_(self), cast_(cast)
  {
  }

  ~BroadphaseContactResultCallback() = default;

  /** \brief This callback is used for each overlapping pair in a pair cache of the broadphase interface to check if a
   *  collision check should done for the pair. */
  // TODO: Add check for two objects attached to the same link
  bool needsCollision(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) const
  {
    if (cast_)
    {
      return !collisions_.done && !isOnlyKinematic(cow0, cow1) && !acmCheck(cow0->getName(), cow1->getName(), acm_);
    }
    else
    {
      return !collisions_.done && (self_ ? isOnlyKinematic(cow0, cow1) : !isOnlyKinematic(cow0, cow1)) &&
             !acmCheck(cow0->getName(), cow1->getName(), acm_);
    }
  }

  /** \brief This callback is used after btManifoldResult processed a collision result. */
  btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int /*partId0*/,
                           int index0, const btCollisionObjectWrapper* colObj1Wrap, int /*partId1*/, int index1)
  {
    if (cp.m_distance1 > static_cast<btScalar>(contact_distance_))
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Not close enough for collision with " << cp.m_distance1);
      return 0;
    }

    if (cast_)
    {
      return addCastSingleResult(cp, colObj0Wrap, index0, colObj1Wrap, index1, collisions_);
    }
    else
    {
      return addDiscreteSingleResult(cp, colObj0Wrap, colObj1Wrap, collisions_);
    }
  }
};

struct TesseractBroadphaseBridgedManifoldResult : public btManifoldResult
{
  BroadphaseContactResultCallback& result_callback_;

  TesseractBroadphaseBridgedManifoldResult(const btCollisionObjectWrapper* obj0Wrap,
                                           const btCollisionObjectWrapper* obj1Wrap,
                                           BroadphaseContactResultCallback& result_callback)
    : btManifoldResult(obj0Wrap, obj1Wrap), result_callback_(result_callback)
  {
  }

  void addContactPoint(const btVector3& normalOnBInWorld, const btVector3& pointInWorld, btScalar depth) override
  {
    if (result_callback_.collisions_.done || result_callback_.collisions_.pair_done ||
        depth > static_cast<btScalar>(result_callback_.contact_distance_))
    {
      return;
    }

    bool is_swapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
    btVector3 point_a = pointInWorld + normalOnBInWorld * depth;
    btVector3 local_a;
    btVector3 local_b;
    if (is_swapped)
    {
      local_a = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(point_a);
      local_b = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }
    else
    {
      local_a = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(point_a);
      local_b = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
    }

    btManifoldPoint new_pt(local_a, local_b, normalOnBInWorld, depth);
    new_pt.m_positionWorldOnA = point_a;
    new_pt.m_positionWorldOnB = pointInWorld;

    // BP mod, store contact triangles.
    if (is_swapped)
    {
      new_pt.m_partId0 = m_partId1;
      new_pt.m_partId1 = m_partId0;
      new_pt.m_index0 = m_index1;
      new_pt.m_index1 = m_index0;
    }
    else
    {
      new_pt.m_partId0 = m_partId0;
      new_pt.m_partId1 = m_partId1;
      new_pt.m_index0 = m_index0;
      new_pt.m_index1 = m_index1;
    }

    // experimental feature info, for per-triangle material etc.
    const btCollisionObjectWrapper* obj0_wrap = is_swapped ? m_body1Wrap : m_body0Wrap;
    const btCollisionObjectWrapper* obj1_wrap = is_swapped ? m_body0Wrap : m_body1Wrap;
    result_callback_.addSingleResult(new_pt, obj0_wrap, new_pt.m_partId0, new_pt.m_index0, obj1_wrap, new_pt.m_partId1,
                                     new_pt.m_index1);
  }
};

/** @brief A callback function that is called as part of the broadphase collision checking.
 *
 *  If the AABB of two collision objects are overlapping the processOverlap method is called and they are checked for
 *  collision/distance and the results are stored in collision_. */
class TesseractCollisionPairCallback : public btOverlapCallback
{
  const btDispatcherInfo& dispatch_info_;
  btCollisionDispatcher* dispatcher_;

  /** \brief Callback executed for each broadphase pair to check if needs collision */
  BroadphaseContactResultCallback& results_callback_;

public:
  TesseractCollisionPairCallback(const btDispatcherInfo& dispatchInfo, btCollisionDispatcher* dispatcher,
                                 BroadphaseContactResultCallback& results_callback)
    : dispatch_info_(dispatchInfo), dispatcher_(dispatcher), results_callback_(results_callback)
  {
  }

  ~TesseractCollisionPairCallback() override = default;

  bool processOverlap(btBroadphasePair& pair) override
  {
    results_callback_.collisions_.pair_done = false;

    if (results_callback_.collisions_.done)
    {
      return false;
    }

    const CollisionObjectWrapper* cow0 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy0->m_clientObject);
    const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(pair.m_pProxy1->m_clientObject);

    std::pair<std::string, std::string> pair_names{ cow0->getName(), cow1->getName() };
    if (results_callback_.needsCollision(cow0, cow1))
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                             "Processing " << cow0->getName() << " vs " << cow1->getName());
      btCollisionObjectWrapper obj0_wrap(nullptr, cow0->getCollisionShape(), cow0, cow0->getWorldTransform(), -1, -1);
      btCollisionObjectWrapper obj1_wrap(nullptr, cow1->getCollisionShape(), cow1, cow1->getWorldTransform(), -1, -1);

      // dispatcher will keep algorithms persistent in the collision pair
      if (!pair.m_algorithm)
      {
        pair.m_algorithm = dispatcher_->findAlgorithm(&obj0_wrap, &obj1_wrap, nullptr, BT_CLOSEST_POINT_ALGORITHMS);
      }

      if (pair.m_algorithm)
      {
        TesseractBroadphaseBridgedManifoldResult contact_point_result(&obj0_wrap, &obj1_wrap, results_callback_);
        contact_point_result.m_closestPointDistanceThreshold =
            static_cast<btScalar>(results_callback_.contact_distance_);

        // discrete collision detection query
        pair.m_algorithm->processCollision(&obj0_wrap, &obj1_wrap, dispatch_info_, &contact_point_result);
      }
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                             "Not processing " << cow0->getName() << " vs " << cow1->getName());
    }
    return false;
  }
};

/** \brief Casts a geometric shape into a btCollisionShape */
btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                       const CollisionObjectType& collision_object_type, CollisionObjectWrapper* cow);

/** @brief Update a collision objects filters
 *  @param active A list of active collision objects
 *  @param cow The collision object to update.
 *  @param continuous Indicate if the object is a continuous collision object.
 *
 *  Currently continuous collision objects can only be checked against static objects. Continuous to Continuous
 *  collision checking is currently not supports. TODO LEVI: Add support for Continuous to Continuous collision
 *  checking. */
inline void updateCollisionObjectFilters(const std::vector<std::string>& active, CollisionObjectWrapper& cow)
{
  // if not active make cow part of static
  if (!isLinkActive(active, cow.getName()))
  {
    cow.m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    cow.m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }
  else
  {
    cow.m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    cow.m_collisionFilterMask = btBroadphaseProxy::KinematicFilter | btBroadphaseProxy::StaticFilter;
  }

  if (cow.getBroadphaseHandle())
  {
    cow.getBroadphaseHandle()->m_collisionFilterGroup = cow.m_collisionFilterGroup;
    cow.getBroadphaseHandle()->m_collisionFilterMask = cow.m_collisionFilterMask;
  }
  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "COW " << cow.getName() << " group "
                                                              << cow.m_collisionFilterGroup << " mask "
                                                              << cow.m_collisionFilterMask);
}

inline CollisionObjectWrapperPtr makeCastCollisionObject(const CollisionObjectWrapperPtr& cow)
{
  CollisionObjectWrapperPtr new_cow = cow->clone();

  btTransform tf;
  tf.setIdentity();

  if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
  {
    assert(dynamic_cast<btConvexShape*>(new_cow->getCollisionShape()) != nullptr);
    btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());

    // This checks if the collision object is already a cast collision object
    assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);

    CastHullShape* shape = new CastHullShape(convex, tf);

    new_cow->manage(shape);
    new_cow->setCollisionShape(shape);
  }
  else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
  {
    btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
    btCompoundShape* new_compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, compound->getNumChildShapes());

    for (int i = 0; i < compound->getNumChildShapes(); ++i)
    {
      if (btBroadphaseProxy::isConvex(compound->getChildShape(i)->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
        assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

        btTransform geom_trans = compound->getChildTransform(i);

        btCollisionShape* subshape = new CastHullShape(convex, tf);

        new_cow->manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geom_trans, subshape);
      }
      else if (btBroadphaseProxy::isCompound(compound->getChildShape(i)->getShapeType()))
      {
        btCompoundShape* second_compound = static_cast<btCompoundShape*>(compound->getChildShape(i));
        btCompoundShape* new_second_compound =
            new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, second_compound->getNumChildShapes());
        for (int j = 0; j < second_compound->getNumChildShapes(); ++j)
        {
          assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)->getShapeType()));

          btConvexShape* convex = static_cast<btConvexShape*>(second_compound->getChildShape(j));
          assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE);  // This checks if already a cast collision object

          btTransform geom_trans = second_compound->getChildTransform(j);

          btCollisionShape* subshape = new CastHullShape(convex, tf);

          new_cow->manage(subshape);
          subshape->setMargin(BULLET_MARGIN);
          new_second_compound->addChildShape(geom_trans, subshape);
        }

        btTransform geom_trans = compound->getChildTransform(i);

        new_cow->manage(new_second_compound);

        // margin on compound seems to have no effect when positive but has an effect when negative
        new_second_compound->setMargin(BULLET_MARGIN);
        new_compound->addChildShape(geom_trans, new_second_compound);
      }
      else
      {
        ROS_ERROR_NAMED("collision_detection.bullet",
                        "I can only collision check convex shapes and compound shapes made of convex shapes");
        throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
      }
    }

    // margin on compound seems to have no effect when positive but has an effect when negative
    new_compound->setMargin(BULLET_MARGIN);
    new_cow->manage(new_compound);
    new_cow->setCollisionShape(new_compound);
    new_cow->setWorldTransform(cow->getWorldTransform());
  }
  else
  {
    ROS_ERROR_NAMED("collision_detection.bullet",
                    "I can only collision check convex shapes and compound shapes made of convex shapes");
    throw std::runtime_error("I can only collision check convex shapes and compound shapes made of convex shapes");
  }

  return new_cow;
}

/** @brief Update the Broadphase AABB for the input collision object
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void updateBroadphaseAABB(const CollisionObjectWrapperPtr& cow,
                                 const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                 const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  assert(cow->getBroadphaseHandle() != nullptr);
  broadphase->setAabb(cow->getBroadphaseHandle(), aabb_min, aabb_max, dispatcher.get());
}

/** @brief Remove the collision object from broadphase
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void removeCollisionObjectFromBroadphase(const CollisionObjectWrapperPtr& cow,
                                                const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                                const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  btBroadphaseProxy* bp = cow->getBroadphaseHandle();
  if (bp)
  {
    // only clear the cached algorithms
    broadphase->getOverlappingPairCache()->cleanProxyFromPairs(bp, dispatcher.get());
    broadphase->destroyProxy(bp, dispatcher.get());
    cow->setBroadphaseHandle(nullptr);
  }
}

/** @brief Add the collision object to broadphase
 *  @param cow The collision objects
 *  @param broadphase The bullet broadphase interface
 *  @param dispatcher The bullet collision dispatcher */
inline void addCollisionObjectToBroadphase(const CollisionObjectWrapperPtr& cow,
                                           const std::unique_ptr<btBroadphaseInterface>& broadphase,
                                           const std::unique_ptr<btCollisionDispatcher>& dispatcher)
{
  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet", "Added " << cow->getName() << " to broadphase");
  btVector3 aabb_min, aabb_max;
  cow->getAABB(aabb_min, aabb_max);

  int type = cow->getCollisionShape()->getShapeType();
  cow->setBroadphaseHandle(broadphase->createProxy(aabb_min, aabb_max, type, cow.get(), cow->m_collisionFilterGroup,
                                                   cow->m_collisionFilterMask, dispatcher.get()));
}

struct BroadphaseFilterCallback : public btOverlapFilterCallback
{
  bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override
  {
    bool cull = !(proxy0->m_collisionFilterMask & proxy1->m_collisionFilterGroup);
    cull = cull || !(proxy1->m_collisionFilterMask & proxy0->m_collisionFilterGroup);

    if (cull)
      return false;

    const CollisionObjectWrapper* cow0 = static_cast<const CollisionObjectWrapper*>(proxy0->m_clientObject);
    const CollisionObjectWrapper* cow1 = static_cast<const CollisionObjectWrapper*>(proxy1->m_clientObject);

    if (!cow0->m_enabled)
      return false;

    if (!cow1->m_enabled)
      return false;

    if (cow0->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
        cow1->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
      if (cow0->m_touch_links.find(cow1->getName()) != cow0->m_touch_links.end())
        return false;

    if (cow1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
        cow0->getTypeID() == collision_detection::BodyType::ROBOT_LINK)
      if (cow1->m_touch_links.find(cow0->getName()) != cow1->m_touch_links.end())
        return false;

    if (cow0->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED &&
        cow1->getTypeID() == collision_detection::BodyType::ROBOT_ATTACHED)
      if (cow0->m_touch_links == cow1->m_touch_links)
        return false;

    ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                           "Broadphase pass " << cow0->getName() << " vs " << cow1->getName());
    return true;
  }
};

}  // namespace collision_detection_bullet
