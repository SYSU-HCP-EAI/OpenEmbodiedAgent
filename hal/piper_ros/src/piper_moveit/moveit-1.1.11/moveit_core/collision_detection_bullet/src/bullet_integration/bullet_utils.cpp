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

#include "moveit/collision_detection_bullet/bullet_integration/bullet_utils.h"

#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <geometric_shapes/shapes.h>
#include <memory>
#include <octomap/octomap.h>
#include <ros/console.h>

namespace collision_detection_bullet
{
btCollisionShape* createShapePrimitive(const shapes::Box* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  const double* size = geom->size;
  btScalar a = static_cast<btScalar>(size[0] / 2);
  btScalar b = static_cast<btScalar>(size[1] / 2);
  btScalar c = static_cast<btScalar>(size[2] / 2);

  return (new btBoxShape(btVector3(a, b, c)));
}

btCollisionShape* createShapePrimitive(const shapes::Sphere* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  return (new btSphereShape(static_cast<btScalar>(geom->radius)));
}

btCollisionShape* createShapePrimitive(const shapes::Cylinder* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  btScalar r = static_cast<btScalar>(geom->radius);
  btScalar l = static_cast<btScalar>(geom->length / 2);
  return (new btCylinderShapeZ(btVector3(r, r, l)));
}

btCollisionShape* createShapePrimitive(const shapes::Cone* geom, const CollisionObjectType& collision_object_type)
{
  (void)(collision_object_type);
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE);
  btScalar r = static_cast<btScalar>(geom->radius);
  btScalar l = static_cast<btScalar>(geom->length);
  return (new btConeShapeZ(r, l));
}

btCollisionShape* createShapePrimitive(const shapes::Mesh* geom, const CollisionObjectType& collision_object_type,
                                       CollisionObjectWrapper* cow)
{
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE ||
         collision_object_type == CollisionObjectType::CONVEX_HULL ||
         collision_object_type == CollisionObjectType::SDF);

  if (geom->vertex_count > 0 && geom->triangle_count > 0)
  {
    // convert the mesh to the assigned collision object type
    switch (collision_object_type)
    {
      case CollisionObjectType::CONVEX_HULL:
      {
        // Create a convex hull shape to approximate Trimesh
        collision_detection_bullet::AlignedVector<Eigen::Vector3d> input;
        collision_detection_bullet::AlignedVector<Eigen::Vector3d> vertices;
        std::vector<int> faces;

        input.reserve(geom->vertex_count);
        for (unsigned int i = 0; i < geom->vertex_count; ++i)
          input.push_back(Eigen::Vector3d(geom->vertices[3 * i], geom->vertices[3 * i + 1], geom->vertices[3 * i + 2]));

        if (collision_detection_bullet::createConvexHull(vertices, faces, input) < 0)
          return nullptr;

        btConvexHullShape* subshape = new btConvexHullShape();
        for (const Eigen::Vector3d& v : vertices)
          subshape->addPoint(
              btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

        return subshape;
      }
      case CollisionObjectType::USE_SHAPE_TYPE:
      {
        btCompoundShape* compound =
            new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(geom->triangle_count));
        compound->setMargin(
            BULLET_MARGIN);  // margin: compound seems to have no effect when positive but has an effect when negative

        for (unsigned i = 0; i < geom->triangle_count; ++i)
        {
          btVector3 v[3];
          for (unsigned x = 0; x < 3; ++x)
          {
            unsigned idx = geom->triangles[3 * i + x];
            for (unsigned y = 0; y < 3; ++y)
            {
              v[x][y] = static_cast<btScalar>(geom->vertices[3 * idx + y]);
            }
          }

          btCollisionShape* subshape = new btTriangleShapeEx(v[0], v[1], v[2]);
          if (subshape != nullptr)
          {
            cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            btTransform geom_trans;
            geom_trans.setIdentity();
            compound->addChildShape(geom_trans, subshape);
          }
        }

        return compound;
      }
      default:
      {
        ROS_ERROR("This bullet shape type (%d) is not supported for geometry meshs",
                  static_cast<int>(collision_object_type));
        return nullptr;
      }
    }
  }
  ROS_ERROR("The mesh is empty!");
  return nullptr;
}

btCollisionShape* createShapePrimitive(const shapes::OcTree* geom, const CollisionObjectType& collision_object_type,
                                       CollisionObjectWrapper* cow)
{
  assert(collision_object_type == CollisionObjectType::USE_SHAPE_TYPE ||
         collision_object_type == CollisionObjectType::CONVEX_HULL ||
         collision_object_type == CollisionObjectType::SDF ||
         collision_object_type == CollisionObjectType::MULTI_SPHERE);

  btCompoundShape* subshape =
      new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(geom->octree->size()));
  double occupancy_threshold = geom->octree->getOccupancyThres();

  // convert the mesh to the assigned collision object type
  switch (collision_object_type)
  {
    case CollisionObjectType::USE_SHAPE_TYPE:
    {
      for (auto it = geom->octree->begin(static_cast<unsigned char>(geom->octree->getTreeDepth())),
                end = geom->octree->end();
           it != end; ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geom_trans;
          geom_trans.setIdentity();
          geom_trans.setOrigin(btVector3(static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()),
                                         static_cast<btScalar>(it.getZ())));
          btScalar l = static_cast<btScalar>(size / 2);
          btBoxShape* childshape = new btBoxShape(btVector3(l, l, l));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geom_trans, childshape);
        }
      }
      return subshape;
    }
    case CollisionObjectType::MULTI_SPHERE:
    {
      for (auto it = geom->octree->begin(static_cast<unsigned char>(geom->octree->getTreeDepth())),
                end = geom->octree->end();
           it != end; ++it)
      {
        if (it->getOccupancy() >= occupancy_threshold)
        {
          double size = it.getSize();
          btTransform geom_trans;
          geom_trans.setIdentity();
          geom_trans.setOrigin(btVector3(static_cast<btScalar>(it.getX()), static_cast<btScalar>(it.getY()),
                                         static_cast<btScalar>(it.getZ())));
          btSphereShape* childshape =
              new btSphereShape(static_cast<btScalar>(std::sqrt(2 * ((size / 2) * (size / 2)))));
          childshape->setMargin(BULLET_MARGIN);
          cow->manage(childshape);

          subshape->addChildShape(geom_trans, childshape);
        }
      }
      return subshape;
    }
    default:
    {
      ROS_ERROR("This bullet shape type (%d) is not supported for geometry octree",
                static_cast<int>(collision_object_type));
      return nullptr;
    }
  }
}

btCollisionShape* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                       const CollisionObjectType& collision_object_type, CollisionObjectWrapper* cow)
{
  switch (geom->type)
  {
    case shapes::BOX:
    {
      return createShapePrimitive(static_cast<const shapes::Box*>(geom.get()), collision_object_type);
    }
    case shapes::SPHERE:
    {
      return createShapePrimitive(static_cast<const shapes::Sphere*>(geom.get()), collision_object_type);
    }
    case shapes::CYLINDER:
    {
      return createShapePrimitive(static_cast<const shapes::Cylinder*>(geom.get()), collision_object_type);
    }
    case shapes::CONE:
    {
      return createShapePrimitive(static_cast<const shapes::Cone*>(geom.get()), collision_object_type);
    }
    case shapes::MESH:
    {
      return createShapePrimitive(static_cast<const shapes::Mesh*>(geom.get()), collision_object_type, cow);
    }
    case shapes::OCTREE:
    {
      return createShapePrimitive(static_cast<const shapes::OcTree*>(geom.get()), collision_object_type, cow);
    }
    default:
    {
      ROS_ERROR("This geometric shape type (%d) is not supported using BULLET yet", static_cast<int>(geom->type));
      return nullptr;
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                               const std::vector<CollisionObjectType>& collision_object_types,
                                               bool active)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
  , m_collision_object_types(collision_object_types)
{
  if (shapes.empty() || shape_poses.empty() ||
      (shapes.size() != shape_poses.size() || collision_object_types.empty() ||
       shapes.size() != collision_object_types.size()))
  {
    throw std::exception();
  }

  this->setContactProcessingThreshold(BULLET_DEFAULT_CONTACT_DISTANCE);
  assert(!name.empty());

  if (active)
  {
    m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    m_collisionFilterMask = btBroadphaseProxy::KinematicFilter | btBroadphaseProxy::StaticFilter;
  }
  else
  {
    m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
    m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
  }

  if (shapes.size() == 1)
  {
    btCollisionShape* shape = createShapePrimitive(m_shapes[0], collision_object_types[0], this);
    shape->setMargin(BULLET_MARGIN);
    manage(shape);
    setCollisionShape(shape);
    setWorldTransform(convertEigenToBt(m_shape_poses[0]));
  }
  else
  {
    btCompoundShape* compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(m_shapes.size()));
    manage(compound);
    // margin on compound seems to have no effect when positive but has an effect when negative
    compound->setMargin(BULLET_MARGIN);
    setCollisionShape(compound);

    setWorldTransform(convertEigenToBt(m_shape_poses[0]));
    Eigen::Isometry3d inv_world = m_shape_poses[0].inverse();

    for (std::size_t j = 0; j < m_shapes.size(); ++j)
    {
      btCollisionShape* subshape = createShapePrimitive(m_shapes[j], collision_object_types[j], this);
      if (subshape != nullptr)
      {
        manage(subshape);
        subshape->setMargin(BULLET_MARGIN);
        btTransform geom_trans = convertEigenToBt(inv_world * m_shape_poses[j]);
        compound->addChildShape(geom_trans, subshape);
      }
    }
  }
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                               const std::vector<CollisionObjectType>& collision_object_types,
                                               const std::set<std::string>& touch_links)
  : CollisionObjectWrapper(name, type_id, shapes, shape_poses, collision_object_types, true)
{
  m_touch_links = touch_links;
}

CollisionObjectWrapper::CollisionObjectWrapper(const std::string& name, const collision_detection::BodyType& type_id,
                                               const std::vector<shapes::ShapeConstPtr>& shapes,
                                               const AlignedVector<Eigen::Isometry3d>& shape_poses,
                                               const std::vector<CollisionObjectType>& collision_object_types,
                                               const std::vector<std::shared_ptr<void>>& data)
  : m_name(name)
  , m_type_id(type_id)
  , m_shapes(shapes)
  , m_shape_poses(shape_poses)
  , m_collision_object_types(collision_object_types)
  , m_data(data)
{
}
}  // namespace collision_detection_bullet
