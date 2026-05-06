/*********************************************************************
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017, Southwest Research Institute
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

/* Author: Levi Armstrong */

#pragma once

#include <LinearMath/btConvexHullComputer.h>
#include <cstdio>
#include <Eigen/Geometry>
#include <fstream>

#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace collision_detection_bullet
{
/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
inline std::pair<std::string, std::string> getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

/**
 * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
inline bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

/** \brief Stores a single contact result in the requested way.
 *   \param found Indicates if a contact for this pair of objects has already been found
 *   \return Pointer to the newly inserted contact */
inline collision_detection::Contact* processResult(ContactTestData& cdata, collision_detection::Contact& contact,
                                                   const std::pair<std::string, std::string>& key, bool found)
{
  // add deepest penetration / smallest distance to result
  if (cdata.req.distance)
  {
    if (contact.depth < cdata.res.distance)
    {
      cdata.res.distance = contact.depth;
    }
  }

  ROS_DEBUG_STREAM_NAMED("collision_detection.bullet",
                         "Contact btw " << key.first << " and " << key.second << " dist: " << contact.depth);
  // case if pair hasn't a contact yet
  if (!found)
  {
    if (contact.depth <= 0)
    {
      cdata.res.collision = true;
    }

    std::vector<collision_detection::Contact> data;

    // if we dont want contacts we are done here
    if (!cdata.req.contacts)
    {
      if (!cdata.req.distance)
      {
        cdata.done = true;
      }
      return nullptr;
    }
    else
    {
      data.reserve(cdata.req.max_contacts_per_pair);
      data.emplace_back(contact);
      cdata.res.contact_count++;
    }

    if (cdata.res.contact_count >= cdata.req.max_contacts)
    {
      if (!cdata.req.distance)
      {
        cdata.done = true;
      }
    }

    if (cdata.req.max_contacts_per_pair == 1u)
    {
      cdata.pair_done = true;
    }

    return &(cdata.res.contacts.insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    std::vector<collision_detection::Contact>& dr = cdata.res.contacts[key];
    dr.emplace_back(contact);
    cdata.res.contact_count++;

    if (dr.size() >= cdata.req.max_contacts_per_pair)
    {
      cdata.pair_done = true;
    }

    if (cdata.res.contact_count >= cdata.req.max_contacts)
    {
      if (!cdata.req.distance)
      {
        cdata.done = true;
      }
    }

    return &(dr.back());
  }

  return nullptr;
}

/**
 * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
 * @param (Output) vertices A vector of vertices
 * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice
 * index
 * @param (input) input A vector of point to create a convex hull from
 * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
 *                units towards the center along its normal).
 * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
 *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
 * @return The number of faces. If less than zero an error occured when trying to create the convex hull
 */
inline int createConvexHull(AlignedVector<Eigen::Vector3d>& vertices, std::vector<int>& faces,
                            const AlignedVector<Eigen::Vector3d>& input, double shrink = -1, double shrinkClamp = -1)
{
  vertices.clear();
  faces.clear();

  btConvexHullComputer conv;
  btAlignedObjectArray<btVector3> points;
  points.reserve(static_cast<int>(input.size()));
  for (const Eigen::Vector3d& v : input)
  {
    points.push_back(btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));
  }

  btScalar val = conv.compute(&points[0].getX(), sizeof(btVector3), points.size(), static_cast<btScalar>(shrink),
                              static_cast<btScalar>(shrinkClamp));
  if (val < 0)
  {
    ROS_ERROR("Failed to create convex hull");
    return -1;
  }

  int num_verts = conv.vertices.size();
  vertices.reserve(static_cast<size_t>(num_verts));
  for (int i = 0; i < num_verts; i++)
  {
    btVector3& v = conv.vertices[i];
    vertices.push_back(Eigen::Vector3d(v.getX(), v.getY(), v.getZ()));
  }

  int num_faces = conv.faces.size();
  faces.reserve(static_cast<size_t>(3 * num_faces));
  for (int i = 0; i < num_faces; i++)
  {
    std::vector<int> face;
    face.reserve(3);

    const btConvexHullComputer::Edge* source_edge = &(conv.edges[conv.faces[i]]);
    int a = source_edge->getSourceVertex();
    face.push_back(a);

    int b = source_edge->getTargetVertex();
    face.push_back(b);

    const btConvexHullComputer::Edge* edge = source_edge->getNextEdgeOfFace();
    int c = edge->getTargetVertex();
    face.push_back(c);

    edge = edge->getNextEdgeOfFace();
    c = edge->getTargetVertex();
    while (c != a)
    {
      face.push_back(c);

      edge = edge->getNextEdgeOfFace();
      c = edge->getTargetVertex();
    }
    faces.push_back(static_cast<int>(face.size()));
    faces.insert(faces.end(), face.begin(), face.end());
  }

  return num_faces;
}

}  // namespace collision_detection_bullet
