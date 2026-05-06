/*********************************************************************
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
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

#include <octomap_msgs/conversions.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>

#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>

namespace collision_detection_bullet
{
/** \brief Recursively traverses robot from root to get all active links
 *
 *   \param active_links Stores the active links
 *   \param urdf_link The current urdf link representation
 *   \param active Indicates if link is considered active */
static inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                               const urdf::LinkConstSharedPtr& urdf_link, bool active)
{
  if (active)
  {
    active_links.push_back(urdf_link->name);
    for (const auto& child_link : urdf_link->child_links)
    {
      getActiveLinkNamesRecursive(active_links, child_link, active);
    }
  }
  else
  {
    for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
    {
      const urdf::LinkConstSharedPtr child_link = urdf_link->child_links[i];
      if ((child_link->parent_joint) && (child_link->parent_joint->type != urdf::Joint::FIXED))
        getActiveLinkNamesRecursive(active_links, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, child_link, active);
    }
  }
}

inline shapes::ShapePtr constructShape(const urdf::Geometry* geom)
{
  shapes::Shape* result = nullptr;
  switch (geom->type)
  {
    case urdf::Geometry::SPHERE:
      result = new shapes::Sphere(static_cast<const urdf::Sphere*>(geom)->radius);
      break;
    case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = static_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
    case urdf::Geometry::CYLINDER:
      result = new shapes::Cylinder(static_cast<const urdf::Cylinder*>(geom)->radius,
                                    static_cast<const urdf::Cylinder*>(geom)->length);
      break;
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh* mesh = static_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        shapes::Mesh* m = shapes::createMeshFromResource(mesh->filename, scale);
        result = m;
      }
    }
    break;
    default:
      ROS_ERROR("Unknown geometry type: %d", static_cast<int>(geom->type));
      break;
  }

  return shapes::ShapePtr(result);
}

inline Eigen::Isometry3d urdfPose2Eigen(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}
}  // namespace collision_detection_bullet
