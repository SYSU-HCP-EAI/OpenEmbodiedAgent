/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Mrinal Kalakrishnan, Ken Anderson, E. Gil Jones */

#include <moveit/distance_field/distance_field.h>
#include <moveit/distance_field/find_internal_points.h>
#include <geometric_shapes/body_operations.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ros/console.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace distance_field
{
DistanceField::DistanceField(double size_x, double size_y, double size_z, double resolution, double origin_x,
                             double origin_y, double origin_z)
  : size_x_(size_x)
  , size_y_(size_y)
  , size_z_(size_z)
  , origin_x_(origin_x)
  , origin_y_(origin_y)
  , origin_z_(origin_z)
  , resolution_(resolution)
  , inv_twice_resolution_(1.0 / (2.0 * resolution_))
{
}

DistanceField::~DistanceField() = default;

double DistanceField::getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y,
                                          double& gradient_z, bool& in_bounds) const
{
  int gx, gy, gz;

  worldToGrid(x, y, z, gx, gy, gz);

  // if out of bounds, return max_distance, and 0 gradient
  // we need extra padding of 1 to get gradients
  if (gx < 1 || gy < 1 || gz < 1 || gx >= getXNumCells() - 1 || gy >= getYNumCells() - 1 || gz >= getZNumCells() - 1)
  {
    gradient_x = 0.0;
    gradient_y = 0.0;
    gradient_z = 0.0;
    in_bounds = false;
    return getUninitializedDistance();
  }

  gradient_x = (getDistance(gx + 1, gy, gz) - getDistance(gx - 1, gy, gz)) * inv_twice_resolution_;
  gradient_y = (getDistance(gx, gy + 1, gz) - getDistance(gx, gy - 1, gz)) * inv_twice_resolution_;
  gradient_z = (getDistance(gx, gy, gz + 1) - getDistance(gx, gy, gz - 1)) * inv_twice_resolution_;

  in_bounds = true;
  return getDistance(gx, gy, gz);
}

void DistanceField::getIsoSurfaceMarkers(double min_distance, double max_distance, const std::string& frame_id,
                                         const ros::Time stamp, visualization_msgs::Marker& inf_marker) const
{
  inf_marker.points.clear();
  inf_marker.header.frame_id = frame_id;
  inf_marker.header.stamp = stamp;
  inf_marker.ns = "distance_field";
  inf_marker.id = 1;
  inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
  inf_marker.action = visualization_msgs::Marker::MODIFY;
  inf_marker.scale.x = resolution_;
  inf_marker.scale.y = resolution_;
  inf_marker.scale.z = resolution_;
  inf_marker.color.r = 1.0;
  inf_marker.color.g = 0.0;
  inf_marker.color.b = 0.0;
  inf_marker.color.a = 0.1;
  // inf_marker.lifetime = ros::Duration(30.0);

  inf_marker.points.reserve(100000);
  for (int x = 0; x < getXNumCells(); ++x)
  {
    for (int y = 0; y < getYNumCells(); ++y)
    {
      for (int z = 0; z < getZNumCells(); ++z)
      {
        double dist = getDistance(x, y, z);

        if (dist >= min_distance && dist <= max_distance)
        {
          int last = inf_marker.points.size();
          inf_marker.points.resize(last + 1);
          double nx, ny, nz;
          gridToWorld(x, y, z, nx, ny, nz);
          Eigen::Translation3d vec(nx, ny, nz);
          inf_marker.points[last].x = vec.x();
          inf_marker.points[last].y = vec.y();
          inf_marker.points[last].z = vec.z();
        }
      }
    }
  }
}

void DistanceField::getGradientMarkers(double min_distance, double max_distance, const std::string& frame_id,
                                       const ros::Time& stamp, visualization_msgs::MarkerArray& marker_array) const
{
  Eigen::Vector3d unit_x(1, 0, 0);
  Eigen::Vector3d unit_y(0, 1, 0);
  Eigen::Vector3d unit_z(0, 0, 1);

  int id = 0;

  for (int x = 0; x < getXNumCells(); ++x)
  {
    for (int y = 0; y < getYNumCells(); ++y)
    {
      for (int z = 0; z < getZNumCells(); ++z)
      {
        double world_x, world_y, world_z;
        gridToWorld(x, y, z, world_x, world_y, world_z);

        double gradient_x, gradient_y, gradient_z;
        bool in_bounds;
        double distance = getDistanceGradient(world_x, world_y, world_z, gradient_x, gradient_y, gradient_z, in_bounds);
        Eigen::Vector3d gradient(gradient_x, gradient_y, gradient_z);

        if (in_bounds && distance >= min_distance && distance <= max_distance && gradient.norm() > 0)
        {
          visualization_msgs::Marker marker;

          marker.header.frame_id = frame_id;
          marker.header.stamp = stamp;

          marker.ns = "distance_field_gradient";
          marker.id = id++;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = world_x;
          marker.pose.position.y = world_y;
          marker.pose.position.z = world_z;

          // Eigen::Vector3d axis = gradient.cross(unitX).norm() > 0 ? gradient.cross(unitX) : unitY;
          // double angle = -gradient.angle(unitX);
          // Eigen::AngleAxisd rotation(angle, axis);

          // marker.pose.orientation.x = rotation.linear().x();
          // marker.pose.orientation.y = rotation.linear().y();
          // marker.pose.orientation.z = rotation.linear().z();
          // marker.pose.orientation.w = rotation.linear().w();

          marker.scale.x = getResolution();
          marker.scale.y = getResolution();
          marker.scale.z = getResolution();

          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;

          marker_array.markers.push_back(marker);
        }
      }
    }
  }
}

bool DistanceField::getShapePoints(const shapes::Shape* shape, const Eigen::Isometry3d& pose,
                                   EigenSTL::vector_Vector3d* points)
{
  if (shape->type == shapes::OCTREE)
  {
    const shapes::OcTree* oc = dynamic_cast<const shapes::OcTree*>(shape);
    if (!oc)
    {
      ROS_ERROR_NAMED("distance_field", "Problem dynamic casting shape that claims to be OcTree");
      return false;
    }
    getOcTreePoints(oc->octree.get(), points);
  }
  else
  {
    bodies::Body* body = bodies::createEmptyBodyFromShapeType(shape->type);
    body->setDimensionsDirty(shape);
    body->setPoseDirty(pose);
    body->updateInternalData();
    findInternalPointsConvex(*body, resolution_, *points);
    delete body;
  }
  return true;
}

void DistanceField::addShapeToField(const shapes::Shape* shape, const Eigen::Isometry3d& pose)
{
  EigenSTL::vector_Vector3d point_vec;
  getShapePoints(shape, pose, &point_vec);
  addPointsToField(point_vec);
}

// DEPRECATED
void DistanceField::addShapeToField(const shapes::Shape* shape, const geometry_msgs::Pose& pose)
{
  Eigen::Isometry3d pose_e;
  tf2::fromMsg(pose, pose_e);
  addShapeToField(shape, pose_e);
}

void DistanceField::getOcTreePoints(const octomap::OcTree* octree, EigenSTL::vector_Vector3d* points)
{
  // lower extent
  double min_x, min_y, min_z;
  gridToWorld(0, 0, 0, min_x, min_y, min_z);

  octomap::point3d bbx_min(min_x, min_y, min_z);

  int num_x = getXNumCells();
  int num_y = getYNumCells();
  int num_z = getZNumCells();

  // upper extent
  double max_x, max_y, max_z;
  gridToWorld(num_x, num_y, num_z, max_x, max_y, max_z);

  octomap::point3d bbx_max(max_x, max_y, max_z);

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbx_min, bbx_max), end = octree->end_leafs_bbx();
       it != end; ++it)
  {
    if (octree->isNodeOccupied(*it))
    {
      if (it.getSize() <= resolution_)
      {
        Eigen::Vector3d point(it.getX(), it.getY(), it.getZ());
        points->push_back(point);
      }
      else
      {
        double ceil_val = ceil(it.getSize() / resolution_) * resolution_ / 2.0;
        for (double x = it.getX() - ceil_val; x <= it.getX() + ceil_val; x += resolution_)
        {
          for (double y = it.getY() - ceil_val; y <= it.getY() + ceil_val; y += resolution_)
          {
            for (double z = it.getZ() - ceil_val; z <= it.getZ() + ceil_val; z += resolution_)
            {
              points->push_back(Eigen::Vector3d(x, y, z));
            }
          }
        }
      }
    }
  }
}

void DistanceField::addOcTreeToField(const octomap::OcTree* octree)
{
  EigenSTL::vector_Vector3d points;
  getOcTreePoints(octree, &points);
  addPointsToField(points);
}

void DistanceField::moveShapeInField(const shapes::Shape* shape, const Eigen::Isometry3d& old_pose,
                                     const Eigen::Isometry3d& new_pose)
{
  if (shape->type == shapes::OCTREE)
  {
    ROS_WARN_NAMED("distance_field", "Move shape not supported for Octree");
    return;
  }
  bodies::Body* body = bodies::createEmptyBodyFromShapeType(shape->type);
  body->setDimensionsDirty(shape);
  body->setPoseDirty(old_pose);
  body->updateInternalData();
  EigenSTL::vector_Vector3d old_point_vec;
  findInternalPointsConvex(*body, resolution_, old_point_vec);
  body->setPose(new_pose);
  EigenSTL::vector_Vector3d new_point_vec;
  findInternalPointsConvex(*body, resolution_, new_point_vec);
  delete body;
  updatePointsInField(old_point_vec, new_point_vec);
}

// DEPRECATED
void DistanceField::moveShapeInField(const shapes::Shape* shape, const geometry_msgs::Pose& old_pose,
                                     const geometry_msgs::Pose& new_pose)
{
  Eigen::Isometry3d old_pose_e, new_pose_e;
  tf2::fromMsg(old_pose, old_pose_e);
  tf2::fromMsg(new_pose, new_pose_e);
  moveShapeInField(shape, old_pose_e, new_pose_e);
}

void DistanceField::removeShapeFromField(const shapes::Shape* shape, const Eigen::Isometry3d& pose)
{
  bodies::Body* body = bodies::createEmptyBodyFromShapeType(shape->type);
  body->setDimensionsDirty(shape);
  body->setPoseDirty(pose);
  body->updateInternalData();
  EigenSTL::vector_Vector3d point_vec;
  findInternalPointsConvex(*body, resolution_, point_vec);
  delete body;
  removePointsFromField(point_vec);
}

// DEPRECATED
void DistanceField::removeShapeFromField(const shapes::Shape* shape, const geometry_msgs::Pose& pose)
{
  Eigen::Isometry3d pose_e;
  tf2::fromMsg(pose, pose_e);
  removeShapeFromField(shape, pose_e);
}

void DistanceField::getPlaneMarkers(PlaneVisualizationType type, double length, double width, double height,
                                    const Eigen::Vector3d& origin, const std::string& frame_id, const ros::Time stamp,
                                    visualization_msgs::Marker& plane_marker) const
{
  plane_marker.header.frame_id = frame_id;
  plane_marker.header.stamp = stamp;
  plane_marker.ns = "distance_field_plane";
  plane_marker.id = 1;
  plane_marker.type = visualization_msgs::Marker::CUBE_LIST;
  plane_marker.action = visualization_msgs::Marker::ADD;
  plane_marker.scale.x = resolution_;
  plane_marker.scale.y = resolution_;
  plane_marker.scale.z = resolution_;
  // plane_marker.lifetime = ros::Duration(30.0);

  plane_marker.points.reserve(100000);
  plane_marker.colors.reserve(100000);

  double min_x = 0;
  double max_x = 0;
  double min_y = 0;
  double max_y = 0;
  double min_z = 0;
  double max_z = 0;

  switch (type)
  {
    case XY_PLANE:
      min_z = height;
      max_z = height;

      min_x = -length / 2.0;
      max_x = length / 2.0;
      min_y = -width / 2.0;
      max_y = width / 2.0;
      break;
    case XZ_PLANE:
      min_y = height;
      max_y = height;

      min_x = -length / 2.0;
      max_x = length / 2.0;
      min_z = -width / 2.0;
      max_z = width / 2.0;
      break;
    case YZ_PLANE:
      min_x = height;
      max_x = height;

      min_y = -length / 2.0;
      max_y = length / 2.0;
      min_z = -width / 2.0;
      max_z = width / 2.0;
      break;
  }

  min_x += origin.x();
  min_y += origin.y();
  min_z += origin.z();

  max_x += origin.x();
  max_y += origin.y();
  max_z += origin.z();

  int min_x_cell = 0;
  int max_x_cell = 0;
  int min_y_cell = 0;
  int max_y_cell = 0;
  int min_z_cell = 0;
  int max_z_cell = 0;

  worldToGrid(min_x, min_y, min_z, min_x_cell, min_y_cell, min_z_cell);
  worldToGrid(max_x, max_y, max_z, max_x_cell, max_y_cell, max_z_cell);
  plane_marker.color.a = 1.0;
  for (int x = min_x_cell; x <= max_x_cell; ++x)
  {
    for (int y = min_y_cell; y <= max_y_cell; ++y)
    {
      for (int z = min_z_cell; z <= max_z_cell; ++z)
      {
        if (!isCellValid(x, y, z))
        {
          continue;
        }
        double dist = getDistance(x, y, z);
        int last = plane_marker.points.size();
        plane_marker.points.resize(last + 1);
        plane_marker.colors.resize(last + 1);
        double nx, ny, nz;
        gridToWorld(x, y, z, nx, ny, nz);
        Eigen::Vector3d vec(nx, ny, nz);
        plane_marker.points[last].x = vec.x();
        plane_marker.points[last].y = vec.y();
        plane_marker.points[last].z = vec.z();
        if (dist < 0.0)
        {
          plane_marker.colors[last].r = fmax(fmin(0.1 / fabs(dist), 1.0), 0.0);
          plane_marker.colors[last].g = fmax(fmin(0.05 / fabs(dist), 1.0), 0.0);
          plane_marker.colors[last].b = fmax(fmin(0.01 / fabs(dist), 1.0), 0.0);
        }
        else
        {
          plane_marker.colors[last].b = fmax(fmin(0.1 / (dist + 0.001), 1.0), 0.0);
          plane_marker.colors[last].g = fmax(fmin(0.05 / (dist + 0.001), 1.0), 0.0);
          plane_marker.colors[last].r = fmax(fmin(0.01 / (dist + 0.001), 1.0), 0.0);
        }
      }
    }
  }
}

void DistanceField::setPoint(int xCell, int yCell, int zCell, double dist, geometry_msgs::Point& point,
                             std_msgs::ColorRGBA& color, double max_distance) const
{
  double wx, wy, wz;
  gridToWorld(xCell, yCell, zCell, wx, wy, wz);

  point.x = wx;
  point.y = wy;
  point.z = wz;

  color.r = 1.0;
  color.g = dist / max_distance;  // dist/max_distance * 0.5;
  color.b = dist / max_distance;  // dist/max_distance * 0.1;
}

void DistanceField::getProjectionPlanes(const std::string& frame_id, const ros::Time& stamp, double max_dist,
                                        visualization_msgs::Marker& marker) const
{
  int max_x_cell = getXNumCells();
  int max_y_cell = getYNumCells();
  int max_z_cell = getZNumCells();

  double* x_projection = new double[max_y_cell * max_z_cell];
  double* y_projection = new double[max_z_cell * max_x_cell];
  double* z_projection = new double[max_x_cell * max_y_cell];
  double initial_val = sqrt(INT_MAX);

  // Initialize
  for (int y = 0; y < max_y_cell; y++)
    for (int x = 0; x < max_x_cell; x++)
      z_projection[x + y * max_x_cell] = initial_val;

  for (int z = 0; z < max_z_cell; z++)
    for (int y = 0; y < max_y_cell; y++)
      x_projection[y + z * max_y_cell] = initial_val;

  for (int z = 0; z < max_z_cell; z++)
    for (int x = 0; x < max_x_cell; x++)
      y_projection[x + z * max_x_cell] = initial_val;

  // Calculate projections
  for (int z = 0; z < max_z_cell; z++)
  {
    for (int y = 0; y < max_y_cell; y++)
    {
      for (int x = 0; x < max_x_cell; x++)
      {
        double dist = getDistance(x, y, z);
        z_projection[x + y * max_x_cell] = std::min(dist, z_projection[x + y * max_x_cell]);
        x_projection[y + z * max_y_cell] = std::min(dist, x_projection[y + z * max_y_cell]);
        y_projection[x + z * max_x_cell] = std::min(dist, y_projection[x + z * max_x_cell]);
      }
    }
  }

  // Make markers
  marker.points.clear();
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "distance_field_projection_plane";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.scale.x = getResolution();
  marker.scale.y = getResolution();
  marker.scale.z = getResolution();
  marker.color.a = 1.0;
  // marker.lifetime = ros::Duration(30.0);

  int x, y, z;
  int index = 0;
  marker.points.resize(max_x_cell * max_y_cell + max_y_cell * max_z_cell + max_z_cell * max_x_cell);
  marker.colors.resize(max_x_cell * max_y_cell + max_y_cell * max_z_cell + max_z_cell * max_x_cell);

  z = 0;
  for (y = 0; y < max_y_cell; y++)
  {
    for (x = 0; x < max_x_cell; x++)
    {
      double dist = z_projection[x + y * max_x_cell];
      setPoint(x, y, z, dist, marker.points[index], marker.colors[index], max_dist);
      index++;
    }
  }

  x = 0;
  for (z = 0; z < max_z_cell; z++)
  {
    for (y = 0; y < max_y_cell; y++)
    {
      double dist = x_projection[y + z * max_y_cell];
      setPoint(x, y, z, dist, marker.points[index], marker.colors[index], max_dist);
      index++;
    }
  }

  y = 0;
  for (z = 0; z < max_z_cell; z++)
  {
    for (x = 0; x < max_x_cell; x++)
    {
      double dist = y_projection[x + z * max_x_cell];
      setPoint(x, y, z, dist, marker.points[index], marker.colors[index], max_dist);
      index++;
    }
  }

  if (x_projection)
    delete[] x_projection;
  if (y_projection)
    delete[] y_projection;
  if (z_projection)
    delete[] z_projection;
}

}  // end of namespace distance_field
