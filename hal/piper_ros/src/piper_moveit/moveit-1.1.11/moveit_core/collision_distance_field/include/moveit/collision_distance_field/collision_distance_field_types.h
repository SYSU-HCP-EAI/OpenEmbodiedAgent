/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: E. Gil Jones */

#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <float.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <octomap/OcTree.h>

#include <moveit/macros/class_forward.h>
#include <moveit/distance_field/distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/console.h>

namespace collision_detection
{
enum CollisionType
{
  NONE = 0,
  SELF = 1,
  INTRA = 2,
  ENVIRONMENT = 3,
};

struct CollisionSphere
{
  CollisionSphere(const Eigen::Vector3d& rel, double radius)
  {
    relative_vec_ = rel;
    radius_ = radius;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d relative_vec_;
  double radius_;
};

struct GradientInfo
{
  GradientInfo() : closest_distance(DBL_MAX), collision(false)
  {
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double closest_distance;
  bool collision;
  EigenSTL::vector_Vector3d sphere_locations;
  std::vector<double> distances;
  EigenSTL::vector_Vector3d gradients;
  std::vector<CollisionType> types;
  std::vector<double> sphere_radii;
  std::string joint_name;

  void clear()
  {
    closest_distance = DBL_MAX;
    collision = false;
    sphere_locations.clear();
    distances.clear();
    gradients.clear();
    sphere_radii.clear();
    joint_name.clear();
  }
};

MOVEIT_CLASS_FORWARD(PosedDistanceField);
MOVEIT_CLASS_FORWARD(BodyDecomposition);  // Defines BodyDecompositionPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(PosedBodySphereDecomposition);
MOVEIT_CLASS_FORWARD(PosedBodyPointDecomposition);
MOVEIT_CLASS_FORWARD(PosedBodySphereDecompositionVector);
MOVEIT_CLASS_FORWARD(PosedBodyPointDecompositionVector);

class PosedDistanceField : public distance_field::PropagationDistanceField
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedDistanceField(const Eigen::Vector3d& size, const Eigen::Vector3d& origin, double resolution, double max_distance,
                     bool propagate_negative_distances = false)
    : distance_field::PropagationDistanceField(size.x(), size.y(), size.z(), resolution, origin.x(), origin.y(),
                                               origin.z(), max_distance, propagate_negative_distances)
    , pose_(Eigen::Isometry3d::Identity())
  {
  }

  void updatePose(const Eigen::Isometry3d& transform)
  {
    pose_ = transform;
  }

  const Eigen::Isometry3d& getPose() const
  {
    return pose_;
  }

  /**
   * @brief Gets not only the distance to the nearest cell but the gradient
   * direction. The point
   * defined by the x, y and z values is transform into the local distance field
   * coordinate system using
   * the current pose.
   * The gradient is computed as a function of the distances of near-by cells.
   * An uninitialized
   *  distance is returned if the cell is not valid for gradient production
   * purposes.
   * @param x input x value of the query point
   * @param y input y value of the query point
   * @param z input z value of the query point
   * @param gradient_x output argument with the x value of the gradient
   * @param gradient_y output argument with the y value of the gradient
   * @param gradient_z output argument with the z value of the gradient
   * @param in_bounds true if the point is within the bounds of the distance
   * field, false otherwise
   */
  double getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z,
                             bool& in_bounds) const
  {
    Eigen::Vector3d rel_pos = pose_.inverse() * Eigen::Vector3d(x, y, z);
    double gx, gy, gz;
    double res = distance_field::PropagationDistanceField::getDistanceGradient(rel_pos.x(), rel_pos.y(), rel_pos.z(),
                                                                               gx, gy, gz, in_bounds);
    Eigen::Vector3d grad = pose_ * Eigen::Vector3d(gx, gy, gz);
    gradient_x = grad.x();
    gradient_y = grad.y();
    gradient_z = grad.z();
    return res;
  }

  /*
   * @brief determines a set of gradients of the given collision spheres in the
   * distance field
   * @param sphere_list vector of the spheres that approximate a links geometry
   * @param sphere_centers vector of points which indicate the center of each
   * sphere in sphere_list
   * @param gradient output argument to be populated with the resulting gradient
   * calculation
   * @param tolerance
   * @param subtract_radii distance to the sphere centers will be computed by
   * substracting the sphere radius from the nearest point
   * @param maximum_value
   * @param stop_at_first_collision when true the computation is terminated when
   * the first collision is found
   */
  bool getCollisionSphereGradients(const std::vector<CollisionSphere>& sphere_list,
                                   const EigenSTL::vector_Vector3d& sphere_centers, GradientInfo& gradient,
                                   const CollisionType& type, double tolerance, bool subtract_radii,
                                   double maximum_value, bool stop_at_first_collision);

protected:
  Eigen::Isometry3d pose_;
};

// determines set of collision spheres given a posed body; this is BAD!
// Allocation erorrs will happen; change this function so it does not return
// that vector by value
std::vector<CollisionSphere> determineCollisionSpheres(const bodies::Body* body, Eigen::Isometry3d& relativeTransform);

// determines a set of gradients of the given collision spheres in the distance
// field
bool getCollisionSphereGradients(const distance_field::DistanceField* distance_field,
                                 const std::vector<CollisionSphere>& sphere_list,
                                 const EigenSTL::vector_Vector3d& sphere_centers, GradientInfo& gradient,
                                 const CollisionType& type, double tolerance, bool subtract_radii, double maximum_value,
                                 bool stop_at_first_collision);

bool getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                 const std::vector<CollisionSphere>& sphere_list,
                                 const EigenSTL::vector_Vector3d& sphere_centers, double maximum_value,
                                 double tolerance);

bool getCollisionSphereCollision(const distance_field::DistanceField* distance_field,
                                 const std::vector<CollisionSphere>& sphere_list,
                                 const EigenSTL::vector_Vector3d& sphere_centers, double maximum_value,
                                 double tolerance, unsigned int num_coll, std::vector<unsigned int>& colls);

// forward declaration required for friending apparently
class BodyDecompositionVector;

class BodyDecomposition
{
  friend class BodyDecompositionVector;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BodyDecomposition(const shapes::ShapeConstPtr& shape, double resolution, double padding = 0.01);

  BodyDecomposition(const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& poses,
                    double resolution, double padding);

  ~BodyDecomposition();

  Eigen::Isometry3d relative_cylinder_pose_;

  void replaceCollisionSpheres(const std::vector<CollisionSphere>& new_collision_spheres,
                               const Eigen::Isometry3d& new_relative_cylinder_pose)
  {
    // std::cerr << "Replacing " << collision_spheres_.size() << " with " <<
    // new_collision_spheres.size() << std::endl;
    collision_spheres_ = new_collision_spheres;
    relative_cylinder_pose_ = new_relative_cylinder_pose;
  }

  const std::vector<CollisionSphere>& getCollisionSpheres() const
  {
    return collision_spheres_;
  }

  const std::vector<double>& getSphereRadii() const
  {
    return sphere_radii_;
  }

  const EigenSTL::vector_Vector3d& getCollisionPoints() const
  {
    return relative_collision_points_;
  }

  const bodies::Body* getBody(unsigned int i) const
  {
    return bodies_.getBody(i);
  }

  unsigned int getBodiesCount()
  {
    return bodies_.getCount();
  }

  Eigen::Isometry3d getRelativeCylinderPose() const
  {
    return relative_cylinder_pose_;
  }

  const bodies::BoundingSphere& getRelativeBoundingSphere() const
  {
    return relative_bounding_sphere_;
  }

protected:
  void init(const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& poses,
            double resolution, double padding);

protected:
  bodies::BodyVector bodies_;

  bodies::BoundingSphere relative_bounding_sphere_;
  std::vector<double> sphere_radii_;
  std::vector<CollisionSphere> collision_spheres_;
  EigenSTL::vector_Vector3d relative_collision_points_;
};

class PosedBodySphereDecomposition
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedBodySphereDecomposition(const BodyDecompositionConstPtr& body_decomposition);

  const std::vector<CollisionSphere>& getCollisionSpheres() const
  {
    return body_decomposition_->getCollisionSpheres();
  }

  const EigenSTL::vector_Vector3d& getSphereCenters() const
  {
    return sphere_centers_;
  }

  const EigenSTL::vector_Vector3d& getCollisionPoints() const
  {
    return posed_collision_points_;
  }

  const std::vector<double>& getSphereRadii() const
  {
    return body_decomposition_->getSphereRadii();
  }
  const Eigen::Vector3d& getBoundingSphereCenter() const
  {
    return posed_bounding_sphere_center_;
  }

  double getBoundingSphereRadius() const
  {
    return body_decomposition_->getRelativeBoundingSphere().radius;
  }

  // assumed to be in reference frame, updates the pose of the body,
  // the collision spheres, and the posed collision points
  void updatePose(const Eigen::Isometry3d& linkTransform);

protected:
  BodyDecompositionConstPtr body_decomposition_;
  Eigen::Vector3d posed_bounding_sphere_center_;
  EigenSTL::vector_Vector3d posed_collision_points_;
  EigenSTL::vector_Vector3d sphere_centers_;
};

class PosedBodyPointDecomposition
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedBodyPointDecomposition(const BodyDecompositionConstPtr& body_decomposition);

  PosedBodyPointDecomposition(const BodyDecompositionConstPtr& body_decomposition, const Eigen::Isometry3d& pose);

  PosedBodyPointDecomposition(const std::shared_ptr<const octomap::OcTree>& octree);

  const EigenSTL::vector_Vector3d& getCollisionPoints() const
  {
    return posed_collision_points_;
  }
  // the collision spheres, and the posed collision points
  void updatePose(const Eigen::Isometry3d& linkTransform);

protected:
  BodyDecompositionConstPtr body_decomposition_;
  EigenSTL::vector_Vector3d posed_collision_points_;
};

class PosedBodySphereDecompositionVector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedBodySphereDecompositionVector()
  {
  }

  const std::vector<CollisionSphere>& getCollisionSpheres() const
  {
    return collision_spheres_;
  }

  const EigenSTL::vector_Vector3d& getSphereCenters() const
  {
    return posed_collision_spheres_;
  }

  const std::vector<double>& getSphereRadii() const
  {
    return sphere_radii_;
  }

  void addToVector(PosedBodySphereDecompositionPtr& bd)
  {
    sphere_index_map_[decomp_vector_.size()] = collision_spheres_.size();
    decomp_vector_.push_back(bd);
    collision_spheres_.insert(collision_spheres_.end(), bd->getCollisionSpheres().begin(),
                              bd->getCollisionSpheres().end());
    posed_collision_spheres_.insert(posed_collision_spheres_.end(), bd->getSphereCenters().begin(),
                                    bd->getSphereCenters().end());
    sphere_radii_.insert(sphere_radii_.end(), bd->getSphereRadii().begin(), bd->getSphereRadii().end());
  }

  unsigned int getSize() const
  {
    return decomp_vector_.size();
  }

  PosedBodySphereDecompositionConstPtr getPosedBodySphereDecomposition(unsigned int i) const
  {
    if (i >= decomp_vector_.size())
    {
      ROS_INFO_NAMED("collision_distance_field", "No body decomposition");
      return empty_ptr_;
    }
    return decomp_vector_[i];
  }

  void updatePose(unsigned int ind, const Eigen::Isometry3d& pose)
  {
    if (ind >= decomp_vector_.size())
    {
      ROS_WARN_NAMED("collision_distance_field", "Can't update pose");
      return;
    }
    decomp_vector_[ind]->updatePose(pose);
    for (unsigned int i = 0; i < decomp_vector_[ind]->getSphereCenters().size(); i++)
    {
      posed_collision_spheres_[sphere_index_map_[ind] + i] = decomp_vector_[ind]->getSphereCenters()[i];
    }
  }

private:
  PosedBodySphereDecompositionConstPtr empty_ptr_;
  std::vector<PosedBodySphereDecompositionPtr> decomp_vector_;
  std::vector<CollisionSphere> collision_spheres_;
  EigenSTL::vector_Vector3d posed_collision_spheres_;
  std::vector<double> sphere_radii_;
  std::map<unsigned int, unsigned int> sphere_index_map_;
};

class PosedBodyPointDecompositionVector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PosedBodyPointDecompositionVector()
  {
  }

  EigenSTL::vector_Vector3d getCollisionPoints() const
  {
    EigenSTL::vector_Vector3d ret_points;
    for (const PosedBodyPointDecompositionPtr& decomp : decomp_vector_)
    {
      ret_points.insert(ret_points.end(), decomp->getCollisionPoints().begin(), decomp->getCollisionPoints().end());
    }
    return ret_points;
  }

  void addToVector(PosedBodyPointDecompositionPtr& bd)
  {
    decomp_vector_.push_back(bd);
  }

  unsigned int getSize() const
  {
    return decomp_vector_.size();
  }

  PosedBodyPointDecompositionConstPtr getPosedBodyDecomposition(unsigned int i) const
  {
    if (i >= decomp_vector_.size())
    {
      ROS_INFO_NAMED("collision_distance_field", "No body decomposition");
      return empty_ptr_;
    }
    return decomp_vector_[i];
  }

  void updatePose(unsigned int ind, const Eigen::Isometry3d& pose)
  {
    if (ind < decomp_vector_.size())
    {
      decomp_vector_[ind]->updatePose(pose);
    }
    else
    {
      ROS_WARN_NAMED("collision_distance_field", "Can't update pose");
      return;
    }
  }

private:
  PosedBodyPointDecompositionPtr empty_ptr_;
  std::vector<PosedBodyPointDecompositionPtr> decomp_vector_;
};

struct ProximityInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::string link_name;
  std::string attached_object_name;
  double proximity;
  unsigned int sphere_index;
  unsigned int att_index;
  Eigen::Vector3d closest_point;
  Eigen::Vector3d closest_gradient;
};

bool doBoundingSpheresIntersect(const PosedBodySphereDecompositionConstPtr& p1,
                                const PosedBodySphereDecompositionConstPtr& p2);

void getCollisionSphereMarkers(const std_msgs::ColorRGBA& color, const std::string& frame_id, const std::string& ns,
                               const ros::Duration& dur,
                               const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                               visualization_msgs::MarkerArray& arr);

void getProximityGradientMarkers(const std::string& frame_id, const std::string& ns, const ros::Duration& dur,
                                 const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                                 const std::vector<PosedBodySphereDecompositionVectorPtr>& posed_vector_decompositions,
                                 const std::vector<GradientInfo>& gradients, visualization_msgs::MarkerArray& arr);

void getCollisionMarkers(const std::string& frame_id, const std::string& ns, const ros::Duration& dur,
                         const std::vector<PosedBodySphereDecompositionPtr>& posed_decompositions,
                         const std::vector<PosedBodySphereDecompositionVectorPtr>& posed_vector_decompositions,
                         const std::vector<GradientInfo>& gradients, visualization_msgs::MarkerArray& arr);
}  // namespace collision_detection
