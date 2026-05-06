/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/distance_field/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <moveit/macros/class_forward.h>

namespace shapes
{
MOVEIT_CLASS_FORWARD(Shape);  // Defines ShapePtr, ConstPtr, WeakPtr... etc
}
namespace octomap
{
class OcTree;
}

/**
 * \brief Namespace for holding classes that generate distance fields.
 *
 * Distance fields are dense 3D representations containing the
 * distance to the nearest obstacles.
 *
 */
namespace distance_field
{
/// \brief The plane to visualize
enum PlaneVisualizationType
{
  XY_PLANE,
  XZ_PLANE,
  YZ_PLANE
};

MOVEIT_CLASS_FORWARD(DistanceField);  // Defines DistanceFieldPtr, ConstPtr, WeakPtr... etc

/**
 * \brief DistanceField is an abstract base class for computing
 * distances from sets of 3D obstacle points.  The distance assigned to
 * a freespace cell should be the distance to the closest obstacle
 * cell.  Cells that are obstacle cells should either be marked as zero
 * distance, or may have a negative distance if a signed version of the
 * distance field is being used and an obstacle point is internal to an
 * obstacle volume.
 *
 * Inherited classes must contain methods for holding a dense set of 3D
 * voxels as well as methods for computing the required distances.  The
 * distance field parent class doesn't hold the data or have any way to
 * generate distances from that data.
 *
 */
class DistanceField
{
public:
  /**
   * \brief Constructor, where units are arbitrary but are assumed to
   * be meters.
   *
   * @param [in] size_x The X dimension in meters of the volume to represent
   * @param [in] size_y The Y dimension in meters of the volume to represent
   * @param [in] size_z The Z dimension in meters of the volume to represent
   * @param [in] resolution The resolution in meters of the volume
   * @param [in] origin_x The minimum X point of the volume
   * @param [in] origin_y The minimum Y point of the volume
   * @param [in] origin_z The minimum Z point of the volume
   */
  DistanceField(double size_x, double size_y, double size_z, double resolution, double origin_x, double origin_y,
                double origin_z);

  virtual ~DistanceField();

  /**
   * \brief Add a set of obstacle points to the distance field,
   * updating distance values accordingly.  The distance field may
   * already contain obstacle cells.
   *
   * @param [in] points The set of obstacle points to add
   */
  virtual void addPointsToField(const EigenSTL::vector_Vector3d& points) = 0;

  /**
   * \brief Remove a set of obstacle points from the distance field,
   * updating distance values accordingly.
   *
   * This function will invalidate the distance measurements
   * associated with the obstacle points and recompute them.
   * Depending on the implementation of the derived class and the
   * proportion of the total occupied points being removed, it may be
   * more efficient to use the \ref reset() function and add the
   * remaining obstacle points to the grid again.

   * @param [in] points The set of obstacle points that will be set as free
   */
  virtual void removePointsFromField(const EigenSTL::vector_Vector3d& points) = 0;

  /**
   * \brief This function will remove any obstacle points that are in
   * the old point set but not the new point set, and add any obstacle
   * points that are in the new block set but not the old block set.
   * Any points that are in both sets are left unchanged.
   *
   * The primary use case for this function is in moving objects -
   * calculating the set of points associated with an object, moving
   * the object in space, and calculating the new set of points.  If
   * the object has moved substantially, such that the old object
   * position does not overlap with the new object position, then it
   * may be more efficient to call \ref removePointsFromField on the
   * old points and \ref addPointsToField on the new points.  If the
   * object has moved only slighly, however, this function may offer a
   * speed improvement.  All points in the old_points set should have
   * been previously added to the field in order for this function to
   * act as intended - points that are in both the old_points set and
   * the new_points set will not be added to the field, as they are
   * assumed to already be obstacle points.
   *
   * @param [in] old_points The set of points that all should be obstacle cells in the distance field
   * @param [in] new_points The set of points, all of which are intended to be obstacle points in the distance field
   */
  virtual void updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                                   const EigenSTL::vector_Vector3d& new_points) = 0;

  /**
   * @brief Get the points associated with a shape.
   *        This is mainly used when the external application needs to cache points.
   * @param [in] shape The shape to find points for.
   * @param [in] pose The pose of the shape.
   * @param [out] points The points determined for this shape.
   */
  bool getShapePoints(const shapes::Shape* shape, const Eigen::Isometry3d& pose, EigenSTL::vector_Vector3d* points);

  /**
   * \brief Adds the set of points corresponding to the shape at the
   * given pose as obstacle points into the distance field.  If the
   * shape is an OcTree, the pose information is ignored and the
   * OcTree is passed to the \ref addOcTreeToField function.
   *
   * This function uses the Body class in the geometric_shapes package
   * to determine the set of obstacle points, with the exception of
   * OcTrees as mentioned.  A bounding sphere is computed given the
   * shape; the bounding sphere is iterated through in 3D at the
   * resolution of the distance_field, with each point tested for
   * point inclusion.  For more information about the behavior of
   * bodies and poses please see the documentation for
   * geometric_shapes.
   *
   * @param [in] shape The shape to add to the distance field
   * @param [in] pose The pose of the shape
   */
  void addShapeToField(const shapes::Shape* shape, const Eigen::Isometry3d& pose);

  // DEPRECATED form
  [[deprecated]] void addShapeToField(const shapes::Shape* shape, const geometry_msgs::Pose& pose);

  /**
   * \brief Adds an octree to the distance field.  Cells that are
   * occupied in the octree that lie within the voxel grid are added
   * to the distance field.  The octree can represent either a larger
   * or smaller volume than the distance field.  If the resolution of
   * the octree is less than or equal to the resolution of the
   * distance field then the center of each leaf cell of the octree
   * will be added to the distance field.  If the resolution of the
   * octree is greater than a 3D volume of the correct resolution will
   * be added for each occupied leaf node.
   *
   * @param [in] octree The octree to add to the distance field
   */
  void addOcTreeToField(const octomap::OcTree* octree);

  /**
   * \brief Moves the shape in the distance field from the old pose to
   * the new pose, removing points that are no longer obstacle points,
   * and adding points that are now obstacle points at the new pose.
   * This function will discretize both shapes, and call the
   * \ref updatePointsInField function on the old and new point sets.
   *
   * It's important to note that this function has no semantic
   * understanding of an object - this function may be called even if
   * \ref addShapeToField was not previously called.  Furthermore,
   * points will be removed even if they have been added by multiple
   * different sources - a cup resting on a table that is moved make
   * take a chunk out of the top of the table.
   *
   * @param [in] shape The shape to move in the distance field
   * @param [in] old_pose The old pose of the shape
   * @param [in] new_pose The new pose of the shape
   */
  void moveShapeInField(const shapes::Shape* shape, const Eigen::Isometry3d& old_pose,
                        const Eigen::Isometry3d& new_pose);

  // DEPRECATED form
  [[deprecated]] void moveShapeInField(const shapes::Shape* shape, const geometry_msgs::Pose& old_pose,
                                       const geometry_msgs::Pose& new_pose);

  /**
   * \brief All points corresponding to the shape are removed from the
   * distance field.
   *
   * The points needs not have been added using \ref addShapeToField.
   *
   * @param [in] shape The shape to remove from the distance field
   * @param [in] pose The pose of the shape to remove
   */
  void removeShapeFromField(const shapes::Shape* shape, const Eigen::Isometry3d& pose);

  // DEPRECATED form
  [[deprecated]] void removeShapeFromField(const shapes::Shape* shape, const geometry_msgs::Pose& pose);

  /**
   * \brief Resets all points in the distance field to an uninitialize
   * value.
   */
  virtual void reset() = 0;

  /**
   * \brief Gets the distance to the closest obstacle at the given
   * location.  The particulars of this function are heavily dependent
   * on the behavior of the derived class.
   *
   * The particular implementation may return a max distance value if
   * a cell is far away from all obstacles.  Values of 0.0 should
   * represent that a cell is an obstacle, and some implementations
   * may return a signed value representing distance to the surface
   * when a cell is deep inside an obstacle volume.  An implementation
   * may also return some value to represent when a location is
   * outside the represented volume.
   *
   * @param [in] x The cell's X value
   * @param [in] y The cell's Y value
   * @param [in] z The cell's Z value
   *
   * @return The distance to the closest obstacle cell
   */
  virtual double getDistance(double x, double y, double z) const = 0;

  /**
   * \brief Gets not only the distance to the nearest cell but the
   * gradient direction.  The gradient is computed as a function of
   * the distances of near-by cells.  An uninitialized distance is
   * returned if the cell is not valid for gradient production
   * purposes.  The gradient is pointing out of the obstacle - thus to
   * recover the closest obstacle point, the normalized gradient value
   * is multiplied by the distance and subtracted from the cell's
   * location, as shown below.
   *
   * A number of different cells will not have valid gradients.  Any
   * cell that is entirely surrounded by cells of the same distance
   * will not have a valid gradient.  Depending on the implementation
   * of the distance field, such cells may be found far away from
   * obstacles (if a distance is not computed for every cell), or deep
   * within obstacles.  Such points can be detected as having zero
   * magnitude for the gradient.
   *
   * The closest cell to a given cell can be computed given the
   * following formulation (this value will only be within the
   * resolution parameter of the correct location), including a test
   * for a non-zero gradient magnitude:
   *
   *\code
   * Eigen::Vector3d grad;
   * double dist = distance_field.getDistanceGradient(x,y,z,grad.x(),grad.y(),grad.z(),in_bounds);
   * if(grad.norm() > 0) {
   *   double closest_point_x = x-(grad.x()/grad.norm())*dist;
   *   double closest_point_y = y-(grad.y()/grad.norm())*dist;
   *   double closest_point_z = z-(grad.z()/grad.norm())*dist;
   * }
   * \endcode
   *
   * @param [in] x The X location of the cell
   * @param [in] y The X location of the cell
   * @param [in] z The X location of the cell
   * @param [out] gradient_x The X component of the gradient to the closest occupied cell
   * @param [out] gradient_y The Y component of the gradient to the closest occupied cell
   * @param [out] gradient_z The Z component of the gradient to the closest occupied cell
   *
   * @param [out] in_bounds Whether or not the (x,y,z) is valid for
   * gradient purposes.  Gradients are not valid at the boundary of
   * the distance field (cells where one or more of the indexes are at
   * 0 or at the maximum size).
   *
   * @return The distance to the closest occupied cell
   */
  double getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z,
                             bool& in_bounds) const;
  /**
   * \brief Gets the distance to the closest obstacle at the given
   * integer cell location. The particulars of this function are
   * heavily dependent on the behavior of the derived class.
   *
   * @param [in] x The X index of the cell
   * @param [in] y The Y index of the cell
   * @param [in] z The Z index of the cell
   *
   * @return The distance to the closest occupied cell
   */
  virtual double getDistance(int x, int y, int z) const = 0;

  /**
   * \brief Determines whether or not the cell associated with the
   * supplied indices is valid for this distance field.
   *
   * @param [in] x The X index of the cell
   * @param [in] y The Y index of the cell
   * @param [in] z The Z index of the cell
   *
   * @return True if the cell is valid, otherwise false.
   */
  virtual bool isCellValid(int x, int y, int z) const = 0;

  /**
   * \brief Gets the number of cells along the X axis
   *
   *
   * @return The number of cells along the X axis
   */
  virtual int getXNumCells() const = 0;

  /**
   * \brief Gets the number of cells along the Y axis
   *
   *
   * @return The number of cells along the Y axis
   */
  virtual int getYNumCells() const = 0;

  /**
   * \brief Gets the number of cells along the Z axis
   *
   *
   * @return The number of cells along the Z axis
   */
  virtual int getZNumCells() const = 0;

  /**
   * \brief Converts from an set of integer indices to a world
   * location given the origin and resolution parameters.
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   * @param [out] world_x The computed world X location
   * @param [out] world_y The computed world X location
   * @param [out] world_z The computed world X location
   * @return Whether or not the transformation is successful.  An implementation may or may not choose to return false
   *if the indicated cell is not valid for this distance field.
   */
  virtual bool gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const = 0;

  /**
   * \brief Converts from a world location to a set of integer
   * indices.  Should return false if the world location is not valid
   * in the distance field, and should populate the index values in
   * either case.
   *
   * @param [in] world_x The world X location
   * @param [in] world_y The world Y location
   * @param [in] world_z The world Z location
   * @param [out] x The computed integer X location
   * @param [out] y The computed integer X location
   * @param [out] z The computed integer X location
   *
   * @return True if all the world values result in integer indices
   * that pass a validity check; otherwise False.
   */
  virtual bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const = 0;

  /**
   * \brief Writes the contents of the distance field to the supplied stream.
   *
   * @param [out] stream The stream to which to write the distance field contents.
   *
   * @return True if the writing is successful; otherwise, false.
   */
  virtual bool writeToStream(std::ostream& stream) const = 0;

  /**
   * \brief Reads, parameterizes, and populates the distance field
   * based on the supplied stream.
   *
   * @param [in] stream The stream from which to read
   *
   * @return True if reading, parameterizing, and populating the
   * distance field is successful; otherwise False.
   */
  virtual bool readFromStream(std::istream& stream) = 0;

  /**
   * \brief Get an iso-surface for visualization in rviz.  The
   * iso-surface shows every cell that has a distance in a given
   * range in the distance field.  The cells are added as a
   * visualization_msgs::Marker::CUBE_LIST in the namespace
   * "distance_field".
   *
   * @param [in] min_distance Cells of less than this distance will not be added to the marker
   * @param [in] max_distance Cells of greater than this distance will not be added to the marker
   * @param [in] frame_id The frame to use as the header in the marker
   * @param [in] stamp The stamp to use in the header of the marker
   * @param [out] marker The marker that will contain the indicated cells.
   */
  void getIsoSurfaceMarkers(double min_distance, double max_distance, const std::string& frame_id,
                            const ros::Time stamp, visualization_msgs::Marker& marker) const;

  /**
   * \brief Populates the supplied marker array with a series of
   * arrows representing gradients of cells that are within the
   * supplied range in terms of distance.  The markers will be
   * visualization_msgs::Marker::ARROW in the namespace
   * "distance_field_gradient".
   *
   * @param [in] min_distance Cells of less than this distance will not be added to the marker
   * @param [in] max_distance Cells of greater than this distance will not be added to the marker
   * @param [in] frame_id The frame to use as the header in the marker
   * @param [in] stamp The stamp to use in the header of the marker
   * @param [out] marker_array The marker array to populate
   */
  void getGradientMarkers(double min_radius, double max_radius, const std::string& frame_id, const ros::Time& stamp,
                          visualization_msgs::MarkerArray& marker_array) const;

  /**
   * \brief Populates a marker with a slice of the distance field in a
   * particular plane.  All cells in the plane will be added to the
   * field, with colors associated with their distance.
   *
   * @param [in] type Which plane to show in the marker

   * @param [in] length The length of the plane to show.  If the type
   * is XZ or XY, it's interpreted as the dimension along the X axis.
   * If the type is YZ, it's interpreted along the Y axis.

   * @param [in] width The width of the plane to show. If the type is
   * XZ or YZ, it's interpreted along the Z axis.  If the type is XY,
   * it's interpreted along the Y axis.

   * @param [in] height The height of the plane to show.  If the type
   * is XY, it's interpreted along the Z axis.  If the type is XZ,
   * it's interpreted along the Y axis.  If the type is YZ, it's
   * interpeted along the X axis.

   * @param [in] origin The minimum point along each axis to display
   * @param [in] frame_id The frame to use as the header in the marker
   * @param [in] stamp The stamp to use in the header of the marker
   * @param [out] marker The marker that will contain the indicated cells.
   */
  void getPlaneMarkers(PlaneVisualizationType type, double length, double width, double height,
                       const Eigen::Vector3d& origin, const std::string& frame_id, const ros::Time stamp,
                       visualization_msgs::Marker& marker) const;
  /**
   * \brief A function that populates the marker with three planes -
   * one each along the XY, XZ, and YZ axes.  For each of the planes,
   * any column on that plane will be marked according to the minimum
   * distance along that column.
   *
   * @param [in] frame_id The frame to use as the header in the marker
   * @param [in] stamp The stamp to use in the header of the marker
   *
   * @param [in] max_distance A max distance for color calculation.
   * Distances of this value or greater will show up as fully white in
   * the marker.
   *
   * @param [out] marker The marker, which will be populated with a
   * visualization_msgs::Marker::CUBE_LIST .
   */
  void getProjectionPlanes(const std::string& frame_id, const ros::Time& stamp, double max_distance,
                           visualization_msgs::Marker& marker) const;

  /**
   * \brief Gets the distance field size along the X dimension in meters
   *
   *
   * @return The size along the X dimension in meters
   */
  double getSizeX() const
  {
    return size_x_;
  }

  /**
   * \brief Gets the distance field size along the Y dimension in meters
   *
   *
   * @return The size along the Y dimension in meters
   */
  double getSizeY() const
  {
    return size_y_;
  }

  /**
   * \brief Gets the distance field size along the Z dimension in meters
   *
   *
   * @return The size along the Z dimension in meters
   */
  double getSizeZ() const
  {
    return size_z_;
  }

  /**
   * \brief Gets the origin (minimum value) along the X dimension
   *
   *
   * @return The X origin
   */
  double getOriginX() const
  {
    return origin_x_;
  }

  /**
   * \brief Gets the origin (minimum value) along the Y dimension
   *
   *
   * @return The Y origin
   */
  double getOriginY() const
  {
    return origin_y_;
  }

  /**
   * \brief Gets the origin (minimum value) along the Z dimension
   *
   *
   * @return The Z origin
   */
  double getOriginZ() const
  {
    return origin_z_;
  }

  /**
   * \brief Gets the resolution of the distance field in meters
   *
   *
   * @return The resolution of the distance field in meters
   */
  double getResolution() const
  {
    return resolution_;
  }

  /**
   * \brief Gets a distance value for an invalid cell.
   *
   *
   * @return The distance associated with an unitialized cell
   */
  virtual double getUninitializedDistance() const = 0;

protected:
  /**
   * @brief Get the points associated with an octree.
   * @param [in] octree The octree to find points for.
   * @param [out] points The points determined for this octree.
   */
  void getOcTreePoints(const octomap::OcTree* octree, EigenSTL::vector_Vector3d* points);

  /**
   * \brief Helper function that sets the point value and color given
   * the distance.
   *
   * @param [in] xCell The x index of the cell
   * @param [in] yCell The y index of the cell
   * @param [in] zCell The z index of the cell
   * @param [in] dist The distance of the cell
   * @param [out] point World coordinates will be placed here
   *
   * @param [out] color A color will be assigned here that's only red if the distance is 0, and gets progressively
   *whiter as the dist value approaches max_distance.
   *
   * @param [in] max_distance The distance past which all cells will be fully white
   */
  void setPoint(int xCell, int yCell, int zCell, double dist, geometry_msgs::Point& point, std_msgs::ColorRGBA& color,
                double max_distance) const;

  double size_x_;            /**< \brief X size of the distance field */
  double size_y_;            /**< \brief Y size of the distance field */
  double size_z_;            /**< \brief Z size of the distance field */
  double origin_x_;          /**< \brief X origin of the distance field */
  double origin_y_;          /**< \brief Y origin of the distance field */
  double origin_z_;          /**< \brief Z origin of the distance field */
  double resolution_;        /**< \brief Resolution of the distance field */
  int inv_twice_resolution_; /**< \brief Computed value 1.0/(2.0*resolution_) */
};

}  // namespace distance_field
