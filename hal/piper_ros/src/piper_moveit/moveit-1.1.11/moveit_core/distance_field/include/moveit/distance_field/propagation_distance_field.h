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

/* Author: Mrinal Kalakrishnan, Ken Anderson */

#pragma once

#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/distance_field.h>
#include <vector>
#include <Eigen/Core>
#include <set>
#include <octomap/octomap.h>

namespace EigenSTL
{
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> vector_Vector3i;
}

namespace distance_field
{
/**
 * \brief Struct for sorting type Eigen::Vector3i for use in sorted
 * std containers.  Sorts in z order, then y order, then x order.
 */
struct CompareEigenVector3i
{
  bool operator()(const Eigen::Vector3i& loc_1, const Eigen::Vector3i& loc_2) const
  {
    if (loc_1.z() != loc_2.z())
      return (loc_1.z() < loc_2.z());
    else if (loc_1.y() != loc_2.y())
      return (loc_1.y() < loc_2.y());
    else if (loc_1.x() != loc_2.x())
      return (loc_1.x() < loc_2.x());
    return false;
  }
};

/**
 * \brief Structure that holds voxel information for the
 * DistanceField.  Will be used in VoxelGrid.
 */
struct PropDistanceFieldVoxel
{
  /**
   * \brief Constructor.  All fields left uninitialized.
   *
   *
   */
  PropDistanceFieldVoxel();

  /**
   * \brief Constructor.  Sets values for distance_sq_ and
   * negative_distance_square_, and sets all remaining internal values
   * to uninitialized.  These should be integers values which
   * represent the distance in cells squared.
   *
   * @param [in] distance_sq_positive Value to which to initialize
   * distance_sq_ for distance to closest obstalce
   *
   * @param [in] distance_sq_negative Value to which to initialize
   * distance_sq_negative_ for distance to nearest non-obstacle cell
   *
   */
  PropDistanceFieldVoxel(int distance_sq_positive, int distance_sq_negative);

  int distance_square_;                    /**< \brief Distance in cells to the closest obstacle, squared */
  int negative_distance_square_;           /**< \brief Distance in cells to the nearest unoccupied cell, squared */
  Eigen::Vector3i closest_point_;          /**< \brief Closest occupied cell */
  Eigen::Vector3i closest_negative_point_; /**< \brief Closest unoccupied cell */
  int update_direction_; /**< \brief Direction from which this voxel was updated for occupied distance propagation */
  int negative_update_direction_; /**< \brief Direction from which this voxel was updated  for negative distance
                                     propagation*/

  static const int UNINITIALIZED = -1; /**< \brief Value that represents an unitialized voxel */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * \brief A DistanceField implementation that uses a vector
 * propagation method.  Distances propagate outward from occupied
 * cells, or inwards from unoccupied cells if negative distances are
 * to be computed, which is optional.  Outward and inward propagation
 * only occur to a desired maximum distance - cells that are more than
 * this maximum distance from the nearest cell will have maximum
 * distance measurements.
 *
 * This class uses a \ref VoxelGrid to hold all data.  One important
 * decision that must be made on construction is whether or not to
 * create a signed version of the distance field.  If the distance
 * field is unsigned, it means that the minumum obstacle distance is
 * 0, a value that will be assigned to all obstacle cells.  Gradient
 * queries for obstacle cells will not give useful information, as the
 * gradient at an obstacle cell will point to the cell itself.  If
 * this behavior is acceptable, then the performance of this mode will
 * be more efficient, as no propagation will occur for obstacle cells.
 * The other option is to calculate signed distances.  In this case,
 * negative distances up to the maximum distance are calculated for
 * obstacle volumes.  This distance encodes the distance of an
 * obstacle cell to the nearest unoccupied obstacle voxel.  Furthmore,
 * gradients pointing out of the volume will be produced.  Depending
 * on the data, calculating this data can significantly impact the
 * time it takes to add and remove obstacle cells.
 */
class PropagationDistanceField : public DistanceField
{
public:
  /**
   * \brief Constructor that initializes entire distance field to
   * empty - all cells will be assigned maximum distance values.  All
   * units are arbitrary but are assumed for documentation purposes to
   * represent meters.
   *
   * @param [in] size_x The X dimension in meters of the volume to represent
   * @param [in] size_y The Y dimension in meters of the volume to represent
   * @param [in] size_z The Z dimension in meters of the volume to represent
   * @param [in] resolution The resolution in meters of the volume
   * @param [in] origin_x The minimum X point of the volume
   * @param [in] origin_y The minimum Y point of the volume
   * @param [in] origin_z The minimum Z point of the volume

   * @param [in] max_distance The maximum distance to which to
   * propagate distance values.  Cells that are greater than this
   * distance will be assigned the maximum distance value.
   *
   * @param [in] propagate_negative_distances Whether or not to
   * propagate negative distances.  If false, no propagation occurs,
   * and all obstacle cells will be assigned zero distance.  See the
   * \ref PropagationDistanceField description for more information on
   * the implications of this.
   *
   */
  PropagationDistanceField(double size_x, double size_y, double size_z, double resolution, double origin_x,
                           double origin_y, double origin_z, double max_distance,
                           bool propagate_negative_distances = false);

  /**
   * \brief Constructor based on an OcTree and bounding box
   * information.  A distance field will be constructed with
   * dimensions based on the supplied bounding box at the resolution
   * of the OcTree.  All octree obstacle cells will be added to the
   * resulting distance field using the \ref DistanceField::addOcTreeToField
   * function.
   *
   * @param [in] octree The OcTree from which to construct the distance field
   * @param [in] bbx_min The minimum world coordinates of the bounding box
   * @param [in] bbx_max The maximum world coordinates of the bounding box
   *
   * @param [in] max_distance The maximum distance to which to
   * propagate distance values.  Cells that are greater than this
   * distance will be assigned the maximum distance value.
   *
   * @param [in] propagate_negative_distances Whether or not to
   * propagate negative distances.  If false, no propagation occurs,
   * and all obstacle cells will be assigned zero distance.  See the
   * \ref PropagationDistanceField description for more information on
   * the implications of this.
   */
  PropagationDistanceField(const octomap::OcTree& octree, const octomap::point3d& bbx_min,
                           const octomap::point3d& bbx_max, double max_distance,
                           bool propagate_negative_distances = false);

  /**
   * \brief Constructor that takes an istream and reads the contents
   * of a saved distance field, adding all obstacle points and running
   * propagation given the arguments for max_distance and
   * propagate_negative_distances. Calls the function
   * \ref readFromStream.
   *
   * @param [in] stream The stream from which to read the data
   *
   * @param [in] max_distance The maximum distance to which to
   * propagate distance values.  Cells that are greater than this
   * distance will be assigned the maximum distance value.
   *
   * @param [in] propagate_negative_distances Whether or not to
   * propagate negative distances.  If false, no propagation occurs,
   * and all obstacle cells will be assigned zero distance.  See the
   * \ref PropagationDistanceField description for more information on
   * the implications of this.
   *
   * @return
   */
  PropagationDistanceField(std::istream& stream, double max_distance, bool propagate_negative_distances = false);
  /**
   * \brief Empty destructor
   *
   *
   */
  ~PropagationDistanceField() override
  {
  }

  /**
   * \brief Add a set of obstacle points to the distance field,
   * updating distance values accordingly.  The distance field may
   * already contain obstacle cells.
   *
   * The function first checks that each location represents a valid
   * point - only valid points will be added.  It takes the vector of
   * valid points and performs positive propagation on them.  If the
   * class has been set up to propagate negative distance, those will
   * also be propagated.
   *
   * @param [in] points The set of obstacle points to add
   */
  void addPointsToField(const EigenSTL::vector_Vector3d& points) override;

  /**
   * \brief Remove a set of obstacle points from the distance field,
   * updating distance values accordingly.
   *
   * This function is relatively less efficient than adding points to
   * the field in terms of positive distances - adding a given number
   * of points will be less comptationally expensive than removing the
   * same number of points.  This is due to the nature of the
   * propagation algorithm - when removing sets of cells, we must
   * search outward from the freed cells and then propagate inward.
   * Negative distances can be propagated more efficiently, as
   * propagation can occur outward from newly freed cells without
   * requiring a search step.  If the set of occupied points that
   * remain after removal is small it may be more efficient to call
   * \ref reset and then to add the remaining points rather than
   * removing a set of points.
   *
   * @param [in] points The set of obstacle points that will be set as free
   */
  void removePointsFromField(const EigenSTL::vector_Vector3d& points) override;

  /**
   * \brief This function will remove any obstacle points that are in
   * the old point set but not the new point set, and add any obstacle
   * points that are in the new block set but not the old block set.
   * Any points that are in both sets are left unchanged.  For more
   * information see \ref DistanceField::updatePointsInField.
   *
   * The implementation of this function finds the set of points that
   * are in the old_points and not the new_points, and the in the
   * new_points and not the old_points using std::set_difference.  It
   * then calls a removal function on the former set, and an addition
   * function on the latter set.
   *
   * If there is no overlap between the old_points and the new_points
   * it is more efficient to first call \ref removePointsFromField on
   * the old_points and then \ref addPointsToField on the new points -
   * this does not require computing set differences.
   *
   * @param [in] old_points The set of points that all should be obstacle cells in the distance field
   * @param [in] new_points The set of points, all of which are intended to be obstacle points in the distance field
   *
   */
  void updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                           const EigenSTL::vector_Vector3d& new_points) override;

  /**
   * \brief Resets the entire distance field to max_distance for
   * positive values and zero for negative values.
   *
   */
  void reset() override;

  /**
   * \brief Get the distance value associated with the cell indicated
   * by the world coordinate.  If the cell is invalid, max_distance
   * will be returned.  If running without negative distances, all
   * obstacle cells will have zero distance.  If running with negative
   * distances, the distance will be between -max_distance and
   * max_distance, with no values having a 0 distance.
   *
   *
   * @param [in] x The X location of the cell
   * @param [in] y The X location of the cell
   * @param [in] z The X location of the cell
   *
   * @return The distance value
   */
  double getDistance(double x, double y, double z) const override;

  /**
   * \brief Get the distance value associated with the cell indicated
   * by the index coordinates.  If the cell is invalid, max_distance
   * will be returned.  If running without negative distances, all
   * obstacle cells will have zero distance.  If running with negative
   * distances, the distance will be between -max_distance and
   * max_distance, with no values having a 0 distance.
   *
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   *
   * @return The distance value for the cell
   */
  double getDistance(int x, int y, int z) const override;

  bool isCellValid(int x, int y, int z) const override;
  int getXNumCells() const override;
  int getYNumCells() const override;
  int getZNumCells() const override;
  bool gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const override;
  bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const override;

  /**
   * \brief Writes the contents of the distance field to the supplied stream.
   *
   * This function writes the resolution, size, and origin parameters
   * to the file in ASCII.  It then writes the occupancy data only in
   * bit form (with values or 1 representing occupancy, and 0
   * representing empty space).  It further runs Zlib compression on
   * the binary data before actually writing to disk.  The
   * max_distance and propagate_negative_distances values are not
   * written to file, and the distances themselves will need to be
   * recreated on load.
   *
   * @param [out] stream The stream to which to write the distance field contents.
   *
   * @return True
   */
  bool writeToStream(std::ostream& stream) const override;

  /**
   * \brief Reads, parameterizes, and populates the distance field
   * based on the supplied stream.
   *
   * This function assumes that the file begins with ASCII data, and
   * that the binary data has been written in bit formulation and
   * compressed using Zlib.  The function will reinitialize all data
   * members based on the data in the file, using preset values for
   * max_distance_ and propagate_negative_distances_.  All occupied
   * cells will be added to the distance field.
   *
   * @param [in] stream The stream from which to read
   *
   * @return True if reading, parameterizing, and populating the
   * distance field is successful; otherwise False.
   */
  bool readFromStream(std::istream& stream) override;

  // passthrough docs to DistanceField
  double getUninitializedDistance() const override
  {
    return max_distance_;
  }

  /**
   * \brief Gets full cell data given an index.
   *
   * x,y,z MUST be valid or data corruption (SEGFAULTS) will occur.
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   *
   * @return The data in the indicated cell.
   */
  const PropDistanceFieldVoxel& getCell(int x, int y, int z) const
  {
    return voxel_grid_->getCell(x, y, z);
  }

  /**
   * \brief Gets nearest surface cell and returns distance to it.
   *
   * x,y,z MUST be valid or data corruption (SEGFAULTS) will occur.
   *
   * @param [in] x The integer X location of the starting cell
   * @param [in] y The integer Y location of the starting cell
   * @param [in] z The integer Z location of the starting cell
   * @param [out] dist if starting cell is inside, the negative distance to the nearest outside cell
   *                   if starting cell is outside, the positive distance to the nearest inside cell
   *                   if nearby cell is unknown, zero
   * @param [out] pos the position of the nearest cell
   *
   *
   * @return If starting cell is inside, the nearest outside cell
   *         If starting cell is outside, the nearst inside cell
   *         If nearest cell is unknown, return NULL
   */
  const PropDistanceFieldVoxel* getNearestCell(int x, int y, int z, double& dist, Eigen::Vector3i& pos) const
  {
    const PropDistanceFieldVoxel* cell = &voxel_grid_->getCell(x, y, z);
    if (cell->distance_square_ > 0)
    {
      dist = sqrt_table_[cell->distance_square_];
      pos = cell->closest_point_;
      const PropDistanceFieldVoxel* ncell = &voxel_grid_->getCell(pos.x(), pos.y(), pos.z());
      return ncell == cell ? nullptr : ncell;
    }
    if (cell->negative_distance_square_ > 0)
    {
      dist = -sqrt_table_[cell->negative_distance_square_];
      pos = cell->closest_negative_point_;
      const PropDistanceFieldVoxel* ncell = &voxel_grid_->getCell(pos.x(), pos.y(), pos.z());
      return ncell == cell ? nullptr : ncell;
    }
    dist = 0.0;
    pos.x() = x;
    pos.y() = y;
    pos.z() = z;
    return nullptr;
  }

  /**
   * \brief Gets the maximum distance squared value.
   *
   * Produced by taking the ceiling of the maximum distance divided by
   * the resolution, and then squaring that value.
   *
   * @return The maximum distance squared.
   */
  int getMaximumDistanceSquared() const
  {
    return max_distance_sq_;
  }

private:
  /** Typedef for set of integer indices */
  typedef std::set<Eigen::Vector3i, CompareEigenVector3i, Eigen::aligned_allocator<Eigen::Vector3i>> VoxelSet;
  /**
   * \brief Initializes the field, resetting the voxel grid and
   * building a sqrt lookup table for efficiency based on
   * max_distance_.
   *
   */
  void initialize();

  /**
   * \brief Adds a valid set of integer points to the voxel grid
   *
   * @param voxel_points Valid set of voxel points for addition
   */
  void addNewObstacleVoxels(const EigenSTL::vector_Vector3i& voxel_points);

  /**
   * \brief Removes a valid set of integer points from the voxel grid
   *
   * @param voxel_points Valid set of voxel points for removal
   */
  void removeObstacleVoxels(const EigenSTL::vector_Vector3i& voxel_points);

  /**
   * \brief Propagates outward to the maximum distance given the
   * contents of the \ref bucket_queue_, and clears the \ref
   * bucket_queue_.
   *
   */
  void propagatePositive();

  /**
   * \brief Propagates inward to a maximum distance given the contents
   * of the \ref negative_bucket_queue_, and clears the \ref
   * negative_bucket_queue_.
   *
   */
  void propagateNegative();

  /**
   * \brief Determines distance based on actual voxel data
   *
   * @param object Actual voxel data
   *
   * @return The distance reported by the cell
   */
  virtual double getDistance(const PropDistanceFieldVoxel& object) const;

  /**
   * \brief Helper function to get a single number in a 27 connected
   * 3D voxel grid given dx, dy, and dz values.
   *
   * @param dx The change in the X direction
   * @param dy The change in the X direction
   * @param dz The change in the Z direction
   *
   * @return Single number 0-26 representing direction
   */
  int getDirectionNumber(int dx, int dy, int dz) const;

  /**
   * \brief Helper function that gets change values given single
   * number representing update direction.
   *
   * @param directionNumber Direction number 0-26
   *
   * @return Integer changes
   */
  Eigen::Vector3i getLocationDifference(int directionNumber) const;

  /**
   * \brief Helper function for computing location and neighborhood
   * information in 27 connected voxel grid.
   *
   */
  void initNeighborhoods();

  /**
   * \brief Debug function that prints all voxels in a set to ROS_DEBUG_NAMED
   *
   * @param set Voxel set to print
   */
  void print(const VoxelSet& set);

  /**
   * \brief Debug function that prints all points in a vector to ROS_DEBUG_NAMED
   *
   * @param points Points to print
   */
  void print(const EigenSTL::vector_Vector3d& points);

  bool propagate_negative_; /**< \brief Whether or not to propagate negative distances */

  VoxelGrid<PropDistanceFieldVoxel>::Ptr voxel_grid_; /**< \brief Actual container for distance data */

  /// \brief Structure used to hold propagation frontier
  std::vector<EigenSTL::vector_Vector3i> bucket_queue_; /**< \brief Data member that holds points from which to
                                                              propagate, where each vector holds points that are a
                                                              particular integer distance from the closest obstacle
                                                              points*/

  std::vector<EigenSTL::vector_Vector3i> negative_bucket_queue_; /**< \brief Data member that holds points from
                                                                       which to propagate in the negative, where each
                                                                       vector holds points that are a particular
                                                                       integer distance from the closest unoccupied
                                                                       points*/

  double max_distance_; /**< \brief Holds maximum distance  */
  int max_distance_sq_; /**< \brief Holds maximum distance squared in cells */

  std::vector<double> sqrt_table_; /**< \brief Precomputed square root table for faster distance lookups */

  /**
   * \brief Holds information on neighbor direction, with 27 different
   * directions.  Shows where to propagate given an integer distance
   * and an update direction.
   *
   * [0] - for expansion of d=0
   * [1] - for expansion of d>=1
   * Under this, we have the 27 directions
   * Then, a list of neighborhoods for each direction
   *
   */

  std::vector<std::vector<EigenSTL::vector_Vector3i>> neighborhoods_;

  EigenSTL::vector_Vector3i direction_number_to_direction_; /**< \brief Holds conversion from direction number to
                                                                  integer changes */
};

////////////////////////// inline functions follow ////////////////////////////////////////

inline PropDistanceFieldVoxel::PropDistanceFieldVoxel(int distance_square, int negative_distance_squared)
  : distance_square_(distance_square), negative_distance_square_(negative_distance_squared)
{
  closest_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_negative_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_negative_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
  closest_negative_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
}

inline PropDistanceFieldVoxel::PropDistanceFieldVoxel()
{
}

inline double PropagationDistanceField::getDistance(const PropDistanceFieldVoxel& object) const
{
  return sqrt_table_[object.distance_square_] - sqrt_table_[object.negative_distance_square_];
}
}  // namespace distance_field
