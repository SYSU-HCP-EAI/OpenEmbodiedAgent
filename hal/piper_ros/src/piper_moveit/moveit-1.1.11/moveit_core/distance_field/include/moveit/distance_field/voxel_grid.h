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

/* Author: Mrinal Kalakrishnan, Acorn Pooley */

#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <moveit/macros/declare_ptr.h>

namespace distance_field
{
/// \brief Specifies dimension of different axes
enum Dimension
{
  DIM_X = 0,
  DIM_Y = 1,
  DIM_Z = 2
};

/**
 * \brief VoxelGrid holds a dense 3D, axis-aligned set of data at a
 * given resolution, where the data is supplied as a template
 * parameter.
 *
 */
template <typename T>
class VoxelGrid
{
public:
  MOVEIT_DECLARE_PTR_MEMBER(VoxelGrid);

  /**
   * \brief Constructor for the VoxelGrid.
   *
   * Constructs a dense representation of a 3D, axis-aligned volume at
   * a given resolution.  The volume can be represented in any
   * consistent set of units, but for the sake of documentation we
   * assume that the units are meters.  The size of the the volume is
   * given along each of the X, Y, and Z axes.  The volume begins at
   * the minimum point in each dimension, as specified by the origin
   * parameters.  The data structure will remain unintialized until
   * the \ref VoxelGrid::reset function is called.
   *
   * @param [in] size_x Size of the X axis in meters
   * @param [in] size_y Size of the Y axis in meters
   * @param [in] size_z Size of the Z axis in meters
   *
   * @param [in] resolution Resolution of a single cell in meters
   *
   * @param [in] origin_x Minimum point along the X axis of the volume
   * @param [in] origin_y Minimum point along the Y axis of the volume
   * @param [in] origin_z Minimum point along the Z axis of the volume
   *
   * @param [in] default_object An object that will be returned for any
   * future queries that are not valid
   */
  VoxelGrid(double size_x, double size_y, double size_z, double resolution, double origin_x, double origin_y,
            double origin_z, T default_object);
  virtual ~VoxelGrid();

  /**
   * \brief Default constructor for the VoxelGrid.
   *
   * This is only useful if resize() is called after construction to set the
   * size and resolution of the VoxelGrid.
   */
  VoxelGrid();

  /**
   * \brief Resize the VoxelGrid.
   *
   * This discards all the data in the voxel grid and reinitializes
   * it with a new size and resolution.  This is mainly useful if the size or
   * resolution is not known until after the voxelgrid is constructed.
   *
   * @param [in] size_x Size of the X axis in meters
   * @param [in] size_y Size of the Y axis in meters
   * @param [in] size_z Size of the Z axis in meters
   *
   * @param [in] resolution Resolution of a single cell in meters
   *
   * @param [in] origin_x Minimum point along the X axis of the volume
   * @param [in] origin_y Minimum point along the Y axis of the volume
   * @param [in] origin_z Minimum point along the Z axis of the volume
   */
  void resize(double size_x, double size_y, double size_z, double resolution, double origin_x, double origin_y,
              double origin_z, T default_object);

  /**
   * \brief Operator that gets the value of the given location (x, y,
   * z) given the discretization of the volume.  The location
   * represents a location in the original coordinate frame used to
   * construct the voxel grid.
   *
   * @param [in] x X component of the desired location
   * @param [in] y Y component of the desired location
   * @param [in] z Z component of the desired location
   *
   * @return The data stored at that location, or a default value
   * supplied in the constructor if the location is not valid.
   */
  const T& operator()(double x, double y, double z) const;
  const T& operator()(const Eigen::Vector3d& pos) const;

  /**
   * \brief Gives the value of the given location (x,y,z) in the
   * discretized voxel grid space.
   *
   * The address here is in the discretized space of the voxel grid,
   * where the cell indicated by the constructor arguments (origin_x,
   * origin_y, origin_z) is cell (0,0,0), and the cell indicated by
   * (origin_x+x_size, origin_y+y_size, origin_z+z_size) will be
   * (size_x/resolution, size_y/resolution, size_z/resolution).
   *
   * @param [in] x The X index of the desired cell
   * @param [in] y The Y index of the desired cell
   * @param [in] z The Z index of the desired cell
   *
   * @return The data in the indicated cell.  If x,y,z is invalid then
   * corruption and/or SEGFAULTS will occur.
   */
  T& getCell(int x, int y, int z);
  T& getCell(const Eigen::Vector3i& pos);
  const T& getCell(int x, int y, int z) const;
  const T& getCell(const Eigen::Vector3i& pos) const;

  /**
   * \brief Sets the value of the given location (x,y,z) in the
   * discretized voxel grid space to supplied value.
   *
   * The address here is in the discretized space of the voxel grid,
   * where the cell indicated by the constructor arguments (origin_x,
   * origin_y, origin_z) is cell (0,0,0), and the cell indicated by
   * (origin_x+x_size, origin_y+y_size, origin_z+z_size) will be
   * (size_x/resolution, size_y/resolution, size_z/resolution).
   *
   * If the arguments do not indicate a valid cell, corruption and/or SEGFAULTS
   * will occur.
   *
   * @param [in] x The X index of the desired cell
   * @param [in] y The Y index of the desired cell
   * @param [in] z The Z index of the desired cell
   * @param [out] obj The data to place into the given cell
   */
  void setCell(int x, int y, int z, const T& obj);
  void setCell(const Eigen::Vector3i& pos, const T& obj);

  /**
   * \brief Sets every cell in the voxel grid to the supplied data
   *
   * @param [in] initial The template variable to which to set the data
   */
  void reset(const T& initial);

  /**
   * \brief Gets the size in arbitrary units of the indicated dimension
   *
   * @param [in] dim The dimension for the query
   *
   * @return The size in meters
   */
  double getSize(Dimension dim) const;

  /**
   * \brief Gets the resolution in arbitrary consistent units
   *
   * @return The resolution in meters
   */
  double getResolution() const;

  /** \brief deprecated.  Use the version with no arguments. */
  double getResolution(Dimension dim) const;

  /**
   * \brief Gets the origin (miniumum point) of the indicated dimension
   *
   * @param [in] dim The dimension for the query
   *
   * @return The indicated axis origin
   */
  double getOrigin(Dimension dim) const;

  /**
   * \brief Gets the number of cells in the indicated dimension
   *
   * @param [in] dim The dimension for the query
   *
   * @return The number of cells for the indicated dimension
   */
  int getNumCells(Dimension dim) const;

  /**
   * \brief Converts grid coordinates to world coordinates.
   */
  /**
   * \brief Converts from an set of integer indices to a world
   * location given the origin and resolution parameters.  There is no
   * check whether or not the cell or world locations lie within the
   * represented region.
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   * @param [out] world_x The computed world X location
   * @param [out] world_y The computed world X location
   * @param [out] world_z The computed world X location
   *
   * @return True, as there is no check that the integer locations are valid
   */
  void gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const;
  void gridToWorld(const Eigen::Vector3i& grid, Eigen::Vector3d& world) const;

  /**
   * \brief Converts from a world location to a set of integer
   * indices.  Does check whether or not the cell being returned is
   * valid.  The returned indices will be computed even if they are
   * invalid.
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
  bool worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const;
  bool worldToGrid(const Eigen::Vector3d& world, Eigen::Vector3i& grid) const;

  /**
   * \brief Checks if the given cell in integer coordinates is within the voxel grid
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   *
   * @return True if the cell lies within the voxel grid; otherwise False.
   */
  bool isCellValid(int x, int y, int z) const;
  bool isCellValid(const Eigen::Vector3i& pos) const;

  /**
   * \brief Checks if the indicated index is valid along a particular dimension.
   *
   * @param [in] dim The dimension for the query
   * @param [in] cell The index along that dimension
   *
   * @return True if the cell is valid along that dimension; otherwise False.
   */
  bool isCellValid(Dimension dim, int cell) const;

protected:
  T* data_;                /**< \brief Storage for the full set of data elements */
  T default_object_;       /**< \brief The default object to return in case of out-of-bounds query */
  T*** data_ptrs_;         /**< \brief 3D array of pointers to the data elements */
  double size_[3];         /**< \brief The size of each dimension in meters (in Dimension order) */
  double resolution_;      /**< \brief The resolution of each dimension in meters (in Dimension order) */
  double oo_resolution_;   /**< \brief 1.0/resolution_ */
  double origin_[3];       /**< \brief The origin (minumum point) of each dimension in meters (in Dimension order) */
  double origin_minus_[3]; /**< \brief origin - 0.5/resolution */
  int num_cells_[3];       /**< \brief The number of cells in each dimension (in Dimension order) */
  int num_cells_total_;    /**< \brief The total number of voxels in the grid */
  int stride1_;            /**< \brief The step to take when stepping between consecutive X members in the 1D array */
  int stride2_; /**< \brief The step to take when stepping between consecutive Y members given an X in the 1D array */

  /**
   * \brief Gets the 1D index into the array, with no validity check.
   *
   * @param [in] x The integer X location
   * @param [in] y The integer Y location
   * @param [in] z The integer Z location
   *
   * @return The computed 1D index
   */
  int ref(int x, int y, int z) const;

  /**
   * \brief Gets the cell number from the location
   */
  /**
   * \brief Gets the cell number in a given dimension given a world
   * value.  No validity check.
   *
   * @param [in] dim The dimension of the query
   * @param [in] loc The world location along that dimension
   *
   * @return The computed cell index along the given dimension
   */
  int getCellFromLocation(Dimension dim, double loc) const;

  /**
   * \brief Gets the center of the cell in world coordinates along the
   * given dimension.  No validity check.
   *
   * @param [in] dim The dimension of the query
   * @param [in] cell The cell along the given dimension
   *
   * @return The world coordinate of the center of the cell
   */
  double getLocationFromCell(Dimension dim, int cell) const;
};

//////////////////////////// template function definitions follow //////////////////

template <typename T>
VoxelGrid<T>::VoxelGrid(double size_x, double size_y, double size_z, double resolution, double origin_x,
                        double origin_y, double origin_z, T default_object)
  : data_(nullptr)
{
  resize(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, default_object);
}

template <typename T>
VoxelGrid<T>::VoxelGrid() : data_(NULL)
{
  for (int i = DIM_X; i <= DIM_Z; ++i)
  {
    size_[i] = 0;
    origin_[i] = 0;
    origin_minus_[i] = 0;
    num_cells_[i] = 0;
  }
  resolution_ = 1.0;
  oo_resolution_ = 1.0 / resolution_;
  num_cells_total_ = 0;
  stride1_ = 0;
  stride2_ = 0;
}

template <typename T>
void VoxelGrid<T>::resize(double size_x, double size_y, double size_z, double resolution, double origin_x,
                          double origin_y, double origin_z, T default_object)
{
  delete[] data_;
  data_ = nullptr;

  size_[DIM_X] = size_x;
  size_[DIM_Y] = size_y;
  size_[DIM_Z] = size_z;
  origin_[DIM_X] = origin_x;
  origin_[DIM_Y] = origin_y;
  origin_[DIM_Z] = origin_z;
  origin_minus_[DIM_X] = origin_x - 0.5 * resolution;
  origin_minus_[DIM_Y] = origin_y - 0.5 * resolution;
  origin_minus_[DIM_Z] = origin_z - 0.5 * resolution;
  num_cells_total_ = 1;
  resolution_ = resolution;
  oo_resolution_ = 1.0 / resolution_;
  for (int i = DIM_X; i <= DIM_Z; ++i)
  {
    num_cells_[i] = size_[i] * oo_resolution_;
    num_cells_total_ *= num_cells_[i];
  }

  default_object_ = default_object;

  stride1_ = num_cells_[DIM_Y] * num_cells_[DIM_Z];
  stride2_ = num_cells_[DIM_Z];

  // initialize the data:
  if (num_cells_total_ > 0)
    data_ = new T[num_cells_total_];
}

template <typename T>
VoxelGrid<T>::~VoxelGrid()
{
  delete[] data_;
}

template <typename T>
inline bool VoxelGrid<T>::isCellValid(int x, int y, int z) const
{
  return (x >= 0 && x < num_cells_[DIM_X] && y >= 0 && y < num_cells_[DIM_Y] && z >= 0 && z < num_cells_[DIM_Z]);
}

template <typename T>
inline bool VoxelGrid<T>::isCellValid(const Eigen::Vector3i& pos) const
{
  return isCellValid(pos.x(), pos.y(), pos.z());
}

template <typename T>
inline bool VoxelGrid<T>::isCellValid(Dimension dim, int cell) const
{
  return cell >= 0 && cell < num_cells_[dim];
}

template <typename T>
inline int VoxelGrid<T>::ref(int x, int y, int z) const
{
  return x * stride1_ + y * stride2_ + z;
}

template <typename T>
inline double VoxelGrid<T>::getSize(Dimension dim) const
{
  return size_[dim];
}

template <typename T>
inline double VoxelGrid<T>::getResolution() const
{
  return resolution_;
}

template <typename T>
inline double VoxelGrid<T>::getResolution(Dimension /*dim*/) const
{
  return resolution_;
}

template <typename T>
inline double VoxelGrid<T>::getOrigin(Dimension dim) const
{
  return origin_[dim];
}

template <typename T>
inline int VoxelGrid<T>::getNumCells(Dimension dim) const
{
  return num_cells_[dim];
}

template <typename T>
inline const T& VoxelGrid<T>::operator()(double x, double y, double z) const
{
  int cell_x = getCellFromLocation(DIM_X, x);
  int cell_y = getCellFromLocation(DIM_Y, y);
  int cell_z = getCellFromLocation(DIM_Z, z);
  if (!isCellValid(cell_x, cell_y, cell_z))
    return default_object_;
  return getCell(cell_x, cell_y, cell_z);
}

template <typename T>
inline const T& VoxelGrid<T>::operator()(const Eigen::Vector3d& pos) const
{
  return this->operator()(pos.x(), pos.y(), pos.z());
}

template <typename T>
inline T& VoxelGrid<T>::getCell(int x, int y, int z)
{
  return data_[ref(x, y, z)];
}

template <typename T>
inline const T& VoxelGrid<T>::getCell(int x, int y, int z) const
{
  return data_[ref(x, y, z)];
}

template <typename T>
inline T& VoxelGrid<T>::getCell(const Eigen::Vector3i& pos)
{
  return data_[ref(pos.x(), pos.y(), pos.z())];
}

template <typename T>
inline const T& VoxelGrid<T>::getCell(const Eigen::Vector3i& pos) const
{
  return data_[ref(pos.x(), pos.y(), pos.z())];
}

template <typename T>
inline void VoxelGrid<T>::setCell(int x, int y, int z, const T& obj)
{
  data_[ref(x, y, z)] = obj;
}

template <typename T>
inline void VoxelGrid<T>::setCell(const Eigen::Vector3i& pos, const T& obj)
{
  data_[ref(pos.x(), pos.y(), pos.z())] = obj;
}

template <typename T>
inline int VoxelGrid<T>::getCellFromLocation(Dimension dim, double loc) const
{
  // This implements
  //
  //      (  loc - origin         )
  // floor(  ------------  + 0.5  )
  //      (   resolution          )
  //
  // In other words, the rounded quantized location.
  //
  // For speed implemented like this:
  //
  // floor( ( loc - origin_minus ) * oo_resolution )
  //
  // where  origin_minus = origin - 0.5*resolution
  //
  return int(floor((loc - origin_minus_[dim]) * oo_resolution_));
}

template <typename T>
inline double VoxelGrid<T>::getLocationFromCell(Dimension dim, int cell) const
{
  return origin_[dim] + resolution_ * (double(cell));
}

template <typename T>
inline void VoxelGrid<T>::reset(const T& initial)
{
  std::fill(data_, data_ + num_cells_total_, initial);
}

template <typename T>
inline void VoxelGrid<T>::gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const
{
  world_x = getLocationFromCell(DIM_X, x);
  world_y = getLocationFromCell(DIM_Y, y);
  world_z = getLocationFromCell(DIM_Z, z);
}

template <typename T>
inline void VoxelGrid<T>::gridToWorld(const Eigen::Vector3i& grid, Eigen::Vector3d& world) const
{
  world.x() = getLocationFromCell(DIM_X, grid.x());
  world.y() = getLocationFromCell(DIM_Y, grid.y());
  world.z() = getLocationFromCell(DIM_Z, grid.z());
}

template <typename T>
inline bool VoxelGrid<T>::worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const
{
  x = getCellFromLocation(DIM_X, world_x);
  y = getCellFromLocation(DIM_Y, world_y);
  z = getCellFromLocation(DIM_Z, world_z);
  return isCellValid(x, y, z);
}

template <typename T>
inline bool VoxelGrid<T>::worldToGrid(const Eigen::Vector3d& world, Eigen::Vector3i& grid) const
{
  grid.x() = getCellFromLocation(DIM_X, world.x());
  grid.y() = getCellFromLocation(DIM_Y, world.y());
  grid.z() = getCellFromLocation(DIM_Z, world.z());
  return isCellValid(grid.x(), grid.y(), grid.z());
}

}  // namespace distance_field
