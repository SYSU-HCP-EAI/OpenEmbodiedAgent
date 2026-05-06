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

#include <moveit/distance_field/propagation_distance_field.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>

namespace distance_field
{
PropagationDistanceField::PropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
                                                   double origin_x, double origin_y, double origin_z,
                                                   double max_distance, bool propagate_negative)
  : DistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z)
  , propagate_negative_(propagate_negative)
  , max_distance_(max_distance)
{
  initialize();
}

PropagationDistanceField::PropagationDistanceField(const octomap::OcTree& octree, const octomap::point3d& bbx_min,
                                                   const octomap::point3d& bbx_max, double max_distance,
                                                   bool propagate_negative_distances)
  : DistanceField(bbx_max.x() - bbx_min.x(), bbx_max.y() - bbx_min.y(), bbx_max.z() - bbx_min.z(),
                  octree.getResolution(), bbx_min.x(), bbx_min.y(), bbx_min.z())
  , propagate_negative_(propagate_negative_distances)
  , max_distance_(max_distance)
  , max_distance_sq_(0)  // avoid gcc warning about uninitialized value
{
  initialize();
  addOcTreeToField(&octree);
}

PropagationDistanceField::PropagationDistanceField(std::istream& is, double max_distance,
                                                   bool propagate_negative_distances)
  : DistanceField(0, 0, 0, 0, 0, 0, 0), propagate_negative_(propagate_negative_distances), max_distance_(max_distance)
{
  readFromStream(is);
}

void PropagationDistanceField::initialize()
{
  max_distance_sq_ = ceil(max_distance_ / resolution_) * ceil(max_distance_ / resolution_);
  voxel_grid_ =
      std::make_shared<VoxelGrid<PropDistanceFieldVoxel>>(size_x_, size_y_, size_z_, resolution_, origin_x_, origin_y_,
                                                          origin_z_, PropDistanceFieldVoxel(max_distance_sq_, 0));

  initNeighborhoods();

  bucket_queue_.resize(max_distance_sq_ + 1);
  negative_bucket_queue_.resize(max_distance_sq_ + 1);

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_ + 1);
  for (int i = 0; i <= max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i)) * resolution_;

  reset();
}

void PropagationDistanceField::print(const VoxelSet& set)
{
  ROS_DEBUG_NAMED("distance_field", "[");
  VoxelSet::const_iterator it;
  for (it = set.begin(); it != set.end(); ++it)
  {
    Eigen::Vector3i loc1 = *it;
    ROS_DEBUG_NAMED("distance_field", "%d, %d, %d ", loc1.x(), loc1.y(), loc1.z());
  }
  ROS_DEBUG_NAMED("distance_field", "] size=%u\n", (unsigned int)set.size());
}

void PropagationDistanceField::print(const EigenSTL::vector_Vector3d& points)
{
  ROS_DEBUG_NAMED("distance_field", "[");
  EigenSTL::vector_Vector3d::const_iterator it;
  for (it = points.begin(); it != points.end(); ++it)
  {
    Eigen::Vector3d loc1 = *it;
    ROS_DEBUG_NAMED("distance_field", "%g, %g, %g ", loc1.x(), loc1.y(), loc1.z());
  }
  ROS_DEBUG_NAMED("distance_field", "] size=%u\n", (unsigned int)points.size());
}

void PropagationDistanceField::updatePointsInField(const EigenSTL::vector_Vector3d& old_points,
                                                   const EigenSTL::vector_Vector3d& new_points)
{
  VoxelSet old_point_set;
  for (const Eigen::Vector3d& old_point : old_points)
  {
    Eigen::Vector3i voxel_loc;
    bool valid = worldToGrid(old_point.x(), old_point.y(), old_point.z(), voxel_loc.x(), voxel_loc.y(), voxel_loc.z());
    if (valid)
    {
      old_point_set.insert(voxel_loc);
    }
  }

  VoxelSet new_point_set;
  for (const Eigen::Vector3d& new_point : new_points)
  {
    Eigen::Vector3i voxel_loc;
    bool valid = worldToGrid(new_point.x(), new_point.y(), new_point.z(), voxel_loc.x(), voxel_loc.y(), voxel_loc.z());
    if (valid)
    {
      new_point_set.insert(voxel_loc);
    }
  }
  CompareEigenVector3i comp;

  EigenSTL::vector_Vector3i old_not_new;
  std::set_difference(old_point_set.begin(), old_point_set.end(), new_point_set.begin(), new_point_set.end(),
                      std::inserter(old_not_new, old_not_new.end()), comp);

  EigenSTL::vector_Vector3i new_not_old;
  std::set_difference(new_point_set.begin(), new_point_set.end(), old_point_set.begin(), old_point_set.end(),
                      std::inserter(new_not_old, new_not_old.end()), comp);

  EigenSTL::vector_Vector3i new_not_in_current;
  for (Eigen::Vector3i& voxel_loc : new_not_old)
  {
    if (voxel_grid_->getCell(voxel_loc.x(), voxel_loc.y(), voxel_loc.z()).distance_square_ != 0)
    {
      new_not_in_current.push_back(voxel_loc);
    }
    // ROS_INFO_NAMED("distance_field", "Adding obstacle voxel %d %d %d", (*it).x(), (*it).y(), (*it).z());
  }

  removeObstacleVoxels(old_not_new);
  addNewObstacleVoxels(new_not_in_current);

  // ROS_DEBUG_NAMED("distance_field",  "new=" );
  // print(points_added);
  // ROS_DEBUG_NAMED("distance_field",  "removed=" );
  // print(points_removed);
  // ROS_DEBUG_NAMED("distance_field",  "obstacle_voxel_locations_=" );
  // print(object_voxel_locations_);
  // ROS_DEBUG_NAMED("distance_field", "");
}

void PropagationDistanceField::addPointsToField(const EigenSTL::vector_Vector3d& points)
{
  EigenSTL::vector_Vector3i voxel_points;

  for (const Eigen::Vector3d& point : points)
  {
    // Convert to voxel coordinates
    Eigen::Vector3i voxel_loc;
    bool valid = worldToGrid(point.x(), point.y(), point.z(), voxel_loc.x(), voxel_loc.y(), voxel_loc.z());

    if (valid)
    {
      if (voxel_grid_->getCell(voxel_loc.x(), voxel_loc.y(), voxel_loc.z()).distance_square_ > 0)
      {
        voxel_points.push_back(voxel_loc);
      }
    }
  }
  addNewObstacleVoxels(voxel_points);
}

void PropagationDistanceField::removePointsFromField(const EigenSTL::vector_Vector3d& points)
{
  EigenSTL::vector_Vector3i voxel_points;
  // VoxelSet voxel_locs;

  for (const Eigen::Vector3d& point : points)
  {
    // Convert to voxel coordinates
    Eigen::Vector3i voxel_loc;
    bool valid = worldToGrid(point.x(), point.y(), point.z(), voxel_loc.x(), voxel_loc.y(), voxel_loc.z());

    if (valid)
    {
      voxel_points.push_back(voxel_loc);
      // if(voxel_grid_->getCell(voxel_loc.x(),voxel_loc.y(),voxel_loc.z()).distance_square_ == 0) {
      //  voxel_locs.insert(voxel_loc);
      //}
    }
  }

  removeObstacleVoxels(voxel_points);
}

void PropagationDistanceField::addNewObstacleVoxels(const EigenSTL::vector_Vector3i& voxel_points)
{
  int initial_update_direction = getDirectionNumber(0, 0, 0);
  bucket_queue_[0].reserve(voxel_points.size());
  EigenSTL::vector_Vector3i negative_stack;
  if (propagate_negative_)
  {
    negative_stack.reserve(getXNumCells() * getYNumCells() * getZNumCells());
    negative_bucket_queue_[0].reserve(voxel_points.size());
  }

  for (const Eigen::Vector3i& voxel_point : voxel_points)
  {
    PropDistanceFieldVoxel& voxel = voxel_grid_->getCell(voxel_point.x(), voxel_point.y(), voxel_point.z());
    const Eigen::Vector3i& loc = voxel_point;
    voxel.distance_square_ = 0;
    voxel.closest_point_ = loc;
    voxel.update_direction_ = initial_update_direction;
    bucket_queue_[0].push_back(loc);
    if (propagate_negative_)
    {
      voxel.negative_distance_square_ = max_distance_sq_;
      voxel.closest_negative_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
      voxel.closest_negative_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
      voxel.closest_negative_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
      negative_stack.push_back(loc);
    }
  }
  propagatePositive();

  if (propagate_negative_)
  {
    while (!negative_stack.empty())
    {
      Eigen::Vector3i loc = negative_stack.back();
      negative_stack.pop_back();

      for (int neighbor = 0; neighbor < 27; neighbor++)
      {
        Eigen::Vector3i diff = getLocationDifference(neighbor);
        Eigen::Vector3i nloc(loc.x() + diff.x(), loc.y() + diff.y(), loc.z() + diff.z());

        if (isCellValid(nloc.x(), nloc.y(), nloc.z()))
        {
          PropDistanceFieldVoxel& nvoxel = voxel_grid_->getCell(nloc.x(), nloc.y(), nloc.z());
          Eigen::Vector3i& close_point = nvoxel.closest_negative_point_;
          if (!isCellValid(close_point.x(), close_point.y(), close_point.z()))
          {
            close_point = nloc;
          }
          PropDistanceFieldVoxel& closest_point_voxel =
              voxel_grid_->getCell(close_point.x(), close_point.y(), close_point.z());

          // our closest non-obstacle cell has become an obstacle
          if (closest_point_voxel.negative_distance_square_ != 0)
          {
            // find all neigbors inside pre-existing obstacles whose
            // closest_negative_point_ is now an obstacle.  These must all be
            // set to max_distance_sq_ so they will be re-propogated with a new
            // closest_negative_point_ that is outside the obstacle.
            if (nvoxel.negative_distance_square_ != max_distance_sq_)
            {
              nvoxel.negative_distance_square_ = max_distance_sq_;
              nvoxel.closest_negative_point_.x() = PropDistanceFieldVoxel::UNINITIALIZED;
              nvoxel.closest_negative_point_.y() = PropDistanceFieldVoxel::UNINITIALIZED;
              nvoxel.closest_negative_point_.z() = PropDistanceFieldVoxel::UNINITIALIZED;
              negative_stack.push_back(nloc);
            }
          }
          else
          {
            // this cell still has a valid non-obstacle cell, so we need to propogate from it
            nvoxel.negative_update_direction_ = initial_update_direction;
            negative_bucket_queue_[0].push_back(nloc);
          }
        }
      }
    }
    propagateNegative();
  }
}

void PropagationDistanceField::removeObstacleVoxels(const EigenSTL::vector_Vector3i& voxel_points)
// const VoxelSet& locations )
{
  EigenSTL::vector_Vector3i stack;
  EigenSTL::vector_Vector3i negative_stack;
  int initial_update_direction = getDirectionNumber(0, 0, 0);

  stack.reserve(getXNumCells() * getYNumCells() * getZNumCells());
  bucket_queue_[0].reserve(voxel_points.size());
  if (propagate_negative_)
  {
    negative_stack.reserve(getXNumCells() * getYNumCells() * getZNumCells());
    negative_bucket_queue_[0].reserve(voxel_points.size());
  }

  // First reset the obstacle voxels,
  // VoxelSet::const_iterator it = locations.begin();
  // for( it=locations.begin(); it!=locations.end(); ++it)
  // {
  //   Eigen::Vector3i loc = *it;
  //   bool valid = isCellValid( loc.x(), loc.y(), loc.z());
  //   if (!valid)
  //     continue;
  for (const Eigen::Vector3i& voxel_point : voxel_points)
  {
    PropDistanceFieldVoxel& voxel = voxel_grid_->getCell(voxel_point.x(), voxel_point.y(), voxel_point.z());
    voxel.distance_square_ = max_distance_sq_;
    voxel.closest_point_ = voxel_point;
    voxel.update_direction_ = initial_update_direction;  // not needed?
    stack.push_back(voxel_point);
    if (propagate_negative_)
    {
      voxel.negative_distance_square_ = 0.0;
      voxel.closest_negative_point_ = voxel_point;
      voxel.negative_update_direction_ = initial_update_direction;
      negative_bucket_queue_[0].push_back(voxel_point);
    }
  }

  // Reset all neighbors who's closest point is now gone.
  while (!stack.empty())
  {
    Eigen::Vector3i loc = stack.back();
    stack.pop_back();

    for (int neighbor = 0; neighbor < 27; neighbor++)
    {
      Eigen::Vector3i diff = getLocationDifference(neighbor);
      Eigen::Vector3i nloc(loc.x() + diff.x(), loc.y() + diff.y(), loc.z() + diff.z());

      if (isCellValid(nloc.x(), nloc.y(), nloc.z()))
      {
        PropDistanceFieldVoxel& nvoxel = voxel_grid_->getCell(nloc.x(), nloc.y(), nloc.z());
        Eigen::Vector3i& close_point = nvoxel.closest_point_;
        if (!isCellValid(close_point.x(), close_point.y(), close_point.z()))
        {
          close_point = nloc;
        }
        PropDistanceFieldVoxel& closest_point_voxel =
            voxel_grid_->getCell(close_point.x(), close_point.y(), close_point.z());

        if (closest_point_voxel.distance_square_ != 0)
        {  // closest point no longer exists
          if (nvoxel.distance_square_ != max_distance_sq_)
          {
            nvoxel.distance_square_ = max_distance_sq_;
            nvoxel.closest_point_ = nloc;
            nvoxel.update_direction_ = initial_update_direction;  // not needed?
            stack.push_back(nloc);
          }
        }
        else
        {  // add to queue so we can propagate the values
          nvoxel.update_direction_ = initial_update_direction;
          bucket_queue_[0].push_back(nloc);
        }
      }
    }
  }
  propagatePositive();

  if (propagate_negative_)
  {
    propagateNegative();
  }
}

void PropagationDistanceField::propagatePositive()
{
  // now process the queue:
  for (unsigned int i = 0; i < bucket_queue_.size(); ++i)
  {
    EigenSTL::vector_Vector3i::iterator list_it = bucket_queue_[i].begin();
    EigenSTL::vector_Vector3i::iterator list_end = bucket_queue_[i].end();
    for (; list_it != list_end; ++list_it)
    {
      const Eigen::Vector3i& loc = *list_it;
      PropDistanceFieldVoxel* vptr = &voxel_grid_->getCell(loc.x(), loc.y(), loc.z());

      // select the neighborhood list based on the update direction:
      EigenSTL::vector_Vector3i* neighborhood;
      int d = i;
      if (d > 1)
        d = 1;

      // This will never happen.  update_direction_ is always set before voxel is added to bucket queue.
      if (vptr->update_direction_ < 0 || vptr->update_direction_ > 26)
      {
        ROS_ERROR_NAMED("distance_field", "PROGRAMMING ERROR: Invalid update direction detected: %d",
                        vptr->update_direction_);
        continue;
      }

      neighborhood = &neighborhoods_[d][vptr->update_direction_];

      for (const Eigen::Vector3i& diff : *neighborhood)
      {
        Eigen::Vector3i nloc(loc.x() + diff.x(), loc.y() + diff.y(), loc.z() + diff.z());
        if (!isCellValid(nloc.x(), nloc.y(), nloc.z()))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        PropDistanceFieldVoxel* neighbor = &voxel_grid_->getCell(nloc.x(), nloc.y(), nloc.z());
        int new_distance_sq = (vptr->closest_point_ - nloc).squaredNorm();
        if (new_distance_sq > max_distance_sq_)
          continue;

        if (new_distance_sq < neighbor->distance_square_)
        {
          // update the neighboring voxel
          neighbor->distance_square_ = new_distance_sq;
          neighbor->closest_point_ = vptr->closest_point_;
          neighbor->update_direction_ = getDirectionNumber(diff.x(), diff.y(), diff.z());

          // and put it in the queue:
          bucket_queue_[new_distance_sq].push_back(nloc);
        }
      }
    }
    bucket_queue_[i].clear();
  }
}

void PropagationDistanceField::propagateNegative()
{
  // now process the queue:
  for (unsigned int i = 0; i < negative_bucket_queue_.size(); ++i)
  {
    EigenSTL::vector_Vector3i::iterator list_it = negative_bucket_queue_[i].begin();
    EigenSTL::vector_Vector3i::iterator list_end = negative_bucket_queue_[i].end();
    for (; list_it != list_end; ++list_it)
    {
      const Eigen::Vector3i& loc = *list_it;
      PropDistanceFieldVoxel* vptr = &voxel_grid_->getCell(loc.x(), loc.y(), loc.z());

      // select the neighborhood list based on the update direction:
      EigenSTL::vector_Vector3i* neighborhood;
      int d = i;
      if (d > 1)
        d = 1;

      // This will never happen.  negative_update_direction_ is always set before voxel is added to
      // negative_bucket_queue_.
      if (vptr->negative_update_direction_ < 0 || vptr->negative_update_direction_ > 26)
      {
        ROS_ERROR_NAMED("distance_field", "PROGRAMMING ERROR: Invalid update direction detected: %d",
                        vptr->update_direction_);
        continue;
      }

      neighborhood = &neighborhoods_[d][vptr->negative_update_direction_];

      for (const Eigen::Vector3i& diff : *neighborhood)
      {
        Eigen::Vector3i nloc(loc.x() + diff.x(), loc.y() + diff.y(), loc.z() + diff.z());
        if (!isCellValid(nloc.x(), nloc.y(), nloc.z()))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        PropDistanceFieldVoxel* neighbor = &voxel_grid_->getCell(nloc.x(), nloc.y(), nloc.z());
        int new_distance_sq = (vptr->closest_negative_point_ - nloc).squaredNorm();
        if (new_distance_sq > max_distance_sq_)
          continue;
        // std::cout << "Looking at " << nloc.x() << " " << nloc.y() << " " << nloc.z() << " " << new_distance_sq << " "
        // << neighbor->negative_distance_square_ << std::endl;
        if (new_distance_sq < neighbor->negative_distance_square_)
        {
          // std::cout << "Updating " << nloc.x() << " " << nloc.y() << " " << nloc.z() << " " << new_distance_sq <<
          // std::endl;
          // update the neighboring voxel
          neighbor->negative_distance_square_ = new_distance_sq;
          neighbor->closest_negative_point_ = vptr->closest_negative_point_;
          neighbor->negative_update_direction_ = getDirectionNumber(diff.x(), diff.y(), diff.z());

          // and put it in the queue:
          negative_bucket_queue_[new_distance_sq].push_back(nloc);
        }
      }
    }
    negative_bucket_queue_[i].clear();
  }
}

void PropagationDistanceField::reset()
{
  voxel_grid_->reset(PropDistanceFieldVoxel(max_distance_sq_, 0));
  for (int x = 0; x < getXNumCells(); x++)
  {
    for (int y = 0; y < getYNumCells(); y++)
    {
      for (int z = 0; z < getZNumCells(); z++)
      {
        PropDistanceFieldVoxel& voxel = voxel_grid_->getCell(x, y, z);
        voxel.closest_negative_point_.x() = x;
        voxel.closest_negative_point_.y() = y;
        voxel.closest_negative_point_.z() = z;
        voxel.negative_distance_square_ = 0;
      }
    }
  }
  // object_voxel_locations_.clear();
}

void PropagationDistanceField::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dz = -1; dz <= 1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        Eigen::Vector3i n_point(dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n = 0; n < 2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dz = -1; dz <= 1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx = -1; tdx <= 1; ++tdx)
          {
            for (int tdy = -1; tdy <= 1; ++tdy)
            {
              for (int tdz = -1; tdz <= 1; ++tdz)
              {
                if (tdx == 0 && tdy == 0 && tdz == 0)
                  continue;
                if (n >= 1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1)
                    continue;
                  if (dx * tdx < 0 || dy * tdy < 0 || dz * tdz < 0)
                    continue;
                }
                Eigen::Vector3i n_point(tdx, tdy, tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          // printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz,
          // neighborhoods_[n][direction_number].size());
        }
      }
    }
  }
}

int PropagationDistanceField::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx + 1) * 9 + (dy + 1) * 3 + dz + 1;
}

Eigen::Vector3i PropagationDistanceField::getLocationDifference(int directionNumber) const
{
  return direction_number_to_direction_[directionNumber];
}

double PropagationDistanceField::getDistance(double x, double y, double z) const
{
  return getDistance((*voxel_grid_.get())(x, y, z));
}

double PropagationDistanceField::getDistance(int x, int y, int z) const
{
  return getDistance(voxel_grid_->getCell(x, y, z));
}

bool PropagationDistanceField::isCellValid(int x, int y, int z) const
{
  return voxel_grid_->isCellValid(x, y, z);
}

int PropagationDistanceField::getXNumCells() const
{
  return voxel_grid_->getNumCells(DIM_X);
}

int PropagationDistanceField::getYNumCells() const
{
  return voxel_grid_->getNumCells(DIM_Y);
}

int PropagationDistanceField::getZNumCells() const
{
  return voxel_grid_->getNumCells(DIM_Z);
}

bool PropagationDistanceField::gridToWorld(int x, int y, int z, double& world_x, double& world_y, double& world_z) const
{
  voxel_grid_->gridToWorld(x, y, z, world_x, world_y, world_z);
  return true;
}

bool PropagationDistanceField::worldToGrid(double world_x, double world_y, double world_z, int& x, int& y, int& z) const
{
  return voxel_grid_->worldToGrid(world_x, world_y, world_z, x, y, z);
}

bool PropagationDistanceField::writeToStream(std::ostream& os) const
{
  os << "resolution: " << resolution_ << std::endl;
  os << "size_x: " << size_x_ << std::endl;
  os << "size_y: " << size_y_ << std::endl;
  os << "size_z: " << size_z_ << std::endl;
  os << "origin_x: " << origin_x_ << std::endl;
  os << "origin_y: " << origin_y_ << std::endl;
  os << "origin_z: " << origin_z_ << std::endl;
  // now the binary stuff

  // first writing to zlib compressed buffer
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::zlib_compressor());
  out.push(os);

  for (unsigned int x = 0; x < static_cast<unsigned int>(getXNumCells()); x++)
  {
    for (unsigned int y = 0; y < static_cast<unsigned int>(getYNumCells()); y++)
    {
      for (unsigned int z = 0; z < static_cast<unsigned int>(getZNumCells()); z += 8)
      {
        std::bitset<8> bs(0);
        unsigned int zv = std::min((unsigned int)8, getZNumCells() - z);
        for (unsigned int zi = 0; zi < zv; zi++)
        {
          if (getCell(x, y, z + zi).distance_square_ == 0)
          {
            // std::cout << "Marking obs cell " << x << " " << y << " " << z+zi << std::endl;
            bs[zi] = 1;
          }
        }
        out.write((char*)&bs, sizeof(char));
      }
    }
  }
  out.flush();
  return true;
}

bool PropagationDistanceField::readFromStream(std::istream& is)
{
  if (!is.good())
    return false;

  std::string temp;

  is >> temp;
  if (temp != "resolution:")
    return false;
  is >> resolution_;

  is >> temp;
  if (temp != "size_x:")
    return false;
  is >> size_x_;

  is >> temp;
  if (temp != "size_y:")
    return false;
  is >> size_y_;

  is >> temp;
  if (temp != "size_z:")
    return false;
  is >> size_z_;

  is >> temp;
  if (temp != "origin_x:")
    return false;
  is >> origin_x_;

  is >> temp;
  if (temp != "origin_y:")
    return false;
  is >> origin_y_;

  is >> temp;
  if (temp != "origin_z:")
    return false;
  is >> origin_z_;

  // previous values for propogation_negative_ and max_distance_ will be used

  initialize();

  // this should be newline
  char nl;
  is.get(nl);

  // now we start the compressed portion
  boost::iostreams::filtering_istream in;
  in.push(boost::iostreams::zlib_decompressor());
  in.push(is);

  // std::cout << "Nums " << getXNumCells() << " " << getYNumCells() << " " << getZNumCells() << std::endl;

  EigenSTL::vector_Vector3i obs_points;
  for (unsigned int x = 0; x < static_cast<unsigned int>(getXNumCells()); x++)
  {
    for (unsigned int y = 0; y < static_cast<unsigned int>(getYNumCells()); y++)
    {
      for (unsigned int z = 0; z < static_cast<unsigned int>(getZNumCells()); z += 8)
      {
        char inchar;
        if (!in.good())
        {
          return false;
        }
        in.get(inchar);
        std::bitset<8> inbit((unsigned long long)inchar);
        unsigned int zv = std::min((unsigned int)8, getZNumCells() - z);
        for (unsigned int zi = 0; zi < zv; zi++)
        {
          if (inbit[zi] == 1)
          {
            // std::cout << "Adding obs cell " << x << " " << y << " " << z+zi << std::endl;
            obs_points.push_back(Eigen::Vector3i(x, y, z + zi));
          }
        }
      }
    }
  }
  addNewObstacleVoxels(obs_points);
  return true;
}
}  // namespace distance_field
