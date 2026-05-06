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

#include <gtest/gtest.h>

#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit/distance_field/find_internal_points.h>
#include <geometric_shapes/body_operations.h>
#include <tf2_eigen/tf2_eigen.h>
#include <octomap/octomap.h>
#include <ros/console.h>

#include <memory>

using namespace distance_field;

static const double WIDTH = 1.0;
static const double HEIGHT = 1.0;
static const double DEPTH = 1.0;
static const double RESOLUTION = 0.1;
static const double ORIGIN_X = 0.0;
static const double ORIGIN_Y = 0.0;
static const double ORIGIN_Z = 0.0;
static const double MAX_DIST = 0.3;

static const Eigen::Vector3d POINT1(0.1, 0.0, 0.0);
static const Eigen::Vector3d POINT2(0.0, 0.1, 0.2);
static const Eigen::Vector3d POINT3(0.4, 0.0, 0.0);

int dist_sq(int x, int y, int z)
{
  return x * x + y * y + z * z;
}

void print(PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  for (int z = 0; z < numZ; z++)
  {
    for (int y = 0; y < numY; y++)
    {
      for (int x = 0; x < numX; x++)
      {
        std::cout << pdf.getCell(x, y, z).distance_square_ << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
  for (int z = 0; z < numZ; z++)
  {
    for (int y = 0; y < numY; y++)
    {
      for (int x = 0; x < numX; x++)
      {
        if (pdf.getCell(x, y, z).distance_square_ == 0)
        {
          // ROS_INFO_NAMED("distance_field", "Obstacle cell %d %d %d", x, y, z);
        }
      }
    }
  }
}

void printNeg(PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  for (int z = 0; z < numZ; z++)
  {
    for (int y = 0; y < numY; y++)
    {
      for (int x = 0; x < numX; x++)
      {
        std::cout << pdf.getCell(x, y, z).negative_distance_square_ << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

void printPointCoords(const Eigen::Vector3i& p)
{
  if (p.x() < 0)
    std::cout << '-';
  else
    std::cout << p.x();

  if (p.y() < 0)
    std::cout << '-';
  else
    std::cout << p.y();

  if (p.z() < 0)
    std::cout << '-';
  else
    std::cout << p.z();

  std::cout << " ";
}

void printBoth(PropagationDistanceField& pdf, int numX, int numY, int numZ)
{
  std::cout << "Positive distance square ... negative distance square" << std::endl;
  for (int z = 0; z < numZ; z++)
  {
    std::cout << "Z=" << z << std::endl;
    for (int y = 0; y < numY; y++)
    {
      for (int x = 0; x < numX; x++)
      {
        std::cout << pdf.getCell(x, y, z).distance_square_ << " ";
      }
      std::cout << "   ";
      for (int x = 0; x < numX; x++)
      {
        std::cout << pdf.getCell(x, y, z).negative_distance_square_ << " ";
      }
      std::cout << "     ";
      for (int x = 0; x < numX; x++)
      {
        printPointCoords(pdf.getCell(x, y, z).closest_point_);
      }
      std::cout << "   ";
      for (int x = 0; x < numX; x++)
      {
        printPointCoords(pdf.getCell(x, y, z).closest_negative_point_);
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

bool areDistanceFieldsDistancesEqual(const PropagationDistanceField& df1, const PropagationDistanceField& df2)
{
  if (df1.getXNumCells() != df2.getXNumCells())
    return false;
  if (df1.getYNumCells() != df2.getYNumCells())
    return false;
  if (df1.getZNumCells() != df2.getZNumCells())
    return false;
  for (int z = 0; z < df1.getZNumCells(); z++)
  {
    for (int x = 0; x < df1.getXNumCells(); x++)
    {
      for (int y = 0; y < df1.getYNumCells(); y++)
      {
        if (df1.getCell(x, y, z).distance_square_ != df2.getCell(x, y, z).distance_square_)
        {
          printf("Cell %d %d %d distances not equal %d %d\n", x, y, z, df1.getCell(x, y, z).distance_square_,
                 df2.getCell(x, y, z).distance_square_);
          return false;
        }
        if (df1.getCell(x, y, z).negative_distance_square_ != df2.getCell(x, y, z).negative_distance_square_)
        {
          printf("Cell %d %d %d neg distances not equal %d %d\n", x, y, z,
                 df1.getCell(x, y, z).negative_distance_square_, df2.getCell(x, y, z).negative_distance_square_);
          return false;
        }
      }
    }
  }
  return true;
}

bool checkOctomapVersusDistanceField(const PropagationDistanceField& df, const octomap::OcTree& octree)
{
  // just one way for now
  for (int z = 0; z < df.getZNumCells(); z++)
  {
    for (int x = 0; x < df.getXNumCells(); x++)
    {
      for (int y = 0; y < df.getYNumCells(); y++)
      {
        if (df.getCell(x, y, z).distance_square_ == 0)
        {
          // making sure that octomap also shows it as occupied
          double qx, qy, qz;
          df.gridToWorld(x, y, z, qx, qy, qz);
          octomap::point3d query(qx, qy, qz);
          octomap::OcTreeNode* result = octree.search(query);
          if (!result)
          {
            for (float boundary_x = query.x() - df.getResolution();
                 boundary_x <= query.x() + df.getResolution() && !result; boundary_x += df.getResolution())
            {
              for (float boundary_y = query.y() - df.getResolution();
                   boundary_y <= query.y() + df.getResolution() && !result; boundary_y += df.getResolution())
              {
                for (float boundary_z = query.z() - df.getResolution(); boundary_z <= query.z() + df.getResolution();
                     boundary_z += df.getResolution())
                {
                  octomap::point3d query_boundary(boundary_x, boundary_y, boundary_z);
                  result = octree.search(query_boundary);
                  if (result)
                  {
                    break;
                  }
                }
              }
            }
            if (!result)
            {
              std::cout << "No point at potential boundary query " << query.x() << " " << query.y() << " " << query.z()
                        << std::endl;
              return false;
            }
          }
          if (!octree.isNodeOccupied(result))
          {
            std::cout << "Disagreement at " << qx << " " << qy << " " << qz << std::endl;
            return false;
          }
        }
      }
    }
  }
  return true;
}

unsigned int countOccupiedCells(const PropagationDistanceField& df)
{
  unsigned int count = 0;
  for (int z = 0; z < df.getZNumCells(); z++)
  {
    for (int x = 0; x < df.getXNumCells(); x++)
    {
      for (int y = 0; y < df.getYNumCells(); y++)
      {
        if (df.getCell(x, y, z).distance_square_ == 0)
        {
          count++;
        }
      }
    }
  }
  return count;
}

unsigned int countLeafNodes(const octomap::OcTree& octree)
{
  unsigned int count = 0;
  for (octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end = octree.end_leafs(); it != end; ++it)
  {
    if (octree.isNodeOccupied(*it))
    {
      std::cout << "Leaf node " << it.getX() << " " << it.getY() << " " << it.getZ() << std::endl;
      count++;
    }
  }
  return count;
}

// points should contain all occupied points
void check_distance_field(const PropagationDistanceField& df, const EigenSTL::vector_Vector3d& points, int numX,
                          int numY, int numZ, bool do_negs)
{
  EigenSTL::vector_Vector3i points_ind(points.size());
  for (unsigned int i = 0; i < points.size(); i++)
  {
    Eigen::Vector3i loc;
    df.worldToGrid(points[i].x(), points[i].y(), points[i].z(), loc.x(), loc.y(), loc.z());
    points_ind[i] = loc;
  }

  for (int x = 0; x < numX; x++)
  {
    for (int y = 0; y < numY; y++)
    {
      for (int z = 0; z < numZ; z++)
      {
        double dsq = df.getCell(x, y, z).distance_square_;
        double ndsq = df.getCell(x, y, z).negative_distance_square_;
        if (dsq == 0)
        {
          bool found = false;
          for (Eigen::Vector3i& point : points_ind)
          {
            if (point.x() == x && point.y() == y && point.z() == z)
            {
              found = true;
              break;
            }
          }
          if (do_negs)
          {
            ASSERT_GT(ndsq, 0) << "Obstacle point " << x << " " << y << " " << z << " has zero negative value";
          }
          ASSERT_TRUE(found) << "Obstacle point " << x << " " << y << " " << z << " not found";
        }
      }
    }
  }
}

TEST(TestPropagationDistanceField, TestAddRemovePoints)
{
  PropagationDistanceField df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST);

  // Check size
  int num_x = df.getXNumCells();
  int num_y = df.getYNumCells();
  int num_z = df.getZNumCells();

  EXPECT_EQ(num_x, (int)(WIDTH / RESOLUTION + 0.5));
  EXPECT_EQ(num_y, (int)(HEIGHT / RESOLUTION + 0.5));
  EXPECT_EQ(num_z, (int)(DEPTH / RESOLUTION + 0.5));

  // getting a bad point
  double tgx, tgy, tgz;
  bool in_bounds;
  EXPECT_NEAR(df.getDistance(1000.0, 1000.0, 1000.0), MAX_DIST, .0001);
  EXPECT_NEAR(df.getDistanceGradient(1000.0, 1000.0, 1000.0, tgx, tgy, tgz, in_bounds), MAX_DIST, .0001);
  EXPECT_FALSE(in_bounds);

  // Add points to the grid
  EigenSTL::vector_Vector3d points;
  points.push_back(POINT1);
  points.push_back(POINT2);
  ROS_INFO_NAMED("distance_field", "Adding %zu points", points.size());
  df.addPointsToField(points);
  // print(df, numX, numY, numZ);

  // Update - iterative
  points.clear();
  points.push_back(POINT2);
  points.push_back(POINT3);
  EigenSTL::vector_Vector3d old_points;
  old_points.push_back(POINT1);
  df.updatePointsInField(old_points, points);
  // std::cout << "One removal, one addition" << std::endl;
  // print(df, numX, numY, numZ);
  // printNeg(df, numX, numY, numZ);
  check_distance_field(df, points, num_x, num_y, num_z, false);

  // Remove
  points.clear();
  points.push_back(POINT2);
  df.removePointsFromField(points);
  points.clear();
  points.push_back(POINT3);
  check_distance_field(df, points, num_x, num_y, num_z, false);

  // now testing gradient calls
  df.reset();
  points.clear();
  points.push_back(POINT1);
  df.addPointsToField(points);
  bool first = true;
  for (int z = 1; z < df.getZNumCells() - 1; z++)
  {
    for (int x = 1; x < df.getXNumCells() - 1; x++)
    {
      for (int y = 1; y < df.getYNumCells() - 1; y++)
      {
        double dist = df.getDistance(x, y, z);
        double wx, wy, wz;
        df.gridToWorld(x, y, z, wx, wy, wz);
        Eigen::Vector3d grad(0.0, 0.0, 0.0);
        bool grad_in_bounds;
        double dist_grad = df.getDistanceGradient(wx, wy, wz, grad.x(), grad.y(), grad.z(), grad_in_bounds);
        ASSERT_TRUE(grad_in_bounds) << x << " " << y << " " << z;
        ASSERT_NEAR(dist, dist_grad, .0001);
        if (dist > 0 && dist < MAX_DIST)
        {
          double xscale = grad.x() / grad.norm();
          double yscale = grad.y() / grad.norm();
          double zscale = grad.z() / grad.norm();

          double comp_x = wx - xscale * dist;
          double comp_y = wy - yscale * dist;
          double comp_z = wz - zscale * dist;
          if (first)
          {
            first = false;
            std::cout << "Dist " << dist << std::endl;
            std::cout << "Cell " << x << " " << y << " " << z << " " << wx << " " << wy << " " << wz << std::endl;
            std::cout << "Scale " << xscale << " " << yscale << " " << zscale << std::endl;
            std::cout << "Grad " << grad.x() << " " << grad.y() << " " << grad.z() << " comp " << comp_x << " "
                      << comp_y << " " << comp_z << std::endl;
          }
          ASSERT_NEAR(comp_x, POINT1.x(), RESOLUTION)
              << dist << x << " " << y << " " << z << " " << grad.x() << " " << grad.y() << " " << grad.z() << " "
              << xscale << " " << yscale << " " << zscale << std::endl;
          ASSERT_NEAR(comp_y, POINT1.y(), RESOLUTION)
              << x << " " << y << " " << z << " " << grad.x() << " " << grad.y() << " " << grad.z() << " " << xscale
              << " " << yscale << " " << zscale << std::endl;
          ASSERT_NEAR(comp_z, POINT1.z(), RESOLUTION)
              << x << " " << y << " " << z << " " << grad.x() << " " << grad.y() << " " << grad.z() << " " << xscale
              << " " << yscale << " " << zscale << std::endl;
        }
      }
    }
  }
  ASSERT_FALSE(first);
}

TEST(TestSignedPropagationDistanceField, TestSignedAddRemovePoints)
{
  PropagationDistanceField df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST, true);

  // Check size
  int num_x = df.getXNumCells();
  int num_y = df.getYNumCells();
  int num_z = df.getZNumCells();

  EXPECT_EQ(num_x, (int)(WIDTH / RESOLUTION + 0.5));
  EXPECT_EQ(num_y, (int)(HEIGHT / RESOLUTION + 0.5));
  EXPECT_EQ(num_z, (int)(DEPTH / RESOLUTION + 0.5));

  // Error checking
  // print(df, numX, numY, numZ);

  // TODO - check initial values
  // EXPECT_EQ( df.getCell(0,0,0).distance_square_, max_dist_sq_in_voxels );

  // Add points to the grid
  double lwx, lwy, lwz;
  double hwx, hwy, hwz;
  df.gridToWorld(1, 1, 1, lwx, lwy, lwz);
  df.gridToWorld(8, 8, 8, hwx, hwy, hwz);

  EigenSTL::vector_Vector3d points;
  for (double x = lwx; x <= hwx; x += .1)
  {
    for (double y = lwy; y <= hwy; y += .1)
    {
      for (double z = lwz; z <= hwz; z += .1)
      {
        points.push_back(Eigen::Vector3d(x, y, z));
      }
    }
  }

  df.reset();
  ROS_INFO_NAMED("distance_field", "Adding %zu points", points.size());
  df.addPointsToField(points);
  // print(df, numX, numY, numZ);
  // printNeg(df, numX, numY, numZ);

  double cx, cy, cz;
  df.gridToWorld(5, 5, 5, cx, cy, cz);

  Eigen::Vector3d center_point(cx, cy, cz);

  EigenSTL::vector_Vector3d rem_points;
  rem_points.push_back(center_point);
  df.removePointsFromField(rem_points);
  // std::cout << "Pos "<< std::endl;
  // print(df, numX, numY, numZ);
  // std::cout << "Neg "<< std::endl;
  // printNeg(df, numX, numY, numZ);

  // testing equality with initial add of points without the center point
  PropagationDistanceField test_df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST, true);
  EigenSTL::vector_Vector3d test_points;
  for (const Eigen::Vector3d& point : points)
  {
    if (point.x() != center_point.x() || point.y() != center_point.y() || point.z() != center_point.z())
    {
      test_points.push_back(point);
    }
  }
  test_df.addPointsToField(test_points);
  ASSERT_TRUE(areDistanceFieldsDistancesEqual(df, test_df));

  PropagationDistanceField gradient_df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST, true);

  shapes::Sphere sphere(.25);

  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  p.position.x = .5;
  p.position.y = .5;
  p.position.z = .5;

  Eigen::Isometry3d p_eigen;
  tf2::fromMsg(p, p_eigen);

  gradient_df.addShapeToField(&sphere, p_eigen);
  // printBoth(gradient_df, numX, numY, numZ);
  EXPECT_GT(gradient_df.getCell(5, 5, 5).negative_distance_square_, 1);
  // all negative cells should have gradients that point towards cells with distance 1
  for (int z = 1; z < df.getZNumCells() - 1; z++)
  {
    for (int x = 1; x < df.getXNumCells() - 1; x++)
    {
      for (int y = 1; y < df.getYNumCells() - 1; y++)
      {
        double dist = gradient_df.getDistance(x, y, z);
        double ncell_dist;
        Eigen::Vector3i ncell_pos;
        const PropDistanceFieldVoxel* ncell = gradient_df.getNearestCell(x, y, z, ncell_dist, ncell_pos);

        EXPECT_EQ(ncell_dist, dist);

        if (ncell == nullptr)
        {
          if (ncell_dist > 0)
          {
            EXPECT_GE(ncell_dist, gradient_df.getUninitializedDistance())
                << "dist=" << dist << " xyz=" << x << " " << y << " " << z << " ncell=" << ncell_pos.x() << " "
                << ncell_pos.y() << " " << ncell_pos.z() << std::endl;
          }
          else if (ncell_dist < 0)
          {
            EXPECT_LE(ncell_dist, -gradient_df.getUninitializedDistance())
                << "dist=" << dist << " xyz=" << x << " " << y << " " << z << " ncell=" << ncell_pos.x() << " "
                << ncell_pos.y() << " " << ncell_pos.z() << std::endl;
          }
        }

        if (gradient_df.getCell(x, y, z).negative_distance_square_ > 0)
        {
          ASSERT_LT(dist, 0) << "Pos " << gradient_df.getCell(x, y, z).distance_square_ << " "
                             << gradient_df.getCell(x, y, z).negative_distance_square_;
          double wx, wy, wz;
          df.gridToWorld(x, y, z, wx, wy, wz);
          Eigen::Vector3d grad(0.0, 0.0, 0.0);
          bool grad_in_bounds;
          double dist_grad = gradient_df.getDistanceGradient(wx, wy, wz, grad.x(), grad.y(), grad.z(), grad_in_bounds);
          ASSERT_TRUE(grad_in_bounds) << x << " " << y << " " << z;
          ASSERT_NEAR(dist, dist_grad, .0001);

          if (!ncell)
            continue;

          EXPECT_GE(gradient_df.getCell(ncell_pos.x(), ncell_pos.y(), ncell_pos.z()).distance_square_, 1)
              << "dist=" << dist << " xyz=" << x << " " << y << " " << z << " grad=" << grad.x() << " " << grad.y()
              << " " << grad.z() << " ncell=" << ncell_pos.x() << " " << ncell_pos.y() << " " << ncell_pos.z()
              << std::endl;

          double grad_size_sq = grad.squaredNorm();
          if (grad_size_sq < std::numeric_limits<double>::epsilon())
            continue;

          double oo_grad_size = 1.0 / sqrt(grad_size_sq);
          double xscale = grad.x() * oo_grad_size;
          double yscale = grad.y() * oo_grad_size;
          double zscale = grad.z() * oo_grad_size;

          double comp_x = wx - xscale * dist;
          double comp_y = wy - yscale * dist;
          double comp_z = wz - zscale * dist;

          int cell_x, cell_y, cell_z;
          bool cell_in_bounds = gradient_df.worldToGrid(comp_x, comp_y, comp_z, cell_x, cell_y, cell_z);

          ASSERT_EQ(cell_in_bounds, true);
          const PropDistanceFieldVoxel* cell = &gradient_df.getCell(cell_x, cell_y, cell_z);

#if 0
          EXPECT_EQ(ncell_pos.x(), cell_x);
          EXPECT_EQ(ncell_pos.y(), cell_y);
          EXPECT_EQ(ncell_pos.z(), cell_z);
          EXPECT_EQ(ncell, cell);
#endif
          EXPECT_GE(cell->distance_square_, 1)
              << dist << " " << x << " " << y << " " << z << " " << grad.x() << " " << grad.y() << " " << grad.z()
              << " " << xscale << " " << yscale << " " << zscale << " cell " << comp_x << " " << comp_y << " " << comp_z
              << std::endl;
        }
      }
    }
  }
}

TEST(TestSignedPropagationDistanceField, TestShape)
{
  PropagationDistanceField df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST, true);

  int num_x = df.getXNumCells();
  int num_y = df.getYNumCells();
  int num_z = df.getZNumCells();

  shapes::Sphere sphere(.25);

  Eigen::Isometry3d p = Eigen::Translation3d(0.5, 0.5, 0.5) * Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
  Eigen::Isometry3d np = Eigen::Translation3d(0.7, 0.7, 0.7) * Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);

  df.addShapeToField(&sphere, p);

  bodies::Body* body = bodies::createBodyFromShape(&sphere);
  body->setPose(p);
  EigenSTL::vector_Vector3d point_vec;
  findInternalPointsConvex(*body, RESOLUTION, point_vec);
  delete body;
  check_distance_field(df, point_vec, num_x, num_y, num_z, true);

  // std::cout << "Shape pos "<< std::endl;
  // print(df, numX, numY, numZ);
  // std::cout << "Shape neg "<< std::endl;
  // printNeg(df, numX, numY, numZ);

  df.addShapeToField(&sphere, np);

  body = bodies::createBodyFromShape(&sphere);
  body->setPose(np);

  EigenSTL::vector_Vector3d point_vec_2;
  findInternalPointsConvex(*body, RESOLUTION, point_vec_2);
  delete body;
  EigenSTL::vector_Vector3d point_vec_union = point_vec_2;
  point_vec_union.insert(point_vec_union.end(), point_vec.begin(), point_vec.end());
  check_distance_field(df, point_vec_union, num_x, num_y, num_z, true);

  // should get rid of old pose
  df.moveShapeInField(&sphere, p, np);

  check_distance_field(df, point_vec_2, num_x, num_y, num_z, true);

  // should be equivalent to just adding second shape
  PropagationDistanceField test_df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST, true);
  test_df.addShapeToField(&sphere, np);
  ASSERT_TRUE(areDistanceFieldsDistancesEqual(df, test_df));
}

static const double PERF_WIDTH = 3.0;
static const double PERF_HEIGHT = 3.0;
static const double PERF_DEPTH = 4.0;
static const double PERF_RESOLUTION = 0.02;
static const double PERF_ORIGIN_X = 0.0;
static const double PERF_ORIGIN_Y = 0.0;
static const double PERF_ORIGIN_Z = 0.0;
static const double PERF_MAX_DIST = .25;
static const unsigned int UNIFORM_DISTANCE = 10;

TEST(TestSignedPropagationDistanceField, TestPerformance)
{
  std::cout << "Creating distance field with "
            << (PERF_WIDTH / PERF_RESOLUTION) * (PERF_HEIGHT / PERF_RESOLUTION) * (PERF_DEPTH / PERF_RESOLUTION)
            << " entries" << std::endl;

  ros::WallTime dt = ros::WallTime::now();
  PropagationDistanceField df(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X, PERF_ORIGIN_Y,
                              PERF_ORIGIN_Z, PERF_MAX_DIST, false);
  std::cout << "Creating unsigned took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  PropagationDistanceField sdf(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X, PERF_ORIGIN_Y,
                               PERF_ORIGIN_Z, PERF_MAX_DIST, true);

  std::cout << "Creating signed took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  shapes::Box big_table(2.0, 2.0, .5);

  Eigen::Isometry3d p = Eigen::Translation3d(PERF_WIDTH / 2.0, PERF_DEPTH / 2.0, PERF_HEIGHT / 2.0) *
                        Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
  Eigen::Isometry3d np = Eigen::Translation3d(PERF_WIDTH / 2.0 + .01, PERF_DEPTH / 2.0, PERF_HEIGHT / 2.0) *
                         Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);

  unsigned int big_num_points = ceil(2.0 / PERF_RESOLUTION) * ceil(2.0 / PERF_RESOLUTION) * ceil(.5 / PERF_RESOLUTION);

  std::cout << "Adding " << big_num_points << " points" << std::endl;

  dt = ros::WallTime::now();
  df.addShapeToField(&big_table, p);
  std::cout << "Adding to unsigned took " << (ros::WallTime::now() - dt).toSec() << " avg "
            << (ros::WallTime::now() - dt).toSec() / (big_num_points * 1.0) << std::endl;

  dt = ros::WallTime::now();
  df.addShapeToField(&big_table, p);
  std::cout << "Re-adding to unsigned took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  sdf.addShapeToField(&big_table, p);
  std::cout << "Adding to signed took " << (ros::WallTime::now() - dt).toSec() << " avg "
            << (ros::WallTime::now() - dt).toSec() / (big_num_points * 1.0) << std::endl;

  dt = ros::WallTime::now();
  df.moveShapeInField(&big_table, p, np);
  std::cout << "Moving in unsigned took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  sdf.moveShapeInField(&big_table, p, np);
  std::cout << "Moving in signed took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  df.removeShapeFromField(&big_table, np);
  std::cout << "Removing from unsigned took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  sdf.removeShapeFromField(&big_table, np);
  std::cout << "Removing from signed took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  df.reset();

  shapes::Box small_table(.25, .25, .05);

  unsigned int small_num_points = (13) * (13) * (3);

  std::cout << "Adding " << small_num_points << " points" << std::endl;

  dt = ros::WallTime::now();
  df.addShapeToField(&small_table, p);
  std::cout << "Adding to unsigned took " << (ros::WallTime::now() - dt).toSec() << " avg "
            << (ros::WallTime::now() - dt).toSec() / (small_num_points * 1.0) << std::endl;

  dt = ros::WallTime::now();
  sdf.addShapeToField(&small_table, p);
  std::cout << "Adding to signed took " << (ros::WallTime::now() - dt).toSec() << " avg "
            << (ros::WallTime::now() - dt).toSec() / (small_num_points * 1.0) << std::endl;

  dt = ros::WallTime::now();
  df.moveShapeInField(&small_table, p, np);
  std::cout << "Moving in unsigned took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  dt = ros::WallTime::now();
  sdf.moveShapeInField(&small_table, p, np);
  std::cout << "Moving in signed took " << (ros::WallTime::now() - dt).toSec() << std::endl;

  // uniformly spaced points - a worst case scenario
  PropagationDistanceField worstdfu(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X, PERF_ORIGIN_Y,
                                    PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  PropagationDistanceField worstdfs(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X, PERF_ORIGIN_Y,
                                    PERF_ORIGIN_Z, PERF_MAX_DIST, true);

  EigenSTL::vector_Vector3d bad_vec;
  unsigned int count = 0;
  for (unsigned int z = UNIFORM_DISTANCE; z < worstdfu.getZNumCells() - UNIFORM_DISTANCE; z += UNIFORM_DISTANCE)
  {
    for (unsigned int x = UNIFORM_DISTANCE; x < worstdfu.getXNumCells() - UNIFORM_DISTANCE; x += UNIFORM_DISTANCE)
    {
      for (unsigned int y = UNIFORM_DISTANCE; y < worstdfu.getYNumCells() - UNIFORM_DISTANCE; y += UNIFORM_DISTANCE)
      {
        count++;
        Eigen::Vector3d loc;
        bool valid = worstdfu.gridToWorld(x, y, z, loc.x(), loc.y(), loc.z());

        if (!valid)
        {
          ROS_WARN_NAMED("distance_field", "Something wrong");
          continue;
        }
        bad_vec.push_back(loc);
      }
    }
  }

  dt = ros::WallTime::now();
  worstdfu.addPointsToField(bad_vec);
  ros::WallDuration wd = ros::WallTime::now() - dt;
  printf("Time for unsigned adding %u uniform points is %g average %g\n", (unsigned int)bad_vec.size(), wd.toSec(),
         wd.toSec() / (bad_vec.size() * 1.0));
  dt = ros::WallTime::now();
  worstdfs.addPointsToField(bad_vec);
  wd = ros::WallTime::now() - dt;
  printf("Time for signed adding %u uniform points is %g average %g\n", (unsigned int)bad_vec.size(), wd.toSec(),
         wd.toSec() / (bad_vec.size() * 1.0));
}

TEST(TestSignedPropagationDistanceField, TestOcTree)
{
  PropagationDistanceField df(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X, PERF_ORIGIN_Y,
                              PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  octomap::OcTree tree(.02);

  unsigned int count = 0;
  for (float x = 1.01; x < 1.5; x += .02)
  {
    for (float y = 1.01; y < 1.5; y += .02)
    {
      for (float z = 1.01; z < 1.5; z += .02, count++)
      {
        octomap::point3d point(x, y, z);
        tree.updateNode(point, true);
      }
    }
  }

  // more points at the border of the distance field
  for (float x = 2.51; x < 3.5; x += .02)
  {
    for (float y = 1.01; y < 3.5; y += .02)
    {
      for (float z = 1.01; z < 3.5; z += .02, count++)
      {
        octomap::point3d point(x, y, z);
        tree.updateNode(point, true);
      }
    }
  }

  std::cout << "OcTree nodes " << count << std::endl;
  df.addOcTreeToField(&tree);

  EXPECT_TRUE(checkOctomapVersusDistanceField(df, tree));

  // more cells
  for (float x = .01; x < .50; x += .02)
  {
    for (float y = .01; y < .50; y += .02)
    {
      for (float z = .01; z < .50; z += .02, count++)
      {
        octomap::point3d point(x, y, z);
        tree.updateNode(point, true);
      }
    }
  }
  df.addOcTreeToField(&tree);
  EXPECT_TRUE(checkOctomapVersusDistanceField(df, tree));

  PropagationDistanceField df_oct(tree, octomap::point3d(0.5, 0.5, 0.5), octomap::point3d(5.0, 5.0, 5.0), PERF_MAX_DIST,
                                  false);

  EXPECT_TRUE(checkOctomapVersusDistanceField(df_oct, tree));

  // now try different resolutions
  octomap::OcTree tree_lowres(.05);
  octomap::point3d point1(.5, .5, .5);
  octomap::point3d point2(.7, .5, .5);
  octomap::point3d point3(1.0, .5, .5);
  tree_lowres.updateNode(point1, true);
  tree_lowres.updateNode(point2, true);
  tree_lowres.updateNode(point3, true);
  ASSERT_EQ(countLeafNodes(tree_lowres), 3u);

  PropagationDistanceField df_highres(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X,
                                      PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  df_highres.addOcTreeToField(&tree_lowres);
  EXPECT_EQ(countOccupiedCells(df_highres), 3u * (4u * 4u * 4u));
  std::cout << "Occupied cells " << countOccupiedCells(df_highres) << std::endl;

  // testing adding shape that happens to be octree
  std::shared_ptr<octomap::OcTree> tree_shape(new octomap::OcTree(.05));
  octomap::point3d tpoint1(1.0, .5, 1.0);
  octomap::point3d tpoint2(1.7, .5, .5);
  octomap::point3d tpoint3(1.8, .5, .5);
  tree_shape->updateNode(tpoint1, true);
  tree_shape->updateNode(tpoint2, true);
  tree_shape->updateNode(tpoint3, true);

  std::shared_ptr<shapes::OcTree> shape_oc(new shapes::OcTree(tree_shape));

  PropagationDistanceField df_test_shape_1(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X,
                                           PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  PropagationDistanceField df_test_shape_2(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X,
                                           PERF_ORIGIN_Y, PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  df_test_shape_1.addOcTreeToField(tree_shape.get());
  df_test_shape_2.addShapeToField(shape_oc.get(), Eigen::Isometry3d());
  EXPECT_TRUE(areDistanceFieldsDistancesEqual(df_test_shape_1, df_test_shape_2));
}

TEST(TestSignedPropagationDistanceField, TestReadWrite)
{
  PropagationDistanceField small_df(WIDTH, HEIGHT, DEPTH, RESOLUTION, ORIGIN_X, ORIGIN_Y, ORIGIN_Z, MAX_DIST);

  EigenSTL::vector_Vector3d points;
  points.push_back(POINT1);
  points.push_back(POINT2);
  points.push_back(POINT3);
  small_df.addPointsToField(points);

  std::ofstream sf("test_small.df", std::ios::out);

  small_df.writeToStream(sf);
  // must close to make sure that the buffer is written
  sf.close();

  std::ifstream si("test_small.df", std::ios::in | std::ios::binary);
  PropagationDistanceField small_df2(si, PERF_MAX_DIST, false);
  ASSERT_TRUE(areDistanceFieldsDistancesEqual(small_df, small_df2));

  PropagationDistanceField df(PERF_WIDTH, PERF_HEIGHT, PERF_DEPTH, PERF_RESOLUTION, PERF_ORIGIN_X, PERF_ORIGIN_Y,
                              PERF_ORIGIN_Z, PERF_MAX_DIST, false);

  shapes::Sphere sphere(.5);

  Eigen::Isometry3d p = Eigen::Translation3d(0.5, 0.5, 0.5) * Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);

  df.addShapeToField(&sphere, p);

  std::ofstream f("test_big.df", std::ios::out);

  df.writeToStream(f);
  f.close();

  std::ifstream i("test_big.df", std::ios::in);
  PropagationDistanceField df2(i, PERF_MAX_DIST, false);
  EXPECT_TRUE(areDistanceFieldsDistancesEqual(df, df2));

  // we shouldn't segfault if we start with an old ifstream
  PropagationDistanceField dfx(i, PERF_MAX_DIST, false);

  std::ifstream i2("test_big.df", std::ios::in);
  ros::WallTime wt = ros::WallTime::now();
  PropagationDistanceField df3(i2, PERF_MAX_DIST + .02, false);
  std::cout << "Reconstruction for big file took " << (ros::WallTime::now() - wt).toSec() << std::endl;
  EXPECT_FALSE(areDistanceFieldsDistancesEqual(df, df3));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
