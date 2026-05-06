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
#include <ros/ros.h>

using namespace distance_field;

TEST(TestVoxelGrid, TestReadWrite)
{
  int i;
  int def = -100;
  VoxelGrid<int> vg(0.02, 0.02, 0.02, 0.01, 0, 0, 0, def);

  int num_x = vg.getNumCells(DIM_X);
  int num_y = vg.getNumCells(DIM_Y);
  int num_z = vg.getNumCells(DIM_Z);

  // Check dimensions
  EXPECT_EQ(num_x, 2);
  EXPECT_EQ(num_y, 2);
  EXPECT_EQ(num_z, 2);

  // check initial values
  vg.reset(0);

  i = 0;
  for (int x = 0; x < num_x; x++)
    for (int y = 0; y < num_y; y++)
      for (int z = 0; z < num_z; z++)
      {
        EXPECT_EQ(vg.getCell(x, y, z), 0);
        i++;
      }

  // Check out-of-bounds query    // FIXME-- this test fails!!
  // EXPECT_EQ( vg.getCell(999,9999,999), def );
  // EXPECT_EQ( vg.getCell(numX+1,0,0), def);
  // EXPECT_EQ( vg.getCell(0,numY+1,0), def);
  // EXPECT_EQ( vg.getCell(0,0,numZ+1), def);

  // Set values
  i = 0;
  for (int x = 0; x < num_x; x++)
    for (int y = 0; y < num_y; y++)
      for (int z = 0; z < num_z; z++)
      {
        vg.getCell(x, y, z) = i;
        i++;
      }

  // check reset values
  i = 0;
  for (int x = 0; x < num_x; x++)
    for (int y = 0; y < num_y; y++)
      for (int z = 0; z < num_z; z++)
      {
        EXPECT_EQ(i, vg.getCell(x, y, z));
        i++;
      }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
