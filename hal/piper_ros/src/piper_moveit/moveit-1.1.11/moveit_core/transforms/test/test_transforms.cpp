/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <moveit/transforms/transforms.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <gtest/gtest.h>

TEST(Transforms, Simple)
{
  moveit::core::Transforms tf("global");

  Eigen::Isometry3d t1;
  t1.setIdentity();
  t1.translation() = Eigen::Vector3d(10.0, 1.0, 0.0);
  tf.setTransform(t1, "some_frame_1");

  Eigen::Isometry3d t2(Eigen::Translation3d(10.0, 1.0, 0.0) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()));
  tf.setTransform(t2, "some_frame_2");

  Eigen::Isometry3d t3;
  t3.setIdentity();
  t3.translation() = Eigen::Vector3d(0.0, 1.0, -1.0);
  tf.setTransform(t3, "some_frame_3");

  EXPECT_TRUE(tf.isFixedFrame("some_frame_1"));
  EXPECT_FALSE(tf.isFixedFrame("base_footprint"));
  EXPECT_TRUE(tf.isFixedFrame("global"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
