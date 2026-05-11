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

/* Author: Acorn Pooley */

#include <gtest/gtest.h>
#include <moveit/collision_detection/world_diff.h>
#include <geometric_shapes/shapes.h>

TEST(WorldDiff, TrackChanges)
{
  collision_detection::WorldPtr world(new collision_detection::World);
  collision_detection::WorldDiff diff1(world);
  collision_detection::WorldDiff diff2;
  collision_detection::WorldDiff::const_iterator it;

  EXPECT_EQ(diff1.getChanges().size(), 0u);
  EXPECT_EQ(diff2.getChanges().size(), 0u);

  // Create some shapes
  shapes::ShapePtr ball(new shapes::Sphere(1.0));
  shapes::ShapePtr box(new shapes::Box(1, 2, 3));
  shapes::ShapePtr cyl(new shapes::Cylinder(4, 5));

  world->addToObject("obj1", ball, Eigen::Isometry3d::Identity());

  EXPECT_EQ(diff1.getChanges().size(), 1u);
  EXPECT_EQ(diff2.getChanges().size(), 0u);

  it = diff1.getChanges().find("obj1");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  it = diff1.getChanges().find("xyz");
  EXPECT_EQ(diff1.end(), it);

  world->addToObject("obj2", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(2u, diff1.getChanges().size());
  EXPECT_EQ(0u, diff2.getChanges().size());

  it = diff1.getChanges().find("obj2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  world->addToObject("obj2", cyl, Eigen::Isometry3d::Identity());

  EXPECT_EQ(2u, diff1.getChanges().size());
  EXPECT_EQ(0u, diff2.getChanges().size());

  it = diff1.getChanges().find("obj2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  diff2.reset(world);

  bool move_ok = world->moveShapeInObject("obj2", cyl, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)));
  EXPECT_TRUE(move_ok);

  EXPECT_EQ(2u, diff1.getChanges().size());
  EXPECT_EQ(1u, diff2.getChanges().size());

  it = diff1.getChanges().find("obj2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE |
                collision_detection::World::MOVE_SHAPE,
            it->second);

  it = diff2.getChanges().find("obj2");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::MOVE_SHAPE, it->second);
  EXPECT_EQ("obj2", it->first);

  diff1.reset(world);

  EXPECT_EQ(0u, diff1.getChanges().size());
  EXPECT_EQ(1u, diff2.getChanges().size());

  it = diff1.getChanges().find("obj2");
  EXPECT_EQ(diff1.end(), it);

  world->addToObject("obj3", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(1u, diff1.getChanges().size());
  EXPECT_EQ(2u, diff2.getChanges().size());

  world->addToObject("obj3", cyl, Eigen::Isometry3d::Identity());

  world->addToObject("obj3", ball, Eigen::Isometry3d::Identity());

  EXPECT_EQ(1u, diff1.getChanges().size());
  EXPECT_EQ(2u, diff2.getChanges().size());

  diff1.reset();

  move_ok = world->moveShapeInObject("obj3", cyl, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 2)));
  EXPECT_TRUE(move_ok);

  EXPECT_EQ(0u, diff1.getChanges().size());
  EXPECT_EQ(2u, diff2.getChanges().size());

  diff1.reset(world);

  world->removeObject("obj2");

  EXPECT_EQ(1u, diff1.getChanges().size());
  EXPECT_EQ(2u, diff2.getChanges().size());

  it = diff1.getChanges().find("obj2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);
  it = diff2.getChanges().find("obj2");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  world->removeShapeFromObject("obj3", cyl);

  it = diff1.getChanges().find("obj3");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::REMOVE_SHAPE, it->second);
  it = diff2.getChanges().find("obj3");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE |
                collision_detection::World::MOVE_SHAPE | collision_detection::World::REMOVE_SHAPE,
            it->second);

  world->removeShapeFromObject("obj3", box);

  it = diff1.getChanges().find("obj3");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::REMOVE_SHAPE, it->second);
  it = diff2.getChanges().find("obj3");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE |
                collision_detection::World::MOVE_SHAPE | collision_detection::World::REMOVE_SHAPE,
            it->second);

  move_ok = world->moveShapeInObject("obj3", ball, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 3)));
  EXPECT_TRUE(move_ok);

  it = diff1.getChanges().find("obj3");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::REMOVE_SHAPE | collision_detection::World::MOVE_SHAPE, it->second);
  it = diff2.getChanges().find("obj3");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE |
                collision_detection::World::MOVE_SHAPE | collision_detection::World::REMOVE_SHAPE,
            it->second);

  world->removeShapeFromObject("obj3", ball);

  it = diff1.getChanges().find("obj3");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);
  it = diff2.getChanges().find("obj3");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);
}

TEST(WorldDiff, SetWorld)
{
  collision_detection::WorldPtr world1(new collision_detection::World);
  collision_detection::WorldPtr world2(new collision_detection::World);
  collision_detection::WorldDiff diff1(world1);
  collision_detection::WorldDiff diff1b(world1);
  collision_detection::WorldDiff diff2(world2);
  collision_detection::WorldDiff::const_iterator it;

  shapes::ShapePtr ball(new shapes::Sphere(1.0));
  shapes::ShapePtr box(new shapes::Box(1, 2, 3));
  shapes::ShapePtr cyl(new shapes::Cylinder(4, 5));

  world1->addToObject("objA1", ball, Eigen::Isometry3d::Identity());

  world1->addToObject("objA2", ball, Eigen::Isometry3d::Identity());

  world1->addToObject("objA3", ball, Eigen::Isometry3d::Identity());

  world2->addToObject("objB1", box, Eigen::Isometry3d::Identity());

  world2->addToObject("objB2", box, Eigen::Isometry3d::Identity());

  world2->addToObject("objB3", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(3u, diff1.getChanges().size());
  EXPECT_EQ(3u, diff1b.getChanges().size());
  EXPECT_EQ(3u, diff2.getChanges().size());

  diff1b.clearChanges();

  EXPECT_EQ(3u, diff1.getChanges().size());
  EXPECT_EQ(0u, diff1b.getChanges().size());
  EXPECT_EQ(3u, diff2.getChanges().size());

  diff1.setWorld(world2);

  EXPECT_EQ(6u, diff1.getChanges().size());
  EXPECT_EQ(0u, diff1b.getChanges().size());
  EXPECT_EQ(3u, diff2.getChanges().size());

  it = diff1.getChanges().find("objA1");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  it = diff1.getChanges().find("objA2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  it = diff1.getChanges().find("objA2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  it = diff1.getChanges().find("objB1");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  it = diff1.getChanges().find("objB2");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  it = diff1.getChanges().find("objB3");
  EXPECT_NE(diff1.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  diff1b.setWorld(world2);

  EXPECT_EQ(6u, diff1.getChanges().size());
  EXPECT_EQ(6u, diff1b.getChanges().size());
  EXPECT_EQ(3u, diff2.getChanges().size());

  it = diff1b.getChanges().find("objA1");
  EXPECT_NE(diff1b.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  it = diff1b.getChanges().find("objA2");
  EXPECT_NE(diff1b.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  it = diff1b.getChanges().find("objA2");
  EXPECT_NE(diff1b.end(), it);
  EXPECT_EQ(collision_detection::World::DESTROY, it->second);

  it = diff1b.getChanges().find("objB1");
  EXPECT_NE(diff1b.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  it = diff1b.getChanges().find("objB2");
  EXPECT_NE(diff1b.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  it = diff1b.getChanges().find("objB3");
  EXPECT_NE(diff1b.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE, it->second);

  world1->addToObject("objC", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(6u, diff1.getChanges().size());
  EXPECT_EQ(6u, diff1b.getChanges().size());
  EXPECT_EQ(3u, diff2.getChanges().size());

  world2->addToObject("objC", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(7u, diff1.getChanges().size());
  EXPECT_EQ(7u, diff1b.getChanges().size());
  EXPECT_EQ(4u, diff2.getChanges().size());

  diff2.setWorld(world1);

  EXPECT_EQ(7u, diff1.getChanges().size());
  EXPECT_EQ(7u, diff1b.getChanges().size());
  EXPECT_EQ(7u, diff2.getChanges().size());

  it = diff2.getChanges().find("objC");
  EXPECT_NE(diff2.end(), it);
  EXPECT_EQ(collision_detection::World::CREATE | collision_detection::World::ADD_SHAPE |
                collision_detection::World::DESTROY,
            it->second);

  world1->addToObject("objD", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(7u, diff1.getChanges().size());
  EXPECT_EQ(7u, diff1b.getChanges().size());
  EXPECT_EQ(8u, diff2.getChanges().size());

  world2->addToObject("objE", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(8u, diff1.getChanges().size());
  EXPECT_EQ(8u, diff1b.getChanges().size());
  EXPECT_EQ(8u, diff2.getChanges().size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
