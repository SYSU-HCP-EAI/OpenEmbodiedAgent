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
#include <moveit/collision_detection/world.h>
#include <geometric_shapes/shapes.h>
#include <functional>

using namespace collision_detection;

TEST(World, AddRemoveShape)
{
  World world;

  // Create some shapes
  shapes::ShapePtr ball(new shapes::Sphere(1.0));
  shapes::ShapePtr box(new shapes::Box(1, 2, 3));
  shapes::ShapePtr cyl(new shapes::Cylinder(4, 5));

  EXPECT_EQ(1, ball.use_count());

  EXPECT_FALSE(world.hasObject("ball"));

  // Add ball object
  world.addToObject("ball", ball, Eigen::Isometry3d::Identity());

  EXPECT_EQ(2, ball.use_count());
  EXPECT_TRUE(world.hasObject("ball"));

  bool move_ok = world.moveShapeInObject("ball", ball, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 9)));
  EXPECT_TRUE(move_ok);

  EXPECT_EQ(2, ball.use_count());
  EXPECT_TRUE(world.hasObject("ball"));

  bool rm_nonexistant = world.removeShapeFromObject("xyz", ball);
  EXPECT_FALSE(rm_nonexistant);

  bool rm_wrong_shape = world.removeShapeFromObject("ball", box);
  EXPECT_FALSE(rm_wrong_shape);

  EXPECT_EQ(2, ball.use_count());
  EXPECT_EQ(1, box.use_count());

  // remove ball object
  bool rm_ball = world.removeShapeFromObject("ball", ball);
  EXPECT_TRUE(rm_ball);

  EXPECT_EQ(1, ball.use_count());
  EXPECT_FALSE(world.hasObject("ball"));

  // add ball again
  world.addToObject("ball", ball, Eigen::Isometry3d::Identity());

  EXPECT_EQ(2, ball.use_count());
  EXPECT_TRUE(world.hasObject("ball"));

  EXPECT_FALSE(world.hasObject("mix1"));

  {
    std::vector<shapes::ShapeConstPtr> shapes;
    EigenSTL::vector_Isometry3d poses;

    shapes.push_back(box);
    shapes.push_back(cyl);
    shapes.push_back(ball);

    poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)));
    poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(0, 0, 2)));
    poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(0, 0, 3)));

    EXPECT_FALSE(world.hasObject("mix1"));

    // add mix1 object
    world.addToObject("mix1", shapes, poses);
  }

  EXPECT_TRUE(world.hasObject("mix1"));

  EXPECT_EQ(2, box.use_count());
  EXPECT_EQ(2, cyl.use_count());
  EXPECT_EQ(3, ball.use_count());

  // add ball2
  world.addToObject("ball2", ball, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 4)));

  EXPECT_EQ(2, box.use_count());
  EXPECT_EQ(2, cyl.use_count());
  EXPECT_EQ(4, ball.use_count());

  bool rm_cyl = world.removeShapeFromObject("mix1", cyl);
  EXPECT_TRUE(rm_cyl);

  EXPECT_EQ(2, box.use_count());
  EXPECT_EQ(1, cyl.use_count());
  EXPECT_EQ(4, ball.use_count());

  rm_cyl = world.removeShapeFromObject("mix1", cyl);
  EXPECT_FALSE(rm_cyl);

  EXPECT_EQ(2, box.use_count());
  EXPECT_EQ(1, cyl.use_count());
  EXPECT_EQ(4, ball.use_count());

  EXPECT_TRUE(world.hasObject("mix1"));

  EXPECT_EQ(3u, world.size());

  {
    World::ObjectConstPtr obj = world.getObject("mix1");
    EXPECT_EQ(2, obj.use_count());

    ASSERT_EQ(2u, obj->shapes_.size());
    ASSERT_EQ(2u, obj->shape_poses_.size());

    // check translation.z of pose
    EXPECT_EQ(1.0, obj->shape_poses_[0](2, 3));
    EXPECT_EQ(3.0, obj->shape_poses_[1](2, 3));

    move_ok = world.moveShapeInObject("mix1", ball, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 5)));
    EXPECT_TRUE(move_ok);

    World::ObjectConstPtr obj2 = world.getObject("mix1");
    EXPECT_EQ(2, obj2.use_count());
    EXPECT_EQ(1, obj.use_count());

    EXPECT_EQ(1.0, obj2->shape_poses_[0](2, 3));
    EXPECT_EQ(5.0, obj2->shape_poses_[1](2, 3));

    EXPECT_EQ(1.0, obj->shape_poses_[0](2, 3));
    EXPECT_EQ(3.0, obj->shape_poses_[1](2, 3));

    // moving object causes copy, thus extra references to shapes in obj
    EXPECT_EQ(3, box.use_count());
    EXPECT_EQ(1, cyl.use_count());
    EXPECT_EQ(5, ball.use_count());

    world.removeObject("mix1");

    EXPECT_EQ(2u, world.size());

    // no change since obj2 still holds a ref
    EXPECT_EQ(3, box.use_count());
    EXPECT_EQ(1, cyl.use_count());
    EXPECT_EQ(5, ball.use_count());

    EXPECT_FALSE(world.hasObject("mix1"));
    EXPECT_TRUE(world.hasObject("ball2"));

    // ask for nonexistant object
    World::ObjectConstPtr obj3 = world.getObject("abc");
    EXPECT_FALSE(obj3);
  }

  EXPECT_EQ(1, box.use_count());
  EXPECT_EQ(1, cyl.use_count());
  EXPECT_EQ(3, ball.use_count());

  EXPECT_EQ(2u, world.size());

  world.clearObjects();

  EXPECT_EQ(1, box.use_count());
  EXPECT_EQ(1, cyl.use_count());
  EXPECT_EQ(1, ball.use_count());

  EXPECT_FALSE(world.hasObject("mix1"));
  EXPECT_FALSE(world.hasObject("ball"));
  EXPECT_FALSE(world.hasObject("ball2"));

  EXPECT_EQ(0u, world.size());
}

/* structure to hold copy of callback args */
struct TestAction
{
  World::Object obj_;
  World::Action action_;
  int cnt_;
  TestAction() : obj_(""), cnt_(0)
  {
    reset();
  }
  void reset()
  {
    obj_.id_ = "";
    obj_.shapes_.clear();
    obj_.shape_poses_.clear();
    action_ = 0x7f;
  }
};

/* notification callback */
static void TrackChangesNotify(TestAction& ta, const World::ObjectConstPtr& obj, World::Action action)
{
  ta.obj_ = *obj;
  ta.action_ = action;
  ta.cnt_++;
}

TEST(World, TrackChanges)
{
  World world;

  TestAction ta;
  World::ObserverHandle observer_ta;
  observer_ta = world.addObserver([&ta](const World::ObjectConstPtr& object, World::Action action) {
    return TrackChangesNotify(ta, object, action);
  });

  // Create some shapes
  shapes::ShapePtr ball(new shapes::Sphere(1.0));
  shapes::ShapePtr box(new shapes::Box(1, 2, 3));
  shapes::ShapePtr cyl(new shapes::Cylinder(4, 5));

  world.addToObject("obj1", ball, Eigen::Isometry3d::Identity());

  EXPECT_EQ(1, ta.cnt_);
  EXPECT_EQ("obj1", ta.obj_.id_);
  EXPECT_EQ(World::CREATE | World::ADD_SHAPE, ta.action_);
  ta.reset();

  bool move_ok = world.moveShapeInObject("obj1", ball, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)));
  EXPECT_TRUE(move_ok);

  EXPECT_EQ(2, ta.cnt_);
  EXPECT_EQ("obj1", ta.obj_.id_);
  EXPECT_EQ(World::MOVE_SHAPE, ta.action_);
  ta.reset();

  world.addToObject("obj1", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(3, ta.cnt_);
  EXPECT_EQ("obj1", ta.obj_.id_);
  EXPECT_EQ(World::ADD_SHAPE, ta.action_);
  ta.reset();

  TestAction ta2;
  World::ObserverHandle observer_ta2;
  observer_ta2 = world.addObserver([&ta2](const World::ObjectConstPtr& object, World::Action action) {
    return TrackChangesNotify(ta2, object, action);
  });

  world.addToObject("obj2", cyl, Eigen::Isometry3d::Identity());

  EXPECT_EQ(4, ta.cnt_);
  EXPECT_EQ("obj2", ta.obj_.id_);
  EXPECT_EQ(World::CREATE | World::ADD_SHAPE, ta.action_);
  ta.reset();
  EXPECT_EQ(1, ta2.cnt_);
  EXPECT_EQ("obj2", ta2.obj_.id_);
  EXPECT_EQ(World::CREATE | World::ADD_SHAPE, ta2.action_);
  ta2.reset();

  world.addToObject("obj3", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(5, ta.cnt_);
  EXPECT_EQ("obj3", ta.obj_.id_);
  EXPECT_EQ(World::CREATE | World::ADD_SHAPE, ta.action_);
  ta.reset();
  EXPECT_EQ(2, ta2.cnt_);
  EXPECT_EQ("obj3", ta2.obj_.id_);
  EXPECT_EQ(World::CREATE | World::ADD_SHAPE, ta2.action_);
  ta2.reset();

  // remove nonexistent obj
  bool rm_bad = world.removeShapeFromObject("xyz", ball);
  EXPECT_FALSE(rm_bad);
  EXPECT_EQ(5, ta.cnt_);
  EXPECT_EQ(2, ta2.cnt_);

  // remove wrong shape
  rm_bad = world.removeShapeFromObject("obj2", ball);
  EXPECT_FALSE(rm_bad);
  EXPECT_EQ(5, ta.cnt_);
  EXPECT_EQ(2, ta2.cnt_);

  TestAction ta3;
  World::ObserverHandle observer_ta3;
  observer_ta3 = world.addObserver([&ta3](const World::ObjectConstPtr& object, World::Action action) {
    return TrackChangesNotify(ta3, object, action);
  });

  bool rm_good = world.removeShapeFromObject("obj2", cyl);
  EXPECT_TRUE(rm_good);

  EXPECT_EQ(6, ta.cnt_);
  EXPECT_EQ("obj2", ta.obj_.id_);
  EXPECT_EQ(World::DESTROY, ta.action_);
  ta.reset();
  EXPECT_EQ(3, ta2.cnt_);
  EXPECT_EQ("obj2", ta2.obj_.id_);
  EXPECT_EQ(World::DESTROY, ta2.action_);
  ta2.reset();
  EXPECT_EQ(1, ta3.cnt_);
  EXPECT_EQ("obj2", ta3.obj_.id_);
  EXPECT_EQ(World::DESTROY, ta3.action_);
  ta3.reset();

  world.removeObserver(observer_ta2);

  rm_good = world.removeShapeFromObject("obj1", ball);
  EXPECT_TRUE(rm_good);

  EXPECT_EQ(7, ta.cnt_);
  EXPECT_EQ("obj1", ta.obj_.id_);
  EXPECT_EQ(World::REMOVE_SHAPE, ta.action_);
  ta.reset();
  EXPECT_EQ(3, ta2.cnt_);

  EXPECT_EQ(2, ta3.cnt_);
  EXPECT_EQ("obj1", ta3.obj_.id_);
  EXPECT_EQ(World::REMOVE_SHAPE, ta3.action_);
  ta3.reset();

  // remove all 2 objects (should make 2 DESTROY callbacks per ta)
  world.clearObjects();

  EXPECT_EQ(9, ta.cnt_);
  EXPECT_EQ(World::DESTROY, ta.action_);
  ta.reset();
  EXPECT_EQ(3, ta2.cnt_);

  EXPECT_EQ(4, ta3.cnt_);
  EXPECT_EQ(World::DESTROY, ta3.action_);
  ta3.reset();

  world.removeObserver(observer_ta);
  world.removeObserver(observer_ta3);

  EXPECT_EQ(9, ta.cnt_);
  EXPECT_EQ(3, ta2.cnt_);
  EXPECT_EQ(4, ta3.cnt_);

  world.addToObject("obj4", box, Eigen::Isometry3d::Identity());

  EXPECT_EQ(9, ta.cnt_);
  EXPECT_EQ(3, ta2.cnt_);
  EXPECT_EQ(4, ta3.cnt_);
}

TEST(World, ObjectPoseAndSubframes)
{
  World world;

  TestAction ta;
  World::ObserverHandle observer_ta;
  observer_ta = world.addObserver([&ta](const World::ObjectConstPtr& object, World::Action action) {
    return TrackChangesNotify(ta, object, action);
  });

  // Create shapes
  shapes::ShapePtr ball(new shapes::Sphere(1.0));
  shapes::ShapePtr box(new shapes::Box(1, 1, 1));
  shapes::ShapePtr cyl(new shapes::Cylinder(0.5, 3));  // radius, length

  // Confirm that setting object pose creates an object
  world.setObjectPose("mix1", Eigen::Isometry3d::Identity());

  EXPECT_EQ(1, ta.cnt_);
  EXPECT_EQ("mix1", ta.obj_.id_);
  EXPECT_EQ(World::CREATE, ta.action_);

  // Move multi-shape objects, use object pose, use subframes
  world.addToObject("mix1", box, Eigen::Isometry3d::Identity());
  world.addToObject("mix1", cyl, Eigen::Isometry3d(Eigen::Translation3d(0, 0, 2)));

  moveit::core::FixedTransformsMap subframes;
  subframes["frame1"] = Eigen::Isometry3d(Eigen::Translation3d(0, 0, 2));
  subframes["frame2"] = Eigen::Isometry3d(Eigen::Translation3d(0, 1, 0));
  world.setSubframesOfObject("mix1", subframes);

  // Check subframes and shape poses
  bool found_ok, found_bad;
  Eigen::Isometry3d pose = world.getTransform("mix1/frame1", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(2.0, pose(2, 3));  // check translation.z

  pose = world.getTransform("mix1/frame2", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(1.0, pose(1, 3));  // check translation.y
  EXPECT_EQ(0.0, pose(2, 3));  // z

  pose = world.getTransform("mix1/frame3", found_bad);
  EXPECT_FALSE(found_bad);

  // Set new object pose, check that all shapes and subframes moved
  world.setObjectPose("mix1", Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)));

  pose = world.getTransform("mix1/frame1", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(3.0, pose(2, 3));  // z

  pose = world.getTransform("mix1/frame2", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(1.0, pose(1, 3));  // y
  EXPECT_EQ(1.0, pose(2, 3));  // z

  pose = world.getGlobalShapeTransform("mix1", 0);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(1.0, pose(2, 3));  // z

  World::ObjectConstPtr obj = world.getObject("mix1");
  EXPECT_EQ(0.0, obj->shape_poses_[0](2, 3));  // Internal shape poses do *not* change
  EXPECT_EQ(2.0, obj->shape_poses_[1](2, 3));

  // Shift object, check that object pose changed
  world.moveObject("mix1", Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)));

  pose = world.getTransform("mix1", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(2.0, pose(2, 3));  // z

  pose = world.getTransform("mix1/frame1", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(4.0, pose(2, 3));  // z

  EXPECT_EQ(0.0, obj->shape_poses_[0](2, 3));  // Internal shape poses should still be constant
  EXPECT_EQ(2.0, obj->shape_poses_[1](2, 3));

  // Move object absolute, check object pose
  world.setObjectPose("mix1", Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1)));
  pose = world.getTransform("mix1", found_ok);
  EXPECT_TRUE(found_ok);
  EXPECT_EQ(1.0, pose(2, 3));  // z
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
