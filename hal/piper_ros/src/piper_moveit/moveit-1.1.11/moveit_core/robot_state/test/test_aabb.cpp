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

/* Author: Martin Pecka */

#include <moveit/robot_model/aabb.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/utils/robot_model_test_utils.h>

// To visualize bbox of the PR2, set this to 1.
#ifndef VISUALIZE_PR2_RVIZ
#define VISUALIZE_PR2_RVIZ 0
#endif

#if VISUALIZE_PR2_RVIZ
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shape_operations.h>
#endif

class TestAABB : public testing::Test
{
protected:
  void SetUp() override{};

  moveit::core::RobotState loadModel(const std::string& robot_name)
  {
    moveit::core::RobotModelPtr model = moveit::core::loadTestingRobotModel(robot_name);
    return loadModel(model);
  }

  moveit::core::RobotState loadModel(const moveit::core::RobotModelPtr& model)
  {
    moveit::core::RobotState robot_state = moveit::core::RobotState(model);
    robot_state.setToDefaultValues();
    robot_state.update(true);

    return robot_state;
  }

  void TearDown() override
  {
  }
};

TEST_F(TestAABB, TestPR2)
{
  // Contains a link with mesh geometry that is not centered

  moveit::core::RobotState pr2_state = this->loadModel("pr2");

  const Eigen::Vector3d& extents_base_footprint = pr2_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin();
  // values taken from moveit_resources/pr2_description/urdf/robot.xml
  EXPECT_NEAR(extents_base_footprint[0], 0.001, 1e-4);
  EXPECT_NEAR(extents_base_footprint[1], 0.001, 1e-4);
  EXPECT_NEAR(extents_base_footprint[2], 0.001, 1e-4);

  const Eigen::Vector3d& offset_base_footprint =
      pr2_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset();
  EXPECT_NEAR(offset_base_footprint[0], 0.0, 1e-4);
  EXPECT_NEAR(offset_base_footprint[1], 0.0, 1e-4);
  EXPECT_NEAR(offset_base_footprint[2], 0.071, 1e-4);

  const Eigen::Vector3d& extents_base_link = pr2_state.getLinkModel("base_link")->getShapeExtentsAtOrigin();
  // values computed from moveit_resources/pr2_description/urdf/meshes/base_v0/base_L.stl in e.g. Meshlab
  EXPECT_NEAR(extents_base_link[0], 0.668242, 1e-4);
  EXPECT_NEAR(extents_base_link[1], 0.668242, 1e-4);
  EXPECT_NEAR(extents_base_link[2], 0.656175, 1e-4);

  const Eigen::Vector3d& offset_base_link = pr2_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset();
  EXPECT_NEAR(offset_base_link[0], 0.0, 1e-4);
  EXPECT_NEAR(offset_base_link[1], 0.0, 1e-4);
  EXPECT_NEAR(offset_base_link[2], 0.656175 / 2, 1e-4);  // The 3D mesh isn't centered, but is whole above z axis

  std::vector<double> pr2_aabb;
  pr2_state.computeAABB(pr2_aabb);
  ASSERT_EQ(pr2_aabb.size(), 6u);

  EXPECT_NEAR(pr2_aabb[0], -0.3376, 1e-4);
  EXPECT_NEAR(pr2_aabb[1], 0.6499, 1e-4);
  EXPECT_NEAR(pr2_aabb[2], -0.6682 / 2, 1e-4);
  EXPECT_NEAR(pr2_aabb[3], 0.6682 / 2, 1e-4);
  EXPECT_NEAR(pr2_aabb[4], 0.0044, 1e-4);
  EXPECT_NEAR(pr2_aabb[5], 1.6328, 1e-4);

  // Test a specific link known to have some global rotation in the default pose

  const moveit::core::LinkModel* link = pr2_state.getLinkModel("l_forearm_link");
  Eigen::Isometry3d transform = pr2_state.getGlobalLinkTransform(link);  // intentional copy, we will translate
  const Eigen::Vector3d& extents = link->getShapeExtentsAtOrigin();
  transform.translate(link->getCenteredBoundingBoxOffset());
  moveit::core::AABB aabb;
  aabb.extendWithTransformedBox(transform, extents);

  EXPECT_NEAR(aabb.center()[0], 0.5394, 1e-4);
  EXPECT_NEAR(aabb.center()[1], 0.1880, 1e-4);
  EXPECT_NEAR(aabb.center()[2], 1.1665, 1e-4);
  EXPECT_NEAR(aabb.sizes()[0], 0.2209, 1e-4);
  EXPECT_NEAR(aabb.sizes()[1], 0.1201, 1e-4);
  EXPECT_NEAR(aabb.sizes()[2], 0.2901, 1e-4);

#if VISUALIZE_PR2_RVIZ
  std::cout << "Overall bounding box of PR2:" << std::endl;
  std::string dims[] = { "x", "y", "z" };
  for (std::size_t i = 0; i < 3; ++i)
  {
    double dim = pr2_aabb[2 * i + 1] - pr2_aabb[2 * i];
    double center = dim / 2;
    std::cout << dims[i] << ": size=" << dim << ", offset=" << (pr2_aabb[2 * i + 1] - center) << std::endl;
  }

  // Initialize a ROS publisher
  char* argv[0];
  int argc = 0;
  ros::init(argc, argv, "visualize_pr2");
  ros::NodeHandle nh;
  ros::Publisher pub_aabb = nh.advertise<visualization_msgs::Marker>("/visualization_aabb", 10);
  ros::Publisher pub_obb = nh.advertise<visualization_msgs::Marker>("/visualization_obb", 10);

  // Wait for the publishers to establish connections
  sleep(5);

  // Prepare the ROS message we will reuse throughout the rest of the function.
  visualization_msgs::Marker msg;
  msg.header.frame_id = pr2_state.getRobotModel()->getRootLinkName();
  msg.type = visualization_msgs::Marker::CUBE;
  msg.color.a = 0.5;
  msg.lifetime.sec = 3000;

  // Publish the AABB of the whole model
  msg.ns = "pr2";
  msg.pose.position.x = (pr2_aabb[0] + pr2_aabb[1]) / 2;
  msg.pose.position.y = (pr2_aabb[2] + pr2_aabb[3]) / 2;
  msg.pose.position.z = (pr2_aabb[4] + pr2_aabb[5]) / 2;
  msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
  msg.pose.orientation.w = 1;
  msg.scale.x = pr2_aabb[1] - pr2_aabb[0];
  msg.scale.y = pr2_aabb[3] - pr2_aabb[2];
  msg.scale.z = pr2_aabb[5] - pr2_aabb[4];
  pub_aabb.publish(msg);

  // Publish BBs for all links
  std::vector<const moveit::core::LinkModel*> links = pr2_state.getRobotModel()->getLinkModelsWithCollisionGeometry();
  for (std::size_t i = 0; i < links.size(); ++i)
  {
    Eigen::Isometry3d transform = pr2_state.getGlobalLinkTransform(links[i]);  // intentional copy, we will translate
    const Eigen::Vector3d& extents = links[i]->getShapeExtentsAtOrigin();
    transform.translate(links[i]->getCenteredBoundingBoxOffset());
    moveit::core::AABB aabb;
    aabb.extendWithTransformedBox(transform, extents);

    // Publish AABB
    msg.ns = links[i]->getName();
    msg.pose.position.x = transform.translation()[0];
    msg.pose.position.y = transform.translation()[1];
    msg.pose.position.z = transform.translation()[2];
    msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    msg.color.r = 1;
    msg.color.b = 0;
    msg.scale.x = aabb.sizes()[0];
    msg.scale.y = aabb.sizes()[1];
    msg.scale.z = aabb.sizes()[2];
    pub_aabb.publish(msg);

    // Publish OBB (oriented BB)
    msg.ns += "-obb";
    msg.pose.position.x = transform.translation()[0];
    msg.pose.position.y = transform.translation()[1];
    msg.pose.position.z = transform.translation()[2];
    msg.scale.x = extents[0];
    msg.scale.y = extents[1];
    msg.scale.z = extents[2];
    msg.color.r = 0;
    msg.color.b = 1;
    Eigen::Quaterniond q(transform.linear());
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pub_obb.publish(msg);
  }

  // Publish BBs for all attached bodies
  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  pr2_state.getAttachedBodies(attached_bodies);
  for (std::vector<const moveit::core::AttachedBody*>::const_iterator it = attached_bodies.begin();
       it != attached_bodies.end(); ++it)
  {
    const EigenSTL::vector_Isometry3d& transforms = (*it)->getGlobalCollisionBodyTransforms();
    const std::vector<shapes::ShapeConstPtr>& shapes = (*it)->getShapes();
    for (std::size_t i = 0; i < transforms.size(); ++i)
    {
      Eigen::Vector3d extents = shapes::computeShapeExtents(shapes[i].get());
      moveit::core::AABB aabb;
      aabb.extendWithTransformedBox(transforms[i], extents);

      // Publish AABB
      msg.ns = (*it)->getName() + boost::lexical_cast<std::string>(i);
      msg.pose.position.x = transforms[i].translation()[0];
      msg.pose.position.y = transforms[i].translation()[1];
      msg.pose.position.z = transforms[i].translation()[2];
      msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0;
      msg.pose.orientation.w = 1;
      msg.color.r = 1;
      msg.color.b = 0;
      msg.scale.x = aabb.sizes()[0];
      msg.scale.y = aabb.sizes()[1];
      msg.scale.z = aabb.sizes()[2];
      pub_aabb.publish(msg);

      // Publish OBB (oriented BB)
      msg.ns += "-obb";
      msg.pose.position.x = transforms[i].translation()[0];
      msg.pose.position.y = transforms[i].translation()[1];
      msg.pose.position.z = transforms[i].translation()[2];
      msg.scale.x = extents[0];
      msg.scale.y = extents[1];
      msg.scale.z = extents[2];
      msg.color.r = 0;
      msg.color.b = 1;
      Eigen::Quaterniond q(transforms[i].linear());
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      pub_obb.publish(msg);
    }
  }
#endif
}

TEST_F(TestAABB, TestSimple)
{
  // Contains a link with simple geometry and an offset in the collision link
  moveit::core::RobotModelBuilder builder("simple", "base_footprint");
  geometry_msgs::Pose origin;
  tf2::toMsg(tf2::Vector3(0, 0, 0.051), origin.position);
  origin.orientation.w = 1.0;
  builder.addChain("base_footprint->base_link", "fixed", { origin });

  tf2::toMsg(tf2::Vector3(0, 0, 0), origin.position);
  builder.addCollisionMesh("base_link", "package://moveit_resources_pr2_description/urdf/meshes/base_v0/base_L.stl",
                           origin);

  tf2::toMsg(tf2::Vector3(0, 0, 0.071), origin.position);
  builder.addCollisionBox("base_footprint", { 0.001, 0.001, 0.001 }, origin);

  builder.addVirtualJoint("odom_combined", "base_footprint", "planar", "world_joint");
  builder.addGroup({}, { "world_joint" }, "base");

  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotState simple_state = loadModel(builder.build());
  std::vector<double> simple_aabb;
  simple_state.computeAABB(simple_aabb);

  ASSERT_EQ(simple_aabb.size(), 6u);
  EXPECT_NEAR(simple_aabb[0], -0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[1], 0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[2], -0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[3], 0.6682 / 2, 1e-4);
  EXPECT_NEAR(simple_aabb[4], 0.0510, 1e-4);
  EXPECT_NEAR(simple_aabb[5], 0.7071, 1e-4);
}

TEST_F(TestAABB, TestComplex)
{
  // Contains a link with simple geometry and an offset and rotation in the collision link
  moveit::core::RobotModelBuilder builder("complex", "base_footprint");
  geometry_msgs::Pose origin;
  tf2::toMsg(tf2::Vector3(0, 0, 1.0), origin.position);
  tf2::Quaternion q;
  q.setRPY(0, 0, 1.5708);
  origin.orientation = tf2::toMsg(q);
  builder.addChain("base_footprint->base_link", "fixed", { origin });
  tf2::toMsg(tf2::Vector3(5.0, 0, 1.0), origin.position);
  builder.addCollisionBox("base_link", { 1.0, 0.1, 0.1 }, origin);
  tf2::toMsg(tf2::Vector3(4.0, 0, 1.0), origin.position);
  builder.addCollisionBox("base_link", { 1.0, 0.1, 0.1 }, origin);
  tf2::toMsg(tf2::Vector3(-5.0, 0.0, -1.0), origin.position);
  q.setRPY(0, 1.5708, 0);
  origin.orientation = tf2::toMsg(q);
  builder.addCollisionBox("base_footprint", { 0.1, 1.0, 0.1 }, origin);
  builder.addVirtualJoint("odom_combined", "base_footprint", "planar", "world_joint");
  builder.addGroup({}, { "world_joint" }, "base");

  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotState complex_state = this->loadModel(builder.build());

  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin()[0], 0.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin()[1], 1.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getShapeExtentsAtOrigin()[2], 0.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset()[0], -5.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset()[1], 0.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_footprint")->getCenteredBoundingBoxOffset()[2], -1.0, 1e-4);

  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getShapeExtentsAtOrigin()[0], 1.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getShapeExtentsAtOrigin()[1], 1.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getShapeExtentsAtOrigin()[2], 0.1, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset()[0], 4.5, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset()[1], 0.0, 1e-4);
  EXPECT_NEAR(complex_state.getLinkModel("base_link")->getCenteredBoundingBoxOffset()[2], 1.0, 1e-4);

  std::vector<double> complex_aabb;
  complex_state.computeAABB(complex_aabb);

  ASSERT_EQ(complex_aabb.size(), 6u);
  EXPECT_NEAR(complex_aabb[0], -5.05, 1e-4);
  EXPECT_NEAR(complex_aabb[1], 0.5, 1e-4);
  EXPECT_NEAR(complex_aabb[2], -0.5, 1e-4);
  EXPECT_NEAR(complex_aabb[3], 5.05, 1e-4);
  EXPECT_NEAR(complex_aabb[4], -1.05, 1e-4);
  EXPECT_NEAR(complex_aabb[5], 2.05, 1e-4);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
