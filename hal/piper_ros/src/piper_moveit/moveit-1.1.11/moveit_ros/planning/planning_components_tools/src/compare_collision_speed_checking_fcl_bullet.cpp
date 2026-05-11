/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Jens Petit
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Jens Petit */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <geometric_shapes/shape_operations.h>
#include <random_numbers/random_numbers.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/utils/robot_model_test_utils.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

/** \brief Factor to compute the maximum number of trials random clutter generation. */
static const int MAX_SEARCH_FACTOR_CLUTTER = 3;

/** \brief Factor to compute the maximum number of trials for random state generation. */
static const int MAX_SEARCH_FACTOR_STATES = 30;

/** \brief Defines a random robot state. */
enum class RobotStateSelector
{
  IN_COLLISION,
  NOT_IN_COLLISION,
  RANDOM,
};

/** \brief Enumerates the available collision detectors. */
enum class CollisionDetector
{
  FCL,
  BULLET,
};

/** \brief Enumerates the different types of collision objects. */
enum class CollisionObjectType
{
  MESH,
  BOX,
};

/** \brief Clutters the world of the planning scene with random objects in a certain area around the origin. All added
 *  objects are not in collision with the robot.
 *
 *   \param planning_scene The planning scene
 *   \param num_objects The number of objects to be cluttered
 *   \param CollisionObjectType Type of object to clutter (mesh or box) */
void clutterWorld(const planning_scene::PlanningScenePtr& planning_scene, const size_t num_objects,
                  CollisionObjectType type)
{
  ROS_INFO("Cluttering scene...");

  random_numbers::RandomNumberGenerator num_generator = random_numbers::RandomNumberGenerator(123);

  // allow all robot links to be in collision for world check
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      planning_scene->getRobotModel()->getLinkModelNames(), true) };

  // set the robot state to home position
  moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  collision_detection::CollisionRequest req;
  current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  current_state.update();

  // load panda link5 as world collision object
  std::string name;
  shapes::ShapeConstPtr shape;
  std::string kinect = "package://moveit_resources_panda_description/meshes/collision/link5.stl";

  Eigen::Quaterniond quat;
  Eigen::Isometry3d pos{ Eigen::Isometry3d::Identity() };

  size_t added_objects{ 0 };
  size_t i{ 0 };
  // create random objects until as many added as desired or quit if too many attempts
  while (added_objects < num_objects && i < num_objects * MAX_SEARCH_FACTOR_CLUTTER)
  {
    // add with random size and random position
    pos.translation().x() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().y() = num_generator.uniformReal(-1.0, 1.0);
    pos.translation().z() = num_generator.uniformReal(0.0, 1.0);

    quat.x() = num_generator.uniformReal(-1.0, 1.0);
    quat.y() = num_generator.uniformReal(-1.0, 1.0);
    quat.z() = num_generator.uniformReal(-1.0, 1.0);
    quat.w() = num_generator.uniformReal(-1.0, 1.0);
    quat.normalize();
    pos.rotate(quat);

    switch (type)
    {
      case CollisionObjectType::MESH:
      {
        shapes::Mesh* mesh = shapes::createMeshFromResource(kinect);
        mesh->scale(num_generator.uniformReal(0.3, 1.0));
        shape.reset(mesh);
        name = "mesh";
        break;
      }
      case CollisionObjectType::BOX:
      {
        shape =
            std::make_shared<shapes::Box>(num_generator.uniformReal(0.05, 0.2), num_generator.uniformReal(0.05, 0.2),
                                          num_generator.uniformReal(0.05, 0.2));
        name = "box";
        break;
      }
    }

    name.append(std::to_string(i));
    planning_scene->getWorldNonConst()->addToObject(name, shape, pos);

    // try if it isn't in collision if yes, ok, if no remove.
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, current_state, acm);

    if (!res.collision)
    {
      added_objects++;
    }
    else
    {
      ROS_DEBUG_STREAM("Object was in collision, remove");
      planning_scene->getWorldNonConst()->removeObject(name);
    }

    i++;
  }
  ROS_INFO_STREAM("Cluttered the planning scene with " << added_objects << " objects");
}

/** \brief Runs a collision detection benchmark and measures the time.
 *
 *   \param trials The number of repeated collision checks for each state
 *   \param scene The planning scene
 *   \param CollisionDetector The type of collision detector
 *   \param only_self Flag for only self collision check performed */
void runCollisionDetection(unsigned int trials, const planning_scene::PlanningScenePtr& scene,
                           const std::vector<moveit::core::RobotState>& states, const CollisionDetector col_detector,
                           bool only_self, bool distance = false)
{
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      scene->getRobotModel()->getLinkModelNames(), true) };

  ROS_INFO_STREAM("Starting detection using " << (col_detector == CollisionDetector::FCL ? "FCL" : "Bullet"));

  if (col_detector == CollisionDetector::FCL)
  {
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create());
  }
  else
  {
    scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
  }

  collision_detection::CollisionResult res;
  collision_detection::CollisionRequest req;

  req.distance = distance;
  // for world collision request detailed information
  if (!only_self)
  {
    req.contacts = true;
    req.max_contacts = 99;
    req.max_contacts_per_pair = 10;
    // If distance is turned on it will slow down the collision checking a lot. Try reducing the
    // number of contacts consequently.
    // req.distance = true;
  }

  ros::WallTime start = ros::WallTime::now();
  for (unsigned int i = 0; i < trials; ++i)
  {
    for (auto& state : states)
    {
      res.clear();

      if (only_self)
      {
        scene->checkSelfCollision(req, res);
      }
      else
      {
        scene->checkCollision(req, res, state);
      }
    }
  }
  double duration = (ros::WallTime::now() - start).toSec();
  ROS_INFO("Performed %lf collision checks per second", (double)trials * states.size() / duration);
  ROS_INFO_STREAM("Total number was " << trials * states.size() << " checks.");
  ROS_INFO_STREAM("We had " << states.size() << " different robot states which were "
                            << (res.collision ? "in collison " : "not in collision ") << "with " << res.contact_count);

  // color collided objects red
  for (auto& contact : res.contacts)
  {
    ROS_INFO_STREAM("Between: " << contact.first.first << " and " << contact.first.second);
    std_msgs::ColorRGBA red;
    red.a = 0.8;
    red.r = 1;
    red.g = 0;
    red.b = 0;
    scene->setObjectColor(contact.first.first, red);
    scene->setObjectColor(contact.first.second, red);
  }

  scene->setCurrentState(states.back());
}

/** \brief Samples valid states of the robot which can be in collision if desired.
 *  \param desired_states Specifier for type for desired state
 *  \param num_states Number of desired states
 *  \param scene The planning scene
 *  \param robot_states Result vector */
void findStates(const RobotStateSelector desired_states, unsigned int num_states,
                const planning_scene::PlanningScenePtr& scene, std::vector<moveit::core::RobotState>& robot_states)
{
  moveit::core::RobotState& current_state{ scene->getCurrentStateNonConst() };
  collision_detection::CollisionRequest req;

  size_t i{ 0 };
  while (robot_states.size() < num_states && i < num_states * MAX_SEARCH_FACTOR_STATES)
  {
    current_state.setToRandomPositions();
    current_state.update();
    collision_detection::CollisionResult res;
    scene->checkSelfCollision(req, res);
    ROS_INFO_STREAM("Found state " << (res.collision ? "in collision" : "not in collision"));

    switch (desired_states)
    {
      case RobotStateSelector::IN_COLLISION:
        if (res.collision)
          robot_states.push_back(current_state);
        break;
      case RobotStateSelector::NOT_IN_COLLISION:
        if (!res.collision)
          robot_states.push_back(current_state);
        break;
      case RobotStateSelector::RANDOM:
        robot_states.push_back(current_state);
        break;
    }
    i++;
  }

  if (!robot_states.empty())
  {
    scene->setCurrentState(robot_states[0]);
  }
  else
  {
    ROS_ERROR_STREAM("Did not find any correct states.");
  }
}

int main(int argc, char** argv)
{
  moveit::core::RobotModelPtr robot_model;
  ros::init(argc, argv, "compare_collision_checking_speed");
  ros::NodeHandle node_handle;

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  unsigned int trials = 10000;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration sleep_t(2.5);

  robot_model = moveit::core::loadTestingRobotModel("panda");

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene_monitor::PlanningSceneMonitor psm(planning_scene, ROBOT_DESCRIPTION);
  psm.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  psm.startSceneMonitor();
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm{ collision_detection::AllowedCollisionMatrix(
      robot_model->getLinkModelNames(), true) };
  planning_scene->checkCollision(req, res, planning_scene->getCurrentState(), acm);

  ROS_INFO("Starting...");

  if (psm.getPlanningScene())
  {
    ros::Duration(0.5).sleep();

    moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
    current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
    current_state.update();

    std::vector<moveit::core::RobotState> sampled_states;
    sampled_states.push_back(current_state);

    ROS_INFO("Starting benchmark: Robot in empty world, only self collision check");
    runCollisionDetection(trials, planning_scene, sampled_states, CollisionDetector::BULLET, true);
    runCollisionDetection(trials, planning_scene, sampled_states, CollisionDetector::FCL, true);

    planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

    clutterWorld(planning_scene, 100, CollisionObjectType::MESH);

    ROS_INFO("Starting benchmark: Robot in cluttered world, no collision with world");
    runCollisionDetection(trials, planning_scene, sampled_states, CollisionDetector::BULLET, false);
    runCollisionDetection(trials, planning_scene, sampled_states, CollisionDetector::FCL, false);

    // bring the robot into a position which collides with the world clutter
    double joint_2 = 1.5;
    current_state.setJointPositions("panda_joint2", &joint_2);
    current_state.update();

    std::vector<moveit::core::RobotState> sampled_states_2;
    sampled_states_2.push_back(current_state);

    ROS_INFO("Starting benchmark: Robot in cluttered world, in collision with world");
    runCollisionDetection(trials, planning_scene, sampled_states_2, CollisionDetector::BULLET, false);
    runCollisionDetection(trials, planning_scene, sampled_states_2, CollisionDetector::FCL, false);

    bool visualize;
    node_handle.getParam("/compare_collision_checking_speed/visualization", visualize);
    if (visualize)
    {
      // publishes the planning scene to visualize in rviz if possible
      moveit_msgs::PlanningScene planning_scene_msg;
      planning_scene->getPlanningSceneMsg(planning_scene_msg);
      planning_scene_diff_publisher.publish(planning_scene_msg);
    }
  }
  else
  {
    ROS_ERROR("Planning scene not configured");
  }

  return 0;
}
