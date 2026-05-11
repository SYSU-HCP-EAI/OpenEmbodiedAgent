/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2019, PickNik Consulting
 *  All rights reserved.
 *
 *********************************************************************/

/* Author: Omid Heidari
   Desc:   Test the trajectory planned by trajopt
 */

// C++
#include <iostream>
#include <string>
#include <cmath>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

#include <trajopt/common.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <trajopt_interface/problem_description.h>
#include <trajopt_interface/kinematic_terms.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/MotionPlanResponse.h>

class TrajectoryTest : public ::testing::Test
{
public:
  TrajectoryTest()
  {
  }

protected:
  void SetUp() override
  {
    node_handle_ = ros::NodeHandle("~");
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    bool robot_model_ok_ = static_cast<bool>(robot_model_);
    if (!robot_model_ok_)
      ROS_ERROR_STREAM_NAMED("trajectory_test", "robot model is not loaded correctly");
  }

protected:
  moveit::core::RobotModelPtr robot_model_;
  std::vector<std::string> group_joint_names_;
  const std::string PLANNING_GROUP = "panda_arm";
  const double GOAL_TOLERANCE = 0.1;
  ros::NodeHandle node_handle_;
};  // class TrajectoryTest

TEST_F(TrajectoryTest, concatVectorValidation)
{
  std::vector<double> vec_a = { 1, 2, 3, 4, 5 };
  std::vector<double> vec_b = { 6, 7, 8, 9, 10 };
  std::vector<double> vec_c = trajopt_interface::concatVector(vec_a, vec_b);
  EXPECT_EQ(vec_c.size(), vec_a.size() + vec_b.size());

  // Check if the output of concatVector is correct.
  // Loop over the output and the input vectors to see if they match
  std::size_t length_ab = vec_a.size() + vec_b.size();
  for (std::size_t index = 0; index < length_ab; ++index)
  {
    if (index < vec_a.size())
    {
      EXPECT_EQ(vec_c[index], vec_a[index]);
    }
    else
    {
      EXPECT_EQ(vec_c[index], vec_b[index - vec_a.size()]);
    }
  }
}

TEST_F(TrajectoryTest, goalTolerance)
{
  const std::string NODE_NAME = "trajectory_test";

  // Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
  moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(robot_model_));
  current_state->setToDefaultValues();

  const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);
  EXPECT_NE(joint_model_group, nullptr);
  const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
  EXPECT_EQ(joint_names.size(), 7);

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model_));

  // Create response and request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  // Set start state
  // ======================================================================================
  std::vector<double> start_joint_values = { 0.4, 0.3, 0.5, -0.55, 0.88, 1.0, -0.075 };
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model_));
  start_state->setJointGroupPositions(joint_model_group, start_joint_values);
  start_state->update();

  req.start_state.joint_state.name = joint_names;
  req.start_state.joint_state.position = start_joint_values;
  req.goal_constraints.clear();
  req.group_name = PLANNING_GROUP;

  // Set the goal state and joints tolerance
  // ========================================================================================
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(robot_model_));
  std::vector<double> goal_joint_values = { 0.8, 0.7, 1, -1.3, 1.9, 2.2, -0.1 };
  goal_state->setJointGroupPositions(joint_model_group, goal_joint_values);
  goal_state->update();
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(*goal_state, joint_model_group);
  req.goal_constraints.push_back(joint_goal);
  req.goal_constraints[0].name = "goal_pos";
  // Set joint tolerance
  std::vector<moveit_msgs::JointConstraint> goal_joint_constraint = req.goal_constraints[0].joint_constraints;
  for (std::size_t x = 0; x < goal_joint_constraint.size(); ++x)
  {
    req.goal_constraints[0].joint_constraints[x].tolerance_above = 0.001;
    req.goal_constraints[0].joint_constraints[x].tolerance_below = 0.001;
  }

  // Load planner
  // ======================================================================================
  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;

  std::string planner_plugin_name = "trajopt_interface/TrajOptPlanner";
  node_handle_.setParam("planning_plugin", planner_plugin_name);

  // Make sure the planner plugin is loaded
  EXPECT_TRUE(node_handle_.getParam("planning_plugin", planner_plugin_name));
  try
  {
    planner_plugin_loader = std::make_shared<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
        "moveit_core", "planning_interface::PlannerManager");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM_NAMED(NODE_NAME, "Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model_, node_handle_.getNamespace()))
      ROS_FATAL_STREAM_NAMED(NODE_NAME, "Could not initialize planner instance");
    ROS_INFO_STREAM_NAMED(NODE_NAME, "Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM_NAMED(NODE_NAME, "Exception while loading planner '" << planner_plugin_name << "': " << ex.what()
                                                                          << std::endl
                                                                          << "Available plugins: " << ss.str());
  }

  // Creat planning context
  // ========================================================================================
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  context->solve(res);
  EXPECT_EQ(res.error_code_.val, res.error_code_.SUCCESS);

  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  // Check the difference between the last step in the solution and the goal
  // ========================================================================================
  std::vector<double> joints_values_last_step = response.trajectory.joint_trajectory.points.back().positions;

  for (std::size_t joint_index = 0; joint_index < joints_values_last_step.size(); ++joint_index)
  {
    double goal_error =
        abs(joints_values_last_step[joint_index] - req.goal_constraints[0].joint_constraints[joint_index].position);
    std::cerr << "goal_error: " << goal_error << std::endl;
    EXPECT_LT(goal_error, GOAL_TOLERANCE);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajectory_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  ros::shutdown();

  return result;
}
