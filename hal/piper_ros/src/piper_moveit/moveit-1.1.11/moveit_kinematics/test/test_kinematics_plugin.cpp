/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Southwest Research Institute
 *                2018, Bielefeld University
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

/* Author: Jorge Nicho, Robert Haschke */

#include <gtest/gtest.h>
#include <memory>
#include <functional>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <xmlrpcpp/XmlRpcValue.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const double DEFAULT_SEARCH_DISCRETIZATION = 0.01f;
const double EXPECTED_SUCCESS_RATE = 0.8;
const double DEFAULT_TOLERANCE = 1e-5;

template <typename T>
inline bool getParam(const std::string& param, T& val)
{
  // first look within private namespace
  ros::NodeHandle pnh("~");
  if (pnh.getParam(param, val))
    return true;

  // then in local namespace
  ros::NodeHandle nh;
  return nh.getParam(param, val);
}

// As loading of parameters is quite slow, we share them across all tests
class SharedData
{
  friend class KinematicsTest;
  typedef pluginlib::ClassLoader<kinematics::KinematicsBase> KinematicsLoader;

  moveit::core::RobotModelPtr robot_model_;
  std::unique_ptr<KinematicsLoader> kinematics_loader_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::vector<std::string> joints_;
  std::vector<double> seed_;
  std::vector<double> consistency_limits_;
  double timeout_;
  double tolerance_;
  int num_fk_tests_;
  int num_ik_cb_tests_;
  int num_ik_tests_;
  int num_ik_multiple_tests_;
  int num_nearest_ik_tests_;

  SharedData(SharedData const&) = delete;  // this is a singleton
  SharedData()
  {
    initialize();
  }

  void initialize()
  {
    ROS_INFO_STREAM("Loading robot model from " << ros::this_node::getNamespace() << "/" << ROBOT_DESCRIPTION_PARAM);
    // load robot model
    rdf_loader::RDFLoader rdf_loader(ROBOT_DESCRIPTION_PARAM);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(rdf_loader.getURDF(), rdf_loader.getSRDF());
    ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";

    // init ClassLoader
    kinematics_loader_ = std::make_unique<KinematicsLoader>("moveit_core", "kinematics::KinematicsBase");
    ASSERT_TRUE(bool(kinematics_loader_)) << "Failed to instantiate ClassLoader";

    // load parameters
    ASSERT_TRUE(getParam("group", group_name_));
    ASSERT_TRUE(getParam("tip_link", tip_link_));
    ASSERT_TRUE(getParam("root_link", root_link_));
    ASSERT_TRUE(getParam("joint_names", joints_));
    getParam("seed", seed_);
    ASSERT_TRUE(seed_.empty() || seed_.size() == joints_.size());
    getParam("consistency_limits", consistency_limits_);
    if (!getParam("ik_timeout", timeout_) || timeout_ < 0.0)
      timeout_ = 1.0;
    if (!getParam("tolerance", tolerance_) || tolerance_ < 0.0)
      tolerance_ = DEFAULT_TOLERANCE;
    ASSERT_TRUE(consistency_limits_.empty() || consistency_limits_.size() == joints_.size());
    ASSERT_TRUE(getParam("num_fk_tests", num_fk_tests_));
    ASSERT_TRUE(getParam("num_ik_cb_tests", num_ik_cb_tests_));
    ASSERT_TRUE(getParam("num_ik_tests", num_ik_tests_));
    ASSERT_TRUE(getParam("num_ik_multiple_tests", num_ik_multiple_tests_));
    ASSERT_TRUE(getParam("num_nearest_ik_tests", num_nearest_ik_tests_));

    ASSERT_TRUE(robot_model_->hasJointModelGroup(group_name_));
    ASSERT_TRUE(robot_model_->hasLinkModel(root_link_));
    ASSERT_TRUE(robot_model_->hasLinkModel(tip_link_));
  }

public:
  auto createUniqueInstance(const std::string& name) const
  {
    return kinematics_loader_->createUniqueInstance(name);
  }

  static const SharedData& instance()
  {
    static SharedData instance;
    return instance;
  }
  static void release()
  {
    SharedData& shared = const_cast<SharedData&>(instance());
    shared.kinematics_loader_.reset();
  }
};

class KinematicsTest : public ::testing::Test
{
protected:
  void operator=(const SharedData& data)
  {
    robot_model_ = data.robot_model_;
    jmg_ = robot_model_->getJointModelGroup(data.group_name_);
    root_link_ = data.root_link_;
    tip_link_ = data.tip_link_;
    group_name_ = data.group_name_;
    joints_ = data.joints_;
    seed_ = data.seed_;
    consistency_limits_ = data.consistency_limits_;
    timeout_ = data.timeout_;
    tolerance_ = data.tolerance_;
    num_fk_tests_ = data.num_fk_tests_;
    num_ik_cb_tests_ = data.num_ik_cb_tests_;
    num_ik_tests_ = data.num_ik_tests_;
    num_ik_multiple_tests_ = data.num_ik_multiple_tests_;
    num_nearest_ik_tests_ = data.num_nearest_ik_tests_;
  }

  void SetUp() override
  {
    *this = SharedData::instance();

    std::string plugin_name;
    ASSERT_TRUE(getParam("ik_plugin_name", plugin_name));
    ROS_INFO_STREAM("Loading " << plugin_name);
    kinematics_solver_ = SharedData::instance().createUniqueInstance(plugin_name);
    ASSERT_TRUE(bool(kinematics_solver_)) << "Failed to load plugin: " << plugin_name;

    // initializing plugin
    ASSERT_TRUE(kinematics_solver_->initialize(*robot_model_, group_name_, root_link_, { tip_link_ },
                                               DEFAULT_SEARCH_DISCRETIZATION) ||
                kinematics_solver_->initialize(ROBOT_DESCRIPTION_PARAM, group_name_, root_link_, { tip_link_ },
                                               DEFAULT_SEARCH_DISCRETIZATION))
        << "Solver failed to initialize";

    jmg_ = robot_model_->getJointModelGroup(kinematics_solver_->getGroupName());
    ASSERT_TRUE(jmg_);

    // Validate chain information
    ASSERT_EQ(root_link_, kinematics_solver_->getBaseFrame());
    ASSERT_FALSE(kinematics_solver_->getTipFrames().empty());
    ASSERT_EQ(tip_link_, kinematics_solver_->getTipFrame());
    ASSERT_EQ(joints_, kinematics_solver_->getJointNames());
  }

public:
  testing::AssertionResult isNear(const char* expr1, const char* expr2, const char* /*abs_error_expr*/,
                                  const geometry_msgs::Point& val1, const geometry_msgs::Point& val2, double abs_error)
  {
    // clang-format off
    if (std::fabs(val1.x - val2.x) <= abs_error &&
        std::fabs(val1.y - val2.y) <= abs_error &&
        std::fabs(val1.z - val2.z) <= abs_error)
      return testing::AssertionSuccess();

    return testing::AssertionFailure()
        << std::setprecision(std::numeric_limits<double>::digits10 + 2)
        << "Expected: " << expr1 << " [" << val1.x << ", " << val1.y << ", " << val1.z << "]\n"
        << "Actual: " << expr2 << " [" << val2.x << ", " << val2.y << ", " << val2.z << "]";
    // clang-format on
  }
  testing::AssertionResult isNear(const char* expr1, const char* expr2, const char* /*abs_error_expr*/,
                                  const geometry_msgs::Quaternion& val1, const geometry_msgs::Quaternion& val2,
                                  double abs_error)
  {
    if ((std::fabs(val1.x - val2.x) <= abs_error && std::fabs(val1.y - val2.y) <= abs_error &&
         std::fabs(val1.z - val2.z) <= abs_error && std::fabs(val1.w - val2.w) <= abs_error) ||
        (std::fabs(val1.x + val2.x) <= abs_error && std::fabs(val1.y + val2.y) <= abs_error &&
         std::fabs(val1.z + val2.z) <= abs_error && std::fabs(val1.w + val2.w) <= abs_error))
      return testing::AssertionSuccess();

    // clang-format off
    return testing::AssertionFailure()
        << std::setprecision(std::numeric_limits<double>::digits10 + 2)
        << "Expected: " << expr1 << " [" << val1.w << ", " << val1.x << ", " << val1.y << ", " << val1.z << "]\n"
        << "Actual: " << expr2 << " [" << val2.w << ", " << val2.x << ", " << val2.y << ", " << val2.z << "]";
    // clang-format on
  }
  testing::AssertionResult expectNearHelper(const char* expr1, const char* expr2, const char* abs_error_expr,
                                            const std::vector<geometry_msgs::Pose>& val1,
                                            const std::vector<geometry_msgs::Pose>& val2, double abs_error)
  {
    if (val1.size() != val2.size())
      return testing::AssertionFailure() << "Different vector sizes"
                                         << "\nExpected: " << expr1 << " (" << val1.size() << ")"
                                         << "\nActual: " << expr2 << " (" << val2.size() << ")";

    for (size_t i = 0; i < val1.size(); ++i)
    {
      ::std::stringstream ss;
      ss << "[" << i << "].position";
      GTEST_ASSERT_(isNear((expr1 + ss.str()).c_str(), (expr2 + ss.str()).c_str(), abs_error_expr, val1[i].position,
                           val2[i].position, abs_error),
                    GTEST_NONFATAL_FAILURE_);

      ss.str("");
      ss << "[" << i << "].orientation";
      GTEST_ASSERT_(isNear((expr1 + ss.str()).c_str(), (expr2 + ss.str()).c_str(), abs_error_expr, val1[i].orientation,
                           val2[i].orientation, abs_error),
                    GTEST_NONFATAL_FAILURE_);
    }
    return testing::AssertionSuccess();
  }

  void searchIKCallback(const std::vector<double>& joint_state, moveit_msgs::MoveItErrorCodes& error_code)
  {
    std::vector<std::string> link_names = { tip_link_ };
    std::vector<geometry_msgs::Pose> poses;
    if (!kinematics_solver_->getPositionFK(link_names, joint_state, poses))
    {
      error_code.val = error_code.PLANNING_FAILED;
      return;
    }

    EXPECT_GT(poses[0].position.z, 0.0f);
    if (poses[0].position.z > 0.0)
      error_code.val = error_code.SUCCESS;
    else
      error_code.val = error_code.PLANNING_FAILED;
  }

public:
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::JointModelGroup* jmg_;
  kinematics::KinematicsBasePtr kinematics_solver_;
  random_numbers::RandomNumberGenerator rng_{ 42 };
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::vector<std::string> joints_;
  std::vector<double> seed_;
  std::vector<double> consistency_limits_;
  double timeout_;
  double tolerance_;
  unsigned int num_fk_tests_;
  unsigned int num_ik_cb_tests_;
  unsigned int num_ik_tests_;
  unsigned int num_ik_multiple_tests_;
  unsigned int num_nearest_ik_tests_;
};

#define EXPECT_NEAR_POSES(lhs, rhs, near)                                                                              \
  SCOPED_TRACE("EXPECT_NEAR_POSES(" #lhs ", " #rhs ")");                                                               \
  GTEST_ASSERT_(expectNearHelper(#lhs, #rhs, #near, lhs, rhs, near), GTEST_NONFATAL_FAILURE_);

TEST_F(KinematicsTest, getFK)
{
  std::vector<double> joints(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (unsigned int i = 0; i < num_fk_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, joints);
    std::vector<geometry_msgs::Pose> fk_poses;
    EXPECT_TRUE(kinematics_solver_->getPositionFK(tip_frames, joints, fk_poses));

    robot_state.updateLinkTransforms();
    std::vector<geometry_msgs::Pose> model_poses;
    model_poses.reserve(tip_frames.size());
    for (const auto& tip : tip_frames)
      model_poses.emplace_back(tf2::toMsg(robot_state.getGlobalLinkTransform(tip)));
    EXPECT_NEAR_POSES(model_poses, fk_poses, tolerance_);
  }
}

// perform random walk in joint-space, reaching poses via IK
TEST_F(KinematicsTest, randomWalkIK)
{
  std::vector<double> seed, goal, solution;
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  if (!seed_.empty())
    robot_state.setJointGroupPositions(jmg_, seed_);

  bool publish_trajectory = false;
  getParam<bool>("publish_trajectory", publish_trajectory);
  moveit_msgs::DisplayTrajectory msg;
  msg.model_id = robot_model_->getName();
  moveit::core::robotStateToRobotStateMsg(robot_state, msg.trajectory_start);
  msg.trajectory.resize(1);
  robot_trajectory::RobotTrajectory traj(robot_model_, jmg_);

  unsigned int failures = 0;
  static constexpr double NEAR_JOINT = 0.1;
  const std::vector<double> consistency_limits(jmg_->getVariableCount(), 1.05 * NEAR_JOINT);
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    // remember previous pose
    robot_state.copyJointGroupPositions(jmg_, seed);
    // sample a new pose nearby
    robot_state.setToRandomPositionsNearBy(jmg_, robot_state, NEAR_JOINT);
    // get joints of new pose
    robot_state.copyJointGroupPositions(jmg_, goal);
    // compute target tip_frames
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(tip_frames, goal, poses));

    // compute IK
    moveit_msgs::MoveItErrorCodes error_code;
    kinematics_solver_->searchPositionIK(poses[0], seed, 0.1, consistency_limits, solution, error_code);
    if (error_code.val != error_code.SUCCESS)
    {
      ++failures;
      continue;
    }

    // on success: validate reached poses
    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(tip_frames, solution, reached_poses);
    EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);

    // validate closeness of solution pose to goal
    auto diff = Eigen::Map<Eigen::ArrayXd>(solution.data(), solution.size()) -
                Eigen::Map<Eigen::ArrayXd>(goal.data(), goal.size());
    if (!diff.isZero(1.05 * NEAR_JOINT))
    {
      ++failures;
      ROS_WARN_STREAM("jump in [" << i << "]: " << diff.transpose());
    }

    // update robot state to found pose
    robot_state.setJointGroupPositions(jmg_, solution);
    traj.addSuffixWayPoint(robot_state, 0.1);
  }
  EXPECT_LE(failures, (1.0 - EXPECTED_SUCCESS_RATE) * num_ik_tests_);

  if (publish_trajectory)
  {
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_random_walk", 1, true);
    traj.getRobotTrajectoryMsg(msg.trajectory[0]);
    pub.publish(msg);
    ros::WallDuration(0.1).sleep();
  }
}

static double parseDouble(XmlRpc::XmlRpcValue& v)
{
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(v);
  else if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(v);
  else
    return 0.0;
}
static void parseVector(XmlRpc::XmlRpcValue& vec, std::vector<double>& values, size_t num = 0)
{
  ASSERT_EQ(vec.getType(), XmlRpc::XmlRpcValue::TypeArray);
  if (num != 0)
  {
    ASSERT_EQ(static_cast<size_t>(vec.size()), num);
  }
  values.reserve(vec.size());
  values.clear();
  for (int i = 0; i < vec.size(); ++i)  // NOLINT(modernize-loop-convert)
    values.push_back(parseDouble(vec[i]));
}
static bool parseGoal(const std::string& name, XmlRpc::XmlRpcValue& value, Eigen::Isometry3d& goal, std::string& desc)
{
  std::ostringstream oss;
  std::vector<double> vec;
  if (name == "pose")
  {
    parseVector(value["orientation"], vec, 4);
    Eigen::Quaterniond q(vec[3], vec[0], vec[1], vec[2]);  // w x y z
    goal = q;
    parseVector(value["position"], vec, 3);
    goal.translation() = Eigen::Map<Eigen::Vector3d>(vec.data());
    oss << name << " " << goal.translation().transpose() << " " << q.vec().transpose() << " " << q.w();
    desc = oss.str();
    return true;
  }
  for (unsigned char axis = 0; axis < 3; ++axis)
  {
    char axis_char = 'x' + axis;
    // position offset
    if (name == (std::string("pos.") + axis_char))
    {
      goal.translation()[axis] += parseDouble(value);
      desc = name + " " + std::to_string(parseDouble(value));
      return true;
    }
    // rotation offset
    else if (name == (std::string("rot.") + axis_char))
    {
      goal *= Eigen::AngleAxisd(parseDouble(value), Eigen::Vector3d::Unit(axis));
      desc = name + " " + std::to_string(parseDouble(value));
      return true;
    }
  }
  return false;
}

TEST_F(KinematicsTest, unitIK)
{
  XmlRpc::XmlRpcValue tests;
  if (!getParam("unit_tests", tests))
    return;

  std::vector<double> seed, sol;
  const std::vector<std::string>& tip_frames = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  // initial joint pose from seed_ or defaults
  if (!seed_.empty())
    robot_state.setJointGroupPositions(jmg_, seed_);
  robot_state.copyJointGroupPositions(jmg_, seed);

  // compute initial end-effector pose
  std::vector<geometry_msgs::Pose> poses;
  ASSERT_TRUE(kinematics_solver_->getPositionFK(tip_frames, seed, poses));
  Eigen::Isometry3d initial, goal;
  tf2::fromMsg(poses[0], initial);

  auto validate_ik = [&](const geometry_msgs::Pose& goal, std::vector<double>& truth) {
    // compute IK
    moveit_msgs::MoveItErrorCodes error_code;
    kinematics_solver_->searchPositionIK(goal, seed, timeout_,
                                         const_cast<const std::vector<double>&>(consistency_limits_), sol, error_code);
    ASSERT_EQ(error_code.val, error_code.SUCCESS);

    // validate reached poses
    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(tip_frames, sol, reached_poses);
    EXPECT_NEAR_POSES({ goal }, reached_poses, tolerance_);

    // validate ground truth
    if (!truth.empty())
    {
      ASSERT_EQ(truth.size(), sol.size()) << "Invalid size of ground truth joints vector";
      Eigen::Map<Eigen::ArrayXd> solution(sol.data(), sol.size());
      Eigen::Map<Eigen::ArrayXd> ground_truth(truth.data(), truth.size());
      EXPECT_TRUE(solution.isApprox(ground_truth, 10 * tolerance_)) << solution.transpose() << std::endl
                                                                    << ground_truth.transpose() << std::endl;
    }
  };

  ASSERT_EQ(tests.getType(), XmlRpc::XmlRpcValue::TypeArray);
  std::vector<double> ground_truth;

  /* process tests definitions on parameter server of the form
     - pos.x: +0.1
       joints: [0, 0, 0, 0, 0, 0]
     - pos.y: -0.1
       joints: [0, 0, 0, 0, 0, 0]
  */
  for (int i = 0; i < tests.size(); ++i)  // NOLINT(modernize-loop-convert)
  {
    goal = initial;  // reset goal to initial
    ground_truth.clear();

    ASSERT_EQ(tests[i].getType(), XmlRpc::XmlRpcValue::TypeStruct);
    std::string desc;
    for (std::pair<const std::string, XmlRpc::XmlRpcValue>& member : tests[i])
    {
      if (member.first == "joints")
        parseVector(member.second, ground_truth);
      else if (!parseGoal(member.first, member.second, goal, desc))
        ROS_WARN_STREAM("unknown unit_tests' key: " << member.first);
    }
    {
      SCOPED_TRACE(desc);
      validate_ik(tf2::toMsg(goal), ground_truth);
    }
  }
}

TEST_F(KinematicsTest, searchIK)
{
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    kinematics_solver_->searchPositionIK(poses[0], seed, timeout_, solution, error_code);
    if (error_code.val == error_code.SUCCESS)
      success++;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_tests_);
  EXPECT_GE(success, EXPECTED_SUCCESS_RATE * num_ik_tests_);
}

TEST_F(KinematicsTest, searchIKWithCallback)
{
  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_cb_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));
    if (poses[0].position.z <= 0.0f)
    {
      --i;  // draw a new random state
      continue;
    }

    kinematics_solver_->searchPositionIK(
        poses[0], fk_values, timeout_, solution,
        [this](const geometry_msgs::Pose& /*unused*/, const std::vector<double>& joints,
               moveit_msgs::MoveItErrorCodes& error_code) { searchIKCallback(joints, error_code); },
        error_code);
    if (error_code.val == error_code.SUCCESS)
      success++;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    kinematics_solver_->getPositionFK(fk_names, solution, reached_poses);
    EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_cb_tests_);
  EXPECT_GE(success, EXPECTED_SUCCESS_RATE * num_ik_cb_tests_);
}

TEST_F(KinematicsTest, getIK)
{
  std::vector<double> fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(kinematics_solver_->getJointNames().size(), 0.0);
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (unsigned int i = 0; i < num_ik_tests_; ++i)
  {
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;

    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));
    kinematics_solver_->getPositionIK(poses[0], fk_values, solution, error_code);
    // starting from the correct solution, should yield the same pose
    EXPECT_EQ(error_code.val, error_code.SUCCESS);

    Eigen::Map<Eigen::ArrayXd> sol(solution.data(), solution.size());
    Eigen::Map<Eigen::ArrayXd> truth(fk_values.data(), fk_values.size());
    EXPECT_TRUE(sol.isApprox(truth, tolerance_)) << sol.transpose() << std::endl << truth.transpose() << std::endl;
  }
}

TEST_F(KinematicsTest, getIKMultipleSolutions)
{
  std::vector<double> seed, fk_values;
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  unsigned int success = 0;
  for (unsigned int i = 0; i < num_ik_multiple_tests_; ++i)
  {
    seed.resize(kinematics_solver_->getJointNames().size(), 0.0);
    fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    solutions.clear();
    kinematics_solver_->getPositionIK(poses, fk_values, solutions, result, options);

    if (result.kinematic_error == kinematics::KinematicErrors::OK)
      success += solutions.empty() ? 0 : 1;
    else
      continue;

    std::vector<geometry_msgs::Pose> reached_poses;
    for (const auto& s : solutions)
    {
      kinematics_solver_->getPositionFK(fk_names, s, reached_poses);
      EXPECT_NEAR_POSES(poses, reached_poses, tolerance_);
    }
  }

  ROS_INFO_STREAM("Success Rate: " << (double)success / num_ik_multiple_tests_);
  EXPECT_GE(success, EXPECTED_SUCCESS_RATE * num_ik_multiple_tests_);
}

// validate that getPositionIK() retrieves closest solution to seed
TEST_F(KinematicsTest, getNearestIKSolution)
{
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsQueryOptions options;
  kinematics::KinematicsResult result;

  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  const std::vector<std::string>& fk_names = kinematics_solver_->getTipFrames();
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  for (unsigned int i = 0; i < num_nearest_ik_tests_; ++i)
  {
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, fk_values);
    std::vector<geometry_msgs::Pose> poses;
    ASSERT_TRUE(kinematics_solver_->getPositionFK(fk_names, fk_values, poses));

    // sample seed vector
    robot_state.setToRandomPositions(jmg_, this->rng_);
    robot_state.copyJointGroupPositions(jmg_, seed);

    // getPositionIK for single solution
    kinematics_solver_->getPositionIK(poses[0], seed, solution, error_code);

    // check if getPositionIK call for single solution returns solution
    if (error_code.val != error_code.SUCCESS)
      continue;

    const Eigen::Map<const Eigen::VectorXd> seed_eigen(seed.data(), seed.size());
    double error_get_ik =
        (Eigen::Map<const Eigen::VectorXd>(solution.data(), solution.size()) - seed_eigen).array().abs().sum();

    // getPositionIK for multiple solutions
    solutions.clear();
    kinematics_solver_->getPositionIK(poses, seed, solutions, result, options);

    // check if getPositionIK call for multiple solutions returns solution
    EXPECT_EQ(result.kinematic_error, kinematics::KinematicErrors::OK)
        << "Multiple solution call failed, while single solution call succeeded";
    if (result.kinematic_error != kinematics::KinematicErrors::OK)
      continue;

    double smallest_error_multiple_ik = std::numeric_limits<double>::max();
    for (const auto& s : solutions)
    {
      double error_multiple_ik =
          (Eigen::Map<const Eigen::VectorXd>(s.data(), s.size()) - seed_eigen).array().abs().sum();
      if (error_multiple_ik <= smallest_error_multiple_ik)
        smallest_error_multiple_ik = error_multiple_ik;
    }
    EXPECT_NEAR(smallest_error_multiple_ik, error_get_ik, tolerance_);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kinematics_plugin_test");
  ros::NodeHandle nh;
  int result = RUN_ALL_TESTS();
  SharedData::release();  // avoid class_loader::LibraryUnloadException
  return result;
}
