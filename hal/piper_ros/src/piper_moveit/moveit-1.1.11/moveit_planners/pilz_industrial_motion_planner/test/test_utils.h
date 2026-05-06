/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <gtest/gtest.h>
#ifndef INSTANTIATE_TEST_SUITE_P  // prior to gtest 1.10
#define INSTANTIATE_TEST_SUITE_P(...) INSTANTIATE_TEST_CASE_P(__VA_ARGS__)
#endif
#ifndef TYPED_TEST_SUITE  // prior to gtest 1.10
#define TYPED_TEST_SUITE(...) TYPED_TEST_CASE(__VA_ARGS__)
#endif

#include "moveit_msgs/MotionSequenceRequest.h"
#include "pilz_industrial_motion_planner/limits_container.h"
#include "pilz_industrial_motion_planner/trajectory_blend_request.h"
#include "pilz_industrial_motion_planner/trajectory_blend_response.h"
#include "pilz_industrial_motion_planner/trajectory_generator.h"
#include <boost/core/demangle.hpp>
#include <math.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>

namespace testutils
{
const std::string JOINT_NAME_PREFIX("prbt_joint_");

static constexpr int32_t DEFAULT_SERVICE_TIMEOUT(10);
static constexpr double DEFAULT_ACCELERATION_EQUALITY_TOLERANCE{ 1e-6 };
static constexpr double DEFAULT_ROTATION_AXIS_EQUALITY_TOLERANCE{ 1e-8 };

/**
 * @brief Convert degree to rad.
 */
inline static constexpr double deg2Rad(double angle)
{
  return (angle / 180.0) * M_PI;
}

inline std::string getJointName(size_t joint_number, const std::string& joint_prefix)
{
  return joint_prefix + std::to_string(joint_number);
}

/**
 * @brief Create limits for tests to avoid the need to get the limits from the
 * parameter server
 */
pilz_industrial_motion_planner::JointLimitsContainer createFakeLimits(const std::vector<std::string>& joint_names);

inline std::string demangle(char const* name)
{
  return boost::core::demangle(name);
}

//********************************************
// Motion plan requests
//********************************************

inline sensor_msgs::JointState generateJointState(const std::vector<double>& pos, const std::vector<double>& vel,
                                                  const std::string& joint_prefix = testutils::JOINT_NAME_PREFIX)
{
  sensor_msgs::JointState state;
  auto posit = pos.cbegin();
  size_t i = 0;

  while (posit != pos.cend())
  {
    state.name.push_back(testutils::getJointName(i + 1, joint_prefix));
    state.position.push_back(*posit);

    i++;
    posit++;
  }
  for (const auto& one_vel : vel)
  {
    state.velocity.push_back(one_vel);
  }
  return state;
}

inline sensor_msgs::JointState generateJointState(const std::vector<double>& pos,
                                                  const std::string& joint_prefix = testutils::JOINT_NAME_PREFIX)
{
  return generateJointState(pos, std::vector<double>(), joint_prefix);
}

inline moveit_msgs::Constraints generateJointConstraint(const std::vector<double>& pos_list,
                                                        const std::string& joint_prefix = testutils::JOINT_NAME_PREFIX)
{
  moveit_msgs::Constraints gc;

  auto pos_it = pos_list.begin();

  for (size_t i = 0; i < pos_list.size(); ++i)
  {
    moveit_msgs::JointConstraint jc;
    jc.joint_name = testutils::getJointName(i + 1, joint_prefix);
    jc.position = *pos_it;
    gc.joint_constraints.push_back(jc);

    ++pos_it;
  }

  return gc;
}

/**
 * @brief Determines the goal position from the given request.
 * TRUE if successful, FALSE otherwise.
 */
bool getExpectedGoalPose(const moveit::core::RobotModelConstPtr& robot_model,
                         const planning_interface::MotionPlanRequest& req, std::string& link_name,
                         Eigen::Isometry3d& goal_pose_expect);

/**
 * @brief create a dummy motion plan request with zero start state
 * No goal constraint is given.
 * @param robot_model
 * @param planning_group
 * @param req
 */
void createDummyRequest(const moveit::core::RobotModelConstPtr& robot_model, const std::string& planning_group,
                        planning_interface::MotionPlanRequest& req);

void createPTPRequest(const std::string& planning_group, const moveit::core::RobotState& start_state,
                      const moveit::core::RobotState& goal_state, planning_interface::MotionPlanRequest& req);

/**
 * @brief check if the goal given in joint space is reached
 * Only the last point in the trajectory is veryfied.
 * @param trajectory: generated trajectory
 * @param goal: goal in joint space
 * @param joint_position_tolerance
 * @param joint_velocity_tolerance
 * @return true if satisfied
 */
bool isGoalReached(const trajectory_msgs::JointTrajectory& trajectory,
                   const std::vector<moveit_msgs::JointConstraint>& goal, const double joint_position_tolerance,
                   const double joint_velocity_tolerance = 1.0e-6);

/**
 * @brief check if the goal given in cartesian space is reached
 * Only the last point in the trajectory is veryfied.
 * @param robot_model
 * @param trajectory
 * @param req
 * @param matrix_norm_tolerance: used to compare the transformation matrix
 * @param joint_velocity_tolerance
 * @return
 */
bool isGoalReached(const robot_model::RobotModelConstPtr& robot_model,
                   const trajectory_msgs::JointTrajectory& trajectory, const planning_interface::MotionPlanRequest& req,
                   const double pos_tolerance, const double orientation_tolerance);

bool isGoalReached(const moveit::core::RobotModelConstPtr& robot_model,
                   const trajectory_msgs::JointTrajectory& trajectory, const planning_interface::MotionPlanRequest& req,
                   const double tolerance);

/**
 * @brief Check that given trajectory is straight line.
 */
bool checkCartesianLinearity(const robot_model::RobotModelConstPtr& robot_model,
                             const trajectory_msgs::JointTrajectory& trajectory,
                             const planning_interface::MotionPlanRequest& req, const double translation_norm_tolerance,
                             const double rot_axis_norm_tolerance, const double rot_angle_tolerance = 10e-5);

/**
 * @brief check SLERP. The orientation should rotate around the same axis
 * linearly.
 * @param start_pose
 * @param goal_pose
 * @param wp_pose
 * @param rot_axis_norm_tolerance
 * @return
 */
bool checkSLERP(const Eigen::Isometry3d& start_pose, const Eigen::Isometry3d& goal_pose,
                const Eigen::Isometry3d& wp_pose, const double rot_axis_norm_tolerance,
                const double rot_angle_tolerance = 10e-5);

/**
 * @brief get the waypoint index from time from start
 * @param trajectory
 * @param time_from_start
 * @return
 */
inline int getWayPointIndex(const robot_trajectory::RobotTrajectoryPtr& trajectory, const double time_from_start)
{
  int index_before, index_after;
  double blend;
  trajectory->findWayPointIndicesForDurationAfterStart(time_from_start, index_before, index_after, blend);
  return blend > 0.5 ? index_after : index_before;
}

/**
 * @brief check joint trajectory of consistency, position, velocity and
 * acceleration limits
 * @param trajectory
 * @param joint_limits
 * @return
 */
bool checkJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                          const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

/**
 * @brief Checks that every waypoint in the trajectory has a non-zero duration
 * between itself and its predecessor
 *
 * Usage in tests:
 * @code
 *    EXPECT_TRUE(HasStrictlyIncreasingTime(trajectory));
 * @endcode
 */
::testing::AssertionResult hasStrictlyIncreasingTime(const robot_trajectory::RobotTrajectoryPtr& trajectory);

/**
 * @brief check if the sizes of the joint position/veloicty/acceleration are
 * correct
 * @param trajectory
 * @return
 */
bool isTrajectoryConsistent(const trajectory_msgs::JointTrajectory& trajectory);

/**
 * @brief is Position Bounded
 * @param trajectory
 * @param joint_limits
 * @return
 */
bool isPositionBounded(const trajectory_msgs::JointTrajectory& trajectory,
                       const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

/**
 * @brief is Velocity Bounded
 * @param trajectory
 * @param joint_limits
 * @return
 */
bool isVelocityBounded(const trajectory_msgs::JointTrajectory& trajectory,
                       const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

/**
 * @brief is Acceleration Bounded
 * @param trajectory
 * @param joint_limits
 * @return
 */
bool isAccelerationBounded(const trajectory_msgs::JointTrajectory& trajectory,
                           const pilz_industrial_motion_planner::JointLimitsContainer& joint_limits);

/**
 * @brief compute the tcp pose from joint values
 * @param robot_model
 * @param link_name
 * @param joint_values
 * @param pose
 * @param joint_prefix Prefix of the joint names
 * @return false if forward kinematics failed
 */
bool toTCPPose(const moveit::core::RobotModelConstPtr& robot_model, const std::string& link_name,
               const std::vector<double>& joint_values, geometry_msgs::Pose& pose,
               const std::string& joint_prefix = testutils::JOINT_NAME_PREFIX);

/**
 * @brief computeLinkFK
 * @param robot_model
 * @param link_name
 * @param joint_state
 * @param pose
 * @return
 */
bool computeLinkFK(const robot_model::RobotModelConstPtr& robot_model, const std::string& link_name,
                   const std::map<std::string, double>& joint_state, Eigen::Isometry3d& pose);

/**
 * @brief checkOriginalTrajectoryAfterBlending
 * @param blend_res
 * @param traj_1_res
 * @param traj_2_res
 * @param time_tolerance
 * @return
 */
bool checkOriginalTrajectoryAfterBlending(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                                          const pilz_industrial_motion_planner::TrajectoryBlendResponse& res,
                                          const double time_tolerance);

/**
 * @brief check the blending result, if the joint space continuity is fulfilled
 * @param res: trajectory blending response, contains three trajectories.
 * Between these three trajectories should be continuous.
 * @return true if joint position/velocity is continuous. joint acceleration can
 * have jumps.
 */
bool checkBlendingJointSpaceContinuity(const pilz_industrial_motion_planner::TrajectoryBlendResponse& res,
                                       double joint_velocity_tolerance, double joint_accleration_tolerance);

bool checkBlendingCartSpaceContinuity(const pilz_industrial_motion_planner::TrajectoryBlendRequest& req,
                                      const pilz_industrial_motion_planner::TrajectoryBlendResponse& res,
                                      const pilz_industrial_motion_planner::LimitsContainer& planner_limits);

/**
 * @brief Checks if all points of the blending trajectory lie within the
 * blending radius.
 */
bool checkThatPointsInRadius(const std::string& link_name, const double& r, Eigen::Isometry3d& circ_pose,
                             const pilz_industrial_motion_planner::TrajectoryBlendResponse& res);

/**
 * @brief compute Cartesian velocity
 * @param pose_1
 * @param pose_2
 * @param duration
 * @param v
 * @param w
 */
void computeCartVelocity(const Eigen::Isometry3d& pose_1, const Eigen::Isometry3d& pose_2, double duration,
                         Eigen::Vector3d& v, Eigen::Vector3d& w);

/**
 * @brief Returns an initial joint state and two poses which
 * can be used to perform a Lin-Lin movement.
 *
 * @param frame_id
 * @param initialJointState As cartesian position: (0.3, 0, 0.65, 0, 0, 0)
 * @param p1 (0.05, 0, 0.65, 0, 0, 0)
 * @param p2 (0.05, 0.4, 0.65, 0, 0, 0)
 */
void getLinLinPosesWithoutOriChange(const std::string& frame_id, sensor_msgs::JointState& initialJointState,
                                    geometry_msgs::PoseStamped& p1, geometry_msgs::PoseStamped& p2);

void getOriChange(Eigen::Matrix3d& ori1, Eigen::Matrix3d& ori2);

void createFakeCartTraj(const robot_trajectory::RobotTrajectoryPtr& traj, const std::string& link_name,
                        moveit_msgs::RobotTrajectory& fake_traj);

inline geometry_msgs::Quaternion fromEuler(double a, double b, double c)
{
  tf2::Vector3 qvz(0., 0., 1.);
  tf2::Vector3 qvy(0., 1., 0.);
  tf2::Quaternion q1(qvz, a);

  q1 = q1 * tf2::Quaternion(qvy, b);
  q1 = q1 * tf2::Quaternion(qvz, c);

  return tf2::toMsg(q1);
}

/**
 * @brief Test data for blending, which contains three joint position vectors of
 * three robot state.
 */
struct BlendTestData
{
  std::vector<double> start_position;
  std::vector<double> mid_position;
  std::vector<double> end_position;
};

/**
 * @brief fetch test datasets from parameter server
 * @param nh
 * @return
 */
bool getBlendTestData(const ros::NodeHandle& nh, const size_t& dataset_num, const std::string& name_prefix,
                      std::vector<BlendTestData>& datasets);

/**
 * @brief check the blending result of lin-lin
 * @param lin_res_1
 * @param lin_res_2
 * @param blend_req
 * @param blend_res
 * @param checkAcceleration
 */
bool checkBlendResult(const pilz_industrial_motion_planner::TrajectoryBlendRequest& blend_req,
                      const pilz_industrial_motion_planner::TrajectoryBlendResponse& blend_res,
                      const pilz_industrial_motion_planner::LimitsContainer& limits, double joint_velocity_tolerance,
                      double joint_acceleration_tolerance, double cartesian_velocity_tolerance,
                      double cartesian_angular_velocity_tolerance);

/**
 * @brief generate two LIN trajectories from test data set
 * @param data: contains joint poisitons of start/mid/end states
 * @param sampling_time_1: sampling time for first LIN
 * @param sampling_time_2: sampling time for second LIN
 * @param[out] res_lin_1: result of the first LIN motion planning
 * @param[out] res_lin_2: result of the second LIN motion planning
 * @param[out] dis_lin_1: translational distance of the first LIN
 * @param[out] dis_lin_2: translational distance of the second LIN
 * @return true if succeed
 */
bool generateTrajFromBlendTestData(const planning_scene::PlanningSceneConstPtr& scene,
                                   const std::shared_ptr<pilz_industrial_motion_planner::TrajectoryGenerator>& tg,
                                   const std::string& group_name, const std::string& link_name,
                                   const BlendTestData& data, const double& sampling_time_1,
                                   const double& sampling_time_2, planning_interface::MotionPlanResponse& res_lin_1,
                                   planning_interface::MotionPlanResponse& res_lin_2, double& dis_lin_1,
                                   double& dis_lin_2);

void generateRequestMsgFromBlendTestData(const moveit::core::RobotModelConstPtr& robot_model, const BlendTestData& data,
                                         const std::string& planner_id, const std::string& group_name,
                                         const std::string& link_name, moveit_msgs::MotionSequenceRequest& req_list);

void checkRobotModel(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                     const std::string& link_name);

/** @brief Check that a given vector of accelerations represents a trapezoid
 * velocity profile
 * @param acc_tol: tolerance for comparing acceleration values
 */
::testing::AssertionResult hasTrapezoidVelocity(std::vector<double> accelerations, const double acc_tol);

/**
 * @brief Check that the translational path of a given trajectory has a
 * trapezoid velocity profile
 * @param acc_tol: tolerance for comparing acceleration values
 */
::testing::AssertionResult
checkCartesianTranslationalPath(const robot_trajectory::RobotTrajectoryConstPtr& trajectory,
                                const std::string& link_name,
                                const double acc_tol = DEFAULT_ACCELERATION_EQUALITY_TOLERANCE);

/**
 * @brief Check that the rotational path of a given trajectory is a quaternion
 * slerp.
 * @param rot_axis_tol: tolerance for comparing rotation axes (in the L2 norm)
 * @param acc_tol: tolerance for comparing angular acceleration values
 */
::testing::AssertionResult
checkCartesianRotationalPath(const robot_trajectory::RobotTrajectoryConstPtr& trajectory, const std::string& link_name,
                             const double rot_axis_tol = DEFAULT_ROTATION_AXIS_EQUALITY_TOLERANCE,
                             const double acc_tol = DEFAULT_ACCELERATION_EQUALITY_TOLERANCE);

inline bool isMonotonouslyDecreasing(const std::vector<double>& vec, const double& tol)
{
  return std::is_sorted(vec.begin(), vec.end(), [tol](double a, double b) {
    return !(std::abs(a - b) < tol || a < b);  // true -> a is ordered before b -> list is not sorted
  });
}

}  // namespace testutils
