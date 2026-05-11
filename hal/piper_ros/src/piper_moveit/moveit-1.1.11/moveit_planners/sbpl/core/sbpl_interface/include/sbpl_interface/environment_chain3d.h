/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Maxim Likhachev
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
 *   * Neither the name of Maxim Likhachev nor the names of its
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

/** \Author: Benjamin Cohen /bcohen@willowgarage.com, E. Gil Jones **/

#pragma once

#include <time.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <string>
#include <list>
#include <algorithm>

#include <sbpl/headers.h>
#include <sbpl_interface/bfs3d/BFS_3D.h>
#include <planning_scene/planning_scene.h>
#include <collision_distance_field/collision_env_hybrid.h>
#include <sbpl_interface/environment_chain3d_types.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <Eigen/Core>

static const double DEFAULT_INTERPOLATION_DISTANCE = .05;
static const double DEFAULT_JOINT_MOTION_PRIMITIVE_DISTANCE = .2;

namespace sbpl_interface
{
struct PlanningStatistics
{
  PlanningStatistics() : total_expansions_(0), coll_checks_(0)
  {
  }

  unsigned int total_expansions_;
  ros::WallDuration total_expansion_time_;
  ros::WallDuration total_coll_check_time_;
  unsigned int coll_checks_;
  ros::WallDuration total_planning_time_;
};

struct PlanningParameters
{
  PlanningParameters()
    : use_bfs_(true)
    , use_standard_collision_checking_(false)
    , attempt_full_shortcut_(true)
    , interpolation_distance_(DEFAULT_INTERPOLATION_DISTANCE)
    , joint_motion_primitive_distance_(DEFAULT_JOINT_MOTION_PRIMITIVE_DISTANCE)
  {
  }

  bool use_bfs_;
  bool use_standard_collision_checking_;
  bool attempt_full_shortcut_;
  double interpolation_distance_;
  double joint_motion_primitive_distance_;
};

/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentChain3D : public DiscreteSpaceInformation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor
   */
  EnvironmentChain3D(const planning_scene::PlanningSceneConstPtr& planning_scene);

  /**
   * @brief Destructor
   */
  ~EnvironmentChain3D();

  //
  // Pure virtual functions from DiscreteSpaceInformation
  //

  /**
   * @brief Initialize the environment from a text file
   * @param name of environment text file
   * @return true if successful, false otherwise
   */
  virtual bool InitializeEnv(const char* sEnvFile);

  /**
   * @brief Initialize the start and goal states of the MDP
   * @param always returns true...
   */
  virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);

  /**
   * @brief Get the heuristic from one state to another state.
   * @param the stateID of the current state
   * @param the stateID of the goal state
   * @return h(s,s')
   */
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

  /**
   * @brief Get the heuristic of a state to the planner's goal state.
   * @param the stateID of the current state
   * @return h(s,s_goal)
   */
  virtual int GetGoalHeuristic(int stateID);

  /**
   * @brief Get the heuristic of a state to the planner's start state.
   * @param the stateID of the current state
   * @return h(s,s_start)
   */
  virtual int GetStartHeuristic(int stateID);

  /**
   * @brief Get the successors of the desired state to be expanded.
   * Return vectors with the successors' state IDs and the cost to move
   * from the current state to that state. If the vectors return to the
   * planner empty then the search quits.
   * @param the state ID of the state to be expanded
   * @param a pointer to a vector that will be populated with the
   * successor state IDs.
   * @param a pointer to a vector that will be populated with the costs of
   * transitioning from the current state to the successor state.
   */
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);

  /** @brief Not defined. */
  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);

  /** @brief Not defined. */
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

  /** @brief Not defined. */
  virtual void SetAllPreds(CMDPSTATE* state);

  /**
   * @brief This function returns the number of hash entries created.
   * @return number of hash entries
   */
  virtual int SizeofCreatedEnv();

  /**
   * @brief This function prints out the state information of a state.
   * @param the state ID of the desired state
   * @param prints out a little extra information
   * @param the file pointer to print to (stdout by default)
   */
  virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

  /** @brief Not defined. */
  virtual void PrintEnv_Config(FILE* fOut);

  //
  // Overloaded virtual functions from DiscreteSpaceInformation
  //

  /**
   * @brief This function is for debugging purposes. It returns the
   * pose of the states that were expanded. The planner node has
   * a function to display these as visualizations in rviz.
   * @param a pointer to a vector of the poses of all of the states
   * expanded during the search (when using ARA*)
   */
  virtual void getExpandedStates(std::vector<std::vector<double> >* ara_states){};

  /**
   * @brief Check if the states with StateID1 & StateID2 are equivalent
   * based on an equivalency function with some declared tolerance.
   * @param stateID of first state
   * @param stateID of second state
   * @return true if equivalent, false otherwise
   */
  virtual bool AreEquivalent(int StateID1, int StateID2);

  bool setupForMotionPlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const moveit_msgs::GetMotionPlan::Request& req, moveit_msgs::GetMotionPlan::Response& res,
                          const PlanningParameters& params);

  const EnvChain3DPlanningData& getPlanningData() const
  {
    return planning_data_;
  }

  bool populateTrajectoryFromStateIDSequence(const std::vector<int>& state_ids,
                                             trajectory_msgs::JointTrajectory& traj) const;

  const PlanningStatistics& getPlanningStatistics() const
  {
    return planning_statistics_;
  }

  bool getPlaneBFSMarker(visualization_msgs::Marker& plane_marker, double z_val);

  const Eigen::Isometry3d& getGoalPose() const
  {
    return goal_pose_;
  }

  void attemptShortcut(const trajectory_msgs::JointTrajectory& traj_in, trajectory_msgs::JointTrajectory& traj_out);

protected:
  bool getGridXYZInt(const Eigen::Isometry3d& pose, int (&xyz)[3]) const;

  void getMotionPrimitives(const std::string& group);

  planning_scene::PlanningSceneConstPtr planning_scene_;

  double angle_discretization_;
  BFS_3D* bfs_;

  std::vector<boost::shared_ptr<JointMotionWrapper> > joint_motion_wrappers_;
  std::vector<boost::shared_ptr<JointMotionPrimitive> > possible_actions_;
  planning_models::RobotState* state_;
  const collision_detection::CollisionEnvHybrid* hy_env_;
  planning_models::RobotState* ::JointStateGroup* joint_state_group_;
  boost::shared_ptr<collision_detection::GroupStateRepresentation> gsr_;
  // boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
  const planning_models::RobotState* ::LinkState* tip_link_state_;
  EnvChain3DPlanningData planning_data_;
  kinematic_constraints::KinematicConstraintSet goal_constraint_set_;
  kinematic_constraints::KinematicConstraintSet path_constraint_set_;
  std::string planning_group_;
  Eigen::Isometry3d goal_pose_;
  PlanningStatistics planning_statistics_;
  PlanningParameters planning_parameters_;
  int maximum_distance_for_motion_;

  planning_models::RobotState* interpolation_state_1_;
  planning_models::RobotState* interpolation_state_2_;
  planning_models::RobotState* interpolation_state_temp_;
  planning_models::RobotState* ::JointStateGroup* interpolation_joint_state_group_1_;
  planning_models::RobotState* ::JointStateGroup* interpolation_joint_state_group_2_;
  planning_models::RobotState* ::JointStateGroup* interpolation_joint_state_group_temp_;
  std::map<int, std::map<int, std::vector<std::vector<double> > > > generated_interpolations_map_;

  void setMotionPrimitives(const std::string& group_name);
  void determineMaximumEndEffectorTravel();

  void convertCoordToJointAngles(const std::vector<int>& coord, std::vector<double>& angles);
  void convertJointAnglesToCoord(const std::vector<double>& angle, std::vector<int>& coord);

  int calculateCost(EnvChain3DHashEntry* HashEntry1, EnvChain3DHashEntry* HashEntry2);
  int getBFSCostToGoal(int x, int y, int z) const;
  int getEndEffectorHeuristic(int FromStateID, int ToStateID);

  double getJointDistanceDoubleSum(const std::vector<double>& angles1, const std::vector<double>& angles2) const;

  int getJointDistanceIntegerSum(const std::vector<double>& angles1, const std::vector<double>& angles2,
                                 double delta) const;

  int getJointDistanceIntegerMax(const std::vector<double>& angles1, const std::vector<double>& angles2,
                                 double delta) const;

  // double getJointDistanceMax(const std::vector<double>& angles1,
  //                            const std::vector<double>& angles2);

  bool interpolateAndCollisionCheck(const std::vector<double> angles1, const std::vector<double> angles2,
                                    std::vector<std::vector<double> >& state_values);

  inline double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) const
  {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
  }

  // temp
  double closest_to_goal_;
};

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentChain3D::convertCoordToJointAngles(const std::vector<int>& coord, std::vector<double>& angles)
{
  angles.resize(coord.size());
  for (size_t i = 0; i < coord.size(); i++)
    angles[i] = coord[i] * angle_discretization_;
}

inline void EnvironmentChain3D::convertJointAnglesToCoord(const std::vector<double>& angle, std::vector<int>& coord)
{
  coord.resize(angle.size());
  for (unsigned int i = 0; i < angle.size(); i++)
  {
    // NOTE: Added 3/1/09
    double pos_angle = angle[i];
    if (pos_angle < 0.0)
      pos_angle += 2 * M_PI;

    coord[i] = (int)((pos_angle + angle_discretization_ * 0.5) / angle_discretization_);
  }
}

// inline double EnvironmentChain3D::getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double
// z2) const
// {
//   return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
// }

}  // namespace sbpl_interface
