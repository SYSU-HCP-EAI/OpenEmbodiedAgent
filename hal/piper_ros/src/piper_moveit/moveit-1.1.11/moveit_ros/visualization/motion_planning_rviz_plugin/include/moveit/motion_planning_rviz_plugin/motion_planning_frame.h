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

/* Author: Ioan Sucan */

#pragma once

#include <QWidget>
#include <QTreeWidgetItem>
#include <QListWidgetItem>

#ifndef Q_MOC_RUN
#include <moveit/macros/class_forward.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>
#include <moveit/semantic_world/semantic_world.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <actionlib/client/simple_action_client.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#endif

#include <map>
#include <string>
#include <memory>

namespace rviz
{
class DisplayContext;
}

namespace Ui
{
class MotionPlanningUI;
}

namespace moveit_warehouse
{
MOVEIT_CLASS_FORWARD(PlanningSceneStorage);  // Defines PlanningSceneStoragePtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(ConstraintsStorage);    // Defines ConstraintsStoragePtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(RobotStateStorage);     // Defines RobotStateStoragePtr, ConstPtr, WeakPtr... etc
}  // namespace moveit_warehouse

namespace moveit_rviz_plugin
{
class MotionPlanningDisplay;
class MotionPlanningFrameJointsWidget;

const std::string OBJECT_RECOGNITION_ACTION = "/recognize_objects";

static const std::string TAB_CONTEXT = "Context";
static const std::string TAB_PLANNING = "Planning";
static const std::string TAB_MANIPULATION = "Manipulation";
static const std::string TAB_OBJECTS = "Scene Objects";
static const std::string TAB_SCENES = "Stored Scenes";
static const std::string TAB_STATES = "Stored States";
static const std::string TAB_STATUS = "Status";

static const double LARGE_MESH_THRESHOLD = 10.0;

class MotionPlanningFrame : public QWidget
{
  friend class MotionPlanningDisplay;
  Q_OBJECT

public:
  MotionPlanningFrame(const MotionPlanningFrame&) = delete;
  MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz::DisplayContext* context, QWidget* parent = nullptr);
  ~MotionPlanningFrame() override;

  void clearRobotModel();
  void changePlanningGroup();
  void enable();
  void disable();
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:
  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void initFromMoveGroupNS();
  void constructPlanningRequest(moveit_msgs::MotionPlanRequest& mreq);

  void updateSceneMarkers(float wall_dt, float ros_dt);

  void updateExternalCommunication();

  MotionPlanningDisplay* planning_display_;
  rviz::DisplayContext* context_;
  Ui::MotionPlanningUI* ui_;
  MotionPlanningFrameJointsWidget* joints_tab_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::semantic_world::SemanticWorldPtr semantic_world_;

  moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_;
  moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage_;
  moveit_warehouse::ConstraintsStoragePtr constraints_storage_;
  moveit_warehouse::RobotStateStoragePtr robot_state_storage_;

  std::shared_ptr<rviz::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::RobotState> RobotStatePair;
  RobotStateMap robot_states_;
  std::string default_planning_pipeline_;
  std::vector<moveit_msgs::PlannerInterfaceDescription> planner_descriptions_;

Q_SIGNALS:
  void planningFinished();
  void configChanged();

private Q_SLOTS:

  // Context tab
  void databaseConnectButtonClicked();
  void planningPipelineIndexChanged(int index);
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked();
  void approximateIKChanged(int state);

  // Planning tab
  bool computeCartesianPlan();
  bool computeJointSpacePlan();
  void planButtonClicked();
  void executeButtonClicked();
  void planAndExecuteButtonClicked();
  void stopButtonClicked();
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void allowExternalProgramCommunication(bool enable);
  void pathConstraintsIndexChanged(int index);
  void onNewPlanningSceneState();
  void startStateTextChanged(const QString& start_state);
  void goalStateTextChanged(const QString& goal_state);
  void planningGroupTextChanged(const QString& planning_group);
  void onClearOctomapClicked();

  // Scene Objects tab
  void clearScene();
  void publishScene();
  void publishSceneIfNeeded();
  void setLocalSceneEdited(bool dirty = true);
  bool isLocalSceneDirty() const;
  void sceneScaleChanged(int value);
  void sceneScaleStartChange();
  void sceneScaleEndChange();
  void shapesComboBoxChanged(const QString& text);
  void addSceneObject();
  void removeSceneObjects();
  void currentCollisionObjectChanged();
  void objectPoseValueChanged(double value);
  void collisionObjectChanged(QListWidgetItem* item);
  void imProcessFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback);
  void copySelectedCollisionObjects();
  void exportGeometryAsTextButtonClicked();
  void importGeometryFromTextButtonClicked();

  // Stored scenes tab
  void saveSceneButtonClicked();
  void planningSceneItemClicked();
  void saveQueryButtonClicked();
  void deleteSceneButtonClicked();
  void deleteQueryButtonClicked();
  void loadSceneButtonClicked();
  void loadQueryButtonClicked();
  void warehouseItemNameChanged(QTreeWidgetItem* item, int column);

  // States tab
  void loadStateButtonClicked();
  void saveStartStateButtonClicked();
  void saveGoalStateButtonClicked();
  void removeStateButtonClicked();
  void clearStatesButtonClicked();
  void setAsStartStateButtonClicked();
  void setAsGoalStateButtonClicked();

  // Pick and place
  void detectObjectsButtonClicked();
  void pickObjectButtonClicked();
  void placeObjectButtonClicked();
  void selectedDetectedObjectChanged();
  void detectedObjectChanged(QListWidgetItem* item);
  void selectedSupportSurfaceChanged();

  // General
  void tabChanged(int index);

private:
  // Context tab
  void computeDatabaseConnectButtonClicked();
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeResetDbButtonClicked(const std::string& db);
  void populatePlannersList(const std::vector<moveit_msgs::PlannerInterfaceDescription>& desc);
  void populatePlannerDescription(const moveit_msgs::PlannerInterfaceDescription& desc);

  // Planning tab
  void computePlanButtonClicked();
  void computeExecuteButtonClicked();
  void computePlanAndExecuteButtonClicked();
  void computePlanAndExecuteButtonClickedDisplayHelper();
  void computeStopButtonClicked();
  void onFinishedExecution(bool success);
  void populateConstraintsList();
  void populateConstraintsList(const std::vector<std::string>& constr);
  void configureForPlanning();
  void configureWorkspace();
  void updateQueryStateHelper(moveit::core::RobotState& state, const std::string& v);
  void fillStateSelectionOptions();
  void fillPlanningGroupOptions();
  void startStateTextChangedExec(const std::string& start_state);
  void goalStateTextChangedExec(const std::string& goal_state);

  // Scene objects tab
  void updateCollisionObjectPose(bool update_marker_position);
  void createSceneInteractiveMarker();
  void renameCollisionObject(QListWidgetItem* item);
  void attachDetachCollisionObject(QListWidgetItem* item);
  QListWidgetItem* addCollisionObjectToList(const std::string& name, int row, bool attached);
  void populateCollisionObjectsList(planning_scene_monitor::LockedPlanningSceneRO* pps = nullptr);
  void computeImportGeometryFromText(const std::string& path);
  void computeExportGeometryAsText(const std::string& path);
  visualization_msgs::InteractiveMarker
  createObjectMarkerMsg(const collision_detection::CollisionEnv::ObjectConstPtr& obj);

  // Stored scenes tab
  void computeSaveSceneButtonClicked();
  void computeSaveQueryButtonClicked(const std::string& scene, const std::string& query_name);
  void computeLoadSceneButtonClicked();
  void computeLoadQueryButtonClicked();
  void populatePlanningSceneTreeView();
  void computeDeleteSceneButtonClicked();
  void computeDeleteQueryButtonClicked();
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem* s);
  void checkPlanningSceneTreeEnabledButtons();

  // States tab
  void saveRobotStateButtonClicked(const moveit::core::RobotState& state);
  void populateRobotStatesList();

  // Pick and place
  void processDetectedObjects();
  void updateDetectedObjectsList(const std::vector<std::string>& object_ids);
  void publishTables();
  void updateSupportSurfacesList();
  ros::Publisher object_recognition_trigger_publisher_;
  std::map<std::string, std::string> pick_object_name_;
  std::string place_object_name_;
  std::vector<geometry_msgs::PoseStamped> place_poses_;
  void pickObject();
  void placeObject();
  void triggerObjectDetection();
  void updateTables();
  std::string support_surface_name_;
  // For coloring
  std::string selected_object_name_;
  std::string selected_support_surface_name_;

  std::unique_ptr<actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> >
      object_recognition_client_;
  template <typename T>
  void waitForAction(const T& action, const ros::Duration& wait_for_server, const std::string& name);
  void listenDetectedObjects(const object_recognition_msgs::RecognizedObjectArrayPtr& msg);
  ros::Subscriber object_recognition_subscriber_;

  ros::Subscriber plan_subscriber_;
  ros::Subscriber execute_subscriber_;
  ros::Subscriber stop_subscriber_;
  ros::Subscriber update_start_state_subscriber_;
  ros::Subscriber update_goal_state_subscriber_;
  ros::Subscriber update_custom_start_state_subscriber_;
  ros::Subscriber update_custom_goal_state_subscriber_;
  // General
  void changePlanningGroupHelper();
  shapes::ShapePtr loadMeshResource(const std::string& url);
  void loadStoredStates(const std::string& pattern);

  void remotePlanCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteExecuteCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteStopCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateStartStateCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateGoalStateCallback(const std_msgs::EmptyConstPtr& msg);
  void remoteUpdateCustomStartStateCallback(const moveit_msgs::RobotStateConstPtr& msg);
  void remoteUpdateCustomGoalStateCallback(const moveit_msgs::RobotStateConstPtr& msg);

  ros::NodeHandle nh_;  // node handle with the namespace of the connected move_group node
  ros::Publisher planning_scene_publisher_;
  ros::Publisher planning_scene_world_publisher_;

  collision_detection::CollisionEnv::ObjectConstPtr scaled_object_;
  moveit::core::FixedTransformsMap scaled_object_subframes_;
  EigenSTL::vector_Isometry3d scaled_object_shape_poses_;

  std::vector<std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  bool first_time_;
  ros::ServiceClient clear_octomap_service_client_;
};

// \todo THIS IS REALLY BAD. NEED TO MOVE THIS AND RELATED FUNCTIONALITY OUT OF HERE
template <typename T>
void MotionPlanningFrame::waitForAction(const T& action, const ros::Duration& wait_for_server, const std::string& name)
{
  ROS_DEBUG("Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    // wait forever until action server connects
    while (ros::ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    // wait for a limited amount of non-simulated time
    ros::WallTime final_time = ros::WallTime::now() + ros::WallDuration(wait_for_server.toSec());
    while (ros::ok() && !action->isServerConnected() && final_time > ros::WallTime::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to move_group action server within allotted time");
  else
    ROS_DEBUG("Connected to '%s'", name.c_str());
};
}  // namespace moveit_rviz_plugin
