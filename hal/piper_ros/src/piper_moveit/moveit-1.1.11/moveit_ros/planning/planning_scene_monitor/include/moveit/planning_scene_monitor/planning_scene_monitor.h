/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/buffer.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <boost/noncopyable.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <memory>

namespace planning_scene_monitor
{
MOVEIT_CLASS_FORWARD(PlanningSceneMonitor);  // Defines PlanningSceneMonitorPtr, ConstPtr, WeakPtr... etc

/**
 * @brief PlanningSceneMonitor
 * Subscribes to the topic \e planning_scene */
class PlanningSceneMonitor : private boost::noncopyable
{
public:
  enum SceneUpdateType
  {
    /** \brief No update */
    UPDATE_NONE = 0,

    /** \brief The state in the monitored scene was updated */
    UPDATE_STATE = 1,

    /** \brief The maintained set of fixed transforms in the monitored scene was updated */
    UPDATE_TRANSFORMS = 2,

    /** \brief The geometry of the scene was updated. This includes receiving new octomaps, collision objects, attached
       objects, scene geometry, etc. */
    UPDATE_GEOMETRY = 4,

    /** \brief The entire scene was updated */
    UPDATE_SCENE = 8 + UPDATE_STATE + UPDATE_TRANSFORMS + UPDATE_GEOMETRY
  };

  /// The name of the topic used by default for receiving joint states
  static const std::string DEFAULT_JOINT_STATES_TOPIC;  // "/joint_states"

  /// The name of the topic used by default for attached collision objects
  static const std::string DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC;  // "/attached_collision_object"

  /// The name of the topic used by default for receiving collision objects in the world
  static const std::string DEFAULT_COLLISION_OBJECT_TOPIC;  // "/collision_object"

  /// The name of the topic used by default for receiving geometry information about a planning scene (complete
  /// overwrite of world geometry)
  static const std::string DEFAULT_PLANNING_SCENE_WORLD_TOPIC;  // "/planning_scene_world"

  /// The name of the topic used by default for receiving full planning scenes or planning scene diffs
  static const std::string DEFAULT_PLANNING_SCENE_TOPIC;  // "/planning_scene"

  /// The name of the service used by default for requesting full planning scene state
  static const std::string DEFAULT_PLANNING_SCENE_SERVICE;  // "/get_planning_scene"

  /// The name of the topic used by default for publishing the monitored planning scene (this is without "/" in the
  /// name, so the topic is prefixed by the node name)
  static const std::string MONITORED_PLANNING_SCENE_TOPIC;  // "monitored_planning_scene"

  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf_buffer A pointer to a tf2_ros::Buffer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const std::string& robot_description,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
                       const std::string& name = "");

  /** @brief Constructor
   *  @param rml A pointer to a kinematic model loader
   *  @param tf_buffer A pointer to a tf2_ros::Buffer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const robot_model_loader::RobotModelLoaderPtr& rml,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
                       const std::string& name = "");

  /** @brief Constructor
   *  @param scene The scene instance to maintain up to date with monitored information
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf_buffer A pointer to a tf2_ros::Buffer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const planning_scene::PlanningScenePtr& scene, const std::string& robot_description,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
                       const std::string& name = "");

  /** @brief Constructor
   *  @param scene The scene instance to maintain up to date with monitored information
   *  @param rml A pointer to a kinematic model loader
   *  @param tf_buffer A pointer to a tf2_ros::Buffer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const planning_scene::PlanningScenePtr& scene,
                       const robot_model_loader::RobotModelLoaderPtr& rml,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
                       const std::string& name = "");

  /** @brief Constructor
   *  @param scene The scene instance to maintain up to date with monitored information
   *  @param rml A pointer to a kinematic model loader
   *  @param nh external parent NodeHandle
   *         The monitors will use this NodeHandle's CallbackQueue for updates.
   *         Usually, this should be a different queue than the global queue, otherwise you might run into timeouts.
   *  @param tf_buffer A pointer to a tf2_ros::Buffer
   *  @param name A name identifying this planning scene monitor
   */
  PlanningSceneMonitor(const planning_scene::PlanningScenePtr& scene,
                       const robot_model_loader::RobotModelLoaderPtr& rml, const ros::NodeHandle& nh,
                       const std::shared_ptr<tf2_ros::Buffer>& tf_buffer = std::shared_ptr<tf2_ros::Buffer>(),
                       const std::string& name = "");

  ~PlanningSceneMonitor();

  /** \brief Get the name of this monitor */
  const std::string& getName() const
  {
    return monitor_name_;
  }

  /** \brief Get the user kinematic model loader */
  const robot_model_loader::RobotModelLoaderPtr& getRobotModelLoader() const
  {
    return rm_loader_;
  }

  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** @brief <b>Avoid this function!</b>  Returns an @b
   *         unsafe pointer to the current planning scene.
   * @warning Most likely you do not want to call this function
   *          directly.  PlanningSceneMonitor has a background thread
   *          which repeatedly updates and clobbers various contents
   *          of its internal PlanningScene instance.  This function
   *          just returns a pointer to that dynamic internal object.
   *          The correct thing is usually to use a
   *          LockedPlanningSceneRO or LockedPlanningSceneRW, which
   *          locks the PlanningSceneMonitor and provides safe access
   *          to the PlanningScene object.
   * @see LockedPlanningSceneRO
   * @see LockedPlanningSceneRW.
   * @return A pointer to the current planning scene.*/
  const planning_scene::PlanningScenePtr& getPlanningScene()
  {
    return scene_;
  }

  /*! @brief <b>Avoid this function!</b>  Returns an @b
   *         unsafe pointer to the current planning scene.
   * @copydetails PlanningSceneMonitor::getPlanningScene() */
  const planning_scene::PlanningSceneConstPtr& getPlanningScene() const
  {
    return scene_const_;
  }

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesScene(const planning_scene::PlanningSceneConstPtr& scene) const;

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesScene(const planning_scene::PlanningScenePtr& scene) const;

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getRobotDescription() const
  {
    return robot_description_;
  }

  /// Get the default robot padding
  double getDefaultRobotPadding() const
  {
    return default_robot_padd_;
  }

  /// Get the default robot scaling
  double getDefaultRobotScale() const
  {
    return default_robot_scale_;
  }

  /// Get the default object padding
  double getDefaultObjectPadding() const
  {
    return default_object_padd_;
  }

  /// Get the default attached padding
  double getDefaultAttachedObjectPadding() const
  {
    return default_attached_padd_;
  }

  /** @brief Get the instance of the TF client that was passed to the constructor of this class. */
  const std::shared_ptr<tf2_ros::Buffer>& getTFClient() const
  {
    return tf_buffer_;
  }

  /** \brief By default, the maintained planning scene does not reason about diffs. When the flag passed in is true, the
     maintained
      scene starts counting diffs. Future updates to the planning scene will be stored as diffs and can be retrieved as
      such. Setting the flag to false restores the default behaviour. Maintaining diffs is automatically enabled when
      publishing planning scenes. */
  void monitorDiffs(bool flag);

  /** \brief Start publishing the maintained planning scene. The first message set out is a complete planning scene.
      Diffs are sent afterwards on updates specified by the \e event bitmask. For UPDATE_SCENE, the full scene is always
     sent. */
  void startPublishingPlanningScene(SceneUpdateType event,
                                    const std::string& planning_scene_topic = MONITORED_PLANNING_SCENE_TOPIC);

  /** \brief Stop publishing the maintained planning scene. */
  void stopPublishingPlanningScene();

  /** \brief Set the maximum frequency at which planning scenes are being published */
  void setPlanningScenePublishingFrequency(double hz);

  /** \brief Get the maximum frequency at which planning scenes are published (Hz) */
  double getPlanningScenePublishingFrequency() const
  {
    return publish_planning_scene_frequency_;
  }

  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  const CurrentStateMonitorPtr& getStateMonitor() const
  {
    return current_state_monitor_;
  }

  CurrentStateMonitorPtr& getStateMonitorNonConst()
  {
    return current_state_monitor_;
  }

  /** @brief Update the transforms for the frames that are not part of the kinematic model using tf.
   *  Examples of these frames are the "map" and "odom_combined" transforms. This function is automatically called when
   * data that uses transforms is received.
   *  However, this function should also be called before starting a planning request, for example.
   */
  void updateFrameTransforms();

  /** @brief Start the current state monitor
      @param joint_states_topic the topic to listen to for joint states
      @param attached_objects_topic the topic to listen to for attached collision objects */
  void startStateMonitor(const std::string& joint_states_topic = DEFAULT_JOINT_STATES_TOPIC,
                         const std::string& attached_objects_topic = DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC);

  /** @brief Stop the state monitor*/
  void stopStateMonitor();

  /** @brief Update the scene using the monitored state. This function is automatically called when an update to the
     current state is received (if startStateMonitor() has been called).
      The updates are throttled to a maximum update frequency however, which is set by setStateUpdateFrequency(). */
  void updateSceneWithCurrentState();

  /** @brief Update the scene using the monitored state at a specified frequency, in Hz. This function has an effect
     only when updates from the CurrentStateMonitor are received at a higher frequency.
      In that case, the updates are throttled down, so that they do not exceed a maximum update frequency specified
     here.
      @param hz the update frequency. By default this is 10Hz. */
  void setStateUpdateFrequency(double hz);

  /** @brief Get the maximum frequency (Hz) at which the current state of the planning scene is updated.*/
  double getStateUpdateFrequency() const
  {
    if (!dt_state_update_.isZero())
      return 1.0 / dt_state_update_.toSec();
    else
      return 0.0;
  }

  /** @brief Start the scene monitor (ROS topic-based)
   *  @param scene_topic The name of the planning scene topic
   */
  void startSceneMonitor(const std::string& scene_topic = DEFAULT_PLANNING_SCENE_TOPIC);

  /** @brief Request a full planning scene state using a service call
   *         Be careful not to use this in conjunction with providePlanningSceneService(),
   *         as it will create a pointless feedback loop.
   *  @param service_name The name of the service to use for requesting the planning scene.
   *         This must be a service of type moveit_msgs::GetPlanningScene and is usually called
   *         "/get_planning_scene".
   */
  bool requestPlanningSceneState(const std::string& service_name = DEFAULT_PLANNING_SCENE_SERVICE);

  /** @brief Create an optional service for getting the complete planning scene
   *         This is useful for satisfying the Rviz PlanningScene display's need for a service
   *         without having to use a move_group node.
   *         Be careful not to use this in conjunction with requestPlanningSceneState(),
   *         as it will create a pointless feedback loop.
   *  @param service_name The topic to provide the service
   */
  void providePlanningSceneService(const std::string& service_name = DEFAULT_PLANNING_SCENE_SERVICE);

  /** @brief Stop the scene monitor*/
  void stopSceneMonitor();

  /** @brief Start the OccupancyMapMonitor and listening for:
   *     - Requests to add/remove/update collision objects to/from the world
   *     - The collision map
   *     - Requests to attached/detach collision objects
   *  @param collision_objects_topic The topic on which to listen for collision objects
   *  @param planning_scene_world_topic The topic to listen to for world scene geometry
   *  @param load_octomap_monitor Flag to disable octomap monitor if desired
   */
  void startWorldGeometryMonitor(const std::string& collision_objects_topic = DEFAULT_COLLISION_OBJECT_TOPIC,
                                 const std::string& planning_scene_world_topic = DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                 const bool load_octomap_monitor = true);

  /** @brief Stop the world geometry monitor */
  void stopWorldGeometryMonitor();

  /** @brief Add a function to be called when an update to the scene is received */
  void addUpdateCallback(const boost::function<void(SceneUpdateType)>& fn);

  /** @brief Clear the functions to be called when an update to the scene is received */
  void clearUpdateCallbacks();

  /** @brief Get the topic names that the monitor is listening to */
  void getMonitoredTopics(std::vector<std::string>& topics) const;

  /** \brief Return the time when the last update was made to the planning scene (by \e any monitor) */
  const ros::Time& getLastUpdateTime() const
  {
    return last_update_time_;
  }

  void publishDebugInformation(bool flag);

  /** @brief This function is called every time there is a change to the planning scene */
  void triggerSceneUpdateEvent(SceneUpdateType update_type);

  /** \brief Wait for robot state to become more recent than time t.
   *
   * If there is no state monitor active, there will be no scene updates.
   * Hence, you can specify a timeout to wait for those updates. Default is 1s.
   */
  bool waitForCurrentRobotState(const ros::Time& t, double wait_time = 1.);

  /** \brief Lock the scene for reading (multiple threads can lock for reading at the same time) */
  void lockSceneRead();

  /** \brief Unlock the scene from reading (multiple threads can lock for reading at the same time) */
  void unlockSceneRead();

  /** \brief Lock the scene for writing (only one thread can lock for writing and no other thread can lock for reading)
   */
  void lockSceneWrite();

  /** \brief Lock the scene from writing (only one thread can lock for writing and no other thread can lock for reading)
   */
  void unlockSceneWrite();

  void clearOctomap();

  // Called to update the planning scene with a new message.
  bool newPlanningSceneMessage(const moveit_msgs::PlanningScene& scene);

protected:
  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated)
   */
  void initialize(const planning_scene::PlanningScenePtr& scene);

  /** @brief Configure the collision matrix for a particular scene */
  void configureCollisionMatrix(const planning_scene::PlanningScenePtr& scene);

  /** @brief Configure the default padding*/
  void configureDefaultPadding();

  /** @brief Callback for a new collision object msg*/
  void collisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr& obj);

  /** @brief Callback for a new planning scene world*/
  void newPlanningSceneWorldCallback(const moveit_msgs::PlanningSceneWorldConstPtr& world);

  /** @brief Callback for octomap updates */
  void octomapUpdateCallback();

  /** @brief Callback for a new attached object msg*/
  void attachObjectCallback(const moveit_msgs::AttachedCollisionObjectConstPtr& obj);

  /** @brief Callback for a change for an attached object of the current state of the planning scene */
  void currentStateAttachedBodyUpdateCallback(moveit::core::AttachedBody* attached_body, bool just_attached);

  /** @brief Callback for a change in the world maintained by the planning scene */
  void currentWorldObjectUpdateCallback(const collision_detection::World::ObjectConstPtr& object,
                                        collision_detection::World::Action action);

  void includeRobotLinksInOctree();
  void excludeRobotLinksFromOctree();

  void excludeWorldObjectsFromOctree();
  void includeWorldObjectsInOctree();
  void excludeWorldObjectFromOctree(const collision_detection::World::ObjectConstPtr& obj);
  void includeWorldObjectInOctree(const collision_detection::World::ObjectConstPtr& obj);

  void excludeAttachedBodiesFromOctree();
  void includeAttachedBodiesInOctree();
  void excludeAttachedBodyFromOctree(const moveit::core::AttachedBody* attached_body);
  void includeAttachedBodyInOctree(const moveit::core::AttachedBody* attached_body);

  bool getShapeTransformCache(const std::string& target_frame, const ros::Time& target_time,
                              occupancy_map_monitor::ShapeTransformCache& cache) const;

  /// The name of this scene monitor
  std::string monitor_name_;

  planning_scene::PlanningScenePtr scene_;
  planning_scene::PlanningSceneConstPtr scene_const_;
  planning_scene::PlanningScenePtr parent_scene_;  /// if diffs are monitored, this is the pointer to the parent scene
  boost::shared_mutex scene_update_mutex_;         /// mutex for stored scene
  ros::Time last_update_time_;                     /// Last time the state was updated
  ros::Time last_robot_motion_time_;               /// Last time the robot has moved

  ros::NodeHandle nh_;
  ros::NodeHandle root_nh_;
  ros::CallbackQueue queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string robot_description_;

  /// default robot padding
  double default_robot_padd_;
  /// default robot scaling
  double default_robot_scale_;
  /// default object padding
  double default_object_padd_;
  /// default attached padding
  double default_attached_padd_;
  /// default robot link padding
  std::map<std::string, double> default_robot_link_padd_;
  /// default robot link scale
  std::map<std::string, double> default_robot_link_scale_;

  // variables for planning scene publishing
  ros::Publisher planning_scene_publisher_;
  std::unique_ptr<boost::thread> publish_planning_scene_;
  double publish_planning_scene_frequency_;
  SceneUpdateType publish_update_types_;
  SceneUpdateType new_scene_update_;
  boost::condition_variable_any new_scene_update_condition_;

  // subscribe to various sources of data
  ros::Subscriber planning_scene_subscriber_;
  ros::Subscriber planning_scene_world_subscriber_;

  ros::Subscriber attached_collision_object_subscriber_;
  ros::Subscriber collision_object_subscriber_;

  // provide an optional service to get the full planning scene state
  // this is used by MoveGroup and related application nodes
  ros::ServiceServer get_scene_service_;

  // include a octomap monitor
  std::unique_ptr<occupancy_map_monitor::OccupancyMapMonitor> octomap_monitor_;

  // include a current state monitor
  CurrentStateMonitorPtr current_state_monitor_;

  typedef std::map<const moveit::core::LinkModel*,
                   std::vector<std::pair<occupancy_map_monitor::ShapeHandle, std::size_t> > >
      LinkShapeHandles;
  using AttachedBodyShapeHandles = std::map<const moveit::core::AttachedBody*,
                                            std::vector<std::pair<occupancy_map_monitor::ShapeHandle, std::size_t> > >;
  using CollisionBodyShapeHandles =
      std::map<std::string, std::vector<std::pair<occupancy_map_monitor::ShapeHandle, const Eigen::Isometry3d*> > >;

  LinkShapeHandles link_shape_handles_;
  AttachedBodyShapeHandles attached_body_shape_handles_;
  CollisionBodyShapeHandles collision_body_shape_handles_;
  mutable boost::recursive_mutex shape_handles_lock_;

  /// lock access to update_callbacks_
  boost::recursive_mutex update_lock_;
  std::vector<boost::function<void(SceneUpdateType)> > update_callbacks_;  /// List of callbacks to trigger when updates
                                                                           /// are received

private:
  void getUpdatedFrameTransforms(std::vector<geometry_msgs::TransformStamped>& transforms);

  // publish planning scene update diffs (runs in its own thread)
  void scenePublishingThread();

  // called by current_state_monitor_ when robot state (as monitored on joint state topic) changes
  void onStateUpdate(const sensor_msgs::JointStateConstPtr& joint_state);

  // called by state_update_timer_ when a state update it pending
  void stateUpdateTimerCallback(const ros::WallTimerEvent& event);

  // Callback for a new planning scene msg
  void newPlanningSceneCallback(const moveit_msgs::PlanningSceneConstPtr& scene);

  // Callback for requesting the full planning scene via service
  bool getPlanningSceneServiceCallback(moveit_msgs::GetPlanningScene::Request& req,
                                       moveit_msgs::GetPlanningScene::Response& res);

  // Lock for state_update_pending_ and dt_state_update_
  boost::mutex state_pending_mutex_;

  /// True when we need to update the RobotState from current_state_monitor_
  // This field is protected by state_pending_mutex_
  volatile bool state_update_pending_;

  /// the amount of time to wait in between updates to the robot state
  // This field is protected by state_pending_mutex_
  ros::WallDuration dt_state_update_;

  /// the amount of time to wait when looking up transforms
  // Setting this to a non-zero value resolves issues when the sensor data is
  // arriving so fast that it is preceding the transform state.
  ros::Duration shape_transform_cache_lookup_wait_time_;

  /// timer for state updates.
  // Check if last_state_update_ is true and if so call updateSceneWithCurrentState()
  // Not safe to access from callback functions.
  ros::WallTimer state_update_timer_;

  /// Last time the state was updated from current_state_monitor_
  // Only access this from callback functions (and constructor)
  ros::WallTime last_robot_state_update_wall_time_;

  robot_model_loader::RobotModelLoaderPtr rm_loader_;
  moveit::core::RobotModelConstPtr robot_model_;

  collision_detection::CollisionPluginLoader collision_loader_;

  class DynamicReconfigureImpl;
  DynamicReconfigureImpl* reconfigure_impl_;
};

/** \brief This is a convenience class for obtaining access to an
 *         instance of a locked PlanningScene.
 *
 * Instances of this class can be used almost exactly like instances
 * of a PlanningScenePtr because of the typecast operator and
 * "operator->" functions.  Therefore you will often see code like this:
 * @code
 *   planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
 *   moveit::core::RobotModelConstPtr model = ls->getRobotModel();
 * @endcode

 * The function "getRobotModel()" is a member of PlanningScene and not
 * a member of this class.  However because of the "operator->" here
 * which returns a PlanningSceneConstPtr, this works.
 *
 * Any number of these "ReadOnly" locks can exist at a given time.
 * The intention is that users which only need to read from the
 * PlanningScene will use these and will thus not interfere with each
 * other.
 *
 * @see LockedPlanningSceneRW */
class LockedPlanningSceneRO
{
public:
  LockedPlanningSceneRO(const PlanningSceneMonitorPtr& planning_scene_monitor)
    : planning_scene_monitor_(planning_scene_monitor)
  {
    initialize(true);
  }

  const PlanningSceneMonitorPtr& getPlanningSceneMonitor()
  {
    return planning_scene_monitor_;
  }

  operator bool() const
  {
    return planning_scene_monitor_ && planning_scene_monitor_->getPlanningScene();
  }

  operator const planning_scene::PlanningSceneConstPtr &() const
  {
    return static_cast<const PlanningSceneMonitor*>(planning_scene_monitor_.get())->getPlanningScene();
  }

  const planning_scene::PlanningSceneConstPtr& operator->() const
  {
    return static_cast<const PlanningSceneMonitor*>(planning_scene_monitor_.get())->getPlanningScene();
  }

protected:
  LockedPlanningSceneRO(const PlanningSceneMonitorPtr& planning_scene_monitor, bool read_only)
    : planning_scene_monitor_(planning_scene_monitor)
  {
    initialize(read_only);
  }

  void initialize(bool read_only)
  {
    if (planning_scene_monitor_)
      lock_ = std::make_shared<SingleUnlock>(planning_scene_monitor_.get(), read_only);
  }

  MOVEIT_STRUCT_FORWARD(SingleUnlock);

  // we use this struct so that lock/unlock are called only once
  // even if the LockedPlanningScene instance is copied around
  struct SingleUnlock
  {
    SingleUnlock(PlanningSceneMonitor* planning_scene_monitor, bool read_only)
      : planning_scene_monitor_(planning_scene_monitor), read_only_(read_only)
    {
      if (read_only)
        planning_scene_monitor_->lockSceneRead();
      else
        planning_scene_monitor_->lockSceneWrite();
    }
    ~SingleUnlock()
    {
      if (read_only_)
        planning_scene_monitor_->unlockSceneRead();
      else
        planning_scene_monitor_->unlockSceneWrite();
    }
    PlanningSceneMonitor* planning_scene_monitor_;
    bool read_only_;
  };

  PlanningSceneMonitorPtr planning_scene_monitor_;
  SingleUnlockPtr lock_;
};

/** \brief This is a convenience class for obtaining access to an
 *         instance of a locked PlanningScene.
 *
 * Instances of this class can be used almost exactly like instances
 * of a PlanningScenePtr because of the typecast operator and
 * "operator->" functions.  Therefore you will often see code like this:
 * @code
 *   planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
 *   moveit::core::RobotModelConstPtr model = ls->getRobotModel();
 * @endcode

 * The function "getRobotModel()" is a member of PlanningScene and not
 * a member of this class.  However because of the "operator->" here
 * which returns a PlanningScenePtr, this works.
 *
 * Only one of these "ReadWrite" locks can exist at a given time.  The
 * intention is that users which need to write to the PlanningScene
 * will use these, preventing other writers and readers from locking
 * the same PlanningScene at the same time.
 *
 * @see LockedPlanningSceneRO */
class LockedPlanningSceneRW : public LockedPlanningSceneRO
{
public:
  LockedPlanningSceneRW(const PlanningSceneMonitorPtr& planning_scene_monitor)
    : LockedPlanningSceneRO(planning_scene_monitor, false)
  {
  }

  operator const planning_scene::PlanningScenePtr &()
  {
    return planning_scene_monitor_->getPlanningScene();
  }

  const planning_scene::PlanningScenePtr& operator->()
  {
    return planning_scene_monitor_->getPlanningScene();
  }
};
}  // namespace planning_scene_monitor
