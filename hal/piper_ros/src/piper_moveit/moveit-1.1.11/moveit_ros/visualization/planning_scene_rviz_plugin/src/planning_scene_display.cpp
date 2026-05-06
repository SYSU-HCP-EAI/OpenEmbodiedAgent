/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/rviz_plugin_render_tools/octomap_render.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf2_ros/buffer.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>

namespace moveit_rviz_plugin
{
// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
PlanningSceneDisplay::PlanningSceneDisplay(bool listen_to_planning_scene, bool show_scene_robot)
  : Display(), planning_scene_needs_render_(true), current_scene_time_(0.0f)
{
  move_group_ns_property_ = new rviz::StringProperty("Move Group Namespace", "",
                                                     "The name of the ROS namespace in "
                                                     "which the move_group node is running",
                                                     this, SLOT(changedMoveGroupNS()), this);
  robot_description_property_ =
      new rviz::StringProperty("Robot Description", "robot_description",
                               "The name of the ROS parameter where the URDF for the robot is loaded", this,
                               SLOT(changedRobotDescription()), this);

  if (listen_to_planning_scene)
    planning_scene_topic_property_ =
        new rviz::RosTopicProperty("Planning Scene Topic", "move_group/monitored_planning_scene",
                                   ros::message_traits::datatype<moveit_msgs::PlanningScene>(),
                                   "The topic on which the moveit_msgs::PlanningScene messages are received", this,
                                   SLOT(changedPlanningSceneTopic()), this);
  else
    planning_scene_topic_property_ = nullptr;

  // Planning scene category -------------------------------------------------------------------------------------------
  scene_category_ = new rviz::Property("Scene Geometry", QVariant(), "", this);

  scene_name_property_ = new rviz::StringProperty("Scene Name", "(noname)", "Shows the name of the planning scene",
                                                  scene_category_, SLOT(changedSceneName()), this);
  scene_name_property_->setShouldBeSaved(false);
  scene_enabled_property_ =
      new rviz::BoolProperty("Show Scene Geometry", true, "Indicates whether planning scenes should be displayed",
                             scene_category_, SLOT(changedSceneEnabled()), this);

  scene_alpha_property_ = new rviz::FloatProperty("Scene Alpha", 0.9f, "Specifies the alpha for the scene geometry",
                                                  scene_category_, SLOT(changedSceneAlpha()), this);
  scene_alpha_property_->setMin(0.0);
  scene_alpha_property_->setMax(1.0);

  scene_color_property_ =
      new rviz::ColorProperty("Scene Color", QColor(50, 230, 50),
                              "The color for the planning scene obstacles (if a color is not defined)", scene_category_,
                              SLOT(changedSceneColor()), this);

  octree_render_property_ = new rviz::EnumProperty("Voxel Rendering", "Occupied Voxels", "Select voxel type.",
                                                   scene_category_, SLOT(changedOctreeRenderMode()), this);

  octree_render_property_->addOption("Occupied Voxels", OCTOMAP_OCCUPIED_VOXELS);
  octree_render_property_->addOption("Free Voxels", OCTOMAP_FREE_VOXELS);
  octree_render_property_->addOption("All Voxels", OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

  octree_coloring_property_ = new rviz::EnumProperty("Voxel Coloring", "Z-Axis", "Select voxel coloring mode",
                                                     scene_category_, SLOT(changedOctreeColorMode()), this);

  octree_coloring_property_->addOption("Z-Axis", OCTOMAP_Z_AXIS_COLOR);
  octree_coloring_property_->addOption("Cell Probability", OCTOMAP_PROBABLILTY_COLOR);

  scene_display_time_property_ = new rviz::FloatProperty("Scene Display Time", 0.01f,
                                                         "The amount of wall-time to wait in between rendering "
                                                         "updates to the planning scene (if any)",
                                                         scene_category_, SLOT(changedSceneDisplayTime()), this);
  scene_display_time_property_->setMin(0.0001);

  if (show_scene_robot)
  {
    robot_category_ = new rviz::Property("Scene Robot", QVariant(), "", this);

    scene_robot_visual_enabled_property_ =
        new rviz::BoolProperty("Show Robot Visual", true,
                               "Indicates whether the robot state specified by the planning scene should be "
                               "displayed as defined for visualisation purposes.",
                               robot_category_, SLOT(changedSceneRobotVisualEnabled()), this);

    scene_robot_collision_enabled_property_ =
        new rviz::BoolProperty("Show Robot Collision", false,
                               "Indicates whether the robot state specified by the planning scene should be "
                               "displayed as defined for collision detection purposes.",
                               robot_category_, SLOT(changedSceneRobotCollisionEnabled()), this);

    robot_alpha_property_ = new rviz::FloatProperty("Robot Alpha", 1.0f, "Specifies the alpha for the robot links",
                                                    robot_category_, SLOT(changedRobotSceneAlpha()), this);
    robot_alpha_property_->setMin(0.0);
    robot_alpha_property_->setMax(1.0);

    attached_body_color_property_ =
        new rviz::ColorProperty("Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies",
                                robot_category_, SLOT(changedAttachedBodyColor()), this);
  }
  else
  {
    robot_category_ = nullptr;
    scene_robot_visual_enabled_property_ = nullptr;
    scene_robot_collision_enabled_property_ = nullptr;
    robot_alpha_property_ = nullptr;
    attached_body_color_property_ = nullptr;
  }
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
PlanningSceneDisplay::~PlanningSceneDisplay()
{
  clearJobs();

  planning_scene_render_.reset();
  if (context_ && context_->getSceneManager() && planning_scene_node_)
    context_->getSceneManager()->destroySceneNode(planning_scene_node_);
  planning_scene_robot_.reset();
  planning_scene_monitor_.reset();
}

void PlanningSceneDisplay::clearJobs()
{
  background_process_.clear();
  {
    boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
    main_loop_jobs_.clear();
  }
}

void PlanningSceneDisplay::onInitialize()
{
  Display::onInitialize();

  // the scene node that contains everything and is located at the planning frame
  planning_scene_node_ = scene_node_->createChildSceneNode();

  if (robot_category_)
  {
    planning_scene_robot_ =
        std::make_shared<RobotStateVisualization>(planning_scene_node_, context_, "Planning Scene", robot_category_);
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    changedRobotSceneAlpha();
    changedAttachedBodyColor();
  }
}

void PlanningSceneDisplay::reset()
{
  if (planning_scene_robot_)
    planning_scene_robot_->clear();
  Display::reset();

  if (isEnabled())
    addBackgroundJob([this] { loadRobotModel(); }, "loadRobotModel");

  if (planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
  }
}

void PlanningSceneDisplay::addBackgroundJob(const boost::function<void()>& job, const std::string& name)
{
  background_process_.addJob(job, name);
}

void PlanningSceneDisplay::spawnBackgroundJob(const boost::function<void()>& job)
{
  boost::thread t(job);
}

void PlanningSceneDisplay::addMainLoopJob(const boost::function<void()>& job)
{
  boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void PlanningSceneDisplay::waitForAllMainLoopJobs()
{
  boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
  while (!main_loop_jobs_.empty())
    main_loop_jobs_empty_condition_.wait(ulock);
}

void PlanningSceneDisplay::executeMainLoopJobs()
{
  main_loop_jobs_lock_.lock();
  while (!main_loop_jobs_.empty())
  {
    boost::function<void()> fn = main_loop_jobs_.front();
    main_loop_jobs_.pop_front();
    main_loop_jobs_lock_.unlock();
    try
    {
      fn();
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
    }
    main_loop_jobs_lock_.lock();
  }
  main_loop_jobs_empty_condition_.notify_all();
  main_loop_jobs_lock_.unlock();
}

const planning_scene_monitor::PlanningSceneMonitorPtr& PlanningSceneDisplay::getPlanningSceneMonitor()
{
  return planning_scene_monitor_;
}

const std::string PlanningSceneDisplay::getMoveGroupNS() const
{
  return move_group_ns_property_->getStdString();
}

const moveit::core::RobotModelConstPtr& PlanningSceneDisplay::getRobotModel() const
{
  if (planning_scene_monitor_)
    return planning_scene_monitor_->getRobotModel();
  else
  {
    static moveit::core::RobotModelConstPtr empty;
    return empty;
  }
}

bool PlanningSceneDisplay::waitForCurrentRobotState(const ros::Time& t)
{
  if (planning_scene_monitor_)
    return planning_scene_monitor_->waitForCurrentRobotState(t);
  return false;
}

planning_scene_monitor::LockedPlanningSceneRO PlanningSceneDisplay::getPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

planning_scene_monitor::LockedPlanningSceneRW PlanningSceneDisplay::getPlanningSceneRW()
{
  return planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
}

void PlanningSceneDisplay::changedAttachedBodyColor()
{
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedSceneColor()
{
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedMoveGroupNS()
{
  if (isEnabled())
    reset();
}

void PlanningSceneDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void PlanningSceneDisplay::changedSceneName()
{
  planning_scene_monitor::LockedPlanningSceneRW ps = getPlanningSceneRW();
  if (ps)
    ps->setName(scene_name_property_->getStdString());
}

void PlanningSceneDisplay::renderPlanningScene()
{
  QColor color = scene_color_property_->getColor();
  rviz::Color env_color(color.redF(), color.greenF(), color.blueF());
  if (attached_body_color_property_)
    color = attached_body_color_property_->getColor();
  rviz::Color attached_color(color.redF(), color.greenF(), color.blueF());

  try
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = getPlanningSceneRO();
    if (planning_scene_needs_render_)
    {
      planning_scene_render_->renderPlanningScene(
          ps, env_color, attached_color, static_cast<OctreeVoxelRenderMode>(octree_render_property_->getOptionInt()),
          static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt()),
          scene_alpha_property_->getFloat());
    }
    else
    {
      planning_scene_render_->updateRobotPosition(ps);
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("Caught %s while rendering planning scene", ex.what());
  }
  planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
}

void PlanningSceneDisplay::changedSceneAlpha()
{
  queueRenderSceneGeometry();
}

void PlanningSceneDisplay::changedRobotSceneAlpha()
{
  if (planning_scene_robot_)
  {
    planning_scene_robot_->setAlpha(robot_alpha_property_->getFloat());
    queueRenderSceneGeometry();
  }
}

void PlanningSceneDisplay::changedPlanningSceneTopic()
{
  if (planning_scene_monitor_ && planning_scene_topic_property_)
  {
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    std::string service_name = planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE;
    if (!getMoveGroupNS().empty())
      service_name = ros::names::append(getMoveGroupNS(), service_name);
    auto bg_func = [=]() {
      if (planning_scene_monitor_->requestPlanningSceneState(service_name))
        addMainLoopJob([this] { onNewPlanningSceneState(); });
      else
        setStatus(rviz::StatusProperty::Warn, "PlanningScene", "Requesting initial scene failed");
    };
    addBackgroundJob(bg_func, "requestPlanningSceneState");
  }
}

void PlanningSceneDisplay::changedSceneDisplayTime()
{
}

void PlanningSceneDisplay::changedOctreeRenderMode()
{
}

void PlanningSceneDisplay::changedOctreeColorMode()
{
}

void PlanningSceneDisplay::changedSceneRobotVisualEnabled()
{
  if (isEnabled() && planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_needs_render_ = true;
  }
}

void PlanningSceneDisplay::changedSceneRobotCollisionEnabled()
{
  if (isEnabled() && planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    planning_scene_needs_render_ = true;
  }
}

void PlanningSceneDisplay::changedSceneEnabled()
{
  if (planning_scene_render_)
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
}

void PlanningSceneDisplay::setGroupColor(rviz::Robot* robot, const std::string& group_name, const QColor& color)
{
  if (getRobotModel())
  {
    const moveit::core::JointModelGroup* jmg = getRobotModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string>& links = jmg->getLinkModelNamesWithCollisionGeometry();
      for (const std::string& link : links)
        setLinkColor(robot, link, color);
    }
  }
}

void PlanningSceneDisplay::unsetAllColors(rviz::Robot* robot)
{
  if (getRobotModel())
  {
    const std::vector<std::string>& links = getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (const std::string& link : links)
      unsetLinkColor(robot, link);
  }
}

void PlanningSceneDisplay::unsetGroupColor(rviz::Robot* robot, const std::string& group_name)
{
  if (getRobotModel())
  {
    const moveit::core::JointModelGroup* jmg = getRobotModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string>& links = jmg->getLinkModelNamesWithCollisionGeometry();
      for (const std::string& link : links)
        unsetLinkColor(robot, link);
    }
  }
}

void PlanningSceneDisplay::setLinkColor(const std::string& link_name, const QColor& color)
{
  if (planning_scene_robot_)
    setLinkColor(&planning_scene_robot_->getRobot(), link_name, color);
}

void PlanningSceneDisplay::unsetLinkColor(const std::string& link_name)
{
  if (planning_scene_robot_)
    unsetLinkColor(&planning_scene_robot_->getRobot(), link_name);
}

void PlanningSceneDisplay::setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor& color)
{
  rviz::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->setColor(color.redF(), color.greenF(), color.blueF());
}

void PlanningSceneDisplay::unsetLinkColor(rviz::Robot* robot, const std::string& link_name)
{
  rviz::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->unsetColor();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
planning_scene_monitor::PlanningSceneMonitorPtr PlanningSceneDisplay::createPlanningSceneMonitor()
{
#ifdef RVIZ_TF1
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = moveit::planning_interface::getSharedTF();
#else
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = context_->getFrameManager()->getTF2BufferPtr();
#endif
  return std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_description_property_->getStdString(), tf_buffer, getNameStd() + "_planning_scene_monitor");
}

void PlanningSceneDisplay::clearRobotModel()
{
  planning_scene_render_.reset();
  // Ensure old PSM is destroyed before we attempt to create a new one
  planning_scene_monitor_.reset();
}

void PlanningSceneDisplay::loadRobotModel()
{
  // wait for other robot loadRobotModel() calls to complete;
  boost::mutex::scoped_lock _(robot_model_loading_lock_);

  // we need to make sure the clearing of the robot model is in the main thread,
  // so that rendering operations do not have data removed from underneath,
  // so the next function is executed in the main loop
  addMainLoopJob([this] { clearRobotModel(); });

  waitForAllMainLoopJobs();

  planning_scene_monitor::PlanningSceneMonitorPtr psm = createPlanningSceneMonitor();
  if (psm->getPlanningScene())
  {
    planning_scene_monitor_.swap(psm);
    planning_scene_monitor_->addUpdateCallback(
        [this](planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type) {
          sceneMonitorReceivedUpdate(type);
        });
    addMainLoopJob([this] { onRobotModelLoaded(); });
    waitForAllMainLoopJobs();
  }
  else
  {
    addMainLoopJob([this]() { setStatus(rviz::StatusProperty::Error, "PlanningScene", "No Planning Scene Loaded"); });
  }
}

// This should always run in the main GUI thread!
void PlanningSceneDisplay::onRobotModelLoaded()
{
  changedPlanningSceneTopic();
  planning_scene_render_ = std::make_shared<PlanningSceneRender>(planning_scene_node_, context_, planning_scene_robot_);
  planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

  const planning_scene_monitor::LockedPlanningSceneRO& ps = getPlanningSceneRO();
  if (planning_scene_robot_)
  {
    planning_scene_robot_->load(*getRobotModel()->getURDF());
    moveit::core::RobotState* rs = new moveit::core::RobotState(ps->getCurrentState());
    rs->update();
    planning_scene_robot_->update(moveit::core::RobotStateConstPtr(rs));
  }

  bool old_state = scene_name_property_->blockSignals(true);
  scene_name_property_->setStdString(ps->getName());
  scene_name_property_->blockSignals(old_state);

  setStatus(rviz::StatusProperty::Ok, "PlanningScene", "Planning Scene Loaded Successfully");
}

void PlanningSceneDisplay::onNewPlanningSceneState()
{
}

void PlanningSceneDisplay::sceneMonitorReceivedUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  onSceneMonitorReceivedUpdate(update_type);
}

void PlanningSceneDisplay::onSceneMonitorReceivedUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType /*update_type*/)
{
  getPlanningSceneRW()->getCurrentStateNonConst().update();
  QMetaObject::invokeMethod(this, "setSceneName", Qt::QueuedConnection,
                            Q_ARG(QString, QString::fromStdString(getPlanningSceneRO()->getName())));
  planning_scene_needs_render_ = true;
}

void PlanningSceneDisplay::setSceneName(const QString& name)
{
  scene_name_property_->setString(name);
}

void PlanningSceneDisplay::onEnable()
{
  Display::onEnable();

  addBackgroundJob([this] { loadRobotModel(); }, "loadRobotModel");

  if (planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
  }
  if (planning_scene_render_)
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

  calculateOffsetPosition();
  planning_scene_needs_render_ = true;
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void PlanningSceneDisplay::onDisable()
{
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->stopSceneMonitor();
    if (planning_scene_render_)
      planning_scene_render_->getGeometryNode()->setVisible(false);
  }
  if (planning_scene_robot_)
    planning_scene_robot_->setVisible(false);
  Display::onDisable();
}

void PlanningSceneDisplay::queueRenderSceneGeometry()
{
  planning_scene_needs_render_ = true;
}

void PlanningSceneDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  executeMainLoopJobs();

  calculateOffsetPosition();

  if (planning_scene_monitor_)
    updateInternal(wall_dt, ros_dt);
}

void PlanningSceneDisplay::updateInternal(float wall_dt, float /*ros_dt*/)
{
  current_scene_time_ += wall_dt;
  if (planning_scene_render_ &&
      ((current_scene_time_ > scene_display_time_property_->getFloat() && robot_state_needs_render_) ||
       planning_scene_needs_render_))
  {
    renderPlanningScene();
    current_scene_time_ = 0.0f;
    robot_state_needs_render_ = false;
    planning_scene_needs_render_ = false;
  }
}

void PlanningSceneDisplay::load(const rviz::Config& config)
{
  Display::load(config);
}

void PlanningSceneDisplay::save(rviz::Config config) const
{
  Display::save(config);
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void PlanningSceneDisplay::calculateOffsetPosition()
{
  if (!getRobotModel())
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(getRobotModel()->getModelFrame(), ros::Time(0), position, orientation);

  planning_scene_node_->setPosition(position);
  planning_scene_node_->setOrientation(orientation);
}

void PlanningSceneDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}

}  // namespace moveit_rviz_plugin
