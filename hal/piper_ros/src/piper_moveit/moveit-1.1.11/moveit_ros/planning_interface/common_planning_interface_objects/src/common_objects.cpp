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

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_ros/transform_listener.h>

using namespace planning_scene_monitor;
using namespace robot_model_loader;

namespace
{
struct SharedStorage
{
  SharedStorage()
  {
  }

  ~SharedStorage()
  {
    tf_buffer_.reset();
    state_monitors_.clear();
    models_.clear();
  }

  boost::mutex lock_;
  std::weak_ptr<tf2_ros::Buffer> tf_buffer_;
  std::map<std::string, moveit::core::RobotModelWeakPtr> models_;
  std::map<std::string, CurrentStateMonitorWeakPtr> state_monitors_;
};

SharedStorage& getSharedStorage()
{
#if 0  // destruction of static storage interferes with static destruction in class_loader
  // More specifically, class_loader's static variables might be already destroyed
  // while being accessed again in the destructor of the class_loader-based kinematics plugin.
  static SharedStorage storage;
  return storage;
#else  // thus avoid destruction at all (until class_loader is fixed)
  static SharedStorage* storage = new SharedStorage;
  return *storage;
#endif
}

// Deleter that, additionally to T*, deletes another object too
template <typename T, typename O>
struct CoupledDeleter
{
  const O* other_;
  CoupledDeleter(const O* other = nullptr) : other_(other)
  {
  }

  void operator()(const T* p)
  {
    delete other_;
    delete p;
  }
};
}  // namespace

namespace moveit
{
namespace planning_interface
{
std::shared_ptr<tf2_ros::Buffer> getSharedTF()
{
  SharedStorage& s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);

  typedef CoupledDeleter<tf2_ros::Buffer, tf2_ros::TransformListener> Deleter;
  std::shared_ptr<tf2_ros::Buffer> buffer = s.tf_buffer_.lock();
  if (!buffer)
  {
    tf2_ros::Buffer* raw = new tf2_ros::Buffer();
    // assign custom deleter to also delete associated TransformListener
    buffer.reset(raw, Deleter(new tf2_ros::TransformListener(*raw)));
    s.tf_buffer_ = buffer;
  }
  return buffer;
}

moveit::core::RobotModelConstPtr getSharedRobotModel(const std::string& robot_description)
{
  SharedStorage& s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  auto it = s.models_.insert(std::make_pair(robot_description, moveit::core::RobotModelWeakPtr())).first;
  moveit::core::RobotModelPtr model = it->second.lock();
  if (!model)
  {
    RobotModelLoader::Options opt(robot_description);
    opt.load_kinematics_solvers_ = true;
    RobotModelLoaderPtr loader(new RobotModelLoader(opt));
    // create an aliasing shared_ptr
    model = moveit::core::RobotModelPtr(loader, loader->getModel().get());
    it->second = model;
  }
  return model;
}

CurrentStateMonitorPtr getSharedStateMonitor(const moveit::core::RobotModelConstPtr& robot_model,
                                             const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
  return getSharedStateMonitor(robot_model, tf_buffer, ros::NodeHandle());
}

CurrentStateMonitorPtr getSharedStateMonitor(const moveit::core::RobotModelConstPtr& robot_model,
                                             const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                             const ros::NodeHandle& nh)
{
  SharedStorage& s = getSharedStorage();
  boost::mutex::scoped_lock slock(s.lock_);
  auto it = s.state_monitors_.insert(std::make_pair(robot_model->getName(), CurrentStateMonitorWeakPtr())).first;
  CurrentStateMonitorPtr monitor = it->second.lock();
  if (!monitor)
  {
    // if there was no valid entry, create one
    monitor = std::make_shared<CurrentStateMonitor>(robot_model, tf_buffer, nh);
    it->second = monitor;
  }
  return monitor;
}

}  // namespace planning_interface
}  // namespace moveit
