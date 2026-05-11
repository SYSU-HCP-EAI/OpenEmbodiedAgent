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

/* Author: E. Gil Jones */

#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <ros/console.h>
#include <boost/thread/mutex.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <memory>

namespace collision_detection
{
struct BodyDecompositionCache
{
  using Comperator = std::owner_less<shapes::ShapeConstWeakPtr>;
  using Map = std::map<shapes::ShapeConstWeakPtr, BodyDecompositionConstPtr, Comperator>;

  BodyDecompositionCache() : clean_count_(0)
  {
  }
  static const unsigned int MAX_CLEAN_COUNT = 100;
  Map map_;
  unsigned int clean_count_;
  boost::mutex lock_;
};

BodyDecompositionCache& getBodyDecompositionCache()
{
  static BodyDecompositionCache cache;
  return cache;
}

BodyDecompositionConstPtr getBodyDecompositionCacheEntry(const shapes::ShapeConstPtr& shape, double resolution)
{
  // TODO - deal with changing resolution?
  BodyDecompositionCache& cache = getBodyDecompositionCache();
  shapes::ShapeConstWeakPtr wptr(shape);
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    BodyDecompositionCache::Map::const_iterator cache_it = cache.map_.find(wptr);
    if (cache_it != cache.map_.end())
    {
      return cache_it->second;
    }
  }

  BodyDecompositionConstPtr bdcp(new BodyDecomposition(shape, resolution));
  {
    boost::mutex::scoped_lock slock(cache.lock_);
    cache.map_[wptr] = bdcp;
    cache.clean_count_++;
    return bdcp;
  }
  // TODO - clean cache
}

PosedBodyPointDecompositionVectorPtr getCollisionObjectPointDecomposition(const collision_detection::World::Object& obj,
                                                                          double resolution)
{
  PosedBodyPointDecompositionVectorPtr ret(new PosedBodyPointDecompositionVector());
  for (unsigned int i = 0; i < obj.shapes_.size(); i++)
  {
    PosedBodyPointDecompositionPtr pbd(
        new PosedBodyPointDecomposition(getBodyDecompositionCacheEntry(obj.shapes_[i], resolution)));
    ret->addToVector(pbd);
    ret->updatePose(ret->getSize() - 1, obj.global_shape_poses_[i]);
  }
  return ret;
}

PosedBodySphereDecompositionVectorPtr getAttachedBodySphereDecomposition(const moveit::core::AttachedBody* att,
                                                                         double resolution)
{
  PosedBodySphereDecompositionVectorPtr ret(new PosedBodySphereDecompositionVector());
  for (unsigned int i = 0; i < att->getShapes().size(); i++)
  {
    PosedBodySphereDecompositionPtr pbd(
        new PosedBodySphereDecomposition(getBodyDecompositionCacheEntry(att->getShapes()[i], resolution)));
    pbd->updatePose(att->getGlobalCollisionBodyTransforms()[i]);
    ret->addToVector(pbd);
  }
  return ret;
}

PosedBodyPointDecompositionVectorPtr getAttachedBodyPointDecomposition(const moveit::core::AttachedBody* att,
                                                                       double resolution)
{
  PosedBodyPointDecompositionVectorPtr ret(new PosedBodyPointDecompositionVector());
  for (unsigned int i = 0; i < att->getShapes().size(); i++)
  {
    PosedBodyPointDecompositionPtr pbd(
        new PosedBodyPointDecomposition(getBodyDecompositionCacheEntry(att->getShapes()[i], resolution)));
    ret->addToVector(pbd);
    ret->updatePose(ret->getSize() - 1, att->getGlobalCollisionBodyTransforms()[i]);
  }
  return ret;
}

void getBodySphereVisualizationMarkers(const GroupStateRepresentationConstPtr& gsr, const std::string& reference_frame,
                                       visualization_msgs::MarkerArray& body_marker_array)
{
  // creating namespaces
  std::string robot_ns = gsr->dfce_->group_name_ + "_sphere_decomposition";
  std::string attached_ns = "attached_sphere_decomposition";

  // creating colors
  std_msgs::ColorRGBA robot_color;
  robot_color.r = 0;
  robot_color.b = 0.8f;
  robot_color.g = 0;
  robot_color.a = 0.5;

  std_msgs::ColorRGBA attached_color;
  attached_color.r = 1;
  attached_color.g = 1;
  attached_color.b = 0;
  attached_color.a = 0.5;

  // creating sphere marker
  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = reference_frame;
  sphere_marker.header.stamp = ros::Time(0);
  sphere_marker.ns = robot_ns;
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::Marker::ADD;
  sphere_marker.pose.orientation.x = 0;
  sphere_marker.pose.orientation.y = 0;
  sphere_marker.pose.orientation.z = 0;
  sphere_marker.pose.orientation.w = 1;
  sphere_marker.color = robot_color;
  sphere_marker.lifetime = ros::Duration(0);

  const moveit::core::RobotState& state = *(gsr->dfce_->state_);
  unsigned int id = 0;
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++)
  {
    const moveit::core::LinkModel* ls = state.getLinkModel(gsr->dfce_->link_names_[i]);
    if (gsr->dfce_->link_has_geometry_[i])
    {
      gsr->link_body_decompositions_[i]->updatePose(state.getFrameTransform(ls->getName()));

      collision_detection::PosedBodySphereDecompositionConstPtr sphere_representation =
          gsr->link_body_decompositions_[i];
      for (unsigned int j = 0; j < sphere_representation->getCollisionSpheres().size(); j++)
      {
        sphere_marker.pose.position = tf2::toMsg(sphere_representation->getSphereCenters()[j]);
        sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z =
            sphere_representation->getCollisionSpheres()[j].radius_;
        sphere_marker.id = id;
        id++;

        body_marker_array.markers.push_back(sphere_marker);
      }
    }
  }

  sphere_marker.ns = attached_ns;
  sphere_marker.color = attached_color;
  for (unsigned int i = 0; i < gsr->dfce_->attached_body_names_.size(); i++)
  {
    const moveit::core::AttachedBody* att = state.getAttachedBody(gsr->dfce_->attached_body_names_[i]);
    if (!att)
    {
      ROS_WARN("Attached body '%s' was not found, skipping sphere "
               "decomposition visualization",
               gsr->dfce_->attached_body_names_[i].c_str());
      continue;
    }

    if (gsr->attached_body_decompositions_[i]->getSize() != att->getShapes().size())
    {
      ROS_WARN("Attached body size discrepancy");
      continue;
    }

    for (unsigned int j = 0; j < att->getShapes().size(); j++)
    {
      PosedBodySphereDecompositionVectorPtr sphere_decp = gsr->attached_body_decompositions_[i];
      sphere_decp->updatePose(j, att->getGlobalCollisionBodyTransforms()[j]);

      sphere_marker.pose.position = tf2::toMsg(sphere_decp->getSphereCenters()[j]);
      sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z =
          sphere_decp->getCollisionSpheres()[j].radius_;
      sphere_marker.id = id;
      body_marker_array.markers.push_back(sphere_marker);
      id++;
    }
  }
}
}  // namespace collision_detection
