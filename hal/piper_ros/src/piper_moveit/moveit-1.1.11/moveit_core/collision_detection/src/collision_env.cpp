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

/* Author: Ioan Sucan, Jens Petit */

#include <moveit/collision_detection/collision_env.h>
#include <limits>

static inline bool validateScale(double scale)
{
  if (scale < std::numeric_limits<double>::epsilon())
  {
    ROS_ERROR_NAMED("collision_detection", "Scale must be positive");
    return false;
  }
  if (scale > std::numeric_limits<double>::max())
  {
    ROS_ERROR_NAMED("collision_detection", "Scale must be finite");
    return false;
  }
  return true;
}

static inline bool validatePadding(double padding)
{
  if (padding < 0.0)
  {
    ROS_ERROR_NAMED("collision_detection", "Padding cannot be negative");
    return false;
  }
  if (padding > std::numeric_limits<double>::max())
  {
    ROS_ERROR_NAMED("collision_detection", "Padding must be finite");
    return false;
  }
  return true;
}

namespace collision_detection
{
CollisionEnv::CollisionEnv(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : robot_model_(model), world_(new World()), world_const_(world_)
{
  if (!validateScale(scale))
    scale = 1.0;
  if (!validatePadding(padding))
    padding = 0.0;

  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (auto link : links)
  {
    link_padding_[link->getName()] = padding;
    link_scale_[link->getName()] = scale;
  }
}

CollisionEnv::CollisionEnv(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding,
                           double scale)
  : robot_model_(model), world_(world), world_const_(world_)
{
  if (!validateScale(scale))
    scale = 1.0;
  if (!validatePadding(padding))
    padding = 0.0;

  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (auto link : links)
  {
    link_padding_[link->getName()] = padding;
    link_scale_[link->getName()] = scale;
  }
}

CollisionEnv::CollisionEnv(const CollisionEnv& other, const WorldPtr& world)
  : robot_model_(other.robot_model_), world_(world), world_const_(world)
{
  link_padding_ = other.link_padding_;
  link_scale_ = other.link_scale_;
}
void CollisionEnv::setPadding(double padding)
{
  if (!validatePadding(padding))
    return;
  std::vector<std::string> u;
  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (auto link : links)
  {
    if (getLinkPadding(link->getName()) != padding)
      u.push_back(link->getName());
    link_padding_[link->getName()] = padding;
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

void CollisionEnv::setScale(double scale)
{
  if (!validateScale(scale))
    return;
  std::vector<std::string> u;
  const std::vector<const moveit::core::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  for (auto link : links)
  {
    if (getLinkScale(link->getName()) != scale)
      u.push_back(link->getName());
    link_scale_[link->getName()] = scale;
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

void CollisionEnv::setLinkPadding(const std::string& link_name, double padding)
{
  validatePadding(padding);
  bool update = getLinkPadding(link_name) != padding;
  link_padding_[link_name] = padding;
  if (update)
  {
    std::vector<std::string> u(1, link_name);
    updatedPaddingOrScaling(u);
  }
}

double CollisionEnv::getLinkPadding(const std::string& link_name) const
{
  auto it = link_padding_.find(link_name);
  if (it != link_padding_.end())
    return it->second;
  else
    return 0.0;
}

void CollisionEnv::setLinkPadding(const std::map<std::string, double>& padding)
{
  std::vector<std::string> u;
  for (const auto& link_pad_pair : padding)
  {
    validatePadding(link_pad_pair.second);
    bool update = getLinkPadding(link_pad_pair.first) != link_pad_pair.second;
    link_padding_[link_pad_pair.first] = link_pad_pair.second;
    if (update)
      u.push_back(link_pad_pair.first);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

const std::map<std::string, double>& CollisionEnv::getLinkPadding() const
{
  return link_padding_;
}

void CollisionEnv::setLinkScale(const std::string& link_name, double scale)
{
  validateScale(scale);
  bool update = getLinkScale(link_name) != scale;
  link_scale_[link_name] = scale;
  if (update)
  {
    std::vector<std::string> u(1, link_name);
    updatedPaddingOrScaling(u);
  }
}

double CollisionEnv::getLinkScale(const std::string& link_name) const
{
  auto it = link_scale_.find(link_name);
  if (it != link_scale_.end())
    return it->second;
  else
    return 1.0;
}

void CollisionEnv::setLinkScale(const std::map<std::string, double>& scale)
{
  std::vector<std::string> u;
  for (const auto& link_scale_pair : scale)
  {
    bool update = getLinkScale(link_scale_pair.first) != link_scale_pair.second;
    link_scale_[link_scale_pair.first] = link_scale_pair.second;
    if (update)
      u.push_back(link_scale_pair.first);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

const std::map<std::string, double>& CollisionEnv::getLinkScale() const
{
  return link_scale_;
}

void CollisionEnv::setPadding(const std::vector<moveit_msgs::LinkPadding>& padding)
{
  std::vector<std::string> u;
  for (const auto& p : padding)
  {
    validatePadding(p.padding);
    bool update = getLinkPadding(p.link_name) != p.padding;
    link_padding_[p.link_name] = p.padding;
    if (update)
      u.push_back(p.link_name);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

void CollisionEnv::setScale(const std::vector<moveit_msgs::LinkScale>& scale)
{
  std::vector<std::string> u;
  for (const auto& s : scale)
  {
    validateScale(s.scale);
    bool update = getLinkScale(s.link_name) != s.scale;
    link_scale_[s.link_name] = s.scale;
    if (update)
      u.push_back(s.link_name);
  }
  if (!u.empty())
    updatedPaddingOrScaling(u);
}

void CollisionEnv::getPadding(std::vector<moveit_msgs::LinkPadding>& padding) const
{
  padding.clear();
  for (const auto& lp : link_padding_)
  {
    moveit_msgs::LinkPadding lp_msg;
    lp_msg.link_name = lp.first;
    lp_msg.padding = lp.second;
    padding.push_back(lp_msg);
  }
}

void CollisionEnv::getScale(std::vector<moveit_msgs::LinkScale>& scale) const
{
  scale.clear();
  for (const auto& ls : link_scale_)
  {
    moveit_msgs::LinkScale ls_msg;
    ls_msg.link_name = ls.first;
    ls_msg.scale = ls.second;
    scale.push_back(ls_msg);
  }
}

void CollisionEnv::updatedPaddingOrScaling(const std::vector<std::string>& /*links*/)
{
}

void CollisionEnv::setWorld(const WorldPtr& world)
{
  world_ = world;
  if (!world_)
    world_ = std::make_shared<World>();

  world_const_ = world;
}

void CollisionEnv::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                  const moveit::core::RobotState& state) const
{
  checkSelfCollision(req, res, state);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, state);
}

void CollisionEnv::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                  const moveit::core::RobotState& state, const AllowedCollisionMatrix& acm) const
{
  checkSelfCollision(req, res, state, acm);
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    checkRobotCollision(req, res, state, acm);
}

}  // end of namespace collision_detection
