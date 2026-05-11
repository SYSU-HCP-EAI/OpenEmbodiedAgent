/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <limits>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <ros/console.h>
#include <vector>

namespace trajectory_processing
{
const std::string LOGNAME = "trajectory_processing.time_optimal_trajectory_generation";
constexpr double EPS = 0.000001;
class LinearPathSegment : public PathSegment
{
public:
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
    : PathSegment((end - start).norm()), end_(end), start_(start)
  {
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    s /= length_;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start_ + s * end_;
  }

  Eigen::VectorXd getTangent(double /* s */) const override
  {
    return (end_ - start_) / length_;
  }

  Eigen::VectorXd getCurvature(double /* s */) const override
  {
    return Eigen::VectorXd::Zero(start_.size());
  }

  std::list<double> getSwitchingPoints() const override
  {
    return std::list<double>();
  }

  LinearPathSegment* clone() const override
  {
    return new LinearPathSegment(*this);
  }

private:
  Eigen::VectorXd end_;
  Eigen::VectorXd start_;
};

class CircularPathSegment : public PathSegment
{
public:
  CircularPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& intersection, const Eigen::VectorXd& end,
                      double max_deviation)
  {
    if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001)
    {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const Eigen::VectorXd start_direction = (intersection - start).normalized();
    const Eigen::VectorXd end_direction = (end - intersection).normalized();
    const double start_dot_end = start_direction.dot(end_direction);

    // catch division by 0 in computations below
    if (start_dot_end > 0.999999 || start_dot_end < -0.999999)
    {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const double angle = acos(start_dot_end);
    const double start_distance = (start - intersection).norm();
    const double end_distance = (end - intersection).norm();

    // enforce max deviation
    double distance = std::min(start_distance, end_distance);
    distance = std::min(distance, max_deviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));

    radius = distance / tan(0.5 * angle);
    length_ = angle * radius;

    center = intersection + (end_direction - start_direction).normalized() * radius / cos(0.5 * angle);
    x = (intersection - distance * start_direction - center).normalized();
    y = start_direction;
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    const double angle = s / radius;
    return center + radius * (x * cos(angle) + y * sin(angle));
  }

  Eigen::VectorXd getTangent(double s) const override
  {
    const double angle = s / radius;
    return -x * sin(angle) + y * cos(angle);
  }

  Eigen::VectorXd getCurvature(double s) const override
  {
    const double angle = s / radius;
    return -1.0 / radius * (x * cos(angle) + y * sin(angle));
  }

  std::list<double> getSwitchingPoints() const override
  {
    std::list<double> switching_points;
    const double dim = x.size();
    for (unsigned int i = 0; i < dim; ++i)
    {
      double switching_angle = atan2(y[i], x[i]);
      if (switching_angle < 0.0)
      {
        switching_angle += M_PI;
      }
      const double switching_point = switching_angle * radius;
      if (switching_point < length_)
      {
        switching_points.push_back(switching_point);
      }
    }
    switching_points.sort();
    return switching_points;
  }

  CircularPathSegment* clone() const override
  {
    return new CircularPathSegment(*this);
  }

private:
  double radius;
  Eigen::VectorXd center;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
};

Path::Path(const std::list<Eigen::VectorXd>& path, double max_deviation) : length_(0.0)
{
  if (path.size() < 2)
    return;
  std::list<Eigen::VectorXd>::const_iterator path_iterator1 = path.begin();
  std::list<Eigen::VectorXd>::const_iterator path_iterator2 = path_iterator1;
  ++path_iterator2;
  std::list<Eigen::VectorXd>::const_iterator path_iterator3;
  Eigen::VectorXd start_config = *path_iterator1;
  while (path_iterator2 != path.end())
  {
    path_iterator3 = path_iterator2;
    ++path_iterator3;
    if (max_deviation > 0.0 && path_iterator3 != path.end())
    {
      CircularPathSegment* blend_segment =
          new CircularPathSegment(0.5 * (*path_iterator1 + *path_iterator2), *path_iterator2,
                                  0.5 * (*path_iterator2 + *path_iterator3), max_deviation);
      Eigen::VectorXd end_config = blend_segment->getConfig(0.0);
      if ((end_config - start_config).norm() > 0.000001)
      {
        path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, end_config));
      }
      path_segments_.emplace_back(blend_segment);

      start_config = blend_segment->getConfig(blend_segment->getLength());
    }
    else
    {
      path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, *path_iterator2));
      start_config = *path_iterator2;
    }
    path_iterator1 = path_iterator2;
    ++path_iterator2;
  }

  // Create list of switching point candidates, calculate total path length and
  // absolute positions of path segments
  for (std::unique_ptr<PathSegment>& path_segment : path_segments_)
  {
    path_segment->position_ = length_;
    std::list<double> local_switching_points = path_segment->getSwitchingPoints();
    for (std::list<double>::const_iterator point = local_switching_points.begin();
         point != local_switching_points.end(); ++point)
    {
      switching_points_.push_back(std::make_pair(length_ + *point, false));
    }
    length_ += path_segment->getLength();
    while (!switching_points_.empty() && switching_points_.back().first >= length_)
      switching_points_.pop_back();
    switching_points_.push_back(std::make_pair(length_, true));
  }
  switching_points_.pop_back();
}

Path::Path(const Path& path) : length_(path.length_), switching_points_(path.switching_points_)
{
  for (const std::unique_ptr<PathSegment>& path_segment : path.path_segments_)
  {
    path_segments_.emplace_back(path_segment->clone());
  }
}

double Path::getLength() const
{
  return length_;
}

PathSegment* Path::getPathSegment(double& s) const
{
  std::list<std::unique_ptr<PathSegment>>::const_iterator it = path_segments_.begin();
  std::list<std::unique_ptr<PathSegment>>::const_iterator next = it;
  ++next;
  while (next != path_segments_.end() && s >= (*next)->position_)
  {
    it = next;
    ++next;
  }
  s -= (*it)->position_;
  return (*it).get();
}

Eigen::VectorXd Path::getConfig(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getConfig(s);
}

Eigen::VectorXd Path::getTangent(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getTangent(s);
}

Eigen::VectorXd Path::getCurvature(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool& discontinuity) const
{
  std::list<std::pair<double, bool>>::const_iterator it = switching_points_.begin();
  while (it != switching_points_.end() && it->first <= s)
  {
    ++it;
  }
  if (it == switching_points_.end())
  {
    discontinuity = true;
    return length_;
  }
  discontinuity = it->second;
  return it->first;
}

std::list<std::pair<double, bool>> Path::getSwitchingPoints() const
{
  return switching_points_;
}

Trajectory::Trajectory(const Path& path, const Eigen::VectorXd& max_velocity, const Eigen::VectorXd& max_acceleration,
                       double time_step)
  : path_(path)
  , max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , joint_num_(max_velocity.size())
  , valid_(true)
  , time_step_(time_step)
  , cached_time_(std::numeric_limits<double>::max())
{
  trajectory_.push_back(TrajectoryStep(0.0, 0.0));
  double after_acceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
  while (valid_ && !integrateForward(trajectory_, after_acceleration) && valid_)
  {
    double before_acceleration;
    TrajectoryStep switching_point;
    if (getNextSwitchingPoint(trajectory_.back().path_pos_, switching_point, before_acceleration, after_acceleration))
    {
      break;
    }
    integrateBackward(trajectory_, switching_point.path_pos_, switching_point.path_vel_, before_acceleration);
  }

  if (valid_)
  {
    double before_acceleration = getMinMaxPathAcceleration(path_.getLength(), 0.0, false);
    integrateBackward(trajectory_, path_.getLength(), 0.0, before_acceleration);
  }

  if (valid_)
  {
    // Calculate timing
    std::list<TrajectoryStep>::iterator previous = trajectory_.begin();
    std::list<TrajectoryStep>::iterator it = previous;
    it->time_ = 0.0;
    ++it;
    while (it != trajectory_.end())
    {
      it->time_ =
          previous->time_ + (it->path_pos_ - previous->path_pos_) / ((it->path_vel_ + previous->path_vel_) / 2.0);
      previous = it;
      ++it;
    }
  }
}

Trajectory::~Trajectory()
{
}

// Returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double path_pos, TrajectoryStep& next_switching_point,
                                       double& before_acceleration, double& after_acceleration)
{
  TrajectoryStep acceleration_switching_point(path_pos, 0.0);
  double acceleration_before_acceleration, acceleration_after_acceleration;
  bool acceleration_reached_end;
  do
  {
    acceleration_reached_end =
        getNextAccelerationSwitchingPoint(acceleration_switching_point.path_pos_, acceleration_switching_point,
                                          acceleration_before_acceleration, acceleration_after_acceleration);
  } while (!acceleration_reached_end &&
           acceleration_switching_point.path_vel_ > getVelocityMaxPathVelocity(acceleration_switching_point.path_pos_));

  TrajectoryStep velocity_switching_point(path_pos, 0.0);
  double velocity_before_acceleration, velocity_after_acceleration;
  bool velocity_reached_end;
  do
  {
    velocity_reached_end = getNextVelocitySwitchingPoint(velocity_switching_point.path_pos_, velocity_switching_point,
                                                         velocity_before_acceleration, velocity_after_acceleration);
  } while (
      !velocity_reached_end && velocity_switching_point.path_pos_ <= acceleration_switching_point.path_pos_ &&
      (velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(velocity_switching_point.path_pos_ - EPS) ||
       velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(velocity_switching_point.path_pos_ + EPS)));

  if (acceleration_reached_end && velocity_reached_end)
  {
    return true;
  }
  else if (!acceleration_reached_end &&
           (velocity_reached_end || acceleration_switching_point.path_pos_ <= velocity_switching_point.path_pos_))
  {
    next_switching_point = acceleration_switching_point;
    before_acceleration = acceleration_before_acceleration;
    after_acceleration = acceleration_after_acceleration;
    return false;
  }
  else
  {
    next_switching_point = velocity_switching_point;
    before_acceleration = velocity_before_acceleration;
    after_acceleration = velocity_after_acceleration;
    return false;
  }
}

bool Trajectory::getNextAccelerationSwitchingPoint(double path_pos, TrajectoryStep& next_switching_point,
                                                   double& before_acceleration, double& after_acceleration)
{
  double switching_path_pos = path_pos;
  double switching_path_vel;
  while (true)
  {
    bool discontinuity;
    switching_path_pos = path_.getNextSwitchingPoint(switching_path_pos, discontinuity);

    if (switching_path_pos > path_.getLength() - EPS)
    {
      return true;
    }

    if (discontinuity)
    {
      const double before_path_vel = getAccelerationMaxPathVelocity(switching_path_pos - EPS);
      const double after_path_vel = getAccelerationMaxPathVelocity(switching_path_pos + EPS);
      switching_path_vel = std::min(before_path_vel, after_path_vel);
      before_acceleration = getMinMaxPathAcceleration(switching_path_pos - EPS, switching_path_vel, false);
      after_acceleration = getMinMaxPathAcceleration(switching_path_pos + EPS, switching_path_vel, true);

      if ((before_path_vel > after_path_vel ||
           getMinMaxPhaseSlope(switching_path_pos - EPS, switching_path_vel, false) >
               getAccelerationMaxPathVelocityDeriv(switching_path_pos - 2.0 * EPS)) &&
          (before_path_vel < after_path_vel || getMinMaxPhaseSlope(switching_path_pos + EPS, switching_path_vel, true) <
                                                   getAccelerationMaxPathVelocityDeriv(switching_path_pos + 2.0 * EPS)))
      {
        break;
      }
    }
    else
    {
      switching_path_vel = getAccelerationMaxPathVelocity(switching_path_pos);
      before_acceleration = 0.0;
      after_acceleration = 0.0;

      if (getAccelerationMaxPathVelocityDeriv(switching_path_pos - EPS) < 0.0 &&
          getAccelerationMaxPathVelocityDeriv(switching_path_pos + EPS) > 0.0)
      {
        break;
      }
    }
  }

  next_switching_point = TrajectoryStep(switching_path_pos, switching_path_vel);
  return false;
}

bool Trajectory::getNextVelocitySwitchingPoint(double path_pos, TrajectoryStep& next_switching_point,
                                               double& before_acceleration, double& after_acceleration)
{
  const double step_size = 0.001;
  const double accuracy = 0.000001;

  bool start = false;
  path_pos -= step_size;
  do
  {
    path_pos += step_size;

    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >=
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      start = true;
    }
  } while ((!start || getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
                          getVelocityMaxPathVelocityDeriv(path_pos)) &&
           path_pos < path_.getLength());

  if (path_pos >= path_.getLength())
  {
    return true;  // end of trajectory reached
  }

  double before_path_pos = path_pos - step_size;
  double after_path_pos = path_pos;
  while (after_path_pos - before_path_pos > accuracy)
  {
    path_pos = (before_path_pos + after_path_pos) / 2.0;
    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      before_path_pos = path_pos;
    }
    else
    {
      after_path_pos = path_pos;
    }
  }

  before_acceleration = getMinMaxPathAcceleration(before_path_pos, getVelocityMaxPathVelocity(before_path_pos), false);
  after_acceleration = getMinMaxPathAcceleration(after_path_pos, getVelocityMaxPathVelocity(after_path_pos), true);
  next_switching_point = TrajectoryStep(after_path_pos, getVelocityMaxPathVelocity(after_path_pos));
  return false;
}

// Returns true if end of path is reached
bool Trajectory::integrateForward(std::list<TrajectoryStep>& trajectory, double acceleration)
{
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;

  std::list<std::pair<double, bool>> switching_points = path_.getSwitchingPoints();
  std::list<std::pair<double, bool>>::iterator next_discontinuity = switching_points.begin();

  while (true)
  {
    while ((next_discontinuity != switching_points.end()) &&
           (next_discontinuity->first <= path_pos || !next_discontinuity->second))
    {
      ++next_discontinuity;
    }

    double old_path_pos = path_pos;
    double old_path_vel = path_vel;

    path_vel += time_step_ * acceleration;
    path_pos += time_step_ * 0.5 * (old_path_vel + path_vel);

    if (next_discontinuity != switching_points.end() && path_pos > next_discontinuity->first)
    {
      // Avoid having a TrajectoryStep with path_pos near a switching point which will cause an almost identical
      // TrajectoryStep get added in the next run (https://github.com/ros-planning/moveit/issues/1665)
      if (path_pos - next_discontinuity->first < EPS)
      {
        continue;
      }
      path_vel = old_path_vel +
                 (next_discontinuity->first - old_path_pos) * (path_vel - old_path_vel) / (path_pos - old_path_pos);
      path_pos = next_discontinuity->first;
    }

    if (path_pos > path_.getLength())
    {
      trajectory.push_back(TrajectoryStep(path_pos, path_vel));
      return true;
    }
    else if (path_vel < 0.0)
    {
      valid_ = false;
      ROS_ERROR_NAMED(LOGNAME, "Error while integrating forward: Negative path velocity");
      return true;
    }

    if (path_vel > getVelocityMaxPathVelocity(path_pos) &&
        getMinMaxPhaseSlope(old_path_pos, getVelocityMaxPathVelocity(old_path_pos), false) <=
            getVelocityMaxPathVelocityDeriv(old_path_pos))
    {
      path_vel = getVelocityMaxPathVelocity(path_pos);
    }

    trajectory.push_back(TrajectoryStep(path_pos, path_vel));
    acceleration = getMinMaxPathAcceleration(path_pos, path_vel, true);

    if (path_vel > getAccelerationMaxPathVelocity(path_pos) || path_vel > getVelocityMaxPathVelocity(path_pos))
    {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;
      while (after - before > EPS)
      {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

        if (midpoint_path_vel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <=
                getVelocityMaxPathVelocityDeriv(before))
        {
          midpoint_path_vel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpoint_path_vel > getAccelerationMaxPathVelocity(midpoint) ||
            midpoint_path_vel > getVelocityMaxPathVelocity(midpoint))
        {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        }
        else
        {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, before_path_vel));

      if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
      {
        if (after > next_discontinuity->first)
        {
          return false;
        }
        else if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, true) >
                 getAccelerationMaxPathVelocityDeriv(trajectory.back().path_pos_))
        {
          return false;
        }
      }
      else
      {
        if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
            getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
        {
          return false;
        }
      }
    }
  }
}

void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory, double path_pos, double path_vel,
                                   double acceleration)
{
  std::list<TrajectoryStep>::iterator start2 = start_trajectory.end();
  --start2;
  std::list<TrajectoryStep>::iterator start1 = start2;
  --start1;
  std::list<TrajectoryStep> trajectory;
  double slope;
  assert(start1->path_pos_ <= path_pos);

  while (start1 != start_trajectory.begin() || path_pos >= 0.0)
  {
    if (start1->path_pos_ <= path_pos)
    {
      trajectory.push_front(TrajectoryStep(path_pos, path_vel));
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
      acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);
      slope = (trajectory.front().path_vel_ - path_vel) / (trajectory.front().path_pos_ - path_pos);

      if (path_vel < 0.0)
      {
        valid_ = false;
        ROS_ERROR_NAMED(LOGNAME, "Error while integrating backward: Negative path velocity");
        end_trajectory_ = trajectory;
        return;
      }
    }
    else
    {
      --start1;
      --start2;
    }

    // Check for intersection between current start trajectory and backward
    // trajectory segments
    const double start_slope = (start2->path_vel_ - start1->path_vel_) / (start2->path_pos_ - start1->path_pos_);
    const double intersection_path_pos =
        (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) / (slope - start_slope);
    if (std::max(start1->path_pos_, path_pos) - EPS <= intersection_path_pos &&
        intersection_path_pos <= EPS + std::min(start2->path_pos_, trajectory.front().path_pos_))
    {
      const double intersection_path_vel =
          start1->path_vel_ + start_slope * (intersection_path_pos - start1->path_pos_);
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.push_back(TrajectoryStep(intersection_path_pos, intersection_path_vel));
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  ROS_ERROR_NAMED(LOGNAME, "Error while integrating backward: Did not hit start trajectory");
  end_trajectory_ = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double path_pos, double path_vel, bool max)
{
  Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      max_path_acceleration =
          std::min(max_path_acceleration, max_acceleration_[i] / std::abs(config_deriv[i]) -
                                              factor * config_deriv2[i] * path_vel * path_vel / config_deriv[i]);
    }
  }
  return factor * max_path_acceleration;
}

double Trajectory::getMinMaxPhaseSlope(double path_pos, double path_vel, bool max)
{
  return getMinMaxPathAcceleration(path_pos, path_vel, max) / path_vel;
}

double Trajectory::getAccelerationMaxPathVelocity(double path_pos) const
{
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  const Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      for (unsigned int j = i + 1; j < joint_num_; ++j)
      {
        if (config_deriv[j] != 0.0)
        {
          double a_ij = config_deriv2[i] / config_deriv[i] - config_deriv2[j] / config_deriv[j];
          if (a_ij != 0.0)
          {
            max_path_velocity = std::min(max_path_velocity, sqrt((max_acceleration_[i] / std::abs(config_deriv[i]) +
                                                                  max_acceleration_[j] / std::abs(config_deriv[j])) /
                                                                 std::abs(a_ij)));
          }
        }
      }
    }
    else if (config_deriv2[i] != 0.0)
    {
      max_path_velocity = std::min(max_path_velocity, sqrt(max_acceleration_[i] / std::abs(config_deriv2[i])));
    }
  }
  return max_path_velocity;
}

double Trajectory::getVelocityMaxPathVelocity(double path_pos) const
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    max_path_velocity = std::min(max_path_velocity, max_velocity_[i] / std::abs(tangent[i]));
  }
  return max_path_velocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double path_pos)
{
  return (getAccelerationMaxPathVelocity(path_pos + EPS) - getAccelerationMaxPathVelocity(path_pos - EPS)) /
         (2.0 * EPS);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double path_pos)
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  unsigned int active_constraint;
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    const double this_max_path_velocity = max_velocity_[i] / std::abs(tangent[i]);
    if (this_max_path_velocity < max_path_velocity)
    {
      max_path_velocity = this_max_path_velocity;
      active_constraint = i;
    }
  }
  return -(max_velocity_[active_constraint] * path_.getCurvature(path_pos)[active_constraint]) /
         (tangent[active_constraint] * std::abs(tangent[active_constraint]));
}

bool Trajectory::isValid() const
{
  return valid_;
}

double Trajectory::getDuration() const
{
  return trajectory_.back().time_;
}

std::list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegment(double time) const
{
  if (time >= trajectory_.back().time_)
  {
    std::list<TrajectoryStep>::const_iterator last = trajectory_.end();
    last--;
    return last;
  }
  else
  {
    if (time < cached_time_)
    {
      cached_trajectory_segment_ = trajectory_.begin();
    }
    while (time >= cached_trajectory_segment_->time_)
    {
      ++cached_trajectory_segment_;
    }
    cached_time_ = time;
    return cached_trajectory_segment_;
  }
}

Eigen::VectorXd Trajectory::getPosition(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration;

  return path_.getConfig(path_pos);
}

Eigen::VectorXd Trajectory::getVelocity(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;

  return path_.getTangent(path_pos) * path_vel;
}

Eigen::VectorXd Trajectory::getAcceleration(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;
  Eigen::VectorXd path_acc =
      (path_.getTangent(path_pos) * path_vel - path_.getTangent(previous->path_pos_) * previous->path_vel_);
  if (time_step > 0.0)
    path_acc /= time_step;
  return path_acc;
}

TimeOptimalTrajectoryGeneration::TimeOptimalTrajectoryGeneration(const double path_tolerance, const double resample_dt,
                                                                 const double min_angle_change)
  : path_tolerance_(path_tolerance), resample_dt_(resample_dt), min_angle_change_(min_angle_change)
{
}

bool TimeOptimalTrajectoryGeneration::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                                        const double max_velocity_scaling_factor,
                                                        const double max_acceleration_scaling_factor) const
{
  if (trajectory.empty())
    return true;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    ROS_ERROR_NAMED(LOGNAME, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  // Validate scaling
  double velocity_scaling_factor = 1.0;
  if (max_velocity_scaling_factor > 0.0 && max_velocity_scaling_factor <= 1.0)
  {
    velocity_scaling_factor = max_velocity_scaling_factor;
  }
  else if (max_velocity_scaling_factor == 0.0)
  {
    ROS_DEBUG_NAMED(LOGNAME, "A max_velocity_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                    velocity_scaling_factor);
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "Invalid max_velocity_scaling_factor %f specified, defaulting to %f instead.",
                   max_velocity_scaling_factor, velocity_scaling_factor);
  }

  double acceleration_scaling_factor = 1.0;
  if (max_acceleration_scaling_factor > 0.0 && max_acceleration_scaling_factor <= 1.0)
  {
    acceleration_scaling_factor = max_acceleration_scaling_factor;
  }
  else if (max_acceleration_scaling_factor == 0.0)
  {
    ROS_DEBUG_NAMED(LOGNAME, "A max_acceleration_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                    acceleration_scaling_factor);
  }
  else
  {
    ROS_WARN_NAMED(LOGNAME, "Invalid max_acceleration_scaling_factor %f specified, defaulting to %f instead.",
                   max_acceleration_scaling_factor, acceleration_scaling_factor);
  }

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  // This is pretty much copied from IterativeParabolicTimeParameterization::applyVelocityConstraints
  const std::vector<std::string>& vars = group->getVariableNames();
  const std::vector<int>& idx = group->getVariableIndexList();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  const unsigned num_joints = group->getVariableCount();
  const unsigned num_points = trajectory.getWayPointCount();

  // Get the limits (we do this at same time, unlike IterativeParabolicTimeParameterization)
  Eigen::VectorXd max_velocity(num_joints);
  Eigen::VectorXd max_acceleration(num_joints);
  for (size_t j = 0; j < num_joints; ++j)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[j]);

    // Limits need to be non-zero, otherwise we never exit
    max_velocity[j] = 1.0;
    if (bounds.velocity_bounded_)
    {
      if (bounds.max_velocity_ < std::numeric_limits<double>::epsilon())
      {
        ROS_ERROR_NAMED(LOGNAME, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                        bounds.max_velocity_, vars[j].c_str());
        return false;
      }
      max_velocity[j] =
          std::min(std::fabs(bounds.max_velocity_), std::fabs(bounds.min_velocity_)) * velocity_scaling_factor;
    }

    max_acceleration[j] = 1.0;
    if (bounds.acceleration_bounded_)
    {
      if (bounds.max_acceleration_ < std::numeric_limits<double>::epsilon())
      {
        ROS_ERROR_NAMED(LOGNAME, "Invalid max_acceleration %f specified for '%s', must be greater than 0.0",
                        bounds.max_acceleration_, vars[j].c_str());
        return false;
      }
      max_acceleration[j] = std::min(std::fabs(bounds.max_acceleration_), std::fabs(bounds.min_acceleration_)) *
                            acceleration_scaling_factor;
    }
  }

  // Have to convert into Eigen data structs and remove repeated points
  //  (https://github.com/tobiaskunz/trajectories/issues/3)
  std::list<Eigen::VectorXd> points;
  for (size_t p = 0; p < num_points; ++p)
  {
    moveit::core::RobotStatePtr waypoint = trajectory.getWayPointPtr(p);
    Eigen::VectorXd new_point(num_joints);
    // The first point should always be kept
    bool diverse_point = (p == 0);

    for (size_t j = 0; j < num_joints; ++j)
    {
      new_point[j] = waypoint->getVariablePosition(idx[j]);
      // If any joint angle is different, it's a unique waypoint
      if (p > 0 && std::fabs(new_point[j] - points.back()[j]) > min_angle_change_)
      {
        diverse_point = true;
      }
    }

    if (diverse_point)
      points.push_back(new_point);
  }

  // Return trajectory with only the first waypoint if there are not multiple diverse points
  if (points.size() == 1)
  {
    ROS_DEBUG_NAMED(LOGNAME,
                    "Trajectory is parameterized with 0.0 dynamics since it only contains a single distinct waypoint.");
    moveit::core::RobotState waypoint = moveit::core::RobotState(trajectory.getWayPoint(0));
    waypoint.zeroVelocities();
    waypoint.zeroAccelerations();
    trajectory.clear();
    trajectory.addSuffixWayPoint(waypoint, 0.0);
    return true;
  }

  // Now actually call the algorithm
  Trajectory parameterized(Path(points, path_tolerance_), max_velocity, max_acceleration, 0.001);
  if (!parameterized.isValid())
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to parameterize trajectory.");
    return false;
  }

  // Compute sample count
  size_t sample_count = std::ceil(parameterized.getDuration() / resample_dt_);

  // Resample and fill in trajectory
  moveit::core::RobotState waypoint = moveit::core::RobotState(trajectory.getWayPoint(0));
  trajectory.clear();
  double last_t = 0;
  for (size_t sample = 0; sample <= sample_count; ++sample)
  {
    // always sample the end of the trajectory as well
    double t = std::min(parameterized.getDuration(), sample * resample_dt_);
    Eigen::VectorXd position = parameterized.getPosition(t);
    Eigen::VectorXd velocity = parameterized.getVelocity(t);
    Eigen::VectorXd acceleration = parameterized.getAcceleration(t);

    for (size_t j = 0; j < num_joints; ++j)
    {
      waypoint.setVariablePosition(idx[j], position[j]);
      waypoint.setVariableVelocity(idx[j], velocity[j]);
      waypoint.setVariableAcceleration(idx[j], acceleration[j]);
    }

    trajectory.addSuffixWayPoint(waypoint, t - last_t);
    last_t = t;
  }

  return true;
}
}  // namespace trajectory_processing
