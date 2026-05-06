/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: Sachin Chitta
 *********************************************************************/

#include <moveit/kinematics_cache/kinematics_cache.h>
#include <fstream>
#include <iostream>

namespace kinematics_cache
{
KinematicsCache::KinematicsCache() : min_squared_distance_(1e6), max_squared_distance_(0.0)
{
}

bool KinematicsCache::initialize(kinematics::KinematicsBaseConstPtr& kinematics_solver,
                                 const moveit::core::RobotModelConstPtr& kinematic_model,
                                 const KinematicsCache::Options& opt)
{
  options_ = opt;
  kinematics_solver_ = kinematics_solver;
  kinematic_model_ = kinematic_model;
  joint_model_group_ = kinematic_model_->getJointModelGroup(kinematics_solver_->getGroupName());
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);
  joint_state_group_ = std::make_shared<moveit::core::JointStateGroup>(kinematic_state_.get(), joint_model_group_);

  setup(opt);
  return true;
}

void KinematicsCache::setup(const KinematicsCache::Options& opt)
{
  cache_origin_ = opt.origin;
  cache_resolution_x_ = opt.resolution[0];
  cache_resolution_y_ = opt.resolution[1];
  cache_resolution_z_ = opt.resolution[2];

  cache_size_x_ = (unsigned int)(opt.workspace_size[0] / opt.resolution[0]);
  cache_size_y_ = (unsigned int)(opt.workspace_size[1] / opt.resolution[1]);
  cache_size_z_ = (unsigned int)(opt.workspace_size[2] / opt.resolution[2]);
  max_solutions_per_grid_location_ = std::max((unsigned int)1, opt.max_solutions_per_grid_location);
  solution_dimension_ = joint_model_group_->getVariableCount();
  size_grid_node_ = max_solutions_per_grid_location_ * solution_dimension_;
  kinematics_cache_size_ = cache_size_x_ * cache_size_y_ * cache_size_z_;
  kinematics_cache_points_with_solution_ = 0;
  kinematics_cache_vector_.resize(kinematics_cache_size_ * size_grid_node_, 0.0);
  num_solutions_vector_.resize(kinematics_cache_size_, 0);
  ROS_DEBUG_NAMED("kinematics_cache", "Origin: %f %f %f", cache_origin_.x, cache_origin_.y, cache_origin_.z);
  ROS_DEBUG_NAMED("kinematics_cache", "Cache size (num points x,y,z): %d %d %d", cache_size_x_, cache_size_y_,
                  cache_size_z_);
  ROS_DEBUG_NAMED("kinematics_cache", "Cache resolution: %f %f %f", cache_resolution_x_, cache_resolution_y_,
                  cache_resolution_z_);
  ROS_DEBUG_NAMED("kinematics_cache", "Solutions per grid location: %d", (int)max_solutions_per_grid_location_);
  ROS_DEBUG_NAMED("kinematics_cache", "Solution dimension: %d", (int)solution_dimension_);
}

bool KinematicsCache::generateCacheMap(double timeout)
{
  ros::WallTime start_time = ros::WallTime::now();
  std::vector<std::string> fk_names;
  std::vector<double> fk_values;
  std::vector<geometry_msgs::Pose> poses;

  fk_names.push_back(kinematics_solver_->getTipFrame());
  fk_values.resize(kinematics_solver_->getJointNames().size(), 0.0);
  poses.resize(1);

  while ((ros::WallTime::now() - start_time).toSec() <= timeout &&
         kinematics_cache_points_with_solution_ <= kinematics_cache_size_)
  {
    joint_state_group_->setToRandomValues();
    joint_state_group_->getVariableValues(fk_values);
    if (!kinematics_solver_->getPositionFK(fk_names, fk_values, poses))
    {
      ROS_ERROR_NAMED("kinematics_cache", "Fk failed");
      return false;
    }
    if (!addToCache(poses[0], fk_values))
    {
      ROS_DEBUG_NAMED("kinematics_cache", "Adding to cache failed for: %f %f %f", poses[0].position.x,
                      poses[0].position.y, poses[0].position.z);
    }
    ROS_DEBUG_NAMED("kinematics_cache", "Adding: %d", kinematics_cache_points_with_solution_);
  }
  ROS_DEBUG_NAMED("kinematics_cache", "Cache map generated with %d valid points",
                  kinematics_cache_points_with_solution_);
  return true;
}

bool KinematicsCache::addToCache(const geometry_msgs::Pose& pose, const std::vector<double>& joint_values,
                                 bool overwrite)
{
  unsigned int grid_index;
  if (!getGridIndex(pose, grid_index))
  {
    ROS_DEBUG_NAMED("kinematics_cache", "Failed to get grid index");
    return false;
  }
  unsigned int num_solutions = num_solutions_vector_[grid_index];
  if (!overwrite && num_solutions >= max_solutions_per_grid_location_)
  {
    ROS_DEBUG_NAMED("kinematics_cache", "Pose already has max number of solutions");
    return true;
  }
  if (num_solutions == 0)
    kinematics_cache_points_with_solution_++;
  if (overwrite && num_solutions >= max_solutions_per_grid_location_)
    num_solutions = 0;
  unsigned int start_index = getSolutionLocation(grid_index, num_solutions);
  for (unsigned int i = 0; i < joint_values.size(); ++i)
  {
    //    ROS_INFO("Joint value[%d]: %f, localtion: %d",i,joint_values[i],start_index+i);
    kinematics_cache_vector_[start_index + i] = joint_values[i];
  }
  if (num_solutions_vector_[grid_index] < max_solutions_per_grid_location_)
    num_solutions_vector_[grid_index]++;
  updateDistances(pose);
  return true;
}

bool KinematicsCache::getGridIndex(const geometry_msgs::Pose& pose, unsigned int& grid_index) const
{
  int x_index = (int)((pose.position.x - cache_origin_.x) / cache_resolution_x_);
  int y_index = (int)((pose.position.y - cache_origin_.y) / cache_resolution_y_);
  int z_index = (int)((pose.position.z - cache_origin_.z) / cache_resolution_z_);

  if (x_index >= (int)cache_size_x_ || x_index < 0)
  {
    ROS_DEBUG_NAMED("kinematics_cache", "X position %f,%d lies outside grid: %d %d", pose.position.x, x_index, 0,
                    cache_size_x_);
    return false;
  }
  if (y_index >= (int)cache_size_y_ || y_index < 0)
  {
    ROS_DEBUG_NAMED("kinematics_cache", "Y position %f,%d lies outside grid: %d %d", pose.position.y, y_index, 0,
                    cache_size_y_);
    return false;
  }
  if (z_index >= (int)cache_size_z_ || z_index < 0)
  {
    ROS_DEBUG_NAMED("kinematics_cache", "Z position %f,%d lies outside grid: %d %d", pose.position.z, z_index, 0,
                    cache_size_z_);
    return false;
  }
  ROS_DEBUG_NAMED("kinematics_cache", "Grid indices: %d %d %d", x_index, y_index, z_index);
  grid_index = (x_index + y_index * cache_size_x_ + z_index * cache_size_x_ * cache_size_y_);
  return true;
}

bool KinematicsCache::getNumSolutions(const geometry_msgs::Pose& pose, unsigned int& num_solutions) const
{
  unsigned int grid_index;
  if (!getGridIndex(pose, grid_index))
    return false;

  num_solutions = num_solutions_vector_[grid_index];
  return true;
}

unsigned int KinematicsCache::getSolutionLocation(unsigned int& grid_index, unsigned int& solution_index) const
{
  ROS_DEBUG_NAMED("kinematics_cache", "[Grid Index, Solution number location]: %d, %d", grid_index,
                  grid_index * size_grid_node_ + solution_index * solution_dimension_);
  return (grid_index * size_grid_node_ + solution_index * solution_dimension_);
}

bool KinematicsCache::getSolution(const geometry_msgs::Pose& pose, unsigned int solution_index,
                                  std::vector<double>& solution) const
{
  unsigned int grid_index;
  if (!getGridIndex(pose, grid_index))
    return false;
  if (solution_index >= max_solutions_per_grid_location_)
    return false;
  if (solution.size() < solution_dimension_)
    return false;

  solution = getSolution(grid_index, solution_index);
  return true;
}

bool KinematicsCache::getSolutions(const geometry_msgs::Pose& pose, std::vector<std::vector<double> >& solution) const
{
  unsigned int grid_index;
  if (!getGridIndex(pose, grid_index))
    return false;
  if (solution.size() != num_solutions_vector_[grid_index])
    return false;
  for (unsigned int i = 0; i < solution.size(); i++)
  {
    if (solution[i].size() != solution_dimension_)
      return false;
    solution[i] = getSolution(grid_index, i);
  }
  return true;
}

std::vector<double> KinematicsCache::getSolution(unsigned int grid_index, unsigned int solution_index) const
{
  std::vector<double> solution_local(solution_dimension_);
  unsigned int solution_location = getSolutionLocation(grid_index, solution_index);
  for (unsigned int i = 0; i < solution_dimension_; ++i)
    solution_local[i] = kinematics_cache_vector_[solution_location + i];
  return solution_local;
}

bool KinematicsCache::readFromFile(const std::string& filename)
{
  std::ifstream file(filename.c_str());
  if (!file.is_open())
  {
    ROS_DEBUG_NAMED("kinematics_cache", "Could not open file: %s", filename.c_str());
    return false;
  }
  std::string group_name;
  std::getline(file, group_name);
  if (group_name.empty())
  {
    ROS_ERROR_NAMED("kinematics_cache", "Could not find group_name in file: %s", group_name.c_str());
    file.close();
    return false;
  }
  if (group_name != kinematics_solver_->getGroupName())
  {
    ROS_ERROR_NAMED("kinematics_cache", "Input file group name %s does not match solver group name %s",
                    group_name.c_str(), kinematics_solver_->getGroupName().c_str());
    file.close();
    return false;
  }

  std::string line_string;
  std::getline(file, line_string);
  std::stringstream line_stream(line_string);
  line_stream >> options_.origin.x >> options_.origin.y >> options_.origin.z;

  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  line_stream >> options_.workspace_size[0] >> options_.workspace_size[1] >> options_.workspace_size[2];

  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  line_stream >> options_.resolution[0] >> options_.resolution[1] >> options_.resolution[2];

  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  line_stream >> options_.max_solutions_per_grid_location;

  setup(options_);

  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  line_stream >> min_squared_distance_;

  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  line_stream >> max_squared_distance_;

  std::vector<double> kinematics_cache_vector;
  std::vector<unsigned int> num_solutions_vector;
  std::getline(file, line_string);
  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  double d;
  while (line_stream >> d)
  {
    kinematics_cache_vector.push_back(d);
  }

  std::getline(file, line_string);
  std::getline(file, line_string);
  line_stream.clear();
  line_stream.str("");
  line_stream << line_string;
  unsigned int index;
  while (line_stream >> index)
  {
    num_solutions_vector.push_back(index);
  }

  file.close();

  kinematics_cache_vector_ = kinematics_cache_vector;
  num_solutions_vector_ = num_solutions_vector;
  ROS_DEBUG_NAMED("kinematics_cache", "Read %d total points from file: %s", (int)num_solutions_vector_.size(),
                  filename.c_str());
  return true;
}

bool KinematicsCache::writeToFile(const std::string& filename)
{
  ROS_DEBUG_NAMED("kinematics_cache", "Writing %d total points to file: %s", (int)num_solutions_vector_.size(),
                  filename.c_str());
  std::ofstream file;
  file.open(filename.c_str());
  if (!file.is_open())
  {
    ROS_DEBUG_NAMED("kinematics_cache", "Could not open file: %s", filename.c_str());
    return false;
  }
  if (file.good())
  {
    std::string group_name = kinematics_solver_->getGroupName();
    file << group_name << std::endl;

    file << options_.origin.x << " " << options_.origin.y << " " << options_.origin.z << std::endl;
    file << options_.workspace_size[0] << " " << options_.workspace_size[1] << " " << options_.workspace_size[2]
         << std::endl;
    file << options_.resolution[0] << " " << options_.resolution[1] << " " << options_.resolution[2] << std::endl;
    file << options_.max_solutions_per_grid_location << std::endl;
    file << min_squared_distance_ << std::endl;
    file << max_squared_distance_ << std::endl;
    file << kinematics_cache_vector_.size() << std::endl;
    std::copy(kinematics_cache_vector_.begin(), kinematics_cache_vector_.end(),
              std::ostream_iterator<double>(file, " "));
    file << std::endl;

    file << num_solutions_vector_.size() << std::endl;
    std::copy(num_solutions_vector_.begin(), num_solutions_vector_.end(),
              std::ostream_iterator<unsigned int>(file, " "));
    file << std::endl;
  }

  file.close();
  return true;
}

std::pair<double, double> KinematicsCache::getMinMaxSquaredDistance()
{
  return std::pair<double, double>(min_squared_distance_, max_squared_distance_);
}

void KinematicsCache::updateDistances(const geometry_msgs::Pose& pose)
{
  double distance_squared =
      (pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z);
  min_squared_distance_ = std::min<double>(distance_squared, min_squared_distance_);
  max_squared_distance_ = std::max<double>(distance_squared, max_squared_distance_);
}
}  // namespace kinematics_cache
