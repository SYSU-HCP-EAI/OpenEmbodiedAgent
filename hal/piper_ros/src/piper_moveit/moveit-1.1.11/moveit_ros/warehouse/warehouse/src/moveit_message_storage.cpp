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

#include <moveit/warehouse/moveit_message_storage.h>
#include <warehouse_ros/database_loader.h>
#include <boost/regex.hpp>
#include <memory>
#include <utility>

moveit_warehouse::MoveItMessageStorage::MoveItMessageStorage(warehouse_ros::DatabaseConnection::Ptr conn)
  : conn_(std::move(conn))
{
}

void moveit_warehouse::MoveItMessageStorage::filterNames(const std::string& regex, std::vector<std::string>& names) const
{
  if (!regex.empty())
  {
    std::vector<std::string> fnames;
    boost::regex r(regex);
    for (std::string& name : names)
    {
      boost::cmatch match;
      if (boost::regex_match(name.c_str(), match, r))
        fnames.push_back(name);
    }
    names.swap(fnames);
  }
}

static std::unique_ptr<warehouse_ros::DatabaseLoader> DBLOADER;

typename warehouse_ros::DatabaseConnection::Ptr moveit_warehouse::loadDatabase()
{
  if (!DBLOADER)
  {
    DBLOADER = std::make_unique<warehouse_ros::DatabaseLoader>();
  }
  return DBLOADER->loadDatabase();
  // return typename warehouse_ros::DatabaseConnection::Ptr(new warehouse_ros_mongo::MongoDatabaseConnection());
}
