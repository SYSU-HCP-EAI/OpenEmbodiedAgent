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

/* Author: Ioan Sucan, Dave Coleman */

#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <pluginlib/class_loader.hpp>
#include <boost/thread/mutex.hpp>
#include <sstream>
#include <vector>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <moveit/profiler/profiler.h>

namespace kinematics_plugin_loader
{
static const std::string LOGNAME = "kinematics_plugin_loader";

class KinematicsPluginLoader::KinematicsLoaderImpl
{
public:
  /**
   * \brief Pimpl Implementation of KinematicsLoader
   * \param robot_description
   * \param possible_kinematics_solvers
   * \param search_res
   * \param iksolver_to_tip_links - a map between each ik solver and a vector of custom-specified tip link(s)
   */
  KinematicsLoaderImpl(const std::string& robot_description,
                       const std::map<std::string, std::vector<std::string>>& possible_kinematics_solvers,
                       const std::map<std::string, std::vector<double>>& search_res,
                       const std::map<std::string, std::vector<std::string>>& iksolver_to_tip_links)
    : robot_description_(robot_description)
    , possible_kinematics_solvers_(possible_kinematics_solvers)
    , search_res_(search_res)
    , iksolver_to_tip_links_(iksolver_to_tip_links)
  {
    try
    {
      kinematics_loader_ =
          std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>("moveit_core", "kinematics::"
                                                                                              "KinematicsBase");
    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_ERROR_NAMED(LOGNAME, "Unable to construct kinematics loader. Error: %s", e.what());
    }
  }

  /**
   * \brief Helper function to decide which, and how many, tip frames a planning group has
   * \param jmg - joint model group pointer
   * \return tips - list of valid links in a planning group to plan for
   */
  std::vector<std::string> chooseTipFrames(const moveit::core::JointModelGroup* jmg)
  {
    std::vector<std::string> tips;
    std::map<std::string, std::vector<std::string>>::const_iterator ik_it = iksolver_to_tip_links_.find(jmg->getName());

    // Check if tips were loaded onto the rosparam server previously
    if (ik_it != iksolver_to_tip_links_.end())
    {
      // the tip is being chosen based on a corresponding rosparam ik link
      ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader", "Choosing tip frame of kinematic solver for group "
                                                             << jmg->getName()
                                                             << " based on values in rosparam server.");
      tips = ik_it->second;
    }
    else
    {
      // get the last link in the chain
      ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader", "Choosing tip frame of kinematic solver for group "
                                                             << jmg->getName() << " based on last link in chain");

      tips.push_back(jmg->getLinkModels().back()->getName());
    }

    // Error check
    if (tips.empty())
    {
      ROS_ERROR_STREAM_NAMED("kinematics_plugin_loader", "Error choosing kinematic solver tip frame(s).");
    }

    // Debug tip choices
    std::stringstream tip_debug;
    tip_debug << "Planning group '" << jmg->getName() << "' has tip(s): ";
    for (const std::string& tip : tips)
      tip_debug << tip << ", ";
    ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader", tip_debug.str());

    return tips;
  }

  kinematics::KinematicsBasePtr allocKinematicsSolver(const moveit::core::JointModelGroup* jmg)
  {
    kinematics::KinematicsBasePtr result;
    if (!kinematics_loader_)
    {
      ROS_ERROR_NAMED(LOGNAME, "Invalid kinematics loader.");
      return result;
    }
    if (!jmg)
    {
      ROS_ERROR_NAMED(LOGNAME, "Specified group is NULL. Cannot allocate kinematics solver.");
      return result;
    }
    const std::vector<const moveit::core::LinkModel*>& links = jmg->getLinkModels();
    if (links.empty())
    {
      ROS_ERROR_NAMED(LOGNAME, "No links specified for group '%s'. Cannot allocate kinematics solver.",
                      jmg->getName().c_str());
      return result;
    }

    ROS_DEBUG_NAMED(LOGNAME, "Trying to allocate kinematics solver for group '%s'", jmg->getName().c_str());

    std::map<std::string, std::vector<std::string>>::const_iterator it =
        possible_kinematics_solvers_.find(jmg->getName());
    if (it == possible_kinematics_solvers_.end())
    {
      ROS_DEBUG_NAMED(LOGNAME, "No kinematics solver available for this group");
      return result;
    }

    const std::string& base = links.front()->getParentJointModel()->getParentLinkModel() ?
                                  links.front()->getParentJointModel()->getParentLinkModel()->getName() :
                                  jmg->getParentModel().getModelFrame();

    // just to be sure, do not call the same pluginlib instance allocation function in parallel
    boost::mutex::scoped_lock slock(lock_);
    for (std::size_t i = 0; !result && i < it->second.size(); ++i)
    {
      try
      {
        result = kinematics_loader_->createUniqueInstance(it->second[i]);
        if (result)
        {
          // choose the tip of the IK solver
          const std::vector<std::string> tips = chooseTipFrames(jmg);

          // choose search resolution
          double search_res = search_res_.find(jmg->getName())->second[i];  // we know this exists, by construction

          if (!result->initialize(jmg->getParentModel(), jmg->getName(),
                                  (base.empty() || base[0] != '/') ? base : base.substr(1), tips, search_res) &&
              // on failure: fallback to old method (TODO: remove in future)
              !result->initialize(robot_description_, jmg->getName(),
                                  (base.empty() || base[0] != '/') ? base : base.substr(1), tips, search_res))
          {
            ROS_ERROR_NAMED(LOGNAME, "Kinematics solver of type '%s' could not be initialized for group '%s'",
                            it->second[i].c_str(), jmg->getName().c_str());
            result.reset();
            continue;
          }

          result->setDefaultTimeout(jmg->getDefaultIKTimeout());
          ROS_DEBUG_NAMED(LOGNAME,
                          "Successfully allocated and initialized a kinematics solver of type '%s' with search "
                          "resolution %lf for group '%s' at address %p",
                          it->second[i].c_str(), search_res, jmg->getName().c_str(), result.get());
          break;
        }
      }
      catch (pluginlib::PluginlibException& e)
      {
        ROS_ERROR_NAMED(LOGNAME, "The kinematics plugin (%s) failed to load. Error: %s", it->first.c_str(), e.what());
      }
    }

    if (!result)
    {
      ROS_DEBUG_NAMED(LOGNAME, "No usable kinematics solver was found for this group.\n"
                               "Did you load kinematics.yaml into your node's namespace?");
    }
    return result;
  }

  // cache solver between two consecutive calls
  // first call in RobotModelLoader::loadKinematicsSolvers() is just to check suitability for jmg
  // second call in JointModelGroup::setSolverAllocators() is to actually retrieve the instance for use
  kinematics::KinematicsBasePtr allocKinematicsSolverWithCache(const moveit::core::JointModelGroup* jmg)
  {
    boost::mutex::scoped_lock slock(cache_lock_);
    kinematics::KinematicsBasePtr& cached = instances_[jmg];
    if (cached.unique())
      return std::move(cached);  // pass on unique instance

    // create a new instance and store in instances_
    cached = allocKinematicsSolver(jmg);
    return cached;
  }

  void status() const
  {
    for (std::map<std::string, std::vector<std::string>>::const_iterator it = possible_kinematics_solvers_.begin();
         it != possible_kinematics_solvers_.end(); ++it)
      for (std::size_t i = 0; i < it->second.size(); ++i)
        ROS_INFO_NAMED(LOGNAME, "Solver for group '%s': '%s' (search resolution = %lf)", it->first.c_str(),
                       it->second[i].c_str(), search_res_.at(it->first)[i]);
  }

private:
  std::string robot_description_;
  std::map<std::string, std::vector<std::string>> possible_kinematics_solvers_;
  std::map<std::string, std::vector<double>> search_res_;
  std::map<std::string, std::vector<std::string>> iksolver_to_tip_links_;  // a map between each ik solver and a vector
                                                                           // of custom-specified tip link(s)
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader_;
  std::map<const moveit::core::JointModelGroup*, kinematics::KinematicsBasePtr> instances_;
  boost::mutex lock_;
  boost::mutex cache_lock_;
};

void KinematicsPluginLoader::status() const
{
  if (loader_)
    loader_->status();
  else
    ROS_INFO_NAMED(LOGNAME, "Loader function was never required");
}

moveit::core::SolverAllocatorFn KinematicsPluginLoader::getLoaderFunction()
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("KinematicsPluginLoader::getLoaderFunction");

  if (loader_)
    return [&loader = *loader_](const moveit::core::JointModelGroup* jmg) {
      return loader.allocKinematicsSolverWithCache(jmg);
    };

  rdf_loader::RDFLoader rml(robot_description_);
  robot_description_ = rml.getRobotDescription();
  return getLoaderFunction(rml.getSRDF());
}

moveit::core::SolverAllocatorFn KinematicsPluginLoader::getLoaderFunction(const srdf::ModelSharedPtr& srdf_model)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("KinematicsPluginLoader::getLoaderFunction(SRDF)");

  if (!loader_)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Configuring kinematics solvers");
    groups_.clear();

    std::map<std::string, std::vector<std::string>> possible_kinematics_solvers;
    std::map<std::string, std::vector<double>> search_res;
    std::map<std::string, std::vector<std::string>> iksolver_to_tip_links;

    if (srdf_model)
    {
      const std::vector<srdf::Model::Group>& known_groups = srdf_model->getGroups();
      if (default_search_resolution_ <= std::numeric_limits<double>::epsilon())
        default_search_resolution_ = kinematics::KinematicsBase::DEFAULT_SEARCH_DISCRETIZATION;

      if (default_solver_plugin_.empty())
      {
        ROS_DEBUG_NAMED(LOGNAME, "Loading settings for kinematics solvers from the ROS param server ...");

        // read data using ROS params
        ros::NodeHandle nh("~");

        // read the list of plugin names for possible kinematics solvers
        for (const srdf::Model::Group& known_group : known_groups)
        {
          std::string base_param_name = known_group.name_;
          ROS_DEBUG_NAMED("kinematics_plugin_loader", "Looking for param %s ",
                          (base_param_name + "/kinematics_solver").c_str());
          std::string ksolver_param_name;
          bool found = nh.searchParam(base_param_name + "/kinematics_solver", ksolver_param_name);
          if (!found || !nh.hasParam(ksolver_param_name))
          {
            base_param_name = robot_description_ + "_kinematics/" + known_group.name_;
            ROS_DEBUG_NAMED(LOGNAME, "Looking for param %s ", (base_param_name + "/kinematics_solver").c_str());
            found = nh.searchParam(base_param_name + "/kinematics_solver", ksolver_param_name);
          }
          if (found)
          {
            ROS_DEBUG_NAMED(LOGNAME, "Found param %s", ksolver_param_name.c_str());
            std::string ksolver;
            if (nh.getParam(ksolver_param_name, ksolver))
            {
              std::stringstream ss(ksolver);
              bool first = true;
              while (ss.good() && !ss.eof())
              {
                if (first)
                {
                  first = false;
                  groups_.push_back(known_group.name_);
                }
                std::string solver;
                ss >> solver >> std::ws;
                possible_kinematics_solvers[known_group.name_].push_back(solver);
                ROS_DEBUG_NAMED(LOGNAME, "Using kinematics solver '%s' for group '%s'.", solver.c_str(),
                                known_group.name_.c_str());
              }
            }
          }

          std::string ksolver_timeout_param_name;
          if (nh.searchParam(base_param_name + "/kinematics_solver_timeout", ksolver_timeout_param_name))
          {
            double ksolver_timeout;
            if (nh.getParam(ksolver_timeout_param_name, ksolver_timeout))
              ik_timeout_[known_group.name_] = ksolver_timeout;
            else
            {  // just in case this is an int
              int ksolver_timeout_i;
              if (nh.getParam(ksolver_timeout_param_name, ksolver_timeout_i))
                ik_timeout_[known_group.name_] = ksolver_timeout_i;
            }
          }

          // TODO: Remove in future release (deprecated in PR #1288, Jan-2019, Melodic)
          std::string ksolver_attempts_param_name;
          if (nh.searchParam(base_param_name + "/kinematics_solver_attempts", ksolver_attempts_param_name) &&
              nh.hasParam(ksolver_attempts_param_name))
          {
            ROS_WARN_ONCE_NAMED(LOGNAME,
                                "Kinematics solver doesn't support #attempts anymore, but only a timeout.\n"
                                "Please remove the parameter '%s' from your configuration.",
                                ksolver_attempts_param_name.c_str());
          }

          std::string ksolver_res_param_name;
          if (nh.searchParam(base_param_name + "/kinematics_solver_search_resolution", ksolver_res_param_name))
          {
            std::string ksolver_res;
            if (nh.getParam(ksolver_res_param_name, ksolver_res))
            {
              std::stringstream ss(ksolver_res);
              while (ss.good() && !ss.eof())
              {
                double res;
                ss >> res >> std::ws;
                search_res[known_group.name_].push_back(res);
              }
            }
            else
            {  // handle the case this param is just one value and parsed as a double
              double res;
              if (nh.getParam(ksolver_res_param_name, res))
                search_res[known_group.name_].push_back(res);
              else
              {
                int res_i;
                if (nh.getParam(ksolver_res_param_name, res_i))
                  search_res[known_group.name_].push_back(res_i);
              }
            }
          }

          // Allow a kinematic solver's tip link to be specified on the rosparam server
          // Depreciated in favor of array version now
          std::string ksolver_ik_link_param_name;
          if (nh.searchParam(base_param_name + "/kinematics_solver_ik_link", ksolver_ik_link_param_name))
          {
            std::string ksolver_ik_link;
            if (nh.getParam(ksolver_ik_link_param_name, ksolver_ik_link))  // has a custom rosparam-based tip link
            {
              ROS_WARN_STREAM_NAMED(LOGNAME, "Using kinematics_solver_ik_link rosparam is "
                                             "deprecated in favor of kinematics_solver_ik_links "
                                             "rosparam array.");
              iksolver_to_tip_links[known_group.name_].push_back(ksolver_ik_link);
            }
          }

          // Allow a kinematic solver's tip links to be specified on the rosparam server as an array
          std::string ksolver_ik_links_param_name;
          if (nh.searchParam(base_param_name + "/kinematics_solver_ik_links", ksolver_ik_links_param_name))
          {
            XmlRpc::XmlRpcValue ksolver_ik_links;
            if (nh.getParam(ksolver_ik_links_param_name, ksolver_ik_links))  // has custom rosparam-based tip link(s)
            {
              if (ksolver_ik_links.getType() != XmlRpc::XmlRpcValue::TypeArray)
              {
                ROS_WARN_STREAM_NAMED(LOGNAME, "rosparam '" << ksolver_ik_links_param_name
                                                            << "' should be an XmlRpc "
                                                               "value type 'Array'");
              }
              else
              {
                for (int32_t j = 0; j < ksolver_ik_links.size(); ++j)  // NOLINT(modernize-loop-convert)
                {
                  ROS_ASSERT(ksolver_ik_links[j].getType() == XmlRpc::XmlRpcValue::TypeString);
                  ROS_DEBUG_STREAM_NAMED(LOGNAME, "found tip " << static_cast<std::string>(ksolver_ik_links[j])
                                                               << " for group " << known_group.name_);
                  iksolver_to_tip_links[known_group.name_].push_back(static_cast<std::string>(ksolver_ik_links[j]));
                }
              }
            }
          }

          // make sure there is a default resolution at least specified for every solver (in case it was not specified
          // on the param server)
          while (search_res[known_group.name_].size() < possible_kinematics_solvers[known_group.name_].size())
            search_res[known_group.name_].push_back(default_search_resolution_);
        }
      }
      else
      {
        ROS_DEBUG_NAMED(LOGNAME, "Using specified default settings for kinematics solvers ...");
        for (const srdf::Model::Group& known_group : known_groups)
        {
          possible_kinematics_solvers[known_group.name_].resize(1, default_solver_plugin_);
          search_res[known_group.name_].resize(1, default_search_resolution_);
          ik_timeout_[known_group.name_] = default_solver_timeout_;
          groups_.push_back(known_group.name_);
        }
      }
    }

    loader_ = std::make_shared<KinematicsLoaderImpl>(robot_description_, possible_kinematics_solvers, search_res,
                                                     iksolver_to_tip_links);
  }

  return [&loader = *loader_](const moveit::core::JointModelGroup* jmg) {
    return loader.allocKinematicsSolverWithCache(jmg);
  };
}
}  // namespace kinematics_plugin_loader
