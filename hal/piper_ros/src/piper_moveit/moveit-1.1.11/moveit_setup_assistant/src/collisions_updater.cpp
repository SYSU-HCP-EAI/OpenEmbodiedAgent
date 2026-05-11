/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fraunhofer IPA
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
 *   * Neither the name of Fraunhofer IPA nor the names of its
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

/* Author: Mathias Lüdtke */

#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include <moveit/setup_assistant/tools/compute_default_collisions.h>

#include <moveit/rdf_loader/rdf_loader.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

bool loadSetupAssistantConfig(moveit_setup_assistant::MoveItConfigData& config_data, const std::string& pkg_path)
{
  if (!config_data.setPackagePath(pkg_path))
  {
    ROS_ERROR_STREAM("Could not set package path '" << pkg_path << "'");
    return false;
  }

  std::string setup_assistant_path;
  if (!config_data.getSetupAssistantYAMLPath(setup_assistant_path))
  {
    ROS_ERROR_STREAM("Could not resolve path to .setup_assistant");
    return false;
  }

  if (!config_data.inputSetupAssistantYAML(setup_assistant_path))
  {
    ROS_ERROR_STREAM("Could not parse .setup_assistant file from '" << setup_assistant_path << "'");
    return false;
  }

  config_data.createFullURDFPath();  // might fail at this point

  config_data.createFullSRDFPath(config_data.config_pkg_path_);  // might fail at this point

  return true;
}

bool setup(moveit_setup_assistant::MoveItConfigData& config_data, bool keep_old,
           const std::vector<std::string>& xacro_args)
{
  std::string urdf_string;
  if (!rdf_loader::RDFLoader::loadXmlFileToString(urdf_string, config_data.urdf_path_, xacro_args))
  {
    ROS_ERROR_STREAM("Could not load URDF from '" << config_data.urdf_path_ << "'");
    return false;
  }
  if (!config_data.urdf_model_->initString(urdf_string))
  {
    ROS_ERROR_STREAM("Could not parse URDF from '" << config_data.urdf_path_ << "'");
    return false;
  }

  std::string srdf_string;
  if (!rdf_loader::RDFLoader::loadXmlFileToString(srdf_string, config_data.srdf_path_, xacro_args))
  {
    ROS_ERROR_STREAM("Could not load SRDF from '" << config_data.srdf_path_ << "'");
    return false;
  }
  if (!config_data.srdf_->initString(*config_data.urdf_model_, srdf_string))
  {
    ROS_ERROR_STREAM("Could not parse SRDF from '" << config_data.srdf_path_ << "'");
    return false;
  }

  if (!keep_old)
  {
    config_data.srdf_->no_default_collision_links_.clear();
    config_data.srdf_->enabled_collision_pairs_.clear();
    config_data.srdf_->disabled_collision_pairs_.clear();
  }

  return true;
}

moveit_setup_assistant::LinkPairMap compute(moveit_setup_assistant::MoveItConfigData& config_data, uint32_t trials,
                                            double min_collision_fraction, bool verbose)
{
  // TODO: spin thread and print progess if verbose
  unsigned int collision_progress;
  return moveit_setup_assistant::computeDefaultCollisions(config_data.getPlanningScene(), &collision_progress,
                                                          trials > 0, trials, min_collision_fraction, verbose);
}

// less operation for two CollisionPairs
struct CollisionPairLess
{
  bool operator()(const srdf::Model::CollisionPair& left, const srdf::Model::CollisionPair& right) const
  {
    // use std::pair's operator<: (left.link1_, left.link2_) < (right.link1_, right.link2_)
    return left.link1_ < right.link1_ || (!(right.link1_ < left.link1_) && left.link2_ < right.link2_);
  }
};

// Update collision pairs
void updateCollisionLinkPairs(std::vector<srdf::Model::CollisionPair>& target_pairs,
                              const moveit_setup_assistant::LinkPairMap& source_pairs, size_t skip_mask)
{
  // remove duplicates
  std::set<srdf::Model::CollisionPair, CollisionPairLess> filtered;
  for (auto& p : target_pairs)
  {
    if (p.link1_ >= p.link2_)
      std::swap(p.link1_, p.link2_);  // unify link1, link2 sorting
    filtered.insert(p);
  }

  // copy the data in this class's LinkPairMap datastructure to srdf::Model::CollisionPair format
  for (const auto& link_pair : source_pairs)
  {
    // Only copy those that are actually disabled
    if (!link_pair.second.disable_check)
      continue;

    // Filter out pairs matching the skip_mask
    if ((1 << link_pair.second.reason) & skip_mask)
      continue;

    srdf::Model::CollisionPair pair;
    pair.link1_ = link_pair.first.first;
    pair.link2_ = link_pair.first.second;
    if (pair.link1_ >= pair.link2_)
      std::swap(pair.link1_, pair.link2_);
    pair.reason_ = moveit_setup_assistant::disabledReasonToString(link_pair.second.reason);

    filtered.insert(pair);
  }

  target_pairs.assign(filtered.begin(), filtered.end());
}

int main(int argc, char* argv[])
{
  std::string config_pkg_path;
  std::string urdf_path;
  std::string srdf_path;

  std::string output_path;

  bool include_default = false, include_always = false, keep_old = false, verbose = false;

  double min_collision_fraction = 1.0;

  uint32_t never_trials = 0;

  po::options_description desc("Allowed options");
  desc.add_options()("help", "show help")("config-pkg", po::value(&config_pkg_path), "path to MoveIt config package")(
      "urdf", po::value(&urdf_path),
      "path to URDF ( or xacro)")("srdf", po::value(&srdf_path),
                                  "path to SRDF ( or xacro)")("output", po::value(&output_path), "output path for SRDF")

      ("xacro-args", po::value<std::vector<std::string> >()->composing(), "additional arguments for xacro")

          ("default", po::bool_switch(&include_default), "disable default colliding pairs")(
              "always", po::bool_switch(&include_always), "disable always colliding pairs")

              ("keep", po::bool_switch(&keep_old), "keep disabled link from SRDF")("verbose", po::bool_switch(&verbose),
                                                                                   "verbose output")

                  ("trials", po::value(&never_trials), "number of trials for searching never colliding pairs")(
                      "min-collision-fraction", po::value(&min_collision_fraction),
                      "fraction of small sample size to determine links that are alwas colliding");

  po::positional_options_description pos_desc;
  pos_desc.add("xacro-args", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  moveit_setup_assistant::MoveItConfigData config_data;

  if (!config_pkg_path.empty())
  {
    if (!loadSetupAssistantConfig(config_data, config_pkg_path))
    {
      ROS_ERROR_STREAM("Could not load config at '" << config_pkg_path << "'");
      return 1;
    }
  }
  else if (urdf_path.empty() || srdf_path.empty())
  {
    ROS_ERROR_STREAM("Please provide config package or URDF and SRDF path");
    return 1;
  }
  else if (rdf_loader::RDFLoader::isXacroFile(srdf_path) && output_path.empty())
  {
    ROS_ERROR_STREAM("Please provide a different output file for SRDF xacro input file");
    return 1;
  }

  // overwrite config paths if applicable
  if (!urdf_path.empty())
    config_data.urdf_path_ = urdf_path;
  if (!srdf_path.empty())
    config_data.srdf_path_ = srdf_path;

  std::vector<std::string> xacro_args;
  if (vm.count("xacro-args"))
    xacro_args = vm["xacro-args"].as<std::vector<std::string> >();

  if (!setup(config_data, keep_old, xacro_args))
  {
    ROS_ERROR_STREAM("Could not setup updater");
    return 1;
  }

  moveit_setup_assistant::LinkPairMap link_pairs = compute(config_data, never_trials, min_collision_fraction, verbose);

  size_t skip_mask = 0;
  if (!include_default)
    skip_mask |= (1 << moveit_setup_assistant::DEFAULT);
  if (!include_always)
    skip_mask |= (1 << moveit_setup_assistant::ALWAYS);

  updateCollisionLinkPairs(config_data.srdf_->disabled_collision_pairs_, link_pairs, skip_mask);

  config_data.srdf_->writeSRDF(output_path.empty() ? config_data.srdf_path_ : output_path);

  return 0;
}
