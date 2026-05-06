/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Mohamad Ayman.
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

/* Author: Mohamad Ayman */
#include <gtest/gtest.h>
#include <sstream>
#include <ctype.h>
#include <iostream>  // For writing yaml and launch files
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <moveit/setup_assistant/tools/moveit_config_data.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <ros/package.h>

// This tests writing/parsing of ros_controller.yaml
class MoveItConfigData : public testing::Test
{
protected:
  void SetUp() override
  {
    std::string robot_name = "panda";
    urdf_model_ = moveit::core::loadModelInterface(robot_name);
    srdf_model_ = moveit::core::loadSRDFModel(robot_name);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_model_);
  };

protected:
  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(MoveItConfigData, ReadingControllers)
{
  // Contains all the configuration data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data;

  config_data = std::make_shared<moveit_setup_assistant::MoveItConfigData>();
  config_data->srdf_->srdf_model_ = srdf_model_;
  config_data->setRobotModel(robot_model_);

  // Initially no controllers
  EXPECT_EQ(config_data->getControllers().size(), 0u);

  // Adding default controllers, a controller for each planning group
  config_data->addDefaultControllers();

  // Number of the planning groups defined in the model srdf
  size_t group_count = config_data->srdf_->srdf_model_->getGroups().size();

  // Test that addDefaultControllers() did accually add a controller for the new_group
  EXPECT_EQ(config_data->getControllers().size(), group_count);

  // Temporary file used during the test and is deleted when the test is finished
  char test_file[] = "/tmp/msa_unittest_ros_controller.yaml";

  // ros_controller.yaml written correctly
  EXPECT_EQ(config_data->outputROSControllersYAML(test_file), true);

  // Reset MoveIt config MoveItConfigData
  config_data = std::make_shared<moveit_setup_assistant::MoveItConfigData>();

  // Initially no controllers
  EXPECT_EQ(config_data->getControllers().size(), 0u);

  // ros_controllers.yaml read correctly
  EXPECT_EQ(config_data->inputROSControllersYAML(test_file), true);

  // ros_controllers.yaml parsed correctly
  EXPECT_EQ(config_data->getControllers().size(), group_count);

  // Remove ros_controllers.yaml temp file which was used in testing
  boost::filesystem::remove(test_file);
}

// This tests parsing of sensors_3d.yaml
TEST_F(MoveItConfigData, ReadingSensorsConfig)
{
  // Contains all the config data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data;
  config_data = std::make_shared<moveit_setup_assistant::MoveItConfigData>();

  boost::filesystem::path setup_assistant_path(config_data->setup_assistant_path_);

  // Before parsing, no config available
  EXPECT_EQ(config_data->getSensorPluginConfig().size(), 0u);

  // Read the file containing the default config parameters
  config_data->input3DSensorsYAML(
      (setup_assistant_path / "templates/moveit_config_pkg_template/config/sensors_3d.yaml").string());
  auto configs = config_data->getSensorPluginConfig();

  // Default config for the two available sensor plugins
  // Make sure both are parsed correctly
  ASSERT_EQ(configs.size(), 2u);

  EXPECT_EQ(configs[0]["sensor_plugin"].getValue(), std::string("occupancy_map_monitor/PointCloudOctomapUpdater"));

  EXPECT_EQ(configs[1]["sensor_plugin"].getValue(), std::string("occupancy_map_monitor/DepthImageOctomapUpdater"));
}

// This tests writing of sensors_3d.yaml
TEST_F(MoveItConfigData, WritingSensorsConfig)
{
  // Contains all the config data for the setup assistant
  moveit_setup_assistant::MoveItConfigDataPtr config_data;
  config_data = std::make_shared<moveit_setup_assistant::MoveItConfigData>();

  // Empty Config Should have No Sensors
  EXPECT_EQ(config_data->getSensorPluginConfig().size(), 0u);

  // Temporary file used during the test and is deleted when the test is finished
  char test_file[] = "/tmp/msa_unittest_sensors.yaml";

  // empty sensors.yaml written correctly
  EXPECT_EQ(config_data->output3DSensorPluginYAML(test_file), true);

  // Set default file
  std::string default_file_path =
      config_data->setup_assistant_path_ + "/templates/moveit_config_pkg_template/config/sensors_3d.yaml";

  // Read from the written (empty) file
  auto config = moveit_setup_assistant::MoveItConfigData::load3DSensorsYAML(test_file);
  // Should have No Sensors
  EXPECT_EQ(config.size(), 0u);

  // Now load the default file and write it to a file
  config_data = std::make_shared<moveit_setup_assistant::MoveItConfigData>();
  config_data->input3DSensorsYAML(default_file_path);
  EXPECT_EQ(config_data->getSensorPluginConfig().size(), 2u);
  EXPECT_EQ(config_data->output3DSensorPluginYAML(test_file), true);

  // Read from the written file
  config = moveit_setup_assistant::MoveItConfigData::load3DSensorsYAML(test_file);
  EXPECT_EQ(config.size(), 2u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
