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

/* Author: Dave Coleman */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>  // for getting kinematic model
#include <yaml-cpp/yaml.h>                         // outputing yaml config files
#include <urdf/model.h>                            // to share throughout app
#include <srdfdom/srdf_writer.h>                   // for writing srdf data

#include <utility>

namespace moveit_setup_assistant
{
// ******************************************************************************************
// Constants
// ******************************************************************************************

// Used for loading kinematic model
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MOVEIT_ROBOT_STATE = "moveit_robot_state";

// Default kin solver values
static const double DEFAULT_KIN_SOLVER_SEARCH_RESOLUTION = 0.005;
static const double DEFAULT_KIN_SOLVER_TIMEOUT = 0.005;

// ******************************************************************************************
// Structs
// ******************************************************************************************

/**
 * Planning groups extra data not found in srdf but used in config files
 */
struct GroupMetaData
{
  std::string kinematics_solver_;               // Name of kinematics plugin to use
  double kinematics_solver_search_resolution_;  // resolution to use with solver
  double kinematics_solver_timeout_;            // solver timeout
  std::string kinematics_parameters_file_;      // file for additional kinematics parameters
  std::string default_planner_;                 // Name of the default planner to use
};

/**
 * Controllers settings which may be set in the config files
 */
struct ControllerConfig
{
  std::string name_;                 // controller name
  std::string type_;                 // controller type
  std::vector<std::string> joints_;  // joints controller by this controller
};

/**
 * Planning parameters which may be set in the config files
 */
struct OmplPlanningParameter
{
  std::string name;     // name of parameter
  std::string value;    // value parameter will receive (but as a string)
  std::string comment;  // comment briefly describing what this parameter does
};

/** \brief This class describes the OMPL planners by name, type, and parameter list, used to create the
 * ompl_planning.yaml file */
class OMPLPlannerDescription
{
public:
  /** \brief Constructor
   *  @param name: name of planner
   *  @parameter type: type of planner
   */
  OMPLPlannerDescription(const std::string& name, const std::string& type)
  {
    name_ = name;
    type_ = type;
  };
  /** \brief Destructor */
  ~OMPLPlannerDescription()
  {
    parameter_list_.clear();
  };
  /** \brief adds a parameter to the planner description
   * @param name: name of parameter to add
   * @parameter: value: value of parameter as a string
   *  @parameter: value: value of parameter as a string
   */
  void addParameter(const std::string& name, const std::string& value = "", const std::string& comment = "")
  {
    OmplPlanningParameter temp;
    temp.name = name;
    temp.value = value;
    temp.comment = comment;
    parameter_list_.push_back(temp);
  }
  std::vector<OmplPlanningParameter> parameter_list_;
  std::string name_;  // name of planner
  std::string type_;  // type of planner (geometric)
};

/**
 * Reusable parameter class which may be used in reading and writing configuration files
 */
class GenericParameter
{
public:
  GenericParameter()
  {
    comment_ = "";
  };

  void setName(std::string name)
  {
    name_ = std::move(name);
  };
  void setValue(std::string value)
  {
    value_ = std::move(value);
  };
  void setComment(std::string comment)
  {
    comment_ = std::move(comment);
  };
  const std::string& getName() const
  {
    return name_;
  };
  const std::string& getValue() const
  {
    return value_;
  };
  const std::string& getComment() const
  {
    return comment_;
  };

private:
  std::string name_;     // name of parameter
  std::string value_;    // value parameter will receive (but as a string)
  std::string comment_;  // comment briefly describing what this parameter does
};

MOVEIT_CLASS_FORWARD(MoveItConfigData);  // Defines MoveItConfigDataPtr, ConstPtr, WeakPtr... etc

/** \brief This class is shared with all widgets and contains the common configuration data
    needed for generating each robot's MoveIt configuration package.

    All SRDF data is contained in a subclass of this class -
    srdf_writer.cpp. This class also contains the functions for writing
    out the configuration files. */
class MoveItConfigData
{
public:
  MoveItConfigData();
  ~MoveItConfigData();

  // bits of information that can be entered in Setup Assistant
  enum InformationFields
  {
    COLLISIONS = 1 << 1,
    VIRTUAL_JOINTS = 1 << 2,
    GROUPS = 1 << 3,
    GROUP_CONTENTS = 1 << 4,
    GROUP_KINEMATICS = 1 << 5,
    POSES = 1 << 6,
    END_EFFECTORS = 1 << 7,
    PASSIVE_JOINTS = 1 << 8,
    SIMULATION = 1 << 9,
    AUTHOR_INFO = 1 << 10,
    SENSORS_CONFIG = 1 << 11,
    SRDF = COLLISIONS | VIRTUAL_JOINTS | GROUPS | GROUP_CONTENTS | POSES | END_EFFECTORS | PASSIVE_JOINTS
  };
  unsigned long changes;  // bitfield of changes (composed of InformationFields)

  // All of the data needed for creating a MoveIt Configuration Files

  // ******************************************************************************************
  // URDF Data
  // ******************************************************************************************

  /// Full file-system path to urdf
  std::string urdf_path_;

  /// Name of package containig urdf (note: this may be empty b/c user may not have urdf in pkg)
  std::string urdf_pkg_name_;

  /// Path relative to urdf package (note: this may be same as urdf_path_)
  std::string urdf_pkg_relative_path_;

  /// Flag indicating whether the URDF was loaded from .xacro format
  bool urdf_from_xacro_;
  /// xacro arguments
  std::string xacro_args_;

  /// URDF robot model
  urdf::ModelSharedPtr urdf_model_;

  /// URDF robot model string
  std::string urdf_string_;

  /// Gazebo URDF robot model string
  // NOTE: Created when the robot urdf is not compatible with Gazebo.
  std::string gazebo_urdf_string_;

  // ******************************************************************************************
  // SRDF Data
  // ******************************************************************************************

  /// Full file-system path to srdf
  std::string srdf_path_;

  /// Path relative to loaded configuration package
  std::string srdf_pkg_relative_path_;

  /// SRDF Data and Writer
  srdf::SRDFWriterPtr srdf_;

  // ******************************************************************************************
  // Other Data
  // ******************************************************************************************

  /// Planning groups extra data not found in srdf but used in config files
  std::map<std::string, GroupMetaData> group_meta_data_;

  /// Setup Assistants package's path for when we use its templates
  std::string setup_assistant_path_;

  /// Loaded configuration package path - if an existing package was loaded, holds that path
  std::string config_pkg_path_;

  /// Location that moveit_setup_assistant stores its templates
  std::string template_package_path_;

  /// Is this application in debug mode?
  bool debug_;

  /// Allowed collision matrix for robot poses
  collision_detection::AllowedCollisionMatrix allowed_collision_matrix_;

  /// Timestamp when configuration package was generated, if it was previously generated
  std::time_t config_pkg_generated_timestamp_;

  /// Name of the author of this config
  std::string author_name_;

  /// Email of the author of this config
  std::string author_email_;

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Load a robot model
  void setRobotModel(const moveit::core::RobotModelPtr& robot_model);

  /// Provide a shared kinematic model loader
  moveit::core::RobotModelConstPtr getRobotModel();

  /// Update the Kinematic Model with latest SRDF modifications
  void updateRobotModel();

  /// Provide a shared planning scene
  planning_scene::PlanningScenePtr getPlanningScene();

  /**
   * Find the associated group by name
   *
   * @param name - name of data to find in datastructure
   * @return pointer to data in datastructure
   */
  srdf::Model::Group* findGroupByName(const std::string& name);

  /// Load the allowed collision matrix from the SRDF's list of link pairs
  void loadAllowedCollisionMatrix(const srdf::SRDFWriter& srdf);

  // ******************************************************************************************
  // Public Functions for outputting configuration and setting files
  // ******************************************************************************************
  std::vector<OMPLPlannerDescription> getOMPLPlanners() const;
  std::map<std::string, double> getInitialJoints() const;
  bool outputSetupAssistantFile(const std::string& file_path);
  bool outputGazeboURDFFile(const std::string& file_path);
  bool outputOMPLPlanningYAML(const std::string& file_path);
  bool outputSTOMPPlanningYAML(const std::string& file_path);
  bool outputKinematicsYAML(const std::string& file_path);
  bool outputJointLimitsYAML(const std::string& file_path);
  bool outputFakeControllersYAML(const std::string& file_path);
  bool outputSimpleControllersYAML(const std::string& file_path);
  bool outputROSControllersYAML(const std::string& file_path);
  bool output3DSensorPluginYAML(const std::string& file_path);

  /**
   * \brief Helper function to get the controller that is controlling the joint
   * \return controller type
   */
  std::string getJointHardwareInterface(const std::string& joint_name);

  /**
   * \brief Decide the best two joints to be used for the projection evaluator
   * \param planning_group name of group to use
   * \return string - value to insert into yaml file
   */
  std::string decideProjectionJoints(const std::string& planning_group);

  /**
   * Input ompl_planning.yaml file for editing its values
   * @param file_path path to ompl_planning.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputOMPLYAML(const std::string& file_path);

  /**
   * Input kinematics.yaml file for editing its values
   * @param file_path path to kinematics.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputKinematicsYAML(const std::string& file_path);

  /**
   * Input planning_context.launch for editing its values
   * @param file_path path to planning_context.launch in the input package
   * @return true if the file was read correctly
   */
  bool inputPlanningContextLaunch(const std::string& file_path);

  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param YAML::Node - individual controller to be parsed
   * @return true if the file was read correctly
   */
  bool parseROSController(const YAML::Node& controller);

  /**
   * Helper function for parsing ros_controllers.yaml file
   * @param std::ifstream of ros_controller.yaml
   * @return true if the file was read correctly
   */
  bool processROSControllers(std::ifstream& input_stream);

  /**
   * Input ros_controllers.yaml file for editing its values
   * @param file_path path to ros_controllers.yaml in the input package
   * @return true if the file was read correctly
   */
  bool inputROSControllersYAML(const std::string& file_path);

  /**
   * \brief Add a Follow Joint Trajectory action Controller for each Planning Group
   * \return true if controllers were added to the controller_configs_ data structure
   */
  bool addDefaultControllers(const std::string& controller_type = "effort_controllers/JointTrajectoryController");

  /**
   * Set package path; try to resolve path from package name if directory does not exist
   * @param pkg_path path to package or package name
   * @return true if the path was set
   */
  bool setPackagePath(const std::string& pkg_path);

  /**
   * determine the package name containing a given file path
   * @param path to a file
   * @param package_name holds the ros package name if found
   * @param relative_filepath holds the relative path of the file to the package root
   * @return whether the file belongs to a known package
   */
  bool extractPackageNameFromPath(const std::string& path, std::string& package_name,
                                  std::string& relative_filepath) const;

  /**
   * Resolve path to .setup_assistant file
   * @param path resolved path
   * @return true if the path could be resolved
   */
  bool getSetupAssistantYAMLPath(std::string& path);

  /// Make the full URDF path using the loaded .setup_assistant data
  bool createFullURDFPath();

  /// Make the full SRDF path using the loaded .setup_assistant data
  bool createFullSRDFPath(const std::string& package_path);

  /**
   * Input .setup_assistant file - contains data used for the MoveIt Setup Assistant
   *
   * @param file_path path to .setup_assistant file
   * @return true if the file was read correctly
   */
  bool inputSetupAssistantYAML(const std::string& file_path);

  /// Load perception sensor config (sensors_3d.yaml) into internal data structure
  void input3DSensorsYAML(const std::string& file_path);
  /// Load perception sensor config
  static std::vector<std::map<std::string, GenericParameter>> load3DSensorsYAML(const std::string& file_path);

  /**
   * Helper Function for joining a file path and a file name, or two file paths, etc,
   * in a cross-platform way
   *
   * @param path1 first half of path
   * @param path2 second half of path, or filename
   * @return string resulting combined paths
   */
  std::string appendPaths(const std::string& path1, const std::string& path2);

  /**
   * \brief Adds a controller to controller_configs_ vector
   * \param new_controller a new Controller to add
   * \return true if inserted correctly
   */
  bool addController(const ControllerConfig& new_controller);

  /**
   * \brief Gets controller_configs_ vector
   * \return pointer to controller_configs_
   */
  std::vector<ControllerConfig>& getControllers()
  {
    return controller_configs_;
  }

  /**
   * Find the associated controller by name
   *
   * @param controller_name - name of controller to find in datastructure
   * @return pointer to data in datastructure
   */
  ControllerConfig* findControllerByName(const std::string& controller_name);

  /**
   * Delete controller by name
   *
   * @param controller_name - name of controller to delete
   * @return true if deleted, false if not found
   */
  bool deleteController(const std::string& controller_name);

  /**
   * \brief Used for adding a sensor plugin configuation prameter to the sensor plugin configuration parameter list
   */
  void addGenericParameterToSensorPluginConfig(const std::string& name, const std::string& value = "",
                                               const std::string& comment = "");

  /**
   * \brief Clear the sensor plugin configuration parameter list
   */
  void clearSensorPluginConfig();

  /**
   * \brief Used for adding a sensor plugin configuation parameter to the sensor plugin configuration parameter list
   */
  const std::vector<std::map<std::string, GenericParameter>>& getSensorPluginConfig() const
  {
    return sensors_plugin_config_parameter_list_;
  }

  /**
   * \brief Helper function to get the default start pose for moveit_sim_hw_interface
   */
  srdf::Model::GroupState getDefaultStartPose();

  /**
   * \brief Custom std::set comparator, used for sorting the joint_limits.yaml file into alphabetical order
   * \param jm1 - a pointer to the first joint model to compare
   * \param jm2 - a pointer to the second joint model to compare
   * \return bool of alphabetical sorting comparison
   */
  struct JointModelCompare
  {
    bool operator()(const moveit::core::JointModel* jm1, const moveit::core::JointModel* jm2) const
    {
      return jm1->getName() < jm2->getName();
    }
  };

private:
  // ******************************************************************************************
  // Private Vars
  // ******************************************************************************************

  /// Sensor plugin configuration parameter list, each sensor plugin type is a map
  std::vector<std::map<std::string, GenericParameter>> sensors_plugin_config_parameter_list_;

  /// Shared kinematic model
  moveit::core::RobotModelPtr robot_model_;

  /// Controllers config data
  std::vector<ControllerConfig> controller_configs_;

  /// Shared planning scene
  planning_scene::PlanningScenePtr planning_scene_;
};

}  // namespace moveit_setup_assistant
