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

/* Author: Ioan Sucan, Mathias Lüdtke, Dave Coleman */

#pragma once

#include <moveit/macros/class_forward.h>
#include <urdf/model.h>
#include <srdfdom/model.h>

namespace rdf_loader
{
MOVEIT_CLASS_FORWARD(RDFLoader);  // Defines RDFLoaderPtr, ConstPtr, WeakPtr... etc

/** @class RDFLoader
 *  @brief Default constructor
 *  @param robot_description The string name corresponding to the ROS param where the URDF is loaded*/
class RDFLoader
{
public:
  /** @brief Default constructor
   *  @param robot_description The string name corresponding to the ROS param where the URDF is loaded; the SRDF is
   * assumed to be at the same param name + the "_semantic" suffix */
  RDFLoader(const std::string& robot_description = "robot_description");

  /** \brief Initialize the robot model from a string representation of the URDF and SRDF documents */
  RDFLoader(const std::string& urdf_string, const std::string& srdf_string);

  /** @brief Get the resolved parameter name for the robot description */
  const std::string& getRobotDescription() const
  {
    return robot_description_;
  }

  /** @brief Get the parsed URDF model*/
  const urdf::ModelInterfaceSharedPtr& getURDF() const
  {
    return urdf_;
  }

  /** @brief Get the parsed SRDF model*/
  const srdf::ModelSharedPtr& getSRDF() const
  {
    return srdf_;
  }

  /** @brief determine if given path points to a xacro file */
  static bool isXacroFile(const std::string& path);

  /** @brief load file from given path into buffer */
  static bool loadFileToString(std::string& buffer, const std::string& path);

  /** @brief run xacro with the given args on the file, return result in buffer */
  static bool loadXacroFileToString(std::string& buffer, const std::string& path,
                                    const std::vector<std::string>& xacro_args);

  /** @brief helper that branches between loadFileToString() and loadXacroFileToString() based on result of
   * isXacroFile() */
  static bool loadXmlFileToString(std::string& buffer, const std::string& path,
                                  const std::vector<std::string>& xacro_args);

  /** @brief helper that generates a file path based on package name and relative file path to package */
  static bool loadPkgFileToString(std::string& buffer, const std::string& package_name,
                                  const std::string& relative_path, const std::vector<std::string>& xacro_args);

private:
  std::string robot_description_;
  srdf::ModelSharedPtr srdf_;
  urdf::ModelInterfaceSharedPtr urdf_;
};
}  // namespace rdf_loader
