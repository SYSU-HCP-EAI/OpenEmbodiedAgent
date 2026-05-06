/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Suat Gedikli */

#pragma once

#include <map>
#include <moveit/macros/declare_ptr.h>
#include <moveit/mesh_filter/gl_renderer.h>
#include <moveit/mesh_filter/mesh_filter_base.h>

// forward declarations
namespace shapes
{
class Mesh;
}

namespace mesh_filter
{
class GLMesh;

/**
 * \brief MeshFilter filters out points that belong to given meshes in depth-images
 * \author Suat Gedikli (gedikli@willowgarage.com)
 */

template <typename SensorType>
class MeshFilter : public MeshFilterBase
{
public:
  MOVEIT_DECLARE_PTR_MEMBER(MeshFilter);

  /**
   * \brief Constructor
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] transform_callback Callback function that is called for each mesh to obtain the current transformation.
   * \note the callback expects the mesh handle but no time stamp. Its the users responsibility to return the correct
   * transformation.
   */
  MeshFilter(const TransformCallback& transform_callback = TransformCallback(),
             const typename SensorType::Parameters& sensor_parameters = typename SensorType::Parameters());

  /**
   * \brief returns the Sensor Parameters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return reference of the parameters object of the used Sensor
   */
  typename SensorType::Parameters& parameters();

  /**
   * \brief returns the Sensor Parameters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return const reference of the parameters object of the used Sensor
   */
  const typename SensorType::Parameters& parameters() const;
};

template <typename SensorType>
MeshFilter<SensorType>::MeshFilter(const TransformCallback& transform_callback,
                                   const typename SensorType::Parameters& sensor_parameters)
  : MeshFilterBase(transform_callback, sensor_parameters, SensorType::RENDER_VERTEX_SHADER_SOURCE,
                   SensorType::RENDER_FRAGMENT_SHADER_SOURCE, SensorType::FILTER_VERTEX_SHADER_SOURCE,
                   SensorType::FILTER_FRAGMENT_SHADER_SOURCE)
{
}

template <typename SensorType>
typename SensorType::Parameters& MeshFilter<SensorType>::parameters()
{
  return static_cast<typename SensorType::Parameters&>(*sensor_parameters_);
}

template <typename SensorType>
const typename SensorType::Parameters& MeshFilter<SensorType>::parameters() const
{
  return static_cast<typename SensorType::Parameters&>(*sensor_parameters_);
}

}  // namespace mesh_filter
