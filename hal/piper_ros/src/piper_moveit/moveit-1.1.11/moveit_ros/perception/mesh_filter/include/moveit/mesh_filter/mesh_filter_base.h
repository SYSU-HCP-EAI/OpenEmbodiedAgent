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
#include <moveit/macros/class_forward.h>
#include <moveit/mesh_filter/gl_renderer.h>
#include <moveit/mesh_filter/sensor_model.h>
#include <Eigen/Geometry>  // for Isometry3d
#include <queue>
#include <thread>
#include <condition_variable>
#include <mutex>

// forward declarations
namespace shapes
{
class Mesh;
}

namespace mesh_filter
{
MOVEIT_CLASS_FORWARD(Job);     // Defines JobPtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(GLMesh);  // Defines GLMeshPtr, ConstPtr, WeakPtr... etc

typedef unsigned int MeshHandle;
typedef uint32_t LabelType;

class MeshFilterBase
{
  // inner types and typedefs
public:
  typedef std::function<bool(MeshHandle, Eigen::Isometry3d&)> TransformCallback;
  // \todo @suat: to avoid a few comparisons, it would be much nicer if background = 14 and shadow = 15 (near/far clip
  // can be anything below that)
  // this would allow me to do a single comparison instead of 3, in the code i write
  enum
  {
    BACKGROUND = 0,
    SHADOW = 1,
    NEAR_CLIP = 2,
    FAR_CLIP = 3,
    FIRST_LABEL = 16
  };

public:
  /**
   * \brief Constructor
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] transform_callback Callback function that is called for each mesh to obtain the current transformation.
   * \note the callback expects the mesh handle but no time stamp. Its the users responsibility to return the correct
   * transformation.
   */
  MeshFilterBase(const TransformCallback& transform_callback, const SensorModel::Parameters& sensor_parameters,
                 const std::string& render_vertex_shader = "", const std::string& render_fragment_shader = "",
                 const std::string& filter_vertex_shader = "", const std::string& filter_fragment_shader = "");

  /** \brief Desctructor */
  ~MeshFilterBase();

  /**
   * \brief adds a mesh to the filter object.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] mesh the mesh to be added
   * \return handle to the mesh. This handle is used in the transform callback function to identify the mesh and
   * retrieve the correct transformation.
   */
  MeshHandle addMesh(const shapes::Mesh& mesh);

  /**
   * \brief removes a mesh given by its handle
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] mesh_handle the handle of the mesh to be removed.
   */
  void removeMesh(MeshHandle mesh_handle);

  /**
   * \brief label/remove pixels from input depth-image
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] sensor_data pointer to the input depth image from sensor readings.
   * \todo what is type?
   */
  void filter(const void* sensor_data, GLushort type, bool wait = false) const;

  /**
   * \brief retrieves the labels of the input data
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[out] labels pointer to buffer to be filled with labels
   * \note labels are corresponding 1-1 to the mesh handles. 0 and 1 are reserved indicating either background (0) or
   * shadow (1)
   *       The upper 8bit of a label is filled with the user given flag (see addMesh)
   */
  void getFilteredLabels(LabelType* labels) const;

  /**
   * \brief retrieves the filtered depth values
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[out] depth pointer to buffer to be filled with depth values.
   */
  void getFilteredDepth(float* depth) const;

  /**
   * \brief retrieves the labels of the rendered model
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[out] labels pointer to buffer to be filled with labels
   * \note labels are corresponding 1-1 to the mesh handles. 0 and 1 are reserved indicating either background (0) or
   * shadow (1)
   *       The upper 8bit of a label is filled with the user given flag (see addMesh)
   * \todo How is this data different from the filtered labels?
   */
  void getModelLabels(LabelType* labels) const;

  /**
   * \brief retrieves the depth values of the rendered model
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[out] depth pointer to buffer to be filled with depth values.
   */
  void getModelDepth(float* depth) const;

  /**
   * \brief set the shadow threshold. points that are further away than the rendered model are filtered out.
   *        Except they are further away than this threshold. Then these points are kept, but its label is set to
   *        1 indicating that it is in the shadow of the model
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] threshold shadow threshold in meters
   */
  void setShadowThreshold(float threshold);

  /**
   * \brief set the callback for retrieving transformations for each mesh.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] transform_callback the callback
   */
  void setTransformCallback(const TransformCallback& transform_callback);

  /**
   * \brief set the scale component of padding used to multiply with sensor-specific padding coefficients to get final
   * coefficients.
   * \param[in] scale the scale value
   */
  void setPaddingScale(float scale);

  /**
   * \brief set the offset component of padding. This value is added to the scaled sensor-specific constant component.
   * \param[in] offset the offset value
   */
  void setPaddingOffset(float offset);

protected:
  /**
   * \brief initializes OpenGL related things as well as renderers
   */
  void initialize(const std::string& render_vertex_shader, const std::string& render_fragment_shader,
                  const std::string& filter_vertex_shader, const std::string& filter_fragment_shader);

  /**
   * \brief cleaning up
   */
  void deInitialize();

  /**
   * \brief filtering thread
   */
  void run(const std::string& render_vertex_shader, const std::string& render_fragment_shader,
           const std::string& filter_vertex_shader, const std::string& filter_fragment_shader);

  /**
   * \brief the filter method that does the magic
   * \param[in] sensor_data pointer to the buffer containing the depth readings
   * \param[in] encoding the representation of the depth readings in the buffer
   */
  void doFilter(const void* sensor_data, const int encoding) const;

  /**
   * \brief used within a Job to allow the main thread adding meshes
   * \param[in] handle the handle of the mesh that is predetermined and passed
   * \param[in] cmesh the mesh to be added to the corresponding handle
   */
  void addMeshHelper(MeshHandle handle, const shapes::Mesh& cmesh);

  /**
   * \brief used within a Job to allow the main thread removing meshes
   * \param[in] handle the handle of the mesh to be removed
   */
  bool removeMeshHelper(MeshHandle handle);

  /**
   * \brief add a Job for the main thread that needs to be executed there
   * \param[in] job the job object that has the function o be executed
   */
  void addJob(const JobPtr& job) const;

  /**
   * \brief sets the size of the fram buffers
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] width width of frame buffers in pixels
   * \param[in] height height of frame buffers in pixels
   */
  void setSize(unsigned int width, unsigned int height);

  /** \brief storage for meshed to be filtered */
  std::map<MeshHandle, GLMeshPtr> meshes_;

  /** \brief the parameters of the used sensor model*/
  SensorModel::ParametersPtr sensor_parameters_;

  /** \brief next handle to be used for next mesh that is added*/
  MeshHandle next_handle_;

  /** \brief Handle values below this are all taken (this variable is used for more efficient computation of
   * next_label_) */
  MeshHandle min_handle_;

  /** \brief the filtering thread that also holds the OpenGL context*/
  std::thread filter_thread_;

  /** \brief condition variable to notify the filtering thread if a new image arrived*/
  mutable std::condition_variable jobs_condition_;

  /** \brief mutex required for synchronization of condition states*/
  mutable std::mutex jobs_mutex_;

  /** \brief OpenGL job queue that need to be processed by the worker thread*/
  mutable std::queue<JobPtr> jobs_queue_;

  /** \brief mutex for synchronization of updating filtered meshes */
  mutable std::mutex meshes_mutex_;

  /** \brief mutex for synchronization of setting/calling transform_callback_ */
  mutable std::mutex transform_callback_mutex_;

  /** \brief indicates whether the filtering loop should stop*/
  bool stop_;

  /** \brief first pass renderer for rendering the mesh*/
  GLRendererPtr mesh_renderer_;

  /** \brief second pass renderer for filtering the results of first pass*/
  GLRendererPtr depth_filter_;

  /** \brief canvas element (screen-filling quad) for second pass*/
  GLuint canvas_;

  /** \brief handle depth texture from sensor data*/
  GLuint sensor_depth_texture_;

  /** \brief handle to GLSL location of shadow threshold*/
  GLuint shadow_threshold_location_;

  /** \brief callback function for retrieving the mesh transformations*/
  TransformCallback transform_callback_;

  /** \brief padding scale*/
  float padding_scale_;

  /** \brief padding offset*/
  float padding_offset_;

  /** \brief threshold for shadowed pixels vs. filtered pixels*/
  float shadow_threshold_;
};
}  // namespace mesh_filter
