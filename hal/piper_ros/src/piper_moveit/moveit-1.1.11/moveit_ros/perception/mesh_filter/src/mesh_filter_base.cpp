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

#include <moveit/mesh_filter/mesh_filter_base.h>
#include <moveit/mesh_filter/gl_mesh.h>
#include <moveit/mesh_filter/filter_job.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Eigen>

#include <memory>
#include <stdexcept>
#include <sstream>

#include <ros/console.h>

// include SSE headers
#ifdef HAVE_SSE_EXTENSIONS
#include <xmmintrin.h>
#endif

mesh_filter::MeshFilterBase::MeshFilterBase(const TransformCallback& transform_callback,
                                            const SensorModel::Parameters& sensor_parameters,
                                            const std::string& render_vertex_shader,
                                            const std::string& render_fragment_shader,
                                            const std::string& filter_vertex_shader,
                                            const std::string& filter_fragment_shader)
  : sensor_parameters_(sensor_parameters.clone())
  , next_handle_(FIRST_LABEL)  // 0 and 1 are reserved!
  , min_handle_(FIRST_LABEL)
  , stop_(false)
  , transform_callback_(transform_callback)
  , padding_scale_(1.0)
  , padding_offset_(0.01)
  , shadow_threshold_(0.5)
{
  filter_thread_ =
      std::thread([this, render_vertex_shader, render_fragment_shader, filter_vertex_shader, filter_fragment_shader] {
        run(render_vertex_shader, render_fragment_shader, filter_vertex_shader, filter_fragment_shader);
      });
}

void mesh_filter::MeshFilterBase::initialize(const std::string& render_vertex_shader,
                                             const std::string& render_fragment_shader,
                                             const std::string& filter_vertex_shader,
                                             const std::string& filter_fragment_shader)
{
  mesh_renderer_ = std::make_shared<GLRenderer>(sensor_parameters_->getWidth(), sensor_parameters_->getHeight(),
                                                sensor_parameters_->getNearClippingPlaneDistance(),
                                                sensor_parameters_->getFarClippingPlaneDistance());
  depth_filter_ = std::make_shared<GLRenderer>(sensor_parameters_->getWidth(), sensor_parameters_->getHeight(),
                                               sensor_parameters_->getNearClippingPlaneDistance(),
                                               sensor_parameters_->getFarClippingPlaneDistance());

  mesh_renderer_->setShadersFromString(render_vertex_shader, render_fragment_shader);
  depth_filter_->setShadersFromString(filter_vertex_shader, filter_fragment_shader);

  depth_filter_->begin();

  glGenTextures(1, &sensor_depth_texture_);

  glUniform1i(glGetUniformLocation(depth_filter_->getProgramID(), "sensor"), 0);
  glUniform1i(glGetUniformLocation(depth_filter_->getProgramID(), "depth"), 2);
  glUniform1i(glGetUniformLocation(depth_filter_->getProgramID(), "label"), 4);

  shadow_threshold_location_ = glGetUniformLocation(depth_filter_->getProgramID(), "shadow_threshold");

  depth_filter_->end();

  canvas_ = glGenLists(1);
  glNewList(canvas_, GL_COMPILE);
  glBegin(GL_QUADS);

  glColor3f(1, 1, 1);
  glTexCoord2f(0, 0);
  glVertex3f(-1, -1, 1);

  glTexCoord2f(1, 0);
  glVertex3f(1, -1, 1);

  glTexCoord2f(1, 1);
  glVertex3f(1, 1, 1);

  glTexCoord2f(0, 1);
  glVertex3f(-1, 1, 1);

  glEnd();
  glEndList();
}

mesh_filter::MeshFilterBase::~MeshFilterBase()
{
  {
    std::unique_lock<std::mutex> lock(jobs_mutex_);
    stop_ = true;
    while (!jobs_queue_.empty())
    {
      jobs_queue_.front()->cancel();
      jobs_queue_.pop();
    }
  }
  jobs_condition_.notify_one();
  filter_thread_.join();
}

void mesh_filter::MeshFilterBase::addJob(const JobPtr& job) const
{
  {
    std::unique_lock<std::mutex> _(jobs_mutex_);
    jobs_queue_.push(job);
  }
  jobs_condition_.notify_one();
}

void mesh_filter::MeshFilterBase::deInitialize()
{
  glDeleteLists(canvas_, 1);
  glDeleteTextures(1, &sensor_depth_texture_);

  meshes_.clear();
  mesh_renderer_.reset();
  depth_filter_.reset();
}

void mesh_filter::MeshFilterBase::setSize(unsigned int width, unsigned int height)
{
  mesh_renderer_->setBufferSize(width, height);
  mesh_renderer_->setCameraParameters(width, width, width >> 1, height >> 1);

  depth_filter_->setBufferSize(width, height);
  depth_filter_->setCameraParameters(width, width, width >> 1, height >> 1);
}

void mesh_filter::MeshFilterBase::setTransformCallback(const TransformCallback& transform_callback)
{
  std::unique_lock<std::mutex> _(transform_callback_mutex_);
  transform_callback_ = transform_callback;
}

mesh_filter::MeshHandle mesh_filter::MeshFilterBase::addMesh(const shapes::Mesh& mesh)
{
  std::unique_lock<std::mutex> _(meshes_mutex_);

  JobPtr job(new FilterJob<void>([this, &mesh] { addMeshHelper(next_handle_, mesh); }));
  addJob(job);
  job->wait();
  mesh_filter::MeshHandle ret = next_handle_;
  const std::size_t sz = min_handle_ + meshes_.size() + 1;
  for (std::size_t i = min_handle_; i < sz; ++i)
    if (meshes_.find(i) == meshes_.end())
    {
      next_handle_ = i;
      break;
    }
  min_handle_ = next_handle_;
  return ret;
}

void mesh_filter::MeshFilterBase::addMeshHelper(MeshHandle handle, const shapes::Mesh& cmesh)
{
  meshes_[handle] = std::make_shared<GLMesh>(cmesh, handle);
}

void mesh_filter::MeshFilterBase::removeMesh(MeshHandle handle)
{
  std::unique_lock<std::mutex> _(meshes_mutex_);
  FilterJob<bool>* remover = new FilterJob<bool>([this, handle] { return removeMeshHelper(handle); });
  JobPtr job(remover);
  addJob(job);
  job->wait();

  if (!remover->getResult())
    throw std::runtime_error("Could not remove mesh. Mesh not found!");
  min_handle_ = std::min(handle, min_handle_);
}

bool mesh_filter::MeshFilterBase::removeMeshHelper(MeshHandle handle)
{
  std::size_t erased = meshes_.erase(handle);
  return (erased != 0);
}

void mesh_filter::MeshFilterBase::setShadowThreshold(float threshold)
{
  shadow_threshold_ = threshold;
}

void mesh_filter::MeshFilterBase::getModelLabels(LabelType* labels) const
{
  JobPtr job(
      new FilterJob<void>([&renderer = *mesh_renderer_, labels] { renderer.getColorBuffer((unsigned char*)labels); }));
  addJob(job);
  job->wait();
}

void mesh_filter::MeshFilterBase::getModelDepth(float* depth) const
{
  JobPtr job1(new FilterJob<void>([&renderer = *mesh_renderer_, depth] { renderer.getDepthBuffer(depth); }));
  JobPtr job2(new FilterJob<void>(
      [&parameters = *sensor_parameters_, depth] { parameters.transformModelDepthToMetricDepth(depth); }));
  {
    std::unique_lock<std::mutex> lock(jobs_mutex_);
    jobs_queue_.push(job1);
    jobs_queue_.push(job2);
  }
  jobs_condition_.notify_one();
  job1->wait();
  job2->wait();
}

void mesh_filter::MeshFilterBase::getFilteredDepth(float* depth) const
{
  JobPtr job1(new FilterJob<void>([&filter = *depth_filter_, depth] { filter.getDepthBuffer(depth); }));
  JobPtr job2(new FilterJob<void>(
      [&parameters = *sensor_parameters_, depth] { parameters.transformFilteredDepthToMetricDepth(depth); }));
  {
    std::unique_lock<std::mutex> lock(jobs_mutex_);
    jobs_queue_.push(job1);
    jobs_queue_.push(job2);
  }
  jobs_condition_.notify_one();
  job1->wait();
  job2->wait();
}

void mesh_filter::MeshFilterBase::getFilteredLabels(LabelType* labels) const
{
  JobPtr job(new FilterJob<void>([&filter = *depth_filter_, labels] { filter.getColorBuffer((unsigned char*)labels); }));
  addJob(job);
  job->wait();
}

void mesh_filter::MeshFilterBase::run(const std::string& render_vertex_shader,
                                      const std::string& render_fragment_shader,
                                      const std::string& filter_vertex_shader,
                                      const std::string& filter_fragment_shader)
{
  initialize(render_vertex_shader, render_fragment_shader, filter_vertex_shader, filter_fragment_shader);

  while (!stop_)
  {
    std::unique_lock<std::mutex> lock(jobs_mutex_);
    // check if we have new sensor data to be processed. If not, wait until we get notified.
    if (jobs_queue_.empty())
      jobs_condition_.wait(lock);

    if (!jobs_queue_.empty())
    {
      JobPtr job = jobs_queue_.front();
      jobs_queue_.pop();
      lock.unlock();
      job->execute();
      lock.lock();
    }
  }
  deInitialize();
}

void mesh_filter::MeshFilterBase::filter(const void* sensor_data, GLushort type, bool wait) const
{
  if (type != GL_FLOAT && type != GL_UNSIGNED_SHORT)
  {
    std::stringstream msg;
    msg << "unknown type \"" << type << "\". Allowed values are GL_FLOAT or GL_UNSIGNED_SHORT.";
    throw std::runtime_error(msg.str());
  }

  JobPtr job(new FilterJob<void>([this, sensor_data, type] { doFilter(sensor_data, type); }));
  addJob(job);
  if (wait)
    job->wait();
}

void mesh_filter::MeshFilterBase::doFilter(const void* sensor_data, const int encoding) const
{
  std::unique_lock<std::mutex> _(transform_callback_mutex_);

  mesh_renderer_->begin();
  sensor_parameters_->setRenderParameters(*mesh_renderer_);

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_FRONT);
  glDisable(GL_ALPHA_TEST);
  glDisable(GL_BLEND);

  GLuint padding_coefficients_id = glGetUniformLocation(mesh_renderer_->getProgramID(), "padding_coefficients");
  Eigen::Vector3f padding_coefficients =
      sensor_parameters_->getPaddingCoefficients() * padding_scale_ + Eigen::Vector3f(0, 0, padding_offset_);
  glUniform3f(padding_coefficients_id, padding_coefficients[0], padding_coefficients[1], padding_coefficients[2]);

  Eigen::Isometry3d transform;
  for (const std::pair<const MeshHandle, GLMeshPtr>& mesh : meshes_)
    if (transform_callback_(mesh.first, transform))
      mesh.second->render(transform);

  mesh_renderer_->end();

  // now filter the depth_map with the second rendering stage
  // depth_filter_.setBufferSize (width, height);
  // depth_filter_.setCameraParameters (fx, fy, cx, cy);
  depth_filter_->begin();
  sensor_parameters_->setFilterParameters(*depth_filter_);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_ALWAYS);
  glDisable(GL_CULL_FACE);
  glDisable(GL_ALPHA_TEST);
  glDisable(GL_BLEND);

  //  glUniform1f (near_location_, depth_filter_.getNearClippingDistance ());
  //  glUniform1f (far_location_, depth_filter_.getFarClippingDistance ());
  glUniform1f(shadow_threshold_location_, shadow_threshold_);

  GLuint depth_texture = mesh_renderer_->getDepthTexture();
  GLuint color_texture = mesh_renderer_->getColorTexture();

  // bind sensor depth
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, sensor_depth_texture_);

  float scale =
      1.0 / (sensor_parameters_->getFarClippingPlaneDistance() - sensor_parameters_->getNearClippingPlaneDistance());

  if (encoding == GL_UNSIGNED_SHORT)
    // unsigned shorts shorts will be mapped to the range 0-1 during transfer. Afterwards we can apply another scale +
    // offset to
    // map the values between near and far clipping plane to 0 - 1. -> scale = (65535 * depth - near ) / (far - near)
    // we have: [0 - 65535] -> [0 - 1]
    // we want: [near - far] -> [0 - 1]
    glPixelTransferf(GL_DEPTH_SCALE, scale * 65.535);
  else
    glPixelTransferf(GL_DEPTH_SCALE, scale);
  glPixelTransferf(GL_DEPTH_BIAS, -scale * sensor_parameters_->getNearClippingPlaneDistance());

  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, sensor_parameters_->getWidth(), sensor_parameters_->getHeight(), 0,
               GL_DEPTH_COMPONENT, encoding, sensor_data);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  // bind depth map
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, depth_texture);

  // bind labels
  glActiveTexture(GL_TEXTURE4);
  glBindTexture(GL_TEXTURE_2D, color_texture);
  glCallList(canvas_);
  depth_filter_->end();
}

void mesh_filter::MeshFilterBase::setPaddingOffset(float offset)
{
  padding_offset_ = offset;
}

void mesh_filter::MeshFilterBase::setPaddingScale(float scale)
{
  padding_scale_ = scale;
}
