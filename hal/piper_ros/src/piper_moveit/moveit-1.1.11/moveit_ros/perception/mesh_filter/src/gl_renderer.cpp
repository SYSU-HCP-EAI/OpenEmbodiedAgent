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

#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <moveit/mesh_filter/gl_renderer.h>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <ros/console.h>

using namespace std;

mesh_filter::GLRenderer::GLRenderer(unsigned width, unsigned height, float near, float far)
  : width_(width)
  , height_(height)
  , fbo_id_(0)
  , rbo_id_(0)
  , rgb_id_(0)
  , depth_id_(0)
  , program_(0)
  , near_(near)
  , far_(far)
  , fx_(width >> 1)  // 90 degree wide angle
  , fy_(fx_)
  , cx_(width >> 1)
  , cy_(height >> 1)
{
  createGLContext();
  initFrameBuffers();
}

mesh_filter::GLRenderer::~GLRenderer()
{
  glDeleteProgram(program_);
  deleteFrameBuffers();
  deleteGLContext();
}

void mesh_filter::GLRenderer::setBufferSize(unsigned width, unsigned height)
{
  if (width_ != width || height_ != height)
  {
    width_ = width;
    height_ = height;
    deleteFrameBuffers();
    initFrameBuffers();
  }
}

void mesh_filter::GLRenderer::setClippingRange(float near, float far)
{
  if (near_ <= 0)
    throw runtime_error("near clipping plane distance needs to be larger than 0");
  if (far_ <= near_)
    throw runtime_error("far clipping plane needs to be larger than near clipping plane distance");
  near_ = near;
  far_ = far;
}

void mesh_filter::GLRenderer::setCameraParameters(float fx, float fy, float cx, float cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

void mesh_filter::GLRenderer::setCameraParameters() const
{
  float left = near_ * -cx_ / fx_;
  float right = near_ * (width_ - cx_) / fx_;
  float top = near_ * cy_ / fy_;
  float bottom = near_ * (cy_ - height_) / fy_;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(left, right, bottom, top, near_, far_);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
}

void mesh_filter::GLRenderer::initFrameBuffers()
{
  glGenTextures(1, &rgb_id_);
  glBindTexture(GL_TEXTURE_2D, rgb_id_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  glGenTextures(1, &depth_id_);
  glBindTexture(GL_TEXTURE_2D, depth_id_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width_, height_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  glGenFramebuffers(1, &fbo_id_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rgb_id_, 0);

  glGenRenderbuffers(1, &rbo_id_);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo_id_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_id_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_id_, 0);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);

  GLenum draw_buffers[2] = { GL_COLOR_ATTACHMENT0, GL_DEPTH_ATTACHMENT };
  glDrawBuffers(2, draw_buffers);

  GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

  if (status != GL_FRAMEBUFFER_COMPLETE)  // If the frame buffer does not report back as complete
    throw runtime_error("Couldn't create frame buffer");

  glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Unbind our frame buffer
}

void mesh_filter::GLRenderer::deleteFrameBuffers()
{
  if (rbo_id_)
    glDeleteRenderbuffers(1, &rbo_id_);
  if (fbo_id_)
    glDeleteFramebuffers(1, &fbo_id_);
  if (depth_id_)
    glDeleteTextures(1, &depth_id_);
  if (rgb_id_)
    glDeleteTextures(1, &rgb_id_);

  rbo_id_ = fbo_id_ = depth_id_ = rgb_id_ = 0;
}

void mesh_filter::GLRenderer::begin() const
{
  glPushAttrib(GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_POLYGON_BIT | GL_PIXEL_MODE_BIT);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, width_, height_);
  glUseProgram(program_);
  setCameraParameters();
}

void mesh_filter::GLRenderer::callList(GLuint list) const
{
  begin();
  glCallList(list);
  end();
}

void mesh_filter::GLRenderer::end() const
{
  glFlush();
  glPopAttrib();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void mesh_filter::GLRenderer::getColorBuffer(unsigned char* buffer) const
{
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glBindTexture(GL_TEXTURE_2D, rgb_id_);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void mesh_filter::GLRenderer::getDepthBuffer(float* buffer) const
{
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
  glBindTexture(GL_TEXTURE_2D, depth_id_);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GLuint mesh_filter::GLRenderer::setShadersFromFile(const string& vertex_filename, const string& fragment_filename)
{
  if (program_)
    glDeleteProgram(program_);

  string vertex_source, fragment_source;
  readShaderCodeFromFile(vertex_filename, vertex_source);
  readShaderCodeFromFile(fragment_filename, fragment_source);

  program_ = loadShaders(vertex_source, fragment_source);
  return program_;
}

GLuint mesh_filter::GLRenderer::setShadersFromString(const string& vertex_source, const string& fragment_source)
{
  program_ = loadShaders(vertex_source, fragment_source);
  return program_;
}

const GLuint& mesh_filter::GLRenderer::getProgramID() const
{
  return program_;
}

const float& mesh_filter::GLRenderer::getNearClippingDistance() const
{
  return near_;
}

const float& mesh_filter::GLRenderer::getFarClippingDistance() const
{
  return far_;
}

GLuint mesh_filter::GLRenderer::createShader(GLuint shaderType, const string& ShaderCode) const
{
  GLuint shader_id = glCreateShader(shaderType);

  // Compile Shader
  char const* source_pointer = ShaderCode.c_str();
  glShaderSource(shader_id, 1, &source_pointer, nullptr);
  glCompileShader(shader_id);

  // Check Shader
  GLint result = GL_FALSE;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  if (result != GL_TRUE)
  {
    int info_log_length;
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);
    if (info_log_length > 0)
    {
      vector<char> shader_error_message(info_log_length + 1);
      glGetShaderInfoLog(shader_id, info_log_length, nullptr, &shader_error_message[0]);
      stringstream error_stream;
      error_stream << "Could not compile shader: " << (const char*)&shader_error_message[0];

      glDeleteShader(shader_id);
      throw runtime_error(error_stream.str());
    }
  }
  return shader_id;
}

void mesh_filter::GLRenderer::readShaderCodeFromFile(const string& filename, string& shader) const
{
  if (filename.empty())
    shader = "";
  else
  {
    string shader_code;
    fstream shader_file(filename.c_str(), ios::in);
    if (shader_file.is_open())
    {
      stringstream buffer;
      buffer << shader_file.rdbuf();
      shader = buffer.str();
    }
    else
    {
      stringstream error_stream;
      error_stream << "Could not open shader code in file \"" << filename << "\"";
      throw runtime_error(error_stream.str());
    }
  }
}

GLuint mesh_filter::GLRenderer::loadShaders(const string& vertex_source, const string& fragment_source) const
{
  if (vertex_source.empty() && fragment_source.empty())
    return 0;

  GLuint program_id = glCreateProgram();
  GLuint vertex_shader_id = 0;
  GLuint fragment_shader_id = 0;

  if (!vertex_source.empty())
  {
    GLuint vertex_shader_id = createShader(GL_VERTEX_SHADER, vertex_source);
    glAttachShader(program_id, vertex_shader_id);
  }

  if (!fragment_source.empty())
  {
    GLuint fragment_shader_id = createShader(GL_FRAGMENT_SHADER, fragment_source);
    glAttachShader(program_id, fragment_shader_id);
  }

  glLinkProgram(program_id);

  // Check the program
  GLint result = GL_FALSE;
  GLint info_log_length;
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &info_log_length);
  if (info_log_length > 0)
  {
    vector<char> program_error_message(info_log_length + 1);
    glGetProgramInfoLog(program_id, info_log_length, nullptr, &program_error_message[0]);
    std::size_t l = strnlen(&program_error_message[0], program_error_message.size());
    if (l > 0)
      ROS_ERROR("%s\n", &program_error_message[0]);
  }

  if (vertex_shader_id)
    glDeleteShader(vertex_shader_id);

  if (fragment_shader_id)
    glDeleteShader(fragment_shader_id);

  return program_id;
}

map<std::thread::id, pair<unsigned, GLuint> > mesh_filter::GLRenderer::context_;
std::mutex mesh_filter::GLRenderer::context_lock_;
bool mesh_filter::GLRenderer::glutInitialized_ = false;

namespace
{
void nullDisplayFunction(){};
}

void mesh_filter::GLRenderer::createGLContext()
{
  std::unique_lock<std::mutex> _(context_lock_);
  if (!glutInitialized_)
  {
    char buffer[1];
    char* args = buffer;
    int n = 1;

    glutInit(&n, &args);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitialized_ = true;
  }

  // check if our thread is initialized
  std::thread::id thread_id = std::this_thread::get_id();
  map<std::thread::id, pair<unsigned, GLuint> >::iterator context_it = context_.find(thread_id);

  if (context_it == context_.end())
  {
    context_[thread_id] = std::pair<unsigned, GLuint>(1, 0);

    glutInitWindowPosition(glutGet(GLUT_SCREEN_WIDTH) + 30000, 0);
    glutInitWindowSize(1, 1);
    GLuint window_id = glutCreateWindow("mesh_filter");
    glutDisplayFunc(nullDisplayFunction);

    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
      stringstream error_stream;
      error_stream << "Unable to initialize GLEW: " << glewGetErrorString(err);

      throw(runtime_error(error_stream.str()));
    }
    glutIconifyWindow();
    glutHideWindow();

    for (int i = 0; i < 10; ++i)
      glutMainLoopEvent();

    context_[thread_id] = std::pair<unsigned, GLuint>(1, window_id);
  }
  else
    ++(context_it->second.first);
}

void mesh_filter::GLRenderer::deleteGLContext()
{
  std::unique_lock<std::mutex> _(context_lock_);
  std::thread::id thread_id = std::this_thread::get_id();
  map<std::thread::id, pair<unsigned, GLuint> >::iterator context_it = context_.find(thread_id);
  if (context_it == context_.end())
  {
    stringstream error_msg;
    error_msg << "No OpenGL context exists for Thread " << thread_id;
    throw runtime_error(error_msg.str());
  }

  if (--(context_it->second.first) == 0)
  {
    glutDestroyWindow(context_it->second.second);
    context_.erase(context_it);
  }
}

GLuint mesh_filter::GLRenderer::getColorTexture() const
{
  return rgb_id_;
}

GLuint mesh_filter::GLRenderer::getDepthTexture() const
{
  return depth_id_;
}

unsigned mesh_filter::GLRenderer::getWidth() const
{
  return width_;
}

unsigned mesh_filter::GLRenderer::getHeight() const
{
  return height_;
}
