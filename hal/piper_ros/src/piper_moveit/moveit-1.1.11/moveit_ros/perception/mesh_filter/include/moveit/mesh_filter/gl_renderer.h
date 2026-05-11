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

#include <moveit/macros/class_forward.h>
#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <string>
#include <thread>
#include <mutex>
#include <map>

namespace mesh_filter
{
MOVEIT_CLASS_FORWARD(GLRenderer);  // Defines GLRendererPtr, ConstPtr, WeakPtr... etc

/** \brief Abstracts the OpenGL frame buffer objects, and provides an interface to render meshes, and retrieve the color
 * and depth ap from opengl.
 *  \author Suat Gedikli (gedikli@willowgarage.com)
 */
class GLRenderer
{
public:
  /**
   * \brief constructs the frame buffer object in a new OpenGL context.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] width the width of the frame buffers
   * \param[in] height height of the framebuffers
   * \param[in] near distance of the near clipping plane in meters
   * \param[in] far distance of the far clipping plane in meters
   */
  GLRenderer(unsigned width, unsigned height, float near = 0.1, float far = 10.0);

  /** \brief destructor, destroys frame buffer objects and OpenGL context*/
  ~GLRenderer();

  /**
   * \brief initializes the frame buffers for rendering and or manipulating
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void begin() const;

  /**
   * \brief finalizes the frame buffers after rendering and/or manipulating
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void end() const;

  /**
   * \brief executes a OpenGL list
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param [in] list the handle of the OpenGL list to be executed
   */
  void callList(GLuint list) const;

  /**
   * \brief retrieves the color buffer from  OpenGL
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[out] buffer pointer to memory where the color values need to be stored
   */
  void getColorBuffer(unsigned char* buffer) const;

  /**
   * \brief retrieves the depth buffer from OpenGL
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[out] buffer pointer to memory where the depth values need to be stored
   */
  void getDepthBuffer(float* buffer) const;

  /**
   * \brief loads, compiles, links and adds GLSL shaders from files to the current OpenGL context.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] vertex_filename path to vertex shader source code. Can set to "" (empty string) if no vertex shader is
   * used.
   * \param[in] fragment_filename path to fragemnt shader source code. Can be set to "" if no fragment shader is used.
   * \return the programID
   */
  GLuint setShadersFromFile(const std::string& vertex_filename, const std::string& fragment_filename);

  /**
   * \brief loads, compiles, links and adds GLSL shaders from string to the current OpenGL context.
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] vertex_shader source code of the vertex shader. Can be "" if no vertex shader is used.
   * \param[in] fragment_shader source code of the fragment shader. Can be "" if no fragment shader is used.
   * \return  programID
   */
  GLuint setShadersFromString(const std::string& vertex_shader, const std::string& fragment_shader);

  /**
   * \brief set the camera parameters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] fx focal length in x-direction
   * \param[in] fy focal length in y-direction
   * \param[in] cx x component of principal point
   * \param[in] cy y component of principal point
   */
  void setCameraParameters(float fx, float fy, float cx, float cy);

  /**
   * \brief sets the near and far clipping plane distances in meters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] near distance of the near clipping plane in meters
   * \param[in] far distance of the far clipping plane in meters
   */
  void setClippingRange(float near, float far);

  /**
   * \brief returns the distance of the near clipping plane in meters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return distance of near clipping plane in meters
   */
  const float& getNearClippingDistance() const;

  /**
   * \brief returns the distance of the far clipping plane in meters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return distance of the far clipping plane in meters
   */
  const float& getFarClippingDistance() const;

  /**
   * \brief returns the width of the frame buffer objectsin pixels
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return width of frame buffer in pixels
   */
  unsigned getWidth() const;

  /**
   * \brief returns the height of the frame buffer objects in pixels
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return height of frame buffer in pixels
   */
  unsigned getHeight() const;

  /**
   * \brief set the size of fram buffers
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] width width of frame buffer in pixels
   * \param[in] height height of frame buffer in pixels
   */
  void setBufferSize(unsigned width, unsigned height);

  /**
   * \returns the current programID
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return current porgamID. 0 if no shaders are used.
   */
  const GLuint& getProgramID() const;

  /**
   * \brief returns the handle of the depth buffer as an OpenGL texture object
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return handle of the OpenGL texture object for the depth buffer
   */
  GLuint getDepthTexture() const;

  /**
   * \brief returns the handle of the color buffer as an OpenGL texture object
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \return handle of the OpenGL texture object for the color buffer
   */
  GLuint getColorTexture() const;

private:
  /**
   * \brief sets the OpenGL camera parameters
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void setCameraParameters() const;

  /**
   * \brief reads shader source code from file to a string
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] filename path to file containing the shader source code
   * \param[out] source string to be filled with the shader source code
   */
  void readShaderCodeFromFile(const std::string& filename, std::string& source) const;

  /**
   * \brief Compiles, Links and adds the GLSL shaders from strings containing the source codes
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] vertex_source string containing the vertex shader source code
   * \param[in] fragment_source string containing the fragment shader source code
   * \return programID
   */
  GLuint loadShaders(const std::string& vertex_source, const std::string& fragment_source) const;

  /**
   * \brief create a OpenGL shader object from the shader source code
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] shaderID type of shader to be created (e.g. GL_VERTEX_SHADER or GL_FRAGMENT_SHADER)
   * \param[in] source the source code of the shader to be created
   * \return handle to the shader object
   */
  GLuint createShader(GLuint shaderID, const std::string& source) const;

  /**
   * \brief initializes the frame buffer objects
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void initFrameBuffers();

  /**
   * \brief deletes the frame buffer objects
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void deleteFrameBuffers();

  /**
   * \brief create the OpenGL context if required. Only on context is created for each thread
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  static void createGLContext();

  /**
   * \brief deletes OpenGL context for the current thread
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  static void deleteGLContext();

  /** \brief width of frame buffer objects in pixels*/
  unsigned width_;

  /** \brief height of frame buffer objects in pixels*/
  unsigned height_;

  /** \brief handle to frame buffer object*/
  GLuint fbo_id_;

  /** \brief handle to render buffer object*/
  GLuint rbo_id_;

  /** \brief handle to color buffer*/
  GLuint rgb_id_;

  /** \brief handle to depth buffer*/
  GLuint depth_id_;

  /** \brief handle to program that is currently used*/
  GLuint program_;

  /** \brief distance of near clipping plane in meters*/
  float near_;

  /** \brief distance of far clipping plane in meters*/
  float far_;

  /** \brief focal length in x-direction of camera in pixels*/
  float fx_;

  /** \brief focal length in y-direction of camera in pixels*/
  float fy_;

  /** \brief x-coordinate of principal point of camera in pixels*/
  float cx_;

  /** \brief y-coordinate of principal point of camera in pixels*/
  float cy_;

  /** \brief map from thread id to OpenGL context */
  static std::map<std::thread::id, std::pair<unsigned, GLuint> > context_;

  /* \brief lock for context map */
  static std::mutex context_lock_;

  static bool glutInitialized_;
};
}  // namespace mesh_filter
