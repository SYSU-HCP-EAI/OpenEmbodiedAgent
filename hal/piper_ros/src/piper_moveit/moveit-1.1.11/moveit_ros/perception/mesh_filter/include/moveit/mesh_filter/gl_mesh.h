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

#include <Eigen/Geometry>  // for Isometry3d
#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace shapes
{
class Mesh;
}

namespace mesh_filter
{
/**
 * \brief GLMesh represents a mesh from geometric_shapes for rendering in GL frame buffers
 * \author Suat Gedikli (gedikli@willowgarage.com)
 */
class GLMesh
{
public:
  /**
   * \brief Constucts a GLMesh object for given mesh and label
   * \author Suat Gedikli (gedikli@willowgarage.com)
   * \param[in] mesh
   * \param[in] mesh_label
   */
  GLMesh(const shapes::Mesh& mesh, unsigned int mesh_label);

  /** \brief Destructor*/
  ~GLMesh();
  /**
   * \brief renders the mesh in current OpenGL frame buffer (context)
   * \param[in] transform the modelview transformation describing the pose of the mesh in camera coordinate frame
   * \author Suat Gedikli (gedikli@willowgarage.com)
   */
  void render(const Eigen::Isometry3d& transform) const;

private:
  /** \brief the OpenGL mesh represented as a OpenGL list */
  GLuint list_;

  /** \brief label of current mesh*/
  unsigned int mesh_label_;
};
}  // namespace mesh_filter
