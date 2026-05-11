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

#include <moveit/mesh_filter/gl_mesh.h>
#include <geometric_shapes/shapes.h>
#include <stdexcept>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
using shapes::Mesh;

mesh_filter::GLMesh::GLMesh(const Mesh& mesh, unsigned int mesh_label)
{
  if (!mesh.vertex_normals)
    throw std::runtime_error("Vertex normals are not computed for input mesh. Call computeVertexNormals() before "
                             "passing as input to mesh_filter.");

  mesh_label_ = mesh_label;
  list_ = glGenLists(1);
  glNewList(list_, GL_COMPILE);
  glBegin(GL_TRIANGLES);
  glColor4ubv((GLubyte*)&mesh_label_);
  for (unsigned t_idx = 0; t_idx < mesh.triangle_count; ++t_idx)
  {
    unsigned v1 = 3 * mesh.triangles[3 * t_idx];
    unsigned v2 = 3 * mesh.triangles[3 * t_idx + 1];
    unsigned v3 = 3 * mesh.triangles[3 * t_idx + 2];

    glNormal3f(mesh.vertex_normals[v1], mesh.vertex_normals[v1 + 1], mesh.vertex_normals[v1 + 2]);
    glVertex3f(mesh.vertices[v1], mesh.vertices[v1 + 1], mesh.vertices[v1 + 2]);

    glNormal3f(mesh.vertex_normals[v2], mesh.vertex_normals[v2 + 1], mesh.vertex_normals[v2 + 2]);
    glVertex3f(mesh.vertices[v2], mesh.vertices[v2 + 1], mesh.vertices[v2 + 2]);

    glNormal3f(mesh.vertex_normals[v3], mesh.vertex_normals[v3 + 1], mesh.vertex_normals[v3 + 2]);
    glVertex3f(mesh.vertices[v3], mesh.vertices[v3 + 1], mesh.vertices[v3 + 2]);
  }
  glEnd();
  glEndList();
}

mesh_filter::GLMesh::~GLMesh()
{
  glDeleteLists(list_, 1);
}

void mesh_filter::GLMesh::render(const Isometry3d& transform) const
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  if (!(transform.matrix().Flags & RowMajorBit))
    glMultMatrixd(transform.matrix().data());
  else
    glMultTransposeMatrixd(transform.matrix().data());
  glCallList(list_);
  glPopMatrix();
}
