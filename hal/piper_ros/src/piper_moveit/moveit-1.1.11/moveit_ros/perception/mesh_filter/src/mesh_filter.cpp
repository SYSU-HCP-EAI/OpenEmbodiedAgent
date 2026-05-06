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

#include <moveit/mesh_filter/mesh_filter.h>
#include <moveit/mesh_filter/gl_mesh.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Eigen>
#include <stdexcept>
#include <sensor_msgs/image_encodings.h>
#include <stdint.h>

// include SSE headers
#ifdef HAVE_SSE_EXTENSIONS
#include <xmmintrin.h>
#endif

using namespace std;
using namespace Eigen;
using shapes::Mesh;

mesh_filter::MeshFilter::MeshFilter(const std::function<bool(MeshFilter::MeshHandle, Isometry3d&)>& transform_callback)
  : mesh_renderer_(640, 480, 0.3, 10)  // some default values for buffer sizes and clipping planes
  , depth_filter_(640, 480, 0.3, 10)
  , next_handle_(FirstLabel)  // 0 and 1 are reserved!
  , transform_callback_(transform_callback)
  , shadow_threshold_(0.5)
{
  mesh_renderer_.setShadersFromString(padding_vertex_shader_, "");
  depth_filter_.setShadersFromString(filter_vertex_shader_, filter_fragment_shader_);

  depth_filter_.begin();

  glGenTextures(1, &sensor_depth_texture_);

  glUniform1i(glGetUniformLocation(depth_filter_.getProgramID(), "sensor"), 0);
  glUniform1i(glGetUniformLocation(depth_filter_.getProgramID(), "depth"), 2);
  glUniform1i(glGetUniformLocation(depth_filter_.getProgramID(), "label"), 4);

  near_location_ = glGetUniformLocation(depth_filter_.getProgramID(), "near");
  far_location_ = glGetUniformLocation(depth_filter_.getProgramID(), "far");
  shadow_threshold_location_ = glGetUniformLocation(depth_filter_.getProgramID(), "shadow_threshold");

  depth_filter_.end();

  canvas_ = glGenLists(1);
  glNewList(canvas_, GL_COMPILE);
  glBegin(GL_QUADS);

  glColor3f(1, 1, 1);
  glTexCoord2f(0, 0);
  glVertex3f(-1, -1, 0);

  glTexCoord2f(1, 0);
  glVertex3f(1, -1, 0);

  glTexCoord2f(1, 1);
  glVertex3f(1, 1, 0);

  glTexCoord2f(0, 1);
  glVertex3f(-1, 1, 0);

  glEnd();
  glEndList();
}

mesh_filter::MeshFilter::~MeshFilter()
{
  glDeleteLists(1, canvas_);
  glDeleteTextures(1, &sensor_depth_texture_);

  for (map<unsigned int, GLMesh*>::iterator meshIt = meshes_.begin(); meshIt != meshes_.end(); ++meshIt)
    delete (meshIt->second);
  meshes_.clear();
}

void mesh_filter::MeshFilter::setSize(unsigned width, unsigned height)
{
  mesh_renderer_.setBufferSize(width, height);
  mesh_renderer_.setCameraParameters(width, width, width >> 1, height >> 1);

  depth_filter_.setBufferSize(width, height);
  depth_filter_.setCameraParameters(width, width, width >> 1, height >> 1);
}

void mesh_filter::MeshFilter::setTransformCallback(
    const std::function<bool(mesh_filter::MeshFilter::MeshHandle, Isometry3d&)>& transform_callback)
{
  transform_callback_ = transform_callback;
}

void mesh_filter::MeshFilter::setPaddingCoefficients(const Eigen::Vector3f& padding_coefficients)
{
  padding_coefficients_ = padding_coefficients;
}

mesh_filter::MeshFilter::MeshHandle mesh_filter::MeshFilter::addMesh(const Mesh& mesh)
{
  Mesh collapsedMesh(1, 1);
  mergeVertices(mesh, collapsedMesh);
  meshes_[next_handle_] = new GLMesh(collapsedMesh, next_handle_);
  return next_handle_++;
}

void mesh_filter::MeshFilter::removeMesh(MeshFilter::MeshHandle handle)
{
  long unsigned int erased = meshes_.erase(handle);
  if (erased == 0)
    throw runtime_error("Could not remove mesh. Mesh not found!");
}

void mesh_filter::MeshFilter::setShadowThreshold(float threshold)
{
  shadow_threshold_ = threshold;
}

void mesh_filter::MeshFilter::getModelLabels(unsigned* labels) const
{
  mesh_renderer_.getColorBuffer((unsigned char*)labels);
}

namespace
{
inline unsigned alignment16(const void* pointer)
{
  return ((uintptr_t)pointer & 15);
}
inline bool isAligned16(const void* pointer)
{
  return (((uintptr_t)pointer & 15) == 0);
}
}  // namespace

void mesh_filter::MeshFilter::getModelDepth(float* depth) const
{
  mesh_renderer_.getDepthBuffer(depth);
#if HAVE_SSE_EXTENSIONS
  const __m128 mmNear = _mm_set1_ps(mesh_renderer_.getNearClippingDistance());
  const __m128 mmFar = _mm_set1_ps(mesh_renderer_.getFarClippingDistance());
  const __m128 mmNF = _mm_mul_ps(mmNear, mmFar);
  const __m128 mmF_N = _mm_sub_ps(mmFar, mmNear);
  static const __m128 mmOnes = _mm_set1_ps(1);
  static const __m128 mmZeros = _mm_set1_ps(0);

  float* depthEnd = depth + mesh_renderer_.getHeight() * mesh_renderer_.getWidth();
  if (!isAligned16(depth))
  {
    // first depth value without SSE until we reach aligned data
    unsigned first = 16 - alignment16(depth);
    unsigned idx;
    const float near = mesh_renderer_.getNearClippingDistance();
    const float far = mesh_renderer_.getFarClippingDistance();
    const float nf = near * far;
    const float f_n = far - near;

    while (depth < depthEnd && idx++ < first)
      if (*depth != 0 && *depth != 1)
        *depth = nf / (far - *depth * f_n);
      else
        *depth = 0;

    // rest of unaligned data at the end
    unsigned last = (depth_filter_.getWidth() * depth_filter_.getHeight() - first) & 15;
    float* depth2 = depthEnd - last;
    while (depth2 < depthEnd)
      if (*depth2 != 0 && *depth2 != 1)
        *depth2 = nf / (far - *depth2 * f_n);
      else
        *depth2 = 0;

    depthEnd -= last;
  }

  const __m128* mmEnd = (__m128*)depthEnd;
  __m128* mmDepth = (__m128*)depth;
  // rest is aligned
  while (mmDepth < mmEnd)
  {
    __m128 mask = _mm_and_ps(_mm_cmpneq_ps(*mmDepth, mmOnes), _mm_cmpneq_ps(*mmDepth, mmZeros));
    *mmDepth = _mm_mul_ps(*mmDepth, mmF_N);
    *mmDepth = _mm_sub_ps(mmFar, *mmDepth);
    *mmDepth = _mm_div_ps(mmNF, *mmDepth);
    *mmDepth = _mm_and_ps(*mmDepth, mask);
    ++mmDepth;
  }

#else
  // calculate metric depth values from OpenGL normalized depth buffer
  const float near = mesh_renderer_.getNearClippingDistance();
  const float far = mesh_renderer_.getFarClippingDistance();
  const float nf = near * far;
  const float f_n = far - near;

  const float* depthEnd = depth + mesh_renderer_.getHeight() * mesh_renderer_.getWidth();
  while (depth < depthEnd)
  {
    if (*depth != 0 && *depth != 1)
      *depth = nf / (far - *depth * f_n);
    else
      *depth = 0;

    ++depth;
  }
#endif
}

void mesh_filter::MeshFilter::getFilteredDepth(float* depth) const
{
  depth_filter_.getDepthBuffer(depth);
#if HAVE_SSE_EXTENSIONS
  //* SSE version
  const __m128 mmNear = _mm_set1_ps(depth_filter_.getNearClippingDistance());
  const __m128 mmFar = _mm_set1_ps(depth_filter_.getFarClippingDistance());
  const __m128 mmScale = _mm_sub_ps(mmFar, mmNear);
  float* depthEnd = depth + depth_filter_.getWidth() * depth_filter_.getHeight();

  if (!isAligned16(depth))
  {
    // first depth value without SSE until we reach aligned data
    unsigned first = 16 - alignment16(depth);
    unsigned idx;
    const float scale = depth_filter_.getFarClippingDistance() - depth_filter_.getNearClippingDistance();
    const float offset = depth_filter_.getNearClippingDistance();
    while (depth < depthEnd && idx++ < first)
      if (*depth != 0 && *depth != 1.0)
        *depth = *depth * scale + offset;
      else
        *depth = 0;

    // rest of unaligned data at the end
    unsigned last = (depth_filter_.getWidth() * depth_filter_.getHeight() - first) & 15;
    float* depth2 = depthEnd - last;
    while (depth2 < depthEnd)
      if (*depth2 != 0 && *depth != 1.0)
        *depth2 = *depth2 * scale + offset;
      else
        *depth2 = 0;

    depthEnd -= last;
  }

  const __m128* mmEnd = (__m128*)depthEnd;
  __m128* mmDepth = (__m128*)depth;
  // rest is aligned
  while (mmDepth < mmEnd)
  {
    *mmDepth = _mm_mul_ps(*mmDepth, mmScale);
    *mmDepth = _mm_add_ps(*mmDepth, mmNear);
    *mmDepth = _mm_and_ps(*mmDepth, _mm_and_ps(_mm_cmpneq_ps(*mmDepth, mmNear), _mm_cmpneq_ps(*mmDepth, mmFar)));
    ++mmDepth;
  }
#else
  const float* depthEnd = depth + depth_filter_.getWidth() * depth_filter_.getHeight();
  const float scale = depth_filter_.getFarClippingDistance() - depth_filter_.getNearClippingDistance();
  const float offset = depth_filter_.getNearClippingDistance();
  while (depth < depthEnd)
  {
    // 0 = on near clipping plane -> we used 0 to mark invalid points -> not visible
    // points on far clipping plane needs to be removed too
    if (*depth != 0 && *depth != 1.0)
      *depth = *depth * scale + offset;
    else
      *depth = 0;

    ++depth;
  }
#endif
}

void mesh_filter::MeshFilter::getFilteredLabels(unsigned* labels) const
{
  depth_filter_.getColorBuffer((unsigned char*)labels);
}

void mesh_filter::MeshFilter::filter(const float* sensor_data, unsigned width, unsigned height, float fx, float fy,
                                     float cx, float cy) const
{
  mesh_renderer_.setBufferSize(width, height);
  mesh_renderer_.setCameraParameters(fx, fy, cx, cy);
  mesh_renderer_.begin();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_FRONT);
  glDisable(GL_ALPHA_TEST);

  GLuint padding_coefficients_id = glGetUniformLocation(mesh_renderer_.getProgramID(), "padding_coefficients");
  glUniform3f(padding_coefficients_id, padding_coefficients_[0], padding_coefficients_[1], padding_coefficients_[2]);

  Isometry3d transform;
  for (map<unsigned int, GLMesh*>::const_iterator meshIt = meshes_.begin(); meshIt != meshes_.end(); ++meshIt)
    if (transform_callback_(meshIt->first, transform))
      meshIt->second->render(transform);

  mesh_renderer_.end();

  // now filter the depth_map with the second rendering stage
  depth_filter_.setBufferSize(width, height);
  depth_filter_.setCameraParameters(fx, fy, cx, cy);
  depth_filter_.begin();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glDisable(GL_ALPHA_TEST);

  glUniform1f(near_location_, depth_filter_.getNearClippingDistance());
  glUniform1f(far_location_, depth_filter_.getFarClippingDistance());
  glUniform1f(shadow_threshold_location_, shadow_threshold_);

  GLuint depth_texture = mesh_renderer_.getDepthTexture();
  GLuint color_texture = mesh_renderer_.getColorTexture();

  // bind sensor depth
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, sensor_depth_texture_);

  float scale = 1.0 / (depth_filter_.getFarClippingDistance() - depth_filter_.getNearClippingDistance());
  glPixelTransferf(GL_DEPTH_SCALE, scale);
  glPixelTransferf(GL_DEPTH_BIAS, -scale * depth_filter_.getNearClippingDistance());
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, sensor_data);
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
  glDisable(GL_TEXTURE_2D);
  depth_filter_.end();
}

void mesh_filter::MeshFilter::setClippingRange(float near, float far)
{
  mesh_renderer_.setClippingRange(near, far);
  depth_filter_.setClippingRange(near, far);
}

void mesh_filter::MeshFilter::mergeVertices(const Mesh& mesh, Mesh& compressed)
{
  // max allowed distance of vertices to be considered same! 1mm
  // to allow double-error from transformed vertices.
  static const double thresholdSQR = pow(0.00001, 2);

  vector<unsigned> vertex_map(mesh.vertex_count);
  vector<Vector3d> vertices(mesh.vertex_count);
  vector<Vector3d> compressed_vertices;
  vector<Vector3i> triangles(mesh.triangle_count);

  for (unsigned vIdx = 0; vIdx < mesh.vertex_count; ++vIdx)
  {
    vertices[vIdx][0] = mesh.vertices[3 * vIdx];
    vertices[vIdx][1] = mesh.vertices[3 * vIdx + 1];
    vertices[vIdx][2] = mesh.vertices[3 * vIdx + 2];
    vertex_map[vIdx] = vIdx;
  }

  for (unsigned tIdx = 0; tIdx < mesh.triangle_count; ++tIdx)
  {
    triangles[tIdx][0] = mesh.triangles[3 * tIdx];
    triangles[tIdx][1] = mesh.triangles[3 * tIdx + 1];
    triangles[tIdx][2] = mesh.triangles[3 * tIdx + 2];
  }

  for (unsigned vIdx1 = 0; vIdx1 < mesh.vertex_count; ++vIdx1)
  {
    if (vertex_map[vIdx1] != vIdx1)
      continue;

    vertex_map[vIdx1] = compressed_vertices.size();
    compressed_vertices.push_back(vertices[vIdx1]);

    for (unsigned vIdx2 = vIdx1 + 1; vIdx2 < mesh.vertex_count; ++vIdx2)
    {
      double distanceSQR = (vertices[vIdx1] - vertices[vIdx2]).squaredNorm();
      if (distanceSQR <= thresholdSQR)
        vertex_map[vIdx2] = vertex_map[vIdx1];
    }
  }

  // redirect triangles to new vertices!
  for (unsigned tIdx = 0; tIdx < mesh.triangle_count; ++tIdx)
  {
    triangles[tIdx][0] = vertex_map[triangles[tIdx][0]];
    triangles[tIdx][1] = vertex_map[triangles[tIdx][1]];
    triangles[tIdx][2] = vertex_map[triangles[tIdx][2]];
  }

  for (int vIdx = 0; vIdx < vertices.size(); ++vIdx)
  {
    if (vertices[vIdx][0] == vertices[vIdx][1] || vertices[vIdx][0] == vertices[vIdx][2] ||
        vertices[vIdx][1] == vertices[vIdx][2])
    {
      vertices[vIdx] = vertices.back();
      vertices.pop_back();
      --vIdx;
    }
  }

  // create new Mesh structure
  if (compressed.vertex_count > 0 && compressed.vertices)
    delete[] compressed.vertices;
  if (compressed.triangle_count > 0 && compressed.triangles)
    delete[] compressed.triangles;
  if (compressed.triangle_count > 0 && compressed.normals)
    delete[] compressed.normals;

  compressed.vertex_count = compressed_vertices.size();
  compressed.vertices = new double[compressed.vertex_count * 3];
  for (unsigned vIdx = 0; vIdx < compressed.vertex_count; ++vIdx)
  {
    compressed.vertices[3 * vIdx + 0] = compressed_vertices[vIdx][0];
    compressed.vertices[3 * vIdx + 1] = compressed_vertices[vIdx][1];
    compressed.vertices[3 * vIdx + 2] = compressed_vertices[vIdx][2];
  }

  compressed.triangle_count = triangles.size();
  compressed.triangles = new unsigned int[compressed.triangle_count * 3];
  for (unsigned tIdx = 0; tIdx < compressed.triangle_count; ++tIdx)
  {
    compressed.triangles[3 * tIdx + 0] = triangles[tIdx][0];
    compressed.triangles[3 * tIdx + 1] = triangles[tIdx][1];
    compressed.triangles[3 * tIdx + 2] = triangles[tIdx][2];
  }

  compressed.normals = new double[compressed.triangle_count * 3];
  for (unsigned tIdx = 0; tIdx < compressed.triangle_count; ++tIdx)
  {
    Vector3d d1 = compressed_vertices[triangles[tIdx][1]] - compressed_vertices[triangles[tIdx][0]];
    Vector3d d2 = compressed_vertices[triangles[tIdx][2]] - compressed_vertices[triangles[tIdx][0]];
    Vector3d normal = d1.cross(d2);
    normal.normalize();
    Vector3d normal_;
    normal_[0] = mesh.normals[3 * tIdx];
    normal_[1] = mesh.normals[3 * tIdx + 1];
    normal_[2] = mesh.normals[3 * tIdx + 2];
    double cosAngle = normal.dot(normal_);

    if (cosAngle < 0)
    {
      swap(compressed.triangles[3 * tIdx + 1], compressed.triangles[3 * tIdx + 2]);
      normal *= -1;
    }

    compressed.normals[tIdx * 3 + 0] = normal[0];
    compressed.normals[tIdx * 3 + 1] = normal[1];
    compressed.normals[tIdx * 3 + 2] = normal[2];
  }
}

string mesh_filter::MeshFilter::padding_vertex_shader_ =
    "#version 120\n"
    "uniform vec3 padding_coefficients;"
    "void main()"
    "{gl_FrontColor = gl_Color;"
    "vec4 vertex = gl_ModelViewMatrix * gl_Vertex;"
    "vec3 normal = normalize(gl_NormalMatrix * gl_Normal);"
    "float lambda = padding_coefficients.x * vertex.z * vertex.z + padding_coefficients.y * vertex.z + "
    "padding_coefficients.z;"
    "gl_Position = gl_ProjectionMatrix * (vertex + lambda * vec4(normal,0) );"
    "gl_Position.y = -gl_Position.y;}";

string mesh_filter::MeshFilter::filter_vertex_shader_ = "#version 120\n"
                                                        "void main ()"
                                                        "{"
                                                        "    gl_FrontColor = gl_Color;"
                                                        "    gl_TexCoord[0] = gl_MultiTexCoord0;"
                                                        "    gl_Position = gl_Vertex;"
                                                        "  gl_Position.w = 1.0;"
                                                        "}";

string mesh_filter::MeshFilter::filter_fragment_shader_ =
    "#version 120\n"
    "uniform sampler2D sensor;"
    "uniform sampler2D depth;"
    "uniform sampler2D label;"
    "uniform float near;"
    "uniform float far;"
    "uniform float shadow_threshold;"
    "float f_n = far - near;"
    "float threshold = shadow_threshold / f_n;"
    "void main()"
    "{"
    "    float dValue = float(texture2D(depth, gl_TexCoord[0].st));"
    "    float zValue = dValue * near / (far - dValue * f_n);"
    "    float diff = float(texture2D(sensor, gl_TexCoord[0].st)) - zValue;"
    "    if (diff < 0) {"
    "        gl_FragColor = vec4 (0, 0, 0, 0);"
    "        gl_FragDepth = float(texture2D(sensor, gl_TexCoord[0].st));"
    "    }    else if (diff > threshold) {"
    "        gl_FragColor = vec4 (0.003921569, 0, 0, 0);"
    "        gl_FragDepth = float(texture2D(sensor, gl_TexCoord[0].st));"
    " } else {"
    "        gl_FragColor = vec4 (1,1,1,1);"
    "        gl_FragDepth = 0;"
    "    }"
    "}";
