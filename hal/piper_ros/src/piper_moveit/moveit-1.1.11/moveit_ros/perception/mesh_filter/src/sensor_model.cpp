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

#include <moveit/mesh_filter/sensor_model.h>
#include <stdexcept>

mesh_filter::SensorModel::~SensorModel() = default;

mesh_filter::SensorModel::Parameters::Parameters(unsigned width, unsigned height, float near_clipping_plane_distance,
                                                 float far_clipping_plane_distance)
  : width_(width)
  , height_(height)
  , far_clipping_plane_distance_(far_clipping_plane_distance)
  , near_clipping_plane_distance_(near_clipping_plane_distance)
{
}

mesh_filter::SensorModel::Parameters::~Parameters() = default;

void mesh_filter::SensorModel::Parameters::setImageSize(unsigned width, unsigned height)
{
  width_ = width;
  height_ = height;
}

void mesh_filter::SensorModel::Parameters::setDepthRange(float near, float far)
{
  if (near <= 0)
    throw std::runtime_error("Near clipping plane distance needs to be larger than zero!");

  if (far <= near)
    throw std::runtime_error("Far clipping plane distance must be larger than the near clipping plane distance!");

  near_clipping_plane_distance_ = near;
  far_clipping_plane_distance_ = far;
}

unsigned mesh_filter::SensorModel::Parameters::getWidth() const
{
  return width_;
}

unsigned mesh_filter::SensorModel::Parameters::getHeight() const
{
  return height_;
}

float mesh_filter::SensorModel::Parameters::getNearClippingPlaneDistance() const
{
  return near_clipping_plane_distance_;
}

float mesh_filter::SensorModel::Parameters::getFarClippingPlaneDistance() const
{
  return far_clipping_plane_distance_;
}

namespace
{
#if HAVE_SSE_EXTENSIONS
inline unsigned alignment16(const void* pointer)
{
  return ((uintptr_t)pointer & 15);
}
inline bool isAligned16(const void* pointer)
{
  return (((uintptr_t)pointer & 15) == 0);
}
#endif
}  // namespace

void mesh_filter::SensorModel::Parameters::transformModelDepthToMetricDepth(float* depth) const
{
#if HAVE_SSE_EXTENSIONS
  const __m128 mmNear = _mm_set1_ps(near_clipping_plane_distance_);
  const __m128 mmFar = _mm_set1_ps(far_clipping_plane_distance_);
  const __m128 mmNF = _mm_mul_ps(mmNear, mmFar);
  const __m128 mmF_N = _mm_sub_ps(mmFar, mmNear);
  static const __m128 mmOnes = _mm_set1_ps(1);
  static const __m128 mmZeros = _mm_set1_ps(0);

  float* depthEnd = depth + width_ * height_;
  if (!isAligned16(depth))
  {
    // first depth value without SSE until we reach aligned data
    unsigned first = 16 - alignment16(depth);
    unsigned idx;
    const float near = near_clipping_plane_distance_;
    const float far = far_clipping_plane_distance_;
    const float nf = near * far;
    const float f_n = far - near;

    while (depth < depthEnd && idx++ < first)
      if (*depth != 0 && *depth != 1)
        *depth = nf / (far - *depth * f_n);
      else
        *depth = 0;

    // rest of unaligned data at the end
    unsigned last = (width_ * height_ - first) & 15;
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
  const float near = near_clipping_plane_distance_;
  const float far = far_clipping_plane_distance_;
  const float nf = near * far;
  const float f_n = far - near;

  const float* depth_end = depth + width_ * height_;
  while (depth < depth_end)
  {
    if (*depth != 0 && *depth != 1)
      *depth = nf / (far - *depth * f_n);
    else
      *depth = 0;

    ++depth;
  }
#endif
}

void mesh_filter::SensorModel::Parameters::transformFilteredDepthToMetricDepth(float* depth) const
{
#if HAVE_SSE_EXTENSIONS
  //* SSE version
  const __m128 mmNear = _mm_set1_ps(near_clipping_plane_distance_);
  const __m128 mmFar = _mm_set1_ps(far_clipping_plane_distance_);
  const __m128 mmScale = _mm_sub_ps(mmFar, mmNear);
  float* depthEnd = depth + width_ * height_;

  if (!isAligned16(depth))
  {
    // first depth value without SSE until we reach aligned data
    unsigned first = 16 - alignment16(depth);
    unsigned idx;
    const float scale = far_clipping_plane_distance_ - near_clipping_plane_distance_;
    const float offset = near_clipping_plane_distance_;
    while (depth < depthEnd && idx++ < first)
      if (*depth != 0 && *depth != 1.0)
        *depth = *depth * scale + offset;
      else
        *depth = 0;

    // rest of unaligned data at the end
    unsigned last = (width_ * height_ - first) & 15;
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
  const float* depth_end = depth + width_ * height_;
  const float scale = far_clipping_plane_distance_ - near_clipping_plane_distance_;
  const float offset = near_clipping_plane_distance_;
  while (depth < depth_end)
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
