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

#include <moveit/mesh_filter/stereo_camera_model.h>
#include <moveit/mesh_filter/gl_renderer.h>

mesh_filter::StereoCameraModel::Parameters::Parameters(unsigned width, unsigned height,
                                                       float near_clipping_plane_distance,
                                                       float far_clipping_plane_distance, float fx, float fy, float cx,
                                                       float cy, float base_line, float disparity_resolution)
  : SensorModel::Parameters(width, height, near_clipping_plane_distance, far_clipping_plane_distance)
  , fx_(fx)
  , fy_(fy)
  , cx_(cx)
  , cy_(cy)
  , base_line_(base_line)
  , disparity_resolution_(disparity_resolution)
  , padding_coefficients_(Eigen::Vector3f(disparity_resolution_ / (fx_ * base_line_), 0, 0))
{
}

mesh_filter::StereoCameraModel::Parameters::~Parameters() = default;

mesh_filter::SensorModel::Parameters* mesh_filter::StereoCameraModel::Parameters::clone() const
{
  return new StereoCameraModel::Parameters(width_, height_, near_clipping_plane_distance_, far_clipping_plane_distance_,
                                           fx_, fy_, cx_, cy_, base_line_, disparity_resolution_);
}

void mesh_filter::StereoCameraModel::Parameters::setCameraParameters(float fx, float fy, float cx, float cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

void mesh_filter::StereoCameraModel::Parameters::setBaseline(float base_line)
{
  base_line_ = base_line;
}

void mesh_filter::StereoCameraModel::Parameters::setDisparityResolution(float disparity_resolution)
{
  disparity_resolution_ = disparity_resolution;
}

void mesh_filter::StereoCameraModel::Parameters::setRenderParameters(GLRenderer& renderer) const
{
  renderer.setClippingRange(near_clipping_plane_distance_, far_clipping_plane_distance_);
  renderer.setBufferSize(width_, height_);
  renderer.setCameraParameters(fx_, fy_, cx_, cy_);
  // GLuint padding_coefficients_id = glGetUniformLocation (renderer.getProgramID (), "padding_coefficients");

  //  // set device dependent padding coefficients
  //  glUniform3f (padding_coefficients_id, padding_coefficients_1_ * padding_scale_,
  //                                        padding_coefficients_2_ * padding_scale_,
  //                                        padding_coefficients_3_ * padding_scale_  + padding_offset_ );
}

const Eigen::Vector3f& mesh_filter::StereoCameraModel::Parameters::getPaddingCoefficients() const
{
  return padding_coefficients_;
}

void mesh_filter::StereoCameraModel::Parameters::setFilterParameters(GLRenderer& renderer) const
{
  glUniform1f(glGetUniformLocation(renderer.getProgramID(), "near"), near_clipping_plane_distance_);
  glUniform1f(glGetUniformLocation(renderer.getProgramID(), "far"), far_clipping_plane_distance_);

  renderer.setClippingRange(near_clipping_plane_distance_, far_clipping_plane_distance_);
  renderer.setBufferSize(width_, height_);
  renderer.setCameraParameters(fx_, fy_, cx_, cy_);
}

// NOLINTNEXTLINE(readability-identifier-naming)
const mesh_filter::StereoCameraModel::Parameters& mesh_filter::StereoCameraModel::REGISTERED_PSDK_PARAMS =
    mesh_filter::StereoCameraModel::Parameters(640, 480, 0.4, 10.0, 525, 525, 319.5, 239.5, 0.075, 0.125);

const std::string mesh_filter::StereoCameraModel::RENDER_VERTEX_SHADER_SOURCE =
    "#version 120\n"
    "uniform vec3 padding_coefficients;"
    "void main()"
    "{"
    "  gl_FrontColor = gl_Color;"
    "  gl_BackColor = gl_Color;"
    "  vec4 vertex = gl_ModelViewMatrix * gl_Vertex;"
    "  vec3 normal = normalize(gl_NormalMatrix * gl_Normal);"
    "  float lambda = padding_coefficients.x * vertex.z * vertex.z + padding_coefficients.y * vertex.z + "
    "padding_coefficients.z;"
    "  gl_Position = gl_ProjectionMatrix * (vertex + lambda * vec4(normal,0) );"
    "  gl_Position.y = -gl_Position.y;"
    "}";

const std::string mesh_filter::StereoCameraModel::RENDER_FRAGMENT_SHADER_SOURCE = "#version 120\n"
                                                                                  "void main()"
                                                                                  "{"
                                                                                  "  gl_FragColor = gl_Color;"
                                                                                  "}";

const std::string mesh_filter::StereoCameraModel::FILTER_VERTEX_SHADER_SOURCE =
    "#version 120\n"
    "void main ()"
    "{"
    "     gl_FrontColor = gl_Color;"
    "     gl_TexCoord[0] = gl_MultiTexCoord0;"
    "     gl_Position = gl_Vertex;"
    "  gl_Position.w = 1.0;"
    "}";

const std::string mesh_filter::StereoCameraModel::FILTER_FRAGMENT_SHADER_SOURCE =
    "#version 120\n"
    "uniform sampler2D sensor;"
    "uniform sampler2D depth;"
    "uniform sampler2D label;"
    "uniform float near;"
    "uniform float far;"
    "uniform float shadow_threshold;"
    "const float shadowLabel = 1.0 / 255.0;"
    "const float nearLabel = 2.0 / 255.0;"
    "const float farLabel = 3.0 / 255.0;"
    "float f_n = far - near;"
    "float threshold = shadow_threshold / f_n;"
    "void main()"
    "{"
    " float sValue = float(texture2D(sensor, gl_TexCoord[0].st));"
    " if (sValue <= 0) {"
    "   gl_FragColor = vec4 (nearLabel, 0, 0, 0);"
    "   gl_FragDepth = 0;"
    " }"
    " else {"
    "      float dValue = float(texture2D(depth, gl_TexCoord[0].st));"
    "      float zValue = dValue * near / (far - dValue * f_n);"
    "      float diff = sValue - zValue;"
    "      if (diff < 0 && sValue < 1) {"
    "          gl_FragColor = vec4 (0, 0, 0, 0);"
    "          gl_FragDepth = float(texture2D(sensor, gl_TexCoord[0].st));"
    "      }    else if (diff > threshold) {"
    "          gl_FragColor = vec4 (shadowLabel, 0, 0, 0);"
    "          gl_FragDepth = float(texture2D(sensor, gl_TexCoord[0].st));"
    "      }    else if (sValue == 1) {"
    "          gl_FragColor = vec4 (farLabel, 0, 0, 0);"
    "          gl_FragDepth = float(texture2D(sensor, gl_TexCoord[0].st));"
    "   } else {"
    "          gl_FragColor = texture2D(label, gl_TexCoord[0].st);"
    "          gl_FragDepth = 0;"
    "      }"
    " }"
    "}";
