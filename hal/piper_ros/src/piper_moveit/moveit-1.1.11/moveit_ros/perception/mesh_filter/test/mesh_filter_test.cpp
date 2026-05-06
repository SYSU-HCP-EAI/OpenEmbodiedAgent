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

#include <gtest/gtest.h>
#include <moveit/mesh_filter/mesh_filter.h>
#include <moveit/mesh_filter/stereo_camera_model.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <eigen3/Eigen/Eigen>
#include <vector>

using namespace mesh_filter;
using namespace Eigen;
using namespace std;
using namespace std::placeholders;

namespace mesh_filter_test
{
template <typename Type>
inline const Type getRandomNumber(const Type& min, const Type& max)
{
  return Type(min + (max - min) * double(rand()) / double(RAND_MAX));
}

template <typename Type>
class FilterTraits
{
public:
  static const GLushort FILTER_GL_TYPE = GL_ZERO;
};

template <>
class FilterTraits<unsigned short>
{
public:
  static const GLushort FILTER_GL_TYPE = GL_UNSIGNED_SHORT;
  static constexpr double ToMetricScale = 0.001;
};

template <>
class FilterTraits<float>
{
public:
  static const GLushort FILTER_GL_TYPE = GL_FLOAT;
  static constexpr double ToMetricScale = 1.0f;
};

template <typename Type>
class MeshFilterTest : public testing::TestWithParam<double>
{
  BOOST_STATIC_ASSERT_MSG(FilterTraits<Type>::FILTER_GL_TYPE != GL_ZERO, "Only \"float\" and \"unsigned short int\" "
                                                                         "are allowed.");

public:
  MeshFilterTest(unsigned width = 500, unsigned height = 500, double near = 0.5, double far = 5.0, double shadow = 0.1,
                 double epsilon = 1e-7);
  void test();
  void setMeshDistance(double distance)
  {
    distance_ = distance;
  }

private:
  shapes::Mesh createMesh(double z) const;
  bool transformCallback(MeshHandle handle, Isometry3d& transform) const;
  void getGroundTruth(unsigned int* labels, float* depth) const;
  const unsigned int width_;
  const unsigned int height_;
  const double near_;
  const double far_;
  const double shadow_;
  const double epsilon_;
  StereoCameraModel::Parameters sensor_parameters_;
  MeshFilter<StereoCameraModel> filter_;
  MeshHandle handle_;
  vector<Type> sensor_data_;
  double distance_;
};

template <typename Type>
MeshFilterTest<Type>::MeshFilterTest(unsigned width, unsigned height, double near, double far, double shadow,
                                     double epsilon)
  : width_(width)
  , height_(height)
  , near_(near)
  , far_(far)
  , shadow_(shadow)
  , epsilon_(epsilon)
  , sensor_parameters_(width, height, near_, far_, width >> 1, height >> 1, width >> 1, height >> 1, 0.1, 0.1)
  , filter_([this](mesh_filter::MeshHandle mesh, Eigen::Isometry3d& tf) { return transformCallback(mesh, tf); },
            sensor_parameters_)
  , sensor_data_(width_ * height_)
  , distance_(0.0)
{
  filter_.setShadowThreshold(shadow_);
  // no padding
  filter_.setPaddingOffset(0.0);
  filter_.setPaddingScale(0.0);

  // create a large plane that covers the whole visible area -> no boundaries

  shapes::Mesh mesh = createMesh(0);
  handle_ = filter_.addMesh(mesh);

  // make it random but reproducable
  srand(0);
  Type t_near = near_ / FilterTraits<Type>::ToMetricScale;
  Type t_far = far_ / FilterTraits<Type>::ToMetricScale;
  for (typename vector<Type>::iterator s_it = sensor_data_.begin(); s_it != sensor_data_.end(); ++s_it)
  {
    do
    {
      *s_it = getRandomNumber<Type>(0.0, 10.0 / FilterTraits<Type>::ToMetricScale);
    } while (*s_it == t_near || *s_it == t_far);
  }
}

template <typename Type>
shapes::Mesh MeshFilterTest<Type>::createMesh(double z) const
{
  shapes::Mesh mesh(4, 4);
  mesh.vertices[0] = -5;
  mesh.vertices[1] = -5;
  mesh.vertices[2] = z;

  mesh.vertices[3] = -5;
  mesh.vertices[4] = 5;
  mesh.vertices[5] = z;

  mesh.vertices[6] = 5;
  mesh.vertices[7] = 5;
  mesh.vertices[8] = z;

  mesh.vertices[9] = 5;
  mesh.vertices[10] = -5;
  mesh.vertices[11] = z;

  mesh.triangles[0] = 0;
  mesh.triangles[1] = 3;
  mesh.triangles[2] = 2;

  mesh.triangles[3] = 0;
  mesh.triangles[4] = 2;
  mesh.triangles[5] = 1;

  mesh.triangles[6] = 0;
  mesh.triangles[7] = 2;
  mesh.triangles[8] = 3;

  mesh.triangles[9] = 0;
  mesh.triangles[10] = 1;
  mesh.triangles[11] = 2;

  mesh.vertex_normals[0] = 0;
  mesh.vertex_normals[1] = 0;
  mesh.vertex_normals[2] = 1;

  mesh.vertex_normals[3] = 0;
  mesh.vertex_normals[4] = 0;
  mesh.vertex_normals[5] = 1;

  mesh.vertex_normals[6] = 0;
  mesh.vertex_normals[7] = 0;
  mesh.vertex_normals[8] = 1;

  mesh.vertex_normals[9] = 0;
  mesh.vertex_normals[10] = 0;
  mesh.vertex_normals[11] = 1;

  return mesh;
}

template <typename Type>
bool MeshFilterTest<Type>::transformCallback(MeshHandle handle, Isometry3d& transform) const
{
  transform = Isometry3d::Identity();
  if (handle == handle_)
    transform.translation() = Vector3d(0, 0, distance_);
  return true;
}

template <typename Type>
void MeshFilterTest<Type>::test()
{
  shapes::Mesh mesh = createMesh(0);
  mesh_filter::MeshHandle handle = filter_.addMesh(mesh);
  filter_.filter(&sensor_data_[0], FilterTraits<Type>::FILTER_GL_TYPE, false);

  vector<float> gt_depth(width_ * height_);
  vector<unsigned int> gt_labels(width_ * height_);
  getGroundTruth(&gt_labels[0], &gt_depth[0]);

  vector<float> filtered_depth(width_ * height_);
  vector<unsigned int> filtered_labels(width_ * height_);
  filter_.getFilteredDepth(&filtered_depth[0]);
  filter_.getFilteredLabels(&filtered_labels[0]);

  for (unsigned idx = 0; idx < width_ * height_; ++idx)
  {
    // Only test if we are not very close to boundaries of object meshes and shadow-boundaries.
    float sensor_depth = sensor_data_[idx] * FilterTraits<Type>::ToMetricScale;
    if (fabs(sensor_depth - distance_ - shadow_) > epsilon_ && fabs(sensor_depth - distance_) > epsilon_)
    {
      ASSERT_NEAR(filtered_depth[idx], gt_depth[idx], 1e-4);
      ASSERT_EQ(filtered_labels[idx], gt_labels[idx]);
    }
  }
  filter_.removeMesh(handle);
}

template <typename Type>
void MeshFilterTest<Type>::getGroundTruth(unsigned int* labels, float* depth) const
{
  const double scale = FilterTraits<Type>::ToMetricScale;
  if (distance_ <= near_ || distance_ >= far_)
  {
    // no filtering is done -> no shadow values or label values
    for (unsigned y_idx = 0, idx = 0; y_idx < height_; ++y_idx)
    {
      for (unsigned x_idx = 0; x_idx < width_; ++x_idx, ++idx)
      {
        depth[idx] = double(sensor_data_[idx]) * scale;
        if (depth[idx] < near_)
          labels[idx] = MeshFilterBase::NEAR_CLIP;
        else if (depth[idx] >= far_)
          labels[idx] = MeshFilterBase::FAR_CLIP;
        else
          labels[idx] = MeshFilterBase::BACKGROUND;

        if (depth[idx] <= near_ || depth[idx] >= far_)
          depth[idx] = 0;
      }
    }
  }
  else
  {
    for (unsigned y_idx = 0, idx = 0; y_idx < height_; ++y_idx)
    {
      for (unsigned x_idx = 0; x_idx < width_; ++x_idx, ++idx)
      {
        depth[idx] = double(sensor_data_[idx]) * scale;

        if (depth[idx] < near_)
        {
          labels[idx] = MeshFilterBase::NEAR_CLIP;
          depth[idx] = 0;
        }
        else
        {
          double diff = depth[idx] - distance_;
          if (diff < 0 && depth[idx] < far_)
            labels[idx] = MeshFilterBase::BACKGROUND;
          else if (diff > shadow_)
            labels[idx] = MeshFilterBase::SHADOW;
          else if (depth[idx] >= far_)
            labels[idx] = MeshFilterBase::FAR_CLIP;
          else
          {
            labels[idx] = MeshFilterBase::FIRST_LABEL;
            depth[idx] = 0;
          }

          if (depth[idx] >= far_)
            depth[idx] = 0;
        }
      }
    }
  }
}

}  // namespace mesh_filter_test

typedef mesh_filter_test::MeshFilterTest<float> MeshFilterTestFloat;
TEST_P(MeshFilterTestFloat, float)
{
  this->setMeshDistance(this->GetParam());
  this->test();
}
INSTANTIATE_TEST_CASE_P(float_test, MeshFilterTestFloat, ::testing::Range<double>(0.0f, 6.0f, 0.5f));

typedef mesh_filter_test::MeshFilterTest<unsigned short> MeshFilterTestUnsignedShort;
TEST_P(MeshFilterTestUnsignedShort, unsigned_short)
{
  this->setMeshDistance(this->GetParam());
  this->test();
}
INSTANTIATE_TEST_CASE_P(ushort_test, MeshFilterTestUnsignedShort, ::testing::Range<double>(0.0f, 6.0f, 0.5f));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
