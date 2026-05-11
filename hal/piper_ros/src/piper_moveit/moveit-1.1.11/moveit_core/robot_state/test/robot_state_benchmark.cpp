/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, CITEC Bielefeld
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Robert Haschke */
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <gtest/gtest.h>
#include <chrono>

// Helper class to measure time within a scoped block and output the result
class ScopedTimer
{
  const char* const msg_;
  double* const gold_standard_;
  const std::chrono::time_point<std::chrono::steady_clock> start_;

public:
  // if gold_standard is provided, a relative increase/decrease is shown too
  ScopedTimer(const char* msg = "", double* gold_standard = nullptr)
    : msg_(msg), gold_standard_(gold_standard), start_(std::chrono::steady_clock::now())
  {
  }

  ~ScopedTimer()
  {
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start_;
    std::cerr << msg_ << elapsed.count() * 1000. << "ms ";

    if (gold_standard_)
    {
      if (*gold_standard_ == 0)
        *gold_standard_ = elapsed.count();
      std::cerr << 100 * elapsed.count() / *gold_standard_ << "%";
    }
    std::cerr << std::endl;
  }
};

class Timing : public testing::Test
{
protected:
  void SetUp() override
  {
    Eigen::Isometry3d iso = Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxisd(0.13 * M_PI, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(0.29 * M_PI, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(0.42 * M_PI, Eigen::Vector3d::UnitZ());
    transforms_.push_back(Eigen::Isometry3d::Identity());  // result
    transforms_.push_back(iso);                            // input
  }

  void TearDown() override
  {
  }

public:
  const Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
  // put transforms into a vector to avoid compiler optimization on variables
  EigenSTL::vector_Isometry3d transforms_;
  volatile size_t result_idx_ = 0;
  volatile size_t input_idx_ = 1;
};

TEST_F(Timing, stateUpdate)
{
  moveit::core::RobotModelPtr model = moveit::core::loadTestingRobotModel("pr2_description");
  ASSERT_TRUE(bool(model));
  moveit::core::RobotState state(model);
  ScopedTimer t("RobotState updates: ");
  for (unsigned i = 0; i < 1e5; ++i)
  {
    state.setToRandomPositions();
    state.update();
  }
}

TEST_F(Timing, multiply)
{
  size_t runs = 1e7;
  double gold_standard = 0;
  {
    ScopedTimer t("Eigen::Affine * Eigen::Matrix: ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_].affine().noalias() = transforms_[input_idx_].affine() * transforms_[input_idx_].matrix();
  }
  {
    ScopedTimer t("Eigen::Matrix * Eigen::Matrix: ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_].matrix().noalias() = transforms_[input_idx_].matrix() * transforms_[input_idx_].matrix();
  }
  {
    ScopedTimer t("Eigen::Isometry * Eigen::Isometry: ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_] = transforms_[input_idx_] * transforms_[input_idx_];
  }
}

TEST_F(Timing, inverse)
{
  EigenSTL::vector_Affine3d affine(1);
  affine[0].matrix() = transforms_[input_idx_].matrix();
  size_t runs = 1e7;
  double gold_standard = 0;
  {
    ScopedTimer t("Isometry3d::inverse(): ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_] = transforms_[input_idx_].inverse();
  }
  volatile size_t input_idx = 0;
  {
    ScopedTimer t("Affine3d::inverse(Eigen::Isometry): ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_].affine().noalias() = affine[input_idx].inverse(Eigen::Isometry).affine();
  }
  {
    ScopedTimer t("Affine3d::inverse(): ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_].affine().noalias() = affine[input_idx].inverse().affine();
  }
  {
    ScopedTimer t("Matrix4d::inverse(): ", &gold_standard);
    for (size_t i = 0; i < runs; ++i)
      transforms_[result_idx_].matrix().noalias() = affine[input_idx].matrix().inverse();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
