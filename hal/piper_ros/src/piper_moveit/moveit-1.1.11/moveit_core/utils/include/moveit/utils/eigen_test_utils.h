/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
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
 *   * Neither the name of the Bielefeld University nor the names of its
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

#include <gtest/gtest.h>
#include <sstream>
#include <Eigen/Geometry>

/** Provide operator<< for Eigen::Transform */
template <typename _Scalar, int _Dim, int _Mode, int _Options>
std::ostream& operator<<(std::ostream& s, const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& t)
{
  return s << "p=[" << t.translation().transpose() << "] q=[" << Eigen::Quaterniond(t.linear()).coeffs().transpose()
           << "]";
}

/** Predicate to compare two Eigen entities */
template <typename T1, typename T2>
struct IsApprox
{
  double prec_;
  IsApprox(double prec_) : prec_(prec_)
  {
  }

  ::testing::AssertionResult operator()(const char* expr1, const char* expr2, T1 val1, T2 val2)
  {
    if (val1.isApprox(val2, prec_))
      return ::testing::AssertionSuccess();

    std::stringstream msg;
    msg << "Expected equality of these values (up to precision " << prec_ << "):" << std::fixed
        << std::setprecision(1 - std::log10(prec_))        // limit to precision
        << "\n  " << expr1 << "\n    Which is: " << val1   // first
        << "\n  " << expr2 << "\n    Which is: " << val2;  // second
    return ::testing::AssertionFailure() << msg.str();
  }
};

#define EXPECT_EIGEN_EQ(val1, val2)                                                                                    \
  EXPECT_PRED_FORMAT2((IsApprox<decltype(val1), decltype(val2)>(                                                       \
                          Eigen::NumTraits<typename std::decay<decltype(val1)>::type::Scalar>::dummy_precision())),    \
                      val1, val2)
#define EXPECT_EIGEN_NEAR(val1, val2, prec_)                                                                           \
  EXPECT_PRED_FORMAT2((IsApprox<decltype(val1), decltype(val2)>(prec_)), val1, val2)
