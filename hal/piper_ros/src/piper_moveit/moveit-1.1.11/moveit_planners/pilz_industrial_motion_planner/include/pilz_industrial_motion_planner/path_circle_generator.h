/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <kdl/path.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>

#include <memory>

namespace pilz_industrial_motion_planner
{
/**
 * @brief Generator class for KDL::Path_Circle from different circle
 * representations
 */
class PathCircleGenerator
{
public:
  /**
   * @brief set the path circle from start, goal and center point
   *
   * Note that a half circle should use interim point and cannot be defined
   * by circle center since start/goal/center points are colinear.
   * @throws KDL::Error_MotionPlanning in case start and goal have different
   * radii to the center point.
   */
  static std::unique_ptr<KDL::Path> circleFromCenter(const KDL::Frame& start_pose, const KDL::Frame& goal_pose,
                                                     const KDL::Vector& center_point, double eqradius);

  /**
   * @brief set circle from start, goal and interim point

   * @throws KDL::Error_MotionPlanning if the given points are colinear.
   */
  static std::unique_ptr<KDL::Path> circleFromInterim(const KDL::Frame& start_pose, const KDL::Frame& goal_pose,
                                                      const KDL::Vector& interim_point, double eqradius);

private:
  PathCircleGenerator(){};  // no instantiation of this helper class!

  /**
   * @brief law of cosines: returns angle gamma in c² = a²+b²-2ab cos(gamma)
   *
   * @return angle in radians
   */
  static double cosines(const double a, const double b, const double c);

  static constexpr double MAX_RADIUS_DIFF{ 1e-2 };
  static constexpr double MAX_COLINEAR_NORM{ 1e-5 };
};

}  // namespace pilz_industrial_motion_planner

class ErrorMotionPlanningCenterPointDifferentRadius : public KDL::Error_MotionPlanning
{
public:
  const char* Description() const override
  {
    return "Distances between start-center and goal-center are different."
           " A circle cannot be created.";
  }
  int GetType() const override
  {
    return ERROR_CODE_CENTER_POINT_DIFFERENT_RADIUS;
  }  // LCOV_EXCL_LINE

private:
  static constexpr int ERROR_CODE_CENTER_POINT_DIFFERENT_RADIUS{ 3006 };
};
