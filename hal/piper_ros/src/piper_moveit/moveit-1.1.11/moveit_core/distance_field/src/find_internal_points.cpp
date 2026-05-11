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

/* Author: Acorn Pooley */

#include <moveit/distance_field/find_internal_points.h>

void distance_field::findInternalPointsConvex(const bodies::Body& body, double resolution,
                                              EigenSTL::vector_Vector3d& points)
{
  bodies::BoundingSphere sphere;
  body.computeBoundingSphere(sphere);
  double xval_s = std::floor((sphere.center.x() - sphere.radius - resolution) / resolution) * resolution;
  double yval_s = std::floor((sphere.center.y() - sphere.radius - resolution) / resolution) * resolution;
  double zval_s = std::floor((sphere.center.z() - sphere.radius - resolution) / resolution) * resolution;
  double xval_e = sphere.center.x() + sphere.radius + resolution;
  double yval_e = sphere.center.y() + sphere.radius + resolution;
  double zval_e = sphere.center.z() + sphere.radius + resolution;
  Eigen::Vector3d pt;
  for (pt.x() = xval_s; pt.x() <= xval_e; pt.x() += resolution)
  {
    for (pt.y() = yval_s; pt.y() <= yval_e; pt.y() += resolution)
    {
      for (pt.z() = zval_s; pt.z() <= zval_e; pt.z() += resolution)
      {
        if (body.containsPoint(pt))
        {
          points.push_back(pt);
        }
      }
    }
  }
}
