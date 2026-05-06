/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

// This file is a slightly modified version of <ompl/datastructures/NearestNeighbors.h>

#pragma once

#include <vector>
#include <functional>

namespace cached_ik_kinematics_plugin
{
/** \brief Abstract representation of a container that can perform nearest neighbors queries */
template <typename _T>
class NearestNeighbors
{
public:
  /** \brief The definition of a distance function */
  typedef std::function<double(const _T&, const _T&)> DistanceFunction;

  NearestNeighbors() = default;

  virtual ~NearestNeighbors() = default;

  /** \brief Set the distance function to use */
  virtual void setDistanceFunction(const DistanceFunction& distFun)
  {
    distFun_ = distFun;
  }

  /** \brief Get the distance function used */
  const DistanceFunction& getDistanceFunction() const
  {
    return distFun_;
  }

  /** \brief Return true if the solutions reported by this data structure
      are sorted, when calling nearestK / nearestR. */
  virtual bool reportsSortedResults() const = 0;

  /** \brief Clear the datastructure */
  virtual void clear() = 0;

  /** \brief Add an element to the datastructure */
  virtual void add(const _T& data) = 0;

  /** \brief Add a vector of points */
  virtual void add(const std::vector<_T>& data)
  {
    for (auto elt = data.begin(); elt != data.end(); ++elt)
      add(*elt);
  }

  /** \brief Remove an element from the datastructure */
  virtual bool remove(const _T& data) = 0;

  /** \brief Get the nearest neighbor of a point */
  virtual _T nearest(const _T& data) const = 0;

  /** \brief Get the k-nearest neighbors of a point
   *
   * All the nearest neighbor structures currently return the neighbors in
   * sorted order, but this is not required.
   */
  virtual void nearestK(const _T& data, std::size_t k, std::vector<_T>& nbh) const = 0;

  /** \brief Get the nearest neighbors of a point, within a specified radius
   *
   * All the nearest neighbor structures currently return the neighbors in
   * sorted order, but this is not required.
   */
  virtual void nearestR(const _T& data, double radius, std::vector<_T>& nbh) const = 0;

  /** \brief Get the number of elements in the datastructure */
  virtual std::size_t size() const = 0;

  /** \brief Get all the elements in the datastructure */
  virtual void list(std::vector<_T>& data) const = 0;

protected:
  /** \brief The used distance function */
  DistanceFunction distFun_;
};
}  // namespace cached_ik_kinematics_plugin
