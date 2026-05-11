/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Mark Moll */

// This file is a slightly modified version of <ompl/datastructures/GreedyKCenters.h>

#pragma once

#include <functional>
#include <random>
#include <boost/numeric/ublas/matrix.hpp>

namespace cached_ik_kinematics_plugin
{
/** \brief An instance of this class can be used to greedily select a given
    number of representatives from a set of data points that are all far
    apart from each other. */
template <typename _T>
class GreedyKCenters
{
public:
  /** \brief The definition of a distance function */
  using DistanceFunction = std::function<double(const _T&, const _T&)>;
  /** \brief A matrix type for storing distances between points and centers */
  using Matrix = boost::numeric::ublas::matrix<double>;

  GreedyKCenters() = default;

  virtual ~GreedyKCenters() = default;

  /** \brief Set the distance function to use */
  void setDistanceFunction(const DistanceFunction& distFun)
  {
    distFun_ = distFun;
  }

  /** \brief Get the distance function used */
  const DistanceFunction& getDistanceFunction() const
  {
    return distFun_;
  }

  /** \brief Greedy algorithm for selecting k centers
      \param data a vector of data points
      \param k the desired number of centers
      \param centers a vector of length k containing the indices into
          data of the k centers
      \param dists a matrix such that dists(i,j) is the distance
          between data[i] and data[center[j]]
  */
  void kcenters(const std::vector<_T>& data, unsigned int k, std::vector<unsigned int>& centers, Matrix& dists)
  {
    // array containing the minimum distance between each data point
    // and the centers computed so far
    std::vector<double> min_dist(data.size(), std::numeric_limits<double>::infinity());

    centers.clear();
    centers.reserve(k);
    if (dists.size1() < data.size() || dists.size2() < k)
      dists.resize(std::max(2 * dists.size1() + 1, data.size()), k, false);
    // first center is picked randomly
    centers.push_back(std::uniform_int_distribution<size_t>{ 0, data.size() - 1 }(generator_));
    for (unsigned i = 1; i < k; ++i)
    {
      unsigned ind = 0;
      const _T& center = data[centers[i - 1]];
      double max_dist = -std::numeric_limits<double>::infinity();
      for (unsigned j = 0; j < data.size(); ++j)
      {
        if ((dists(j, i - 1) = distFun_(data[j], center)) < min_dist[j])
          min_dist[j] = dists(j, i - 1);
        // the j-th center is the one furthest away from center 0,..,j-1
        if (min_dist[j] > max_dist)
        {
          ind = j;
          max_dist = min_dist[j];
        }
      }
      // no more centers available
      if (max_dist < std::numeric_limits<double>::epsilon())
        break;
      centers.push_back(ind);
    }

    const _T& center = data[centers.back()];
    unsigned i = centers.size() - 1;
    for (unsigned j = 0; j < data.size(); ++j)
      dists(j, i) = distFun_(data[j], center);
  }

protected:
  /** \brief The used distance function */
  DistanceFunction distFun_;

  /** Random number generator used to select first center */
  std::mt19937 generator_{ std::random_device{}() };
};
}  // namespace cached_ik_kinematics_plugin
