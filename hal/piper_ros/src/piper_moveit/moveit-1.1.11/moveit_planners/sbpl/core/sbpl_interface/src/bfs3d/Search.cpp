/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Maxim Likhachev
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
 *   * Neither the name of Maxim Likhachev nor the names of its
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

#include <sbpl_interface/bfs3d/BFS_3D.h>
#include <iostream>
#include <boost/thread.hpp>

namespace sbpl_interface
{
#define EXPAND_NEIGHBOR(offset)                                                                                        \
  if (distance_grid[currentNode + offset] < 0)                                                                         \
  {                                                                                                                    \
    queue[queue_tail++] = currentNode + offset;                                                                        \
    distance_grid[currentNode + offset] = currentCost;                                                                 \
    boost::this_thread::interruption_point();                                                                          \
  }

void BFS_3D::search(int width, int planeSize, int volatile* distance_grid, int* queue, int& queue_head, int& queue_tail)
{
  while (queue_head < queue_tail)
  {
    int currentNode = queue[queue_head++];
    int currentCost = distance_grid[currentNode] + 1;

    EXPAND_NEIGHBOR(-width);
    EXPAND_NEIGHBOR(1);
    EXPAND_NEIGHBOR(width);
    EXPAND_NEIGHBOR(-1);
    EXPAND_NEIGHBOR(-width - 1);
    EXPAND_NEIGHBOR(-width + 1);
    EXPAND_NEIGHBOR(width + 1);
    EXPAND_NEIGHBOR(width - 1);
    EXPAND_NEIGHBOR(planeSize);
    EXPAND_NEIGHBOR(-width + planeSize);
    EXPAND_NEIGHBOR(1 + planeSize);
    EXPAND_NEIGHBOR(width + planeSize);
    EXPAND_NEIGHBOR(-1 + planeSize);
    EXPAND_NEIGHBOR(-width - 1 + planeSize);
    EXPAND_NEIGHBOR(-width + 1 + planeSize);
    EXPAND_NEIGHBOR(width + 1 + planeSize);
    EXPAND_NEIGHBOR(width - 1 + planeSize);
    EXPAND_NEIGHBOR(-planeSize);
    EXPAND_NEIGHBOR(-width - planeSize);
    EXPAND_NEIGHBOR(1 - planeSize);
    EXPAND_NEIGHBOR(width - planeSize);
    EXPAND_NEIGHBOR(-1 - planeSize);
    EXPAND_NEIGHBOR(-width - 1 - planeSize);
    EXPAND_NEIGHBOR(-width + 1 - planeSize);
    EXPAND_NEIGHBOR(width + 1 - planeSize);
    EXPAND_NEIGHBOR(width - 1 - planeSize);
  }
  // std::cerr << "Search thread done" << std::endl;
  running = false;
}
}  // namespace sbpl_interface
