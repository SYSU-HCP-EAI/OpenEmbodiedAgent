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

/** \Author: Benjamin Cohen /bcohen@willowgarage.com, E. Gil Jones **/

#pragma once

#include <boost/thread.hpp>

namespace sbpl_interface
{
#define WALL 0x7FFFFFFF
#define UNDISCOVERED 0xFFFFFFFF

class BFS_3D
{
private:
  int dim_x, dim_y, dim_z;
  int dim_xy, dim_xyz;

  int origin;
  int volatile* distance_grid;

  int* queue;
  int queue_head, queue_tail;

  std::shared_ptr<boost::thread> search_thread_;

  volatile bool running;

  void search(int, int, int volatile*, int*, int&, int&);
  inline int getNode(int, int, int);

public:
  BFS_3D(int, int, int);
  ~BFS_3D();

  void getDimensions(int*, int*, int*);

  void setWall(int, int, int);
  bool isWall(int, int, int);

  void run(int, int, int);

  int getDistance(int, int, int);
};
}  // namespace sbpl_interface
