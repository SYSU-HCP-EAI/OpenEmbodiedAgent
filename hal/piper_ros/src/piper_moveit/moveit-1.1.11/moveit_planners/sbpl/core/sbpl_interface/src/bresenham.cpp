/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Maxim Likhachev
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

/* Author: Benjamin Cohen */

#include <sbpl_interface/bresenham.h>

void get_bresenham3d_parameters(int p1x, int p1y, int p1z, int p2x, int p2y, int p2z, bresenham3d_param_t* params)
{
  params->X1 = p1x;
  params->Y1 = p1y;
  params->Z1 = p1z;
  params->X2 = p2x;
  params->Y2 = p2y;
  params->Z2 = p2z;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
  params->ZIndex = params->Z1;

  params->dx = fabs((double)(p2x - p1x));
  params->dy = fabs((double)(p2y - p1y));
  params->dz = fabs((double)(p2z - p1z));
  params->dx2 = params->dx << 1;
  params->dy2 = params->dy << 1;
  params->dz2 = params->dz << 1;

  // get direction of slope
  if ((double)(p2x - p1x) < 0)
    params->IncX = -1;
  else
    params->IncX = 1;

  if ((double)(p2y - p1y) < 0)
    params->IncY = -1;
  else
    params->IncY = 1;

  if ((double)(p2z - p1z) < 0)
    params->IncZ = -1;
  else
    params->IncZ = 1;

  // choose which axis to use as the index
  if (params->dx >= params->dy && params->dx >= params->dz)
  {
    params->UsingXYZIndex = 0;
    params->err1 = params->dy2 - params->dx;
    params->err2 = params->dz2 - params->dx;
  }
  else if (params->dy >= params->dx && params->dy >= params->dz)
  {
    params->UsingXYZIndex = 1;
    params->err1 = params->dx2 - params->dy;
    params->err2 = params->dz2 - params->dy;
  }
  else
  {
    params->UsingXYZIndex = 2;
    params->err1 = params->dy2 - params->dz;
    params->err2 = params->dx2 - params->dz;
  }
}

void get_current_point3d(bresenham3d_param_t* params, int* x, int* y, int* z)
{
  *x = params->XIndex;
  *y = params->YIndex;
  *z = params->ZIndex;
}

int get_next_point3d(bresenham3d_param_t* params)
{
  // check to see if at end of line
  if (params->XIndex == params->X2 && params->YIndex == params->Y2 && params->ZIndex == params->Z2)
    return 0;

  if (params->UsingXYZIndex == 0)
  {
    if (params->err1 > 0)
    {
      params->YIndex += params->IncY;
      params->err1 -= params->dx2;
    }
    if (params->err2 > 0)
    {
      params->ZIndex += params->IncZ;
      params->err2 -= params->dx2;
    }
    params->err1 += params->dy2;
    params->err2 += params->dz2;
    params->XIndex += params->IncX;
  }
  else if (params->UsingXYZIndex == 1)
  {
    if (params->err1 > 0)
    {
      params->XIndex += params->IncX;
      params->err1 -= params->dy2;
    }
    if (params->err2 > 0)
    {
      params->ZIndex += params->IncZ;
      params->err2 -= params->dy2;
    }
    params->err1 += params->dx2;
    params->err2 += params->dz2;
    params->YIndex += params->IncY;
  }
  else
  {
    if (params->err1 > 0)
    {
      params->YIndex += params->IncY;
      params->err1 -= params->dz2;
    }
    if (params->err2 > 0)
    {
      params->XIndex += params->IncX;
      params->err2 -= params->dz2;
    }
    params->err1 += params->dy2;
    params->err2 += params->dx2;
    params->ZIndex += params->IncZ;
  }
  return 1;
}
