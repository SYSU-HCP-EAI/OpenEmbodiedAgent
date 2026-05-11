/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University.
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

/* Author: Benjamin Scholz */

#pragma once

#include <fcl/config.h>

#define FCL_VERSION_CHECK(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#define MOVEIT_FCL_VERSION FCL_VERSION_CHECK(FCL_MAJOR_VERSION, FCL_MINOR_VERSION, FCL_PATCH_VERSION)

#if (MOVEIT_FCL_VERSION < FCL_VERSION_CHECK(0, 6, 0))
namespace fcl
{
class CollisionGeometry;
using CollisionGeometryd = fcl::CollisionGeometry;
class CollisionObject;
using CollisionObjectd = fcl::CollisionObject;
class BroadPhaseCollisionManager;
using BroadPhaseCollisionManagerd = fcl::BroadPhaseCollisionManager;
class Transform3f;
using Transform3d = fcl::Transform3f;
class Contact;
using Contactd = fcl::Contact;
class CostSource;
using CostSourced = fcl::CostSource;
class CollisionRequest;
using CollisionRequestd = fcl::CollisionRequest;
class CollisionResult;
using CollisionResultd = fcl::CollisionResult;
class DistanceRequest;
using DistanceRequestd = fcl::DistanceRequest;
class DistanceResult;
using DistanceResultd = fcl::DistanceResult;
class Plane;
using Planed = fcl::Plane;
class Sphere;
using Sphered = fcl::Sphere;
class Box;
using Boxd = fcl::Box;
class Cylinder;
using Cylinderd = fcl::Cylinder;
class Cone;
using Coned = fcl::Cone;

namespace details
{
struct sse_meta_f4;
template <typename T>
struct Vec3Data;
}  // namespace details
template <typename T>
class Vec3fX;
#if FCL_HAVE_SSE
using Vector3d = Vec3fX<details::sse_meta_f4>;
#else
using Vector3d = Vec3fX<details::Vec3Data<double> >;
#endif

class OcTree;
using OcTreed = fcl::OcTree;
class OBBRSS;
using OBBRSSd = fcl::OBBRSS;
class DynamicAABBTreeCollisionManager;
using DynamicAABBTreeCollisionManagerd = fcl::DynamicAABBTreeCollisionManager;
}  // namespace fcl
#endif
