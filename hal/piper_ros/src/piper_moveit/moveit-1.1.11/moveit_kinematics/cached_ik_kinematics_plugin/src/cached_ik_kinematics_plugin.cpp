/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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

#include <moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_plugin.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
// compilation error: KDL and LMA kinematics plugins declare same types
//#include <moveit/lma_kinematics_plugin/lma_kinematics_plugin.h>
#include <moveit/srv_kinematics_plugin/srv_kinematics_plugin.h>

#include <pluginlib/class_list_macros.hpp>
// register CachedIKKinematicsPlugin<KDLKinematicsPlugin> as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<kdl_kinematics_plugin::KDLKinematicsPlugin>,
                       kinematics::KinematicsBase);

// register CachedIKKinematicsPlugin<SrvKinematicsPlugin> as a KinematicsBase implementation
// PLUGINLIB_EXPORT_CLASS(cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<lma_kinematics_plugin::LMAKinematicsPlugin>,
// kinematics::KinematicsBase);

// register CachedIKKinematicsPlugin<SrvKinematicsPlugin> as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<srv_kinematics_plugin::SrvKinematicsPlugin>,
                       kinematics::KinematicsBase);

#ifdef CACHED_IK_KINEMATICS_TRAC_IK
#include <trac_ik/trac_ik_kinematics_plugin.hpp>
// register CachedIKKinematicsPlugin<TRAC_IKKinematicsPlugin> as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(
    cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin>,
    kinematics::KinematicsBase);
#endif
