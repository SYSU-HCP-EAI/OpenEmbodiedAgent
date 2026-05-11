/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Delft Robotics B.V.
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
 *   * Neither the name of the copyright holder nor the names of its
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

#include <memory>

/**
 * \def MOVEIT_DELCARE_PTR
 * Macro that given a Name and a Type declares the following types:
 * - ${Name}Ptr            = shared_ptr<${Type}>
 * - ${Name}ConstPtr       = shared_ptr<const ${Type}>
 * - ${Name}WeakPtr        = weak_ptr<${Type}>
 * - ${Name}ConstWeakPtr   = weak_ptr<const ${Type}>
 * - ${Name}UniquePtr      = unique_ptr<${Type}>
 * - ${Name}ConstUniquePtr = unique_ptr<const ${Type}>
 *
 * For best portability the exact type of shared_ptr declared by the macro
 * should be considered to be an implementation detail, liable to change in
 * future releases.
 */

#define MOVEIT_DECLARE_PTR(Name, Type)                                                                                 \
  typedef std::shared_ptr<Type> Name##Ptr;                                                                             \
  typedef std::shared_ptr<const Type> Name##ConstPtr;                                                                  \
  typedef std::weak_ptr<Type> Name##WeakPtr;                                                                           \
  typedef std::weak_ptr<const Type> Name##ConstWeakPtr;                                                                \
  typedef std::unique_ptr<Type> Name##UniquePtr;                                                                       \
  typedef std::unique_ptr<const Type> Name##ConstUniquePtr

/**
 * \def MOVEIT_DELCARE_PTR_MEMBER
 * The macro defines the same typedefs as MOVEIT_DECLARE_PTR, but shortens the new names to their suffix.
 *
 * This can be used to create `Classname::Ptr` style names, but in most situations in MoveIt's codebase,
 * MOVEIT_CLASS_FORWARD and MOVEIT_DECLARE_PTR should be preferred.
 */

#define MOVEIT_DECLARE_PTR_MEMBER(Type)                                                                                \
  typedef std::shared_ptr<Type> Ptr;                                                                                   \
  typedef std::shared_ptr<const Type> ConstPtr;                                                                        \
  typedef std::weak_ptr<Type> WeakPtr;                                                                                 \
  typedef std::weak_ptr<const Type> ConstWeakPtr;                                                                      \
  typedef std::unique_ptr<Type> UniquePtr;                                                                             \
  typedef std::unique_ptr<const Type> ConstUniquePtr
