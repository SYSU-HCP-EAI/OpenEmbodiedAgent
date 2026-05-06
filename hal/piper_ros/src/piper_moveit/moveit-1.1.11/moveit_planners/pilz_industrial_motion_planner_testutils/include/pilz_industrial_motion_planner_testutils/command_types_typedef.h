/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef COMMAND_TYPES_TYPEDEF_H
#define COMMAND_TYPES_TYPEDEF_H

#include <boost/variant.hpp>

#include "ptp.h"
#include "lin.h"
#include "circ.h"
#include "gripper.h"
#include "jointconfiguration.h"
#include "cartesianconfiguration.h"
#include "circ_auxiliary_types.h"

namespace pilz_industrial_motion_planner_testutils
{
typedef Ptp<JointConfiguration, JointConfiguration> PtpJoint;
typedef Ptp<JointConfiguration, CartesianConfiguration> PtpJointCart;
typedef Ptp<CartesianConfiguration, CartesianConfiguration> PtpCart;

typedef Lin<JointConfiguration, JointConfiguration> LinJoint;
typedef Lin<JointConfiguration, CartesianConfiguration> LinJointCart;
typedef Lin<CartesianConfiguration, CartesianConfiguration> LinCart;

typedef Circ<CartesianConfiguration, CartesianCenter, CartesianConfiguration> CircCenterCart;
typedef Circ<CartesianConfiguration, CartesianInterim, CartesianConfiguration> CircInterimCart;

typedef Circ<JointConfiguration, CartesianCenter, JointConfiguration> CircJointCenterCart;
typedef Circ<JointConfiguration, CartesianInterim, JointConfiguration> CircJointInterimCart;

typedef boost::variant<PtpJoint, PtpJointCart, PtpCart, LinJoint, LinCart, CircCenterCart, CircInterimCart,
                       CircJointCenterCart, CircJointInterimCart, Gripper>
    CmdVariant;
}  // namespace pilz_industrial_motion_planner_testutils

#endif  // COMMAND_TYPES_TYPEDEF_H
