/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University
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
 *   * Neither the name of Hamburg University nor the names of its
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

/* Author: Michael Görner, Robert Haschke */

#pragma once
#include <string>
#include <xmlrpcpp/XmlRpc.h>

namespace moveit
{
namespace core
{
/// parse a double value from a scalar XmlRpc
double parseDouble(XmlRpc::XmlRpcValue& v);

/** check that v is an array of given size
 *
 * @param size: check that array has given size, zero value allows for array size
 * @param name: if non-empty, print a warning message "name is not an array[size]"
 * @param description: if non-empty, serves as a descriptor for array items
 */
bool isArray(XmlRpc::XmlRpcValue& v, size_t size = 0, const std::string& name = "", const std::string& description = "");

/** check that v is a struct with given keys
 *
 * @param keys: list of required keys
 * @param name: if non-empty, print a warning message "name is not a struct with keys ..."
 */
bool isStruct(XmlRpc::XmlRpcValue& v, const std::vector<std::string>& keys = {}, const std::string& name = "");
}  // namespace core
}  // namespace moveit
