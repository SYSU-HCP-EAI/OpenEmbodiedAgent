/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, isys vision, GmbH.
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

/* Author: Simon Schmeisser */

#include "moveit/utils/lexical_casts.h"

#include <locale>
#include <sstream>

namespace moveit
{
namespace core
{
template <class InType>
std::string toStringImpl(InType t)
{
  // convert to string using no locale
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << t;
  return oss.str();
}

std::string toString(double d)
{
  return toStringImpl(d);
}

std::string toString(float f)
{
  return toStringImpl(f);
}

template <class OutType>
OutType toRealImpl(const std::string& s)
{
  // convert from string using no locale
  std::istringstream stream(s);
  stream.imbue(std::locale::classic());
  OutType result;
  stream >> result;
  if (stream.fail() || !stream.eof())
  {
    throw std::runtime_error("Failed converting string to real number");
  }
  return result;
}

double toDouble(const std::string& s)
{
  return toRealImpl<double>(s);
}

float toFloat(const std::string& s)
{
  return toRealImpl<float>(s);
}
}  // namespace core
}  // namespace moveit
