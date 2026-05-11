/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University, Inc.
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
 *   * Neither the name of Bielefeld University nor the names of its
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

/* Author: Robert Haschke */

#include <moveit/setup_assistant/tools/xml_manipulation.h>

namespace moveit_setup_assistant
{
namespace
{
bool hasRequiredAttributes(const TiXmlElement& e, const std::vector<Attribute>& attributes)
{
  for (const auto& attr : attributes)
  {
    if (!attr.required)
      continue;  // attribute not required
    const char* value = e.Attribute(attr.name);
    if (value && strcmp(attr.value, value) == 0)
      continue;  // attribute has required value
    else
      return false;
  }
  return true;
};
}  // namespace

TiXmlElement* uniqueInsert(TiXmlElement& element, const char* tag, const std::vector<Attribute>& attributes,
                           const char* text)
{
  // search for existing element with required tag name and attributes
  TiXmlElement* result = element.FirstChildElement(tag);
  while (result && !hasRequiredAttributes(*result, attributes))
    result = result->NextSiblingElement(tag);

  if (!result)  // if not yet present, create new element
    result = element.InsertEndChild(TiXmlElement(tag))->ToElement();

  // set (not-yet existing) attributes
  for (const auto& attr : attributes)
  {
    if (!result->Attribute(attr.name))
      result->SetAttribute(attr.name, attr.value);
  }

  // insert text if required
  if (text && !result->GetText())
    result->InsertEndChild(TiXmlText(text));

  return result;
}

}  // namespace moveit_setup_assistant
