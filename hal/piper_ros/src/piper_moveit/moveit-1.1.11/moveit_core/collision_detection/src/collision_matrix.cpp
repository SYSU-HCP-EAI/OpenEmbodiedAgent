/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, E. Gil Jones */

#include <moveit/collision_detection/collision_matrix.h>
#include <functional>
#include <iomanip>

namespace collision_detection
{
AllowedCollisionMatrix::AllowedCollisionMatrix()
{
}

AllowedCollisionMatrix::AllowedCollisionMatrix(const std::vector<std::string>& names, bool allowed)
{
  for (std::size_t i = 0; i < names.size(); ++i)
    for (std::size_t j = i; j < names.size(); ++j)
      setEntry(names[i], names[j], allowed);
}

AllowedCollisionMatrix::AllowedCollisionMatrix(const srdf::Model& srdf)
{
  // load collision defaults
  for (const std::string& name : srdf.getNoDefaultCollisionLinks())
    setDefaultEntry(name, collision_detection::AllowedCollision::ALWAYS);
  // re-enable specific collision pairs
  for (auto const& collision : srdf.getEnabledCollisionPairs())
    setEntry(collision.link1_, collision.link2_, false);
  // *finally* disable selected collision pairs
  for (auto const& collision : srdf.getDisabledCollisionPairs())
    setEntry(collision.link1_, collision.link2_, true);
}

AllowedCollisionMatrix::AllowedCollisionMatrix(const moveit_msgs::AllowedCollisionMatrix& msg)
{
  if (msg.entry_names.size() != msg.entry_values.size() ||
      msg.default_entry_names.size() != msg.default_entry_values.size())
  {
    ROS_ERROR_NAMED("collision_detection", "The number of links does not match the number of entries "
                                           "in AllowedCollisionMatrix message");
    return;
  }
  for (std::size_t i = 0; i < msg.default_entry_names.size(); ++i)
    setDefaultEntry(msg.default_entry_names[i], msg.default_entry_values[i]);

  for (std::size_t i = 0; i < msg.entry_names.size(); ++i)
  {
    if (msg.entry_values[i].enabled.size() != msg.entry_names.size())
    {
      ROS_ERROR_NAMED("collision_detection",
                      "Number of entries is incorrect for link '%s' in AllowedCollisionMatrix message",
                      msg.entry_names[i].c_str());
      return;
    }
    for (std::size_t j = i + 1; j < msg.entry_values[i].enabled.size(); ++j)
    {
      AllowedCollision::Type allowed_default, allowed_entry;
      if (!getDefaultEntry(msg.entry_names[i], msg.entry_names[j], allowed_default))
        allowed_default = AllowedCollision::NEVER;
      allowed_entry = msg.entry_values[i].enabled[j] ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;

      if (allowed_entry != allowed_default)
        setEntry(msg.entry_names[i], msg.entry_names[j], allowed_entry);
    }
  }
}

bool AllowedCollisionMatrix::getEntry(const std::string& name1, const std::string& name2, DecideContactFn& fn) const
{
  auto it1 = allowed_contacts_.find(name1);
  if (it1 == allowed_contacts_.end())
    return false;
  auto it2 = it1->second.find(name2);
  if (it2 == it1->second.end())
    return false;
  fn = it2->second;
  return true;
}

bool AllowedCollisionMatrix::getEntry(const std::string& name1, const std::string& name2,
                                      AllowedCollision::Type& allowed_collision) const
{
  auto it1 = entries_.find(name1);
  if (it1 == entries_.end())
    return false;
  auto it2 = it1->second.find(name2);
  if (it2 == it1->second.end())
    return false;
  allowed_collision = it2->second;
  return true;
}

bool AllowedCollisionMatrix::hasEntry(const std::string& name) const
{
  return entries_.find(name) != entries_.end();
}

bool AllowedCollisionMatrix::hasEntry(const std::string& name1, const std::string& name2) const
{
  auto it1 = entries_.find(name1);
  if (it1 == entries_.end())
    return false;
  auto it2 = it1->second.find(name2);
  return it2 != it1->second.end();
}

void AllowedCollisionMatrix::setEntry(const std::string& name1, const std::string& name2, bool allowed)
{
  const AllowedCollision::Type v = allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;
  entries_[name1][name2] = entries_[name2][name1] = v;

  // remove boost::function pointers, if any
  auto it = allowed_contacts_.find(name1);
  if (it != allowed_contacts_.end())
  {
    auto jt = it->second.find(name2);
    if (jt != it->second.end())
      it->second.erase(jt);
  }
  it = allowed_contacts_.find(name2);
  if (it != allowed_contacts_.end())
  {
    auto jt = it->second.find(name1);
    if (jt != it->second.end())
      it->second.erase(jt);
  }
}

void AllowedCollisionMatrix::setEntry(const std::string& name1, const std::string& name2, const DecideContactFn& fn)
{
  entries_[name1][name2] = entries_[name2][name1] = AllowedCollision::CONDITIONAL;
  allowed_contacts_[name1][name2] = allowed_contacts_[name2][name1] = fn;
}

void AllowedCollisionMatrix::removeEntry(const std::string& name)
{
  entries_.erase(name);
  allowed_contacts_.erase(name);
  for (auto& entry : entries_)
    entry.second.erase(name);
  for (auto& allowed_contact : allowed_contacts_)
    allowed_contact.second.erase(name);
}

void AllowedCollisionMatrix::removeEntry(const std::string& name1, const std::string& name2)
{
  auto jt = entries_.find(name1);
  if (jt != entries_.end())
  {
    auto it = jt->second.find(name2);
    if (it != jt->second.end())
      jt->second.erase(it);
  }
  jt = entries_.find(name2);
  if (jt != entries_.end())
  {
    auto it = jt->second.find(name1);
    if (it != jt->second.end())
      jt->second.erase(it);
  }

  auto it = allowed_contacts_.find(name1);
  if (it != allowed_contacts_.end())
  {
    auto jt = it->second.find(name2);
    if (jt != it->second.end())
      it->second.erase(jt);
  }
  it = allowed_contacts_.find(name2);
  if (it != allowed_contacts_.end())
  {
    auto jt = it->second.find(name1);
    if (jt != it->second.end())
      it->second.erase(jt);
  }
}

void AllowedCollisionMatrix::setEntry(const std::string& name, const std::vector<std::string>& other_names, bool allowed)
{
  for (const auto& other_name : other_names)
    if (other_name != name)
      setEntry(other_name, name, allowed);
}

void AllowedCollisionMatrix::setEntry(const std::vector<std::string>& names1, const std::vector<std::string>& names2,
                                      bool allowed)
{
  for (const auto& name1 : names1)
    setEntry(name1, names2, allowed);
}

void AllowedCollisionMatrix::setEntry(const std::string& name, bool allowed)
{
  std::string last = name;
  for (auto& entry : entries_)
    if (name != entry.first && last != entry.first)
    {
      last = entry.first;
      setEntry(name, entry.first, allowed);
    }
}

void AllowedCollisionMatrix::setEntry(bool allowed)
{
  const AllowedCollision::Type v = allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;
  for (auto& entry : entries_)
    for (auto& it2 : entry.second)
      it2.second = v;
}

void AllowedCollisionMatrix::setDefaultEntry(const std::string& name, bool allowed)
{
  const AllowedCollision::Type v = allowed ? AllowedCollision::ALWAYS : AllowedCollision::NEVER;
  default_entries_[name] = v;
  default_allowed_contacts_.erase(name);
}

void AllowedCollisionMatrix::setDefaultEntry(const std::string& name, const DecideContactFn& fn)
{
  default_entries_[name] = AllowedCollision::CONDITIONAL;
  default_allowed_contacts_[name] = fn;
}

bool AllowedCollisionMatrix::getDefaultEntry(const std::string& name, AllowedCollision::Type& allowed_collision) const
{
  auto it = default_entries_.find(name);
  if (it == default_entries_.end())
    return false;
  allowed_collision = it->second;
  return true;
}

bool AllowedCollisionMatrix::getDefaultEntry(const std::string& name, DecideContactFn& fn) const
{
  auto it = default_allowed_contacts_.find(name);
  if (it == default_allowed_contacts_.end())
    return false;
  fn = it->second;
  return true;
}

static bool andDecideContact(const DecideContactFn& f1, const DecideContactFn& f2, Contact& contact)
{
  return f1(contact) && f2(contact);
}

bool AllowedCollisionMatrix::getAllowedCollision(const std::string& name1, const std::string& name2,
                                                 DecideContactFn& fn) const
{
  bool found = getEntry(name1, name2, fn);
  if (!found)
  {
    DecideContactFn fn1, fn2;
    bool found1 = getDefaultEntry(name1, fn1);
    bool found2 = getDefaultEntry(name2, fn2);
    if (found1 && !found2)
      fn = fn1;
    else if (!found1 && found2)
      fn = fn2;
    else if (found1 && found2)
      fn = [fn1, fn2](Contact& contact) { return andDecideContact(fn1, fn2, contact); };
    else
      return false;
  }
  return true;
}

bool AllowedCollisionMatrix::getDefaultEntry(const std::string& name1, const std::string& name2,
                                             AllowedCollision::Type& allowed_collision) const
{
  AllowedCollision::Type t1, t2;
  bool found1 = getDefaultEntry(name1, t1);
  bool found2 = getDefaultEntry(name2, t2);
  if (!found1 && !found2)
    return false;
  else if (found1 && !found2)
    allowed_collision = t1;
  else if (!found1 && found2)
    allowed_collision = t2;
  else if (found1 && found2)
  {
    if (t1 == AllowedCollision::NEVER || t2 == AllowedCollision::NEVER)
      allowed_collision = AllowedCollision::NEVER;
    else if (t1 == AllowedCollision::CONDITIONAL || t2 == AllowedCollision::CONDITIONAL)
      allowed_collision = AllowedCollision::CONDITIONAL;
    else  // ALWAYS is the only remaining case
      allowed_collision = AllowedCollision::ALWAYS;
  }
  return true;
}

bool AllowedCollisionMatrix::getAllowedCollision(const std::string& name1, const std::string& name2,
                                                 AllowedCollision::Type& allowed_collision) const
{
  return getEntry(name1, name2, allowed_collision) || getDefaultEntry(name1, name2, allowed_collision);
}

void AllowedCollisionMatrix::clear()
{
  entries_.clear();
  allowed_contacts_.clear();
  default_entries_.clear();
  default_allowed_contacts_.clear();
}

void AllowedCollisionMatrix::getAllEntryNames(std::vector<std::string>& names) const
{
  names.clear();
  for (const auto& entry : entries_)
    names.push_back(entry.first);

  for (const auto& item : default_entries_)
  {
    auto it = std::lower_bound(names.begin(), names.end(), item.first);
    if (it != names.end() && *it != item.first)
      names.insert(it, item.first);
  }
}

void AllowedCollisionMatrix::getMessage(moveit_msgs::AllowedCollisionMatrix& msg) const
{
  msg.entry_names.clear();
  msg.entry_values.clear();
  msg.default_entry_names.clear();
  msg.default_entry_values.clear();

  getAllEntryNames(msg.entry_names);

  msg.entry_values.resize(msg.entry_names.size());
  for (std::size_t i = 0; i < msg.entry_names.size(); ++i)
    msg.entry_values[i].enabled.resize(msg.entry_names.size(), false);

  // there is an approximation here: if we use a boost function to decide
  // whether a collision is allowed or not, we just assume the collision is not allowed.
  for (std::size_t i = 0; i < msg.entry_names.size(); ++i)
  {
    AllowedCollision::Type dtype;
    bool dfound = getDefaultEntry(msg.entry_names[i], dtype);
    if (dfound)
    {
      msg.default_entry_names.push_back(msg.entry_names[i]);
      msg.default_entry_values.push_back(dtype == AllowedCollision::ALWAYS);
    }

    for (std::size_t j = i; j < msg.entry_names.size(); ++j)
    {
      AllowedCollision::Type type = AllowedCollision::NEVER;
      getAllowedCollision(msg.entry_names[i], msg.entry_names[j], type);
      msg.entry_values[i].enabled[j] = msg.entry_values[j].enabled[i] = (type == AllowedCollision::ALWAYS);
    }
  }
}

void AllowedCollisionMatrix::print(std::ostream& out) const
{
  std::vector<std::string> names;
  getAllEntryNames(names);

  std::size_t spacing = 4;
  for (auto& name : names)
  {
    std::size_t length = name.length();
    if (length > spacing)
      spacing = length;
  }
  ++spacing;

  std::size_t number_digits = 2;
  while (names.size() > pow(10, number_digits) - 1)
    number_digits++;

  // print indices along the top of the matrix
  for (std::size_t j = 0; j < number_digits; ++j)
  {
    out << std::setw(spacing + number_digits + 8) << "";
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      std::stringstream ss;
      ss << std::setw(number_digits) << i;
      out << std::setw(3) << ss.str().c_str()[j];
    }
    out << std::endl;
  }

  const char* indicator = "01?";  // ALWAYS / NEVER / CONDITIONAL
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    out << std::setw(spacing) << names[i];
    out << std::setw(number_digits + 1) << i;
    out << " | ";
    // print default value
    AllowedCollision::Type type;
    if (getDefaultEntry(names[i], type))
      out << indicator[type];
    else
      out << '-';
    out << " | ";
    // print pairs
    for (std::size_t j = 0; j < names.size(); ++j)
    {
      bool found = getAllowedCollision(names[i], names[j], type);
      if (found)
        out << std::setw(3) << indicator[type];
      else
        out << std::setw(3) << '-';
    }
    out << std::endl;
  }
}

}  // end of namespace collision_detection
