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

#pragma once

#include <moveit/collision_detection/collision_common.h>
#include <moveit/macros/class_forward.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <boost/function.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace collision_detection
{
/** \brief Any pair of bodies can have a collision state associated to it */
namespace AllowedCollision
{
enum Type
{
  /** \brief Collisions between the pair of bodies is never ok, i.e., if two bodies are in contact in
      a particular configuration of the robot, that configuration is considered to be in collision*/
  NEVER,

  /** \brief Collisions between a particular pair of bodies does not imply that the robot configuration is in
      collision. There is no need to explicitly do a computation (to check for contacts) on this pair of bodies*/
  ALWAYS,

  /** \brief The collision is allowed depending on a predicate evaluated on the produced contact. If the
      predicate returns true, this particular contact is deemed ok (or allowed), i.e., the contact does not
      imply that the two bodies are in collision*/
  CONDITIONAL
};
}  // namespace AllowedCollision

/** \brief Signature of predicate that decides whether a contact is allowed or not (when AllowedCollision::Type is
 * CONDITIONAL) */
using DecideContactFn = boost::function<bool(collision_detection::Contact&)>;

MOVEIT_CLASS_FORWARD(AllowedCollisionMatrix);  // Defines AllowedCollisionMatrixPtr, ConstPtr, WeakPtr... etc

/** @class AllowedCollisionMatrix
 *  @brief Definition of a structure for the allowed collision matrix. All elements in the collision world are referred
 * to by their names.
 *   This class represents which collisions are allowed to happen and which are not. */
class AllowedCollisionMatrix
{
public:
  AllowedCollisionMatrix();

  /** @brief Instantiate using a vector of names (corresponding to all the elements in the collision world).
   *  @param names a vector of names (corresponding to object IDs in the collision world).
   *  @param allowed If false, indicates that collisions between all elements must be checked for and no collisions
   *  will be ignored. */
  AllowedCollisionMatrix(const std::vector<std::string>& names, bool allowed = false);

  /** @brief Construct from an SRDF representation */
  AllowedCollisionMatrix(const srdf::Model& srdf);

  /** @brief Construct the structure from a message representation */
  AllowedCollisionMatrix(const moveit_msgs::AllowedCollisionMatrix& msg);

  /** @brief Copy constructor */
  AllowedCollisionMatrix(const AllowedCollisionMatrix& acm) = default;

  /** @brief Get the type of the allowed collision between two elements.
   *  Return true if the entry is included in the collision matrix. Return false if the entry is not found.
   *  @param name1 name of first element
   *  @param name2 name of second element
   *  @param allowed_collision_type The allowed collision type will be filled here */
  bool getEntry(const std::string& name1, const std::string& name2,
                AllowedCollision::Type& allowed_collision_type) const;

  /** @brief Get the allowed collision predicate between two elements.
   *  Return true if a predicate for this entry is available in the collision matrix.
   *  Return false if the entry is not found.
   *  @param name1 name of first element
   *  @param name2 name of second element
   *  @param fn A callback function that is used to decide if collisions are allowed between the two elements is filled
   * here */
  bool getEntry(const std::string& name1, const std::string& name2, DecideContactFn& fn) const;

  /** @brief Check if the allowed collision matrix has an entry at all for an element.
   *  Returns true if the element is included.
   *  @param name name of the element */
  bool hasEntry(const std::string& name) const;

  /** @brief Check if the allowed collision matrix has an entry for a pair of elements.
   *  Returns true if the pair is included.
   *  @param name1 name of first element
   *  @param name2 name of second element*/
  bool hasEntry(const std::string& name1, const std::string& name2) const;

  /** @brief Remove an entry corresponding to a pair of elements.
   *  Nothing happens if the pair does not exist in the collision matrix.
   *  @param name1 name of first element
   *  @param name2 name of second element*/
  void removeEntry(const std::string& name1, const std::string& name2);

  /** @brief Remove all entries corresponding to a name (all pairs that include this name)
   *  @param name namespace*/
  void removeEntry(const std::string& name);

  /** @brief Set an entry corresponding to a pair of elements
   *  @param name1 name of first element
   *  @param name2 name of second element
   *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions
   *  will be ignored (AllowedCollision::NEVER). If true, indicates that collisions between two elements are ok and an
   *  explicit collision computation is not necessary (AllowedCollision::ALWAYS).*/
  void setEntry(const std::string& name1, const std::string& name2, bool allowed);

  /** @brief Set an entry corresponding to a pair of elements.
   *
   *  This sets the type of the entry to AllowedCollision::CONDITIONAL.
   *  @param name1 name of first element
   *  @param name2 name of second element
   *  @param fn A callback function that is used to decide if collisions are allowed between the two elements */
  void setEntry(const std::string& name1, const std::string& name2, const DecideContactFn& fn);

  /** @brief Set the entries corresponding to a name.
   *  With each of the *known names* in the collision matrix, form a pair using the name
   *  specified as argument to this function and set the entry as indicated by \e allowed.
   *  As the set of known names might change in future, consider using setDefaultEntry() instead!
   *  @param name the object name
   *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions
   *  will be ignored (AllowedCollision::NEVER). If true, indicates that collisions between two elements are ok and an
   *  explicit collision computation is not necessary (AllowedCollision::ALWAYS).*/
  void setEntry(const std::string& name, bool allowed);

  /** @brief Set multiple entries. Pairs of names are formed using \e name and \e other_names
   *  @param name name of first element
   *  @param other_names names of all other elements to pair with first element. The collision
   *  matrix entries will be set for all such pairs.
   *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions
   *  will be ignored (AllowedCollision::NEVER). If true, indicates that collisions between two elements are ok and an
   *  explicit collision computation is not necessary (AllowedCollision::ALWAYS).*/
  void setEntry(const std::string& name, const std::vector<std::string>& other_names, bool allowed);

  /** @brief Set an entry corresponding to all possible pairs between two sets of elements
   *  @param names1 First set of names
   *  @param names2 Second set of names
   *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions
   *  will be ignored (AllowedCollision::NEVER). If true, indicates that collisions between two elements are ok and an
   *  explicit collision computation is not necessary (AllowedCollision::ALWAYS).*/
  void setEntry(const std::vector<std::string>& names1, const std::vector<std::string>& names2, bool allowed);

  /** @brief Set an entry corresponding to all known pairs
   *  @param allowed If false, indicates that collisions between two elements must be checked for and no collisions
   *  will be ignored (AllowedCollision::NEVER). If true, indicates that collisions between two elements are ok and an
   *  explicit collision computation is not necessary (AllowedCollision::ALWAYS).*/
  void setEntry(bool allowed);

  /** @brief Get all the names known to the collision matrix */
  void getAllEntryNames(std::vector<std::string>& names) const;

  /** @brief Get the allowed collision matrix as a message */
  void getMessage(moveit_msgs::AllowedCollisionMatrix& msg) const;

  /** @brief Clear the allowed collision matrix */
  void clear();

  /** @brief Get the size of the allowed collision matrix (number of specified entries) */
  std::size_t getSize() const
  {
    return entries_.size();
  }

  /** @brief Set the default value for entries that include \e name but are not set explicity with setEntry().
   *  @param name The name of the element for which to set the default value
   *  @param allowed If false, indicates that collisions between \e name and any other element must be checked for and
   *  no collisions will be ignored (AllowedCollision::NEVER). If true, indicates that collisions between \e name and
   *  any other element are ok and an explicit collision computation is not necessary (AllowedCollision::ALWAYS).*/
  void setDefaultEntry(const std::string& name, bool allowed);

  /** @brief Set the default value for entries that include \e name but are not set explicity with setEntry().
   *  @param name The name of the element for which to set the default value
   *  @param fn A callback function that is used to decide if collisions are allowed between \e name and some other
   *  element is expected here. */
  void setDefaultEntry(const std::string& name, const DecideContactFn& fn);

  /** @brief Get the type of the allowed collision to be considered by default for an element.
   *  Return true if a default value was found for the specified element. Return false otherwise.
   *  @param name name of the element
   *  @param allowed_collision The default allowed collision type will be filled here */
  bool getDefaultEntry(const std::string& name, AllowedCollision::Type& allowed_collision) const;

  /** @brief Get the type of the allowed collision between to be considered by default for an element.
   *  Return true if a default value was found for the specified element. Return false otherwise.
   *  @param name name of the element
   *  @param fn Return the callback function that is used to decide if collisions are allowed between the two elements. */
  bool getDefaultEntry(const std::string& name, DecideContactFn& fn) const;

  /** @brief Get the allowed collision predicate between two elements.
   *  Return true if a predicate for entry is included in the collision matrix (if the type is AllowedCollision::CONDITIONAL)
   *  or if one was computed from defaults. Return false if the entry is not found.
   *  @param name1 name of first element
   *  @param name2 name of second element
   *  @param fn Return the callback function that is used to decide if collisions are allowed between the two elements. */
  bool getAllowedCollision(const std::string& name1, const std::string& name2, DecideContactFn& fn) const;

  /** @brief Get the type of the allowed collision between two elements.
   *  Return true if the entry is included in the collision matrix or if specified defaults were found.
   *  Return false if the entry is not found.
   *  @param name1 name of first element
   *  @param name2 name of second element
   *  @param allowed_collision The allowed collision type will be filled here */
  bool getAllowedCollision(const std::string& name1, const std::string& name2,
                           AllowedCollision::Type& allowed_collision) const;

  /** @brief Print the allowed collision matrix */
  void print(std::ostream& out) const;

private:
  bool getDefaultEntry(const std::string& name1, const std::string& name2,
                       AllowedCollision::Type& allowed_collision) const;

  std::map<std::string, std::map<std::string, AllowedCollision::Type> > entries_;
  std::map<std::string, std::map<std::string, DecideContactFn> > allowed_contacts_;

  std::map<std::string, AllowedCollision::Type> default_entries_;
  std::map<std::string, DecideContactFn> default_allowed_contacts_;
};
}  // namespace collision_detection
