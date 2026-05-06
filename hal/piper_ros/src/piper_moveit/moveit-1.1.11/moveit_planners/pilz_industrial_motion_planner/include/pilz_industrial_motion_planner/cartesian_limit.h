/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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

#pragma once

namespace pilz_industrial_motion_planner
{
/**
 * @brief Set of cartesian limits, has values for velocity, acceleration and
 * deceleration of both the
 *        translational and rotational part.
 */
class CartesianLimit
{
public:
  CartesianLimit();

  // Translational Velocity Limit

  /**
   * @brief Check if translational velocity limit is set.
   * @return True if limit was set, false otherwise
   */
  bool hasMaxTranslationalVelocity() const;

  /**
   * @brief Set the maximal translational velocity
   * @param Maximum translational velocity [m/s]
   */
  void setMaxTranslationalVelocity(double max_trans_vel);

  /**
   * @brief Return the maximal translational velocity [m/s], 0 if nothing was
   * set
   * @return Maximum translational velocity, 0 if nothing was set
   */
  double getMaxTranslationalVelocity() const;

  // Translational Acceleration Limit

  /**
   * @brief Check if translational acceleration limit is set.
   * @return True if limit was set false otherwise
   */
  bool hasMaxTranslationalAcceleration() const;

  /**
   * @brief Set the maximum translational acceleration
   * @param Maximum translational acceleration [m/s^2]
   */
  void setMaxTranslationalAcceleration(double max_trans_acc);

  /**
   * @brief Return the maximal translational acceleration [m/s^2], 0 if nothing
   * was set
   * @return maximal translational acceleration, 0 if nothing was set
   */
  double getMaxTranslationalAcceleration() const;

  // Translational Deceleration Limit

  /**
   * @brief Check if translational deceleration limit is set.
   * @return True if limit was set false otherwise
   */
  bool hasMaxTranslationalDeceleration() const;

  /**
   * @brief Set the maximum translational deceleration
   * @param Maximum translational deceleration, always <=0 [m/s^2]
   */
  void setMaxTranslationalDeceleration(double max_trans_dec);

  /**
   * @brief Return the maximal translational deceleration [m/s^2], 0 if nothing
   * was set
   * @return maximal translational deceleration, 0 if nothing was set, always
   * <=0 [m/s^2]
   */
  double getMaxTranslationalDeceleration() const;

  // Rotational Velocity Limit

  /**
   * @brief Check if rotational velocity limit is set.
   * @return True if limit was set false otherwise
   */
  bool hasMaxRotationalVelocity() const;

  /**
   * @brief Set the maximum rotational velocity
   * @param Maximum rotational velocity [rad/s]
   */
  void setMaxRotationalVelocity(double max_rot_vel);

  /**
   * @brief Return the maximal rotational velocity [rad/s], 0 if nothing was set
   * @return maximal rotational velocity, 0 if nothing was set
   */
  double getMaxRotationalVelocity() const;

private:
  ///    Flag if a maximum translational velocity was set
  bool has_max_trans_vel_;

  ///    Maximum translational velocity [m/s]
  double max_trans_vel_;

  ///    Flag if a maximum translational acceleration was set
  bool has_max_trans_acc_;

  ///    Maximum translational acceleration [m/s^2]
  double max_trans_acc_;

  ///    Flag if a maximum translational deceleration was set
  bool has_max_trans_dec_;

  ///    Maximum translational deceleration, always <=0 [m/s^2]
  double max_trans_dec_;

  ///    Flag if a maximum rotational velocity was set
  bool has_max_rot_vel_;

  ///    Maximum rotational velocity [rad/s]
  double max_rot_vel_;
};

}  // namespace pilz_industrial_motion_planner
