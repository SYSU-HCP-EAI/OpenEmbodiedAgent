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

#include "pilz_industrial_motion_planner/velocity_profile_atrap.h"

namespace pilz_industrial_motion_planner
{
VelocityProfileATrap::VelocityProfileATrap(double max_vel, double max_acc, double max_dec)
  : max_vel_(fabs(max_vel))
  , max_acc_(fabs(max_acc))
  , max_dec_(fabs(max_dec))
  , start_pos_(0)
  , end_pos_(0)
  , start_vel_(0)
  , a1_(0)
  , a2_(0)
  , a3_(0)
  , b1_(0)
  , b2_(0)
  , b3_(0)
  , c1_(0)
  , c2_(0)
  , c3_(0)
  , t_a_(0)
  , t_b_(0)
  , t_c_(0)
{
}

void VelocityProfileATrap::SetProfile(double pos1, double pos2)
{
  start_pos_ = pos1;
  end_pos_ = pos2;
  start_vel_ = 0.0;

  if (start_pos_ == end_pos_)
  {
    // goal already reached, set everything to zero
    setEmptyProfile();
    return;
  }
  else
  {
    // get the sign
    double s = ((end_pos_ - start_pos_) > 0.0) - ((end_pos_ - start_pos_) < 0.0);

    double dis = fabs(end_pos_ - start_pos_);
    double min_dis_max_vel = 0.5 * max_vel_ * max_vel_ / max_acc_ + 0.5 * max_vel_ * max_vel_ / max_dec_;

    // max_vel can be reached
    if (dis > min_dis_max_vel)
    {
      // acceleration phase
      a1_ = start_pos_;
      a2_ = 0.0;
      a3_ = s * max_acc_ / 2.0;
      t_a_ = max_vel_ / max_acc_;

      // constant phase
      b1_ = a1_ + a3_ * t_a_ * t_a_;
      b2_ = s * max_vel_;
      b3_ = 0;
      t_b_ = (dis - min_dis_max_vel) / max_vel_;

      // deceleration phase
      c1_ = b1_ + b2_ * (t_b_);
      c2_ = s * max_vel_;
      c3_ = -s * max_dec_ / 2.0;
      t_c_ = max_vel_ / max_dec_;
    }
    // max_vel cannot be reached, no constant velocity phase
    else
    {
      // compute the new velocity of constant phase
      double new_vel = s * sqrt(2.0 * dis * max_acc_ * max_dec_ / (max_acc_ + max_dec_));

      // acceleration phase
      a1_ = start_pos_;
      a2_ = 0.0;
      a3_ = s * max_acc_ / 2.0;
      t_a_ = fabs(new_vel) / max_acc_;

      // constant phase
      b1_ = a1_ + a3_ * t_a_ * t_a_;
      b2_ = new_vel;
      b3_ = 0;
      t_b_ = 0.0;

      // deceleration phase
      c1_ = b1_;
      c2_ = new_vel;
      c3_ = -s * max_dec_ / 2.0;
      t_c_ = fabs(new_vel) / max_dec_;
    }
  }
}

void VelocityProfileATrap::SetProfileDuration(double pos1, double pos2, double duration)
{
  // compute the fastest case
  SetProfile(pos1, pos2);

  // cannot be faster
  if (Duration() > duration)
  {
    return;
  }

  double ratio = Duration() / duration;
  a2_ *= ratio;
  a3_ *= ratio * ratio;
  b2_ *= ratio;
  b3_ *= ratio * ratio;
  c2_ *= ratio;
  c3_ *= ratio * ratio;
  t_a_ /= ratio;
  t_b_ /= ratio;
  t_c_ /= ratio;
}

bool VelocityProfileATrap::setProfileAllDurations(double pos1, double pos2, double duration1, double duration2,
                                                  double duration3)
{
  // compute the fastest case
  SetProfile(pos1, pos2);

  assert(duration1 > 0);
  assert(duration3 > 0);

  // cannot be faster
  if (Duration() - (duration1 + duration2 + duration3) > KDL::epsilon)
  {
    return false;
  }

  // get the sign
  double s = ((end_pos_ - start_pos_) > 0.0) - ((end_pos_ - start_pos_) < 0.0);
  // compute the new velocity/acceleration/decel4eration
  double dis = fabs(end_pos_ - start_pos_);
  double new_vel = s * dis / (duration2 + duration1 / 2.0 + duration3 / 2.0);
  double new_acc = new_vel / duration1;
  double new_dec = -new_vel / duration3;
  if ((fabs(new_vel) - max_vel_ > KDL::epsilon) || (fabs(new_acc) - max_acc_ > KDL::epsilon) ||
      (fabs(new_dec) - max_dec_ > KDL::epsilon))
  {
    return false;
  }
  else
  {
    // set profile
    start_pos_ = pos1;
    end_pos_ = pos2;

    // acceleration phase
    a1_ = start_pos_;
    a2_ = 0.0;
    a3_ = new_acc / 2.0;
    t_a_ = duration1;

    // constant phase
    b1_ = a1_ + a3_ * t_a_ * t_a_;
    b2_ = new_vel;
    b3_ = 0;
    t_b_ = duration2;

    // deceleration phase
    c1_ = b1_ + b2_ * (t_b_);
    c2_ = new_vel;
    c3_ = new_dec / 2.0;
    t_c_ = duration3;

    return true;
  }
}

bool VelocityProfileATrap::setProfileStartVelocity(double pos1, double pos2, double vel1)
{
  if (vel1 == 0)
  {
    SetProfile(pos1, pos2);
    return true;
  }

  // get the sign
  double s = ((pos2 - pos1) > 0.0) - ((pos2 - pos1) < 0.0);

  if (s * vel1 <= 0)
  {
    // TODO initial velocity is in opposite derection of start-end vector
    return false;
  }

  start_pos_ = pos1;
  end_pos_ = pos2;
  start_vel_ = vel1;

  // minimum brake distance
  double min_brake_dis = 0.5 * vel1 * vel1 / max_dec_;
  // minimum distance to reach the maximum velocity
  double min_dis_max_vel =
      0.5 * (max_vel_ - start_vel_) * (max_vel_ + start_vel_) / max_acc_ + 0.5 * max_vel_ * max_vel_ / max_dec_;
  double dis = fabs(end_pos_ - start_pos_);

  // brake, acceleration in opposite direction, deceleration
  if (dis <= min_brake_dis)
  {
    // brake to zero velocity
    t_a_ = fabs(start_vel_ / max_dec_);
    a1_ = start_pos_;
    a2_ = start_vel_;
    a3_ = -0.5 * s * max_dec_;

    // compute the velocity in opposite direction
    double new_vel = -s * sqrt(2.0 * fabs(min_brake_dis - dis) * max_acc_ * max_dec_ / (max_acc_ + max_dec_));

    // acceleration in opposite direction
    t_b_ = fabs(new_vel / max_acc_);
    b1_ = a1_ + a2_ * t_a_ + a3_ * t_a_ * t_a_;
    b2_ = 0;
    b3_ = -s * 0.5 * max_acc_;

    // deceleration to zero
    t_c_ = fabs(new_vel / max_dec_);
    c1_ = b1_ + b2_ * t_b_ + b3_ * t_b_ * t_b_;
    c2_ = new_vel;
    c3_ = 0.5 * s * max_dec_;
  }
  else if (dis <= min_dis_max_vel)
  {
    // compute the reached velocity
    double new_vel =
        s * sqrt((dis + 0.5 * start_vel_ * start_vel_ / max_acc_) * 2.0 * max_acc_ * max_dec_ / (max_acc_ + max_dec_));

    // acceleration to new velocity
    t_a_ = fabs(new_vel - start_vel_) / max_acc_;
    a1_ = start_pos_;
    a2_ = start_vel_;
    a3_ = 0.5 * s * max_acc_;

    // no constant velocity phase
    t_b_ = 0;
    b1_ = a1_ + a2_ * t_a_ + a3_ * t_a_ * t_a_;
    b2_ = 0;
    b3_ = 0;

    // deceleration to zero velocity
    t_c_ = fabs(new_vel / max_dec_);
    c1_ = b1_;
    c2_ = new_vel;
    c3_ = -0.5 * s * max_dec_;
  }
  else
  {
    // acceleration to max velocity
    t_a_ = fabs(max_vel_ - start_vel_) / max_acc_;
    a1_ = start_pos_;
    a2_ = start_vel_;
    a3_ = 0.5 * s * max_acc_;

    // constant velocity
    t_b_ = (dis - min_dis_max_vel) / max_vel_;
    b1_ = a1_ + a2_ * t_a_ + a3_ * t_a_ * t_a_;
    b2_ = max_vel_;
    b3_ = 0;

    // deceleration to zero velocity
    t_c_ = max_vel_ / max_dec_;
    c1_ = b1_ + b2_ * t_b_ + b3_ * t_b_ * t_b_;
    c2_ = max_vel_;
    c3_ = -0.5 * s * max_dec_;
  }

  return true;
}

double VelocityProfileATrap::Duration() const
{
  return t_a_ + t_b_ + t_c_;
}

double VelocityProfileATrap::Pos(double time) const
{
  if (time < 0)
  {
    return start_pos_;
  }
  else if (time < t_a_)
  {
    return a1_ + time * (a2_ + a3_ * time);
  }
  else if (time < (t_a_ + t_b_))
  {
    return b1_ + (time - t_a_) * (b2_ + b3_ * (time - t_a_));
  }
  else if (time <= (t_a_ + t_b_ + t_c_))
  {
    return c1_ + (time - t_a_ - t_b_) * (c2_ + c3_ * (time - t_a_ - t_b_));
  }
  else
  {
    return end_pos_;
  }
}

double VelocityProfileATrap::Vel(double time) const
{
  if (time < 0)
  {
    return start_vel_;
  }
  else if (time < t_a_)
  {
    return a2_ + 2 * a3_ * time;
  }
  else if (time < (t_a_ + t_b_))
  {
    return b2_ + 2 * b3_ * (time - t_a_);
  }
  else if (time <= (t_a_ + t_b_ + t_c_))
  {
    return c2_ + 2 * c3_ * (time - t_a_ - t_b_);
  }
  else
  {
    return 0;
  }
}

double VelocityProfileATrap::Acc(double time) const
{
  if (time <= 0)
  {
    return 0;
  }
  else if (time <= t_a_)
  {
    return 2 * a3_;
  }
  else if (time <= (t_a_ + t_b_))
  {
    return 2 * b3_;
  }
  else if (time <= (t_a_ + t_b_ + t_c_))
  {
    return 2 * c3_;
  }
  else
  {
    return 0;
  }
}

KDL::VelocityProfile* VelocityProfileATrap::Clone() const
{
  VelocityProfileATrap* trap = new VelocityProfileATrap(max_vel_, max_acc_, max_dec_);
  trap->setProfileAllDurations(this->start_pos_, this->end_pos_, this->t_a_, this->t_b_, this->t_c_);
  return trap;
}

// LCOV_EXCL_START // No tests for the print function
void VelocityProfileATrap::Write(std::ostream& os) const
{
  os << *this;
}

std::ostream& operator<<(std::ostream& os, const VelocityProfileATrap& p)
{
  os << "Asymmetric Trapezoid " << std::endl
     << "maximal velocity: " << p.max_vel_ << std::endl
     << "maximal acceleration: " << p.max_acc_ << std::endl
     << "maximal deceleration: " << p.max_dec_ << std::endl
     << "start position: " << p.start_pos_ << std::endl
     << "end position: " << p.end_pos_ << std::endl
     << "start velocity: " << p.start_vel_ << std::endl
     << "a1: " << p.a1_ << std::endl
     << "a2: " << p.a2_ << std::endl
     << "a3: " << p.a3_ << std::endl
     << "b1: " << p.b1_ << std::endl
     << "b2: " << p.b2_ << std::endl
     << "b3: " << p.b3_ << std::endl
     << "c1: " << p.c1_ << std::endl
     << "c2: " << p.c2_ << std::endl
     << "c3: " << p.c3_ << std::endl
     << "firstPhaseDuration " << p.firstPhaseDuration() << std::endl
     << "secondPhaseDuration " << p.secondPhaseDuration() << std::endl
     << "thirdPhaseDuration " << p.thirdPhaseDuration() << std::endl;
  return os;
}
// LCOV_EXCL_STOP

bool VelocityProfileATrap::operator==(const VelocityProfileATrap& other) const
{
  return (max_vel_ == other.max_vel_ && max_acc_ == other.max_acc_ && max_dec_ == other.max_dec_ &&
          start_pos_ == other.start_pos_ && end_pos_ == other.end_pos_ && start_vel_ == other.start_vel_ &&
          a1_ == other.a1_ && a2_ == other.a2_ && a3_ == other.a3_ && b1_ == other.b1_ && b2_ == other.b2_ &&
          b3_ == other.b3_ && c1_ == other.c1_ && c2_ == other.c2_ && c3_ == other.c3_ && t_a_ == other.t_a_ &&
          t_b_ == other.t_b_ && t_c_ == other.t_c_);
}

VelocityProfileATrap::~VelocityProfileATrap()
{
}

void VelocityProfileATrap::setEmptyProfile()
{
  a1_ = end_pos_;
  a2_ = 0;
  a3_ = 0;
  b1_ = end_pos_;
  b2_ = 0;
  c1_ = end_pos_;
  c2_ = 0;
  c3_ = 0;

  t_a_ = 0;
  t_b_ = 0;
  t_c_ = 0;
}

}  // namespace pilz_industrial_motion_planner
