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

/* Author: Ioan Sucan */

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <geometric_shapes/check_isometry.h>
#include <boost/math/constants/constants.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <functional>
#include <limits>
#include <memory>

namespace kinematic_constraints
{
static double normalizeAngle(double angle)
{
  double v = fmod(angle, 2.0 * boost::math::constants::pi<double>());
  if (v < -boost::math::constants::pi<double>())
    v += 2.0 * boost::math::constants::pi<double>();
  else if (v > boost::math::constants::pi<double>())
    v -= 2.0 * boost::math::constants::pi<double>();
  return v;
}

// Normalizes an angle to the interval [-pi, +pi] and then take the absolute value
// The returned values will be in the following range [0, +pi]
static double normalizeAbsoluteAngle(const double angle)
{
  const double normalized_angle = std::fmod(std::abs(angle), 2 * M_PI);
  return std::min(2 * M_PI - normalized_angle, normalized_angle);
}

/**
 * This's copied from
 * https://gitlab.com/libeigen/eigen/-/blob/master/unsupported/Eigen/src/EulerAngles/EulerSystem.h#L187
 * Return the intrinsic Roll-Pitch-Yaw euler angles given the input rotation matrix and boolean indicating whether the
 * there's a singularity in the input rotation matrix (true: the input rotation matrix don't have a singularity, false:
 * the input rotation matrix have a singularity) The returned angles are in the ranges [-pi:pi]x[-pi/2:pi/2]x[-pi:pi]
 */
template <typename Derived>
std::tuple<Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar, 3, 1>, bool>
CalcEulerAngles(const Eigen::MatrixBase<Derived>& R)
{
  using std::atan2;
  using std::sqrt;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3)
  using Index = EIGEN_DEFAULT_DENSE_INDEX_TYPE;
  using Scalar = typename Eigen::MatrixBase<Derived>::Scalar;
  const Index i = 0;
  const Index j = 1;
  const Index k = 2;
  Eigen::Matrix<Scalar, 3, 1> res;
  const Scalar rsum = sqrt((R(i, i) * R(i, i) + R(i, j) * R(i, j) + R(j, k) * R(j, k) + R(k, k) * R(k, k)) / 2);
  res[1] = atan2(R(i, k), rsum);
  // There is a singularity when cos(beta) == 0
  if (rsum > 4 * Eigen::NumTraits<Scalar>::epsilon())
  {  // cos(beta) != 0
    res[0] = atan2(-R(j, k), R(k, k));
    res[2] = atan2(-R(i, j), R(i, i));
    return { res, true };
  }
  else if (R(i, k) > 0)
  {                                         // cos(beta) == 0 and sin(beta) == 1
    const Scalar spos = R(j, i) + R(k, j);  // 2*sin(alpha + gamma)
    const Scalar cpos = R(j, j) - R(k, i);  // 2*cos(alpha + gamma)
    res[0] = atan2(spos, cpos);
    res[2] = 0;
    return { res, false };
  }                                       // cos(beta) == 0 and sin(beta) == -1
  const Scalar sneg = R(k, j) - R(j, i);  // 2*sin(alpha + gamma)
  const Scalar cneg = R(j, j) + R(k, i);  // 2*cos(alpha + gamma)
  res[0] = atan2(sneg, cneg);
  res[2] = 0;
  return { res, false };
}

KinematicConstraint::KinematicConstraint(const moveit::core::RobotModelConstPtr& model)
  : type_(UNKNOWN_CONSTRAINT), robot_model_(model), constraint_weight_(std::numeric_limits<double>::epsilon())
{
}

KinematicConstraint::~KinematicConstraint() = default;

bool JointConstraint::configure(const moveit_msgs::JointConstraint& jc)
{
  // clearing before we configure to get rid of any old data
  clear();

  // testing tolerances first
  if (jc.tolerance_above < 0.0 || jc.tolerance_below < 0.0)
  {
    ROS_WARN_NAMED("kinematic_constraints", "JointConstraint tolerance values must be positive.");
    joint_model_ = nullptr;
    return false;
  }

  joint_variable_name_ = jc.joint_name;
  local_variable_name_.clear();
  if (robot_model_->hasJointModel(joint_variable_name_))
    joint_model_ = robot_model_->getJointModel(joint_variable_name_);
  else
  {
    std::size_t pos = jc.joint_name.find_last_of('/');
    if (pos != std::string::npos)
    {
      joint_model_ = robot_model_->getJointModel(jc.joint_name.substr(0, pos));
      if (pos + 1 < jc.joint_name.length())
        local_variable_name_ = jc.joint_name.substr(pos + 1);
    }
    else
      joint_model_ = robot_model_->getJointModel(jc.joint_name);
  }

  if (joint_model_)
  {
    if (local_variable_name_.empty())
    {
      // check if the joint has 1 DOF (the only kind we can handle)
      if (joint_model_->getVariableCount() == 0)
      {
        ROS_ERROR_NAMED("kinematic_constraints", "Joint '%s' has no parameters to constrain", jc.joint_name.c_str());
        joint_model_ = nullptr;
      }
      else if (joint_model_->getVariableCount() > 1)
      {
        ROS_ERROR_NAMED("kinematic_constraints",
                        "Joint '%s' has more than one parameter to constrain. "
                        "This type of constraint is not supported.",
                        jc.joint_name.c_str());
        joint_model_ = nullptr;
      }
    }
    else
    {
      int found = -1;
      const std::vector<std::string>& local_var_names = joint_model_->getLocalVariableNames();
      for (std::size_t i = 0; i < local_var_names.size(); ++i)
        if (local_var_names[i] == local_variable_name_)
        {
          found = i;
          break;
        }
      if (found < 0)
      {
        ROS_ERROR_NAMED("kinematic_constraints", "Local variable name '%s' is not known to joint '%s'",
                        local_variable_name_.c_str(), joint_model_->getName().c_str());
        joint_model_ = nullptr;
      }
    }
  }

  if (joint_model_)
  {
    joint_is_continuous_ = false;
    joint_tolerance_above_ = jc.tolerance_above;
    joint_tolerance_below_ = jc.tolerance_below;
    joint_variable_index_ = robot_model_->getVariableIndex(joint_variable_name_);

    // check if we have to wrap angles when computing distances
    joint_is_continuous_ = false;
    if (joint_model_->getType() == moveit::core::JointModel::REVOLUTE)
    {
      const moveit::core::RevoluteJointModel* rjoint =
          static_cast<const moveit::core::RevoluteJointModel*>(joint_model_);
      if (rjoint->isContinuous())
        joint_is_continuous_ = true;
    }
    else if (joint_model_->getType() == moveit::core::JointModel::PLANAR)
    {
      if (local_variable_name_ == "theta")
        joint_is_continuous_ = true;
    }

    if (joint_is_continuous_)
    {
      joint_position_ = normalizeAngle(jc.position);
    }
    else
    {
      joint_position_ = jc.position;
      const moveit::core::VariableBounds& bounds = joint_model_->getVariableBounds(joint_variable_name_);

      if (bounds.min_position_ > joint_position_ + joint_tolerance_above_)
      {
        joint_position_ = bounds.min_position_;
        joint_tolerance_above_ = std::numeric_limits<double>::epsilon();
        ROS_WARN_NAMED("kinematic_constraints",
                       "Joint %s is constrained to be below the minimum bounds. "
                       "Assuming minimum bounds instead.",
                       jc.joint_name.c_str());
      }
      else if (bounds.max_position_ < joint_position_ - joint_tolerance_below_)
      {
        joint_position_ = bounds.max_position_;
        joint_tolerance_below_ = std::numeric_limits<double>::epsilon();
        ROS_WARN_NAMED("kinematic_constraints",
                       "Joint %s is constrained to be above the maximum bounds. "
                       "Assuming maximum bounds instead.",
                       jc.joint_name.c_str());
      }
    }

    if (jc.weight <= std::numeric_limits<double>::epsilon())
    {
      ROS_WARN_NAMED("kinematic_constraints",
                     "The weight on constraint for joint '%s' is very near zero.  Setting to 1.0.",
                     jc.joint_name.c_str());
      constraint_weight_ = 1.0;
    }
    else
      constraint_weight_ = jc.weight;
  }
  return joint_model_ != nullptr;
}

bool JointConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const JointConstraint& o = static_cast<const JointConstraint&>(other);
  if (o.joint_model_ == joint_model_ && o.local_variable_name_ == local_variable_name_)
    return fabs(joint_position_ - o.joint_position_) <= margin &&
           fabs(joint_tolerance_above_ - o.joint_tolerance_above_) <= margin &&
           fabs(joint_tolerance_below_ - o.joint_tolerance_below_) <= margin;
  return false;
}

ConstraintEvaluationResult JointConstraint::decide(const moveit::core::RobotState& state, bool verbose) const
{
  if (!joint_model_)
    return ConstraintEvaluationResult(true, 0.0);

  double current_joint_position = state.getVariablePosition(joint_variable_index_);
  double dif = 0.0;

  // compute signed shortest distance for continuous joints
  if (joint_is_continuous_)
  {
    dif = normalizeAngle(current_joint_position) - joint_position_;

    if (dif > boost::math::constants::pi<double>())
      dif = 2.0 * boost::math::constants::pi<double>() - dif;
    else if (dif < -boost::math::constants::pi<double>())
      dif += 2.0 * boost::math::constants::pi<double>();  // we include a sign change to have dif > 0
  }
  else
    dif = current_joint_position - joint_position_;

  // check bounds
  bool result = dif <= (joint_tolerance_above_ + 2.0 * std::numeric_limits<double>::epsilon()) &&
                dif >= (-joint_tolerance_below_ - 2.0 * std::numeric_limits<double>::epsilon());
  if (verbose)
    ROS_INFO_NAMED("kinematic_constraints",
                   "Constraint %s:: Joint name: '%s', actual value: %f, desired value: %f, "
                   "tolerance_above: %f, tolerance_below: %f",
                   result ? "satisfied" : "violated", joint_variable_name_.c_str(), current_joint_position,
                   joint_position_, joint_tolerance_above_, joint_tolerance_below_);
  return ConstraintEvaluationResult(result, constraint_weight_ * fabs(dif));
}

bool JointConstraint::enabled() const
{
  return joint_model_;
}

void JointConstraint::clear()
{
  joint_model_ = nullptr;
  joint_variable_index_ = -1;
  joint_is_continuous_ = false;
  local_variable_name_ = "";
  joint_variable_name_ = "";
  joint_position_ = joint_tolerance_below_ = joint_tolerance_above_ = 0.0;
}

void JointConstraint::print(std::ostream& out) const
{
  if (joint_model_)
  {
    out << "Joint constraint for joint " << joint_variable_name_ << ": " << std::endl;
    out << "  value = ";
    out << joint_position_ << "; ";
    out << "  tolerance below = ";
    out << joint_tolerance_below_ << "; ";
    out << "  tolerance above = ";
    out << joint_tolerance_above_ << "; ";
    out << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

bool PositionConstraint::configure(const moveit_msgs::PositionConstraint& pc, const moveit::core::Transforms& tf)
{
  // clearing before we configure to get rid of any old data
  clear();

  link_model_ = robot_model_->getLinkModel(pc.link_name);
  if (link_model_ == nullptr)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "Position constraint link model %s not found in kinematic model. Constraint invalid.",
                   pc.link_name.c_str());
    return false;
  }

  if (pc.header.frame_id.empty())
  {
    ROS_WARN_NAMED("kinematic_constraints", "No frame specified for position constraint on link '%s'!",
                   pc.link_name.c_str());
    return false;
  }

  offset_ = Eigen::Vector3d(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
  has_offset_ = offset_.squaredNorm() > std::numeric_limits<double>::epsilon();

  if (tf.isFixedFrame(pc.header.frame_id))
  {
    constraint_frame_id_ = tf.getTargetFrame();
    mobile_frame_ = false;
  }
  else
  {
    constraint_frame_id_ = pc.header.frame_id;
    mobile_frame_ = true;
  }

  // load primitive shapes, first clearing any we already have
  for (std::size_t i = 0; i < pc.constraint_region.primitives.size(); ++i)
  {
    std::unique_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region.primitives[i]));
    if (shape)
    {
      if (pc.constraint_region.primitive_poses.size() <= i)
      {
        ROS_WARN_NAMED("kinematic_constraints", "Constraint region message does not contain enough primitive poses");
        continue;
      }
      Eigen::Isometry3d t;
      tf2::fromMsg(pc.constraint_region.primitive_poses[i], t);
      ASSERT_ISOMETRY(t)  // unsanitized input, could contain a non-isometry
      constraint_region_pose_.push_back(t);
      if (!mobile_frame_)
        tf.transformPose(pc.header.frame_id, constraint_region_pose_.back(), constraint_region_pose_.back());

      const bodies::BodyPtr body(bodies::createEmptyBodyFromShapeType(shape->type));
      body->setDimensionsDirty(shape.get());
      body->setPoseDirty(constraint_region_pose_.back());
      body->updateInternalData();
      constraint_region_.push_back(body);
    }
    else
      ROS_WARN_NAMED("kinematic_constraints", "Could not construct primitive shape %zu", i);
  }

  // load meshes
  for (std::size_t i = 0; i < pc.constraint_region.meshes.size(); ++i)
  {
    std::unique_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region.meshes[i]));
    if (shape)
    {
      if (pc.constraint_region.mesh_poses.size() <= i)
      {
        ROS_WARN_NAMED("kinematic_constraints", "Constraint region message does not contain enough primitive poses");
        continue;
      }
      Eigen::Isometry3d t;
      tf2::fromMsg(pc.constraint_region.mesh_poses[i], t);
      ASSERT_ISOMETRY(t)  // unsanitized input, could contain a non-isometry
      constraint_region_pose_.push_back(t);
      if (!mobile_frame_)
        tf.transformPose(pc.header.frame_id, constraint_region_pose_.back(), constraint_region_pose_.back());
      const bodies::BodyPtr body(bodies::createEmptyBodyFromShapeType(shape->type));
      body->setDimensionsDirty(shape.get());
      body->setPoseDirty(constraint_region_pose_.back());
      body->updateInternalData();
      constraint_region_.push_back(body);
    }
    else
    {
      ROS_WARN_NAMED("kinematic_constraints", "Could not construct mesh shape %zu", i);
    }
  }

  if (pc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The weight on position constraint for link '%s' is near zero.  Setting to 1.0.",
                   pc.link_name.c_str());
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = pc.weight;

  return !constraint_region_.empty();
}

bool PositionConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const PositionConstraint& o = static_cast<const PositionConstraint&>(other);

  if (link_model_ == o.link_model_ && moveit::core::Transforms::sameFrame(constraint_frame_id_, o.constraint_frame_id_))
  {
    if ((offset_ - o.offset_).norm() > margin)
      return false;
    std::vector<bool> other_region_matches_this(constraint_region_.size(), false);
    for (std::size_t i = 0; i < constraint_region_.size(); ++i)
    {
      bool some_match = false;
      // need to check against all other regions
      for (std::size_t j = 0; j < o.constraint_region_.size(); ++j)
      {
        // constraint_region_pose_ contain only valid isometries, so diff is also a valid isometry
        Eigen::Isometry3d diff = constraint_region_pose_[i].inverse() * o.constraint_region_pose_[j];
        if (diff.translation().norm() < margin && diff.linear().isIdentity(margin) &&
            constraint_region_[i]->getType() == o.constraint_region_[j]->getType() &&
            fabs(constraint_region_[i]->computeVolume() - o.constraint_region_[j]->computeVolume()) < margin)
        {
          some_match = true;
          // can't break, as need to do matches the other way as well
          other_region_matches_this[j] = true;
        }
      }
      if (!some_match)
        return false;
    }
    for (std::size_t i = 0; i < o.constraint_region_.size(); ++i)
      if (!other_region_matches_this[i])
        return false;
    return true;
  }
  return false;
}

// helper function to avoid code duplication
static inline ConstraintEvaluationResult finishPositionConstraintDecision(const Eigen::Vector3d& pt,
                                                                          const Eigen::Vector3d& desired,
                                                                          const std::string& name, double weight,
                                                                          bool result, bool verbose)
{
  double dx = desired.x() - pt.x();
  double dy = desired.y() - pt.y();
  double dz = desired.z() - pt.z();
  if (verbose)
  {
    ROS_INFO_NAMED("kinematic_constraints",
                   "Position constraint %s on link '%s'. Desired: %f, %f, %f, current: %f, %f, %f",
                   result ? "satisfied" : "violated", name.c_str(), desired.x(), desired.y(), desired.z(), pt.x(),
                   pt.y(), pt.z());
    ROS_INFO_NAMED("kinematic_constraints", "Differences %g %g %g", dx, dy, dz);
  }
  return ConstraintEvaluationResult(result, weight * sqrt(dx * dx + dy * dy + dz * dz));
}

ConstraintEvaluationResult PositionConstraint::decide(const moveit::core::RobotState& state, bool verbose) const
{
  if (!link_model_ || constraint_region_.empty())
    return ConstraintEvaluationResult(true, 0.0);

  Eigen::Vector3d pt = state.getGlobalLinkTransform(link_model_) * offset_;
  if (mobile_frame_)
  {
    for (std::size_t i = 0; i < constraint_region_.size(); ++i)
    {
      Eigen::Isometry3d tmp = state.getFrameTransform(constraint_frame_id_) * constraint_region_pose_[i];
      bool result = constraint_region_[i]->cloneAt(tmp)->containsPoint(pt, verbose);
      if (result || (i + 1 == constraint_region_pose_.size()))
        return finishPositionConstraintDecision(pt, tmp.translation(), link_model_->getName(), constraint_weight_,
                                                result, verbose);
      else
        finishPositionConstraintDecision(pt, tmp.translation(), link_model_->getName(), constraint_weight_, result,
                                         verbose);
    }
  }
  else
  {
    for (std::size_t i = 0; i < constraint_region_.size(); ++i)
    {
      bool result = constraint_region_[i]->containsPoint(pt, true);
      if (result || (i + 1 == constraint_region_.size()))
        return finishPositionConstraintDecision(pt, constraint_region_[i]->getPose().translation(),
                                                link_model_->getName(), constraint_weight_, result, verbose);
      else
        finishPositionConstraintDecision(pt, constraint_region_[i]->getPose().translation(), link_model_->getName(),
                                         constraint_weight_, result, verbose);
    }
  }
  return ConstraintEvaluationResult(false, 0.0);
}

void PositionConstraint::print(std::ostream& out) const
{
  if (enabled())
    out << "Position constraint on link '" << link_model_->getName() << "'" << std::endl;
  else
    out << "No constraint" << std::endl;
}

void PositionConstraint::clear()
{
  offset_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  has_offset_ = false;
  constraint_region_.clear();
  constraint_region_pose_.clear();
  mobile_frame_ = false;
  constraint_frame_id_ = "";
  link_model_ = nullptr;
}

bool PositionConstraint::enabled() const
{
  return link_model_ && !constraint_region_.empty();
}

bool OrientationConstraint::configure(const moveit_msgs::OrientationConstraint& oc, const moveit::core::Transforms& tf)
{
  // clearing out any old data
  clear();

  link_model_ = robot_model_->getLinkModel(oc.link_name);
  if (!link_model_)
  {
    ROS_WARN_NAMED("kinematic_constraints", "Could not find link model for link name %s", oc.link_name.c_str());
    return false;
  }
  Eigen::Quaterniond q;
  tf2::fromMsg(oc.orientation, q);
  if (fabs(q.norm() - 1.0) > 1e-3)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "Orientation constraint for link '%s' is probably incorrect: %f, %f, %f, "
                   "%f. Assuming identity instead.",
                   oc.link_name.c_str(), oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w);
    q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }

  if (oc.header.frame_id.empty())
    ROS_WARN_NAMED("kinematic_constraints", "No frame specified for position constraint on link '%s'!",
                   oc.link_name.c_str());

  if (tf.isFixedFrame(oc.header.frame_id))
  {
    tf.transformQuaternion(oc.header.frame_id, q, q);
    desired_rotation_frame_id_ = tf.getTargetFrame();
    desired_rotation_matrix_ = Eigen::Matrix3d(q);
    desired_rotation_matrix_inv_ = desired_rotation_matrix_.transpose();
    mobile_frame_ = false;
  }
  else
  {
    desired_rotation_frame_id_ = oc.header.frame_id;
    desired_rotation_matrix_ = Eigen::Matrix3d(q);
    mobile_frame_ = true;
  }
  std::stringstream matrix_str;
  matrix_str << desired_rotation_matrix_;
  ROS_DEBUG_NAMED("kinematic_constraints", "The desired rotation matrix for link '%s' in frame %s is:\n%s",
                  oc.link_name.c_str(), desired_rotation_frame_id_.c_str(), matrix_str.str().c_str());

  if (oc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The weight on position constraint for link '%s' is near zero.  Setting to 1.0.",
                   oc.link_name.c_str());
    constraint_weight_ = 1.0;
  }
  else
  {
    constraint_weight_ = oc.weight;
  }

  parameterization_type_ = oc.parameterization;
  // validate the parameterization, set to default value if invalid
  if (parameterization_type_ != moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES &&
      parameterization_type_ != moveit_msgs::OrientationConstraint::ROTATION_VECTOR)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "Unknown parameterization for orientation constraint tolerance, using default (XYZ_EULER_ANGLES).");
    parameterization_type_ = moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES;
  }

  absolute_x_axis_tolerance_ = fabs(oc.absolute_x_axis_tolerance);
  if (absolute_x_axis_tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for absolute_x_axis_tolerance");
  absolute_y_axis_tolerance_ = fabs(oc.absolute_y_axis_tolerance);
  if (absolute_y_axis_tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for absolute_y_axis_tolerance");
  absolute_z_axis_tolerance_ = fabs(oc.absolute_z_axis_tolerance);
  if (absolute_z_axis_tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for absolute_z_axis_tolerance");

  return link_model_ != nullptr;
}

bool OrientationConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const OrientationConstraint& o = static_cast<const OrientationConstraint&>(other);

  if (o.link_model_ == link_model_ &&
      moveit::core::Transforms::sameFrame(desired_rotation_frame_id_, o.desired_rotation_frame_id_))
  {
    if (!desired_rotation_matrix_.isApprox(o.desired_rotation_matrix_))
      return false;
    return fabs(absolute_x_axis_tolerance_ - o.absolute_x_axis_tolerance_) <= margin &&
           fabs(absolute_y_axis_tolerance_ - o.absolute_y_axis_tolerance_) <= margin &&
           fabs(absolute_z_axis_tolerance_ - o.absolute_z_axis_tolerance_) <= margin;
  }
  return false;
}

void OrientationConstraint::clear()
{
  link_model_ = nullptr;
  desired_rotation_matrix_ = Eigen::Matrix3d::Identity();
  desired_rotation_matrix_inv_ = Eigen::Matrix3d::Identity();
  desired_rotation_frame_id_ = "";
  mobile_frame_ = false;
  absolute_z_axis_tolerance_ = absolute_y_axis_tolerance_ = absolute_x_axis_tolerance_ = 0.0;
}

bool OrientationConstraint::enabled() const
{
  return link_model_;
}

ConstraintEvaluationResult OrientationConstraint::decide(const moveit::core::RobotState& state, bool verbose) const
{
  if (!link_model_)
    return ConstraintEvaluationResult(true, 0.0);

  Eigen::Isometry3d diff;
  if (mobile_frame_)
  {
    // getFrameTransform() returns a valid isometry by contract
    Eigen::Matrix3d tmp = state.getFrameTransform(desired_rotation_frame_id_).linear() * desired_rotation_matrix_;
    // getGlobalLinkTransform() returns a valid isometry by contract
    diff = Eigen::Isometry3d(tmp.transpose() * state.getGlobalLinkTransform(link_model_).linear());  // valid isometry
  }
  else
  {
    // diff is valid isometry by construction
    diff = Eigen::Isometry3d(desired_rotation_matrix_inv_ * state.getGlobalLinkTransform(link_model_).linear());
  }

  // This needs to live outside the if-block scope (as xyz_rotation points to its data).
  std::tuple<Eigen::Vector3d, bool> euler_angles_error;
  Eigen::Vector3d xyz_rotation;
  if (parameterization_type_ == moveit_msgs::OrientationConstraint::XYZ_EULER_ANGLES)
  {
    euler_angles_error = CalcEulerAngles(diff.linear());
    // Converting from a rotation matrix to intrinsic XYZ Euler angles has 2 singularities:
    // pitch ~= pi/2 ==> roll + yaw = theta
    // pitch ~= -pi/2 ==> roll - yaw = theta
    // in those cases CalcEulerAngles will set roll (xyz_rotation(0)) to theta and yaw (xyz_rotation(2)) to zero, so for
    // us to be able to capture yaw tolerance violations we do the following: If theta violates the absolute yaw
    // tolerance we think of it as a pure yaw rotation and set roll to zero.
    xyz_rotation = std::get<Eigen::Vector3d>(euler_angles_error);
    if (!std::get<bool>(euler_angles_error))
    {
      if (normalizeAbsoluteAngle(xyz_rotation(0)) > absolute_z_axis_tolerance_ + std::numeric_limits<double>::epsilon())
      {
        xyz_rotation(2) = xyz_rotation(0);
        xyz_rotation(0) = 0;
      }
    }
    // Account for angle wrapping
    xyz_rotation = xyz_rotation.unaryExpr(&normalizeAbsoluteAngle);
  }
  else if (parameterization_type_ == moveit_msgs::OrientationConstraint::ROTATION_VECTOR)
  {
    Eigen::AngleAxisd aa(diff.linear());
    xyz_rotation = aa.axis() * aa.angle();
    xyz_rotation(0) = fabs(xyz_rotation(0));
    xyz_rotation(1) = fabs(xyz_rotation(1));
    xyz_rotation(2) = fabs(xyz_rotation(2));
  }
  else
  {
    /* The parameterization type should be validated in configure, so this should never happen. */
    ROS_ERROR_STREAM_NAMED("kinematic_constraints",
                           "The parameterization type for the orientation constraints is invalid.");
  }

  bool result = xyz_rotation(2) < absolute_z_axis_tolerance_ + std::numeric_limits<double>::epsilon() &&
                xyz_rotation(1) < absolute_y_axis_tolerance_ + std::numeric_limits<double>::epsilon() &&
                xyz_rotation(0) < absolute_x_axis_tolerance_ + std::numeric_limits<double>::epsilon();

  if (verbose)
  {
    Eigen::Quaterniond q_act(state.getGlobalLinkTransform(link_model_).linear());
    Eigen::Quaterniond q_des(desired_rotation_matrix_);
    ROS_INFO_NAMED("kinematic_constraints",
                   "Orientation constraint %s for link '%s'. Quaternion desired: %f %f %f %f, quaternion "
                   "actual: %f %f %f %f, error: x=%f, y=%f, z=%f, tolerance: x=%f, y=%f, z=%f",
                   result ? "satisfied" : "violated", link_model_->getName().c_str(), q_des.x(), q_des.y(), q_des.z(),
                   q_des.w(), q_act.x(), q_act.y(), q_act.z(), q_act.w(), xyz_rotation(0), xyz_rotation(1),
                   xyz_rotation(2), absolute_x_axis_tolerance_, absolute_y_axis_tolerance_, absolute_z_axis_tolerance_);
  }

  return ConstraintEvaluationResult(result, constraint_weight_ * (xyz_rotation(0) + xyz_rotation(1) + xyz_rotation(2)));
}

void OrientationConstraint::print(std::ostream& out) const
{
  if (link_model_)
  {
    out << "Orientation constraint on link '" << link_model_->getName() << "'" << std::endl;
    Eigen::Quaterniond q_des(desired_rotation_matrix_);
    out << "Desired orientation:" << q_des.x() << "," << q_des.y() << "," << q_des.z() << "," << q_des.w() << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

VisibilityConstraint::VisibilityConstraint(const moveit::core::RobotModelConstPtr& model)
  : KinematicConstraint(model), collision_env_(new collision_detection::CollisionEnvFCL(model))
{
  type_ = VISIBILITY_CONSTRAINT;
}

void VisibilityConstraint::clear()
{
  mobile_sensor_frame_ = false;
  mobile_target_frame_ = false;
  target_frame_id_ = "";
  sensor_frame_id_ = "";
  sensor_pose_ = Eigen::Isometry3d::Identity();
  sensor_view_direction_ = 0;
  target_pose_ = Eigen::Isometry3d::Identity();
  cone_sides_ = 0;
  points_.clear();
  target_radius_ = -1.0;
  max_view_angle_ = 0.0;
  max_range_angle_ = 0.0;
}

bool VisibilityConstraint::configure(const moveit_msgs::VisibilityConstraint& vc, const moveit::core::Transforms& tf)
{
  clear();
  target_radius_ = fabs(vc.target_radius);

  if (vc.target_radius <= std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints",
                   "The radius of the target disc that must be visible should be strictly positive");

  if (vc.cone_sides < 3)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The number of sides for the visibility region must be 3 or more. "
                   "Assuming 3 sides instead of the specified %d",
                   vc.cone_sides);
    cone_sides_ = 3;
  }
  else
    cone_sides_ = vc.cone_sides;

  // compute the points on the base circle of the cone that make up the cone sides
  points_.clear();
  double delta = 2.0 * boost::math::constants::pi<double>() / (double)cone_sides_;
  double a = 0.0;
  for (unsigned int i = 0; i < cone_sides_; ++i, a += delta)
  {
    double x = sin(a) * target_radius_;
    double y = cos(a) * target_radius_;
    points_.push_back(Eigen::Vector3d(x, y, 0.0));
  }

  tf2::fromMsg(vc.target_pose.pose, target_pose_);
  ASSERT_ISOMETRY(target_pose_)  // unsanitized input, could contain a non-isometry

  if (tf.isFixedFrame(vc.target_pose.header.frame_id))
  {
    tf.transformPose(vc.target_pose.header.frame_id, target_pose_, target_pose_);
    target_frame_id_ = tf.getTargetFrame();
    mobile_target_frame_ = false;
    // transform won't change, so apply it now
    for (Eigen::Vector3d& point : points_)
      point = target_pose_ * point;
  }
  else
  {
    target_frame_id_ = vc.target_pose.header.frame_id;
    mobile_target_frame_ = true;
  }

  tf2::fromMsg(vc.sensor_pose.pose, sensor_pose_);
  ASSERT_ISOMETRY(sensor_pose_)  // unsanitized input, could contain a non-isometry

  if (tf.isFixedFrame(vc.sensor_pose.header.frame_id))
  {
    tf.transformPose(vc.sensor_pose.header.frame_id, sensor_pose_, sensor_pose_);
    sensor_frame_id_ = tf.getTargetFrame();
    mobile_sensor_frame_ = false;
  }
  else
  {
    sensor_frame_id_ = vc.sensor_pose.header.frame_id;
    mobile_sensor_frame_ = true;
  }

  if (vc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints", "The weight of visibility constraint is near zero.  Setting to 1.0.");
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = vc.weight;

  max_view_angle_ = vc.max_view_angle;
  max_range_angle_ = vc.max_range_angle;
  sensor_view_direction_ = vc.sensor_view_direction;

  return target_radius_ > std::numeric_limits<double>::epsilon();
}

bool VisibilityConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const VisibilityConstraint& o = static_cast<const VisibilityConstraint&>(other);

  if (moveit::core::Transforms::sameFrame(target_frame_id_, o.target_frame_id_) &&
      moveit::core::Transforms::sameFrame(sensor_frame_id_, o.sensor_frame_id_) && cone_sides_ == o.cone_sides_ &&
      sensor_view_direction_ == o.sensor_view_direction_)
  {
    if (fabs(max_view_angle_ - o.max_view_angle_) > margin || fabs(target_radius_ - o.target_radius_) > margin)
      return false;
    // sensor_pose_ is valid isometry, checked in configure()
    Eigen::Isometry3d diff = sensor_pose_.inverse() * o.sensor_pose_;  // valid isometry
    if (diff.translation().norm() > margin)
      return false;
    if (!diff.linear().isIdentity(margin))
      return false;
    // target_pose_ is valid isometry, checked in configure()
    diff = target_pose_.inverse() * o.target_pose_;  // valid isometry
    if (diff.translation().norm() > margin)
      return false;
    if (!diff.linear().isIdentity(margin))
      return false;
    return true;
  }
  return false;
}

bool VisibilityConstraint::enabled() const
{
  return target_radius_ > std::numeric_limits<double>::epsilon();
}

shapes::Mesh* VisibilityConstraint::getVisibilityCone(const moveit::core::RobotState& state) const
{
  // the current pose of the sensor

  const Eigen::Isometry3d& sp =
      mobile_sensor_frame_ ? state.getFrameTransform(sensor_frame_id_) * sensor_pose_ : sensor_pose_;
  const Eigen::Isometry3d& tp =
      mobile_target_frame_ ? state.getFrameTransform(target_frame_id_) * target_pose_ : target_pose_;

  // transform the points on the disc to the desired target frame
  const EigenSTL::vector_Vector3d* points = &points_;
  std::unique_ptr<EigenSTL::vector_Vector3d> temp_points;
  if (mobile_target_frame_)
  {
    temp_points = std::make_unique<EigenSTL::vector_Vector3d>(points_.size());
    for (std::size_t i = 0; i < points_.size(); ++i)
      temp_points->at(i) = tp * points_[i];
    points = temp_points.get();
  }

  // allocate memory for a mesh to represent the visibility cone
  shapes::Mesh* m = new shapes::Mesh();
  m->vertex_count = cone_sides_ + 2;
  m->vertices = new double[m->vertex_count * 3];
  m->triangle_count = cone_sides_ * 2;
  m->triangles = new unsigned int[m->triangle_count * 3];
  // we do NOT allocate normals because we do not compute them

  // the sensor origin
  m->vertices[0] = sp.translation().x();
  m->vertices[1] = sp.translation().y();
  m->vertices[2] = sp.translation().z();

  // the center of the base of the cone approximation
  m->vertices[3] = tp.translation().x();
  m->vertices[4] = tp.translation().y();
  m->vertices[5] = tp.translation().z();

  // the points that approximate the base disc
  for (std::size_t i = 0; i < points->size(); ++i)
  {
    m->vertices[i * 3 + 6] = points->at(i).x();
    m->vertices[i * 3 + 7] = points->at(i).y();
    m->vertices[i * 3 + 8] = points->at(i).z();
  }

  // add the triangles
  std::size_t p3 = points->size() * 3;
  for (std::size_t i = 1; i < points->size(); ++i)
  {
    // triangle forming a side of the cone, using the sensor origin
    std::size_t i3 = (i - 1) * 3;
    m->triangles[i3] = i + 1;
    m->triangles[i3 + 1] = 0;
    m->triangles[i3 + 2] = i + 2;
    // triangle forming a part of the base of the cone, using the center of the base
    std::size_t i6 = p3 + i3;
    m->triangles[i6] = i + 1;
    m->triangles[i6 + 1] = 1;
    m->triangles[i6 + 2] = i + 2;
  }

  // last triangles
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 0;
  m->triangles[p3 - 1] = 2;
  p3 *= 2;
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 1;
  m->triangles[p3 - 1] = 2;

  return m;
}

void VisibilityConstraint::getMarkers(const moveit::core::RobotState& state,
                                      visualization_msgs::MarkerArray& markers) const
{
  shapes::Mesh* m = getVisibilityCone(state);
  visualization_msgs::Marker mk;
  shapes::constructMarkerFromShape(m, mk);
  delete m;
  mk.header.frame_id = robot_model_->getModelFrame();
  mk.header.stamp = ros::Time::now();
  mk.ns = "constraints";
  mk.id = 1;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = 0;
  mk.pose.orientation.y = 0;
  mk.pose.orientation.z = 0;
  mk.pose.orientation.w = 1;
  mk.lifetime = ros::Duration(60);
  // this scale necessary to make results look reasonable
  mk.scale.x = .01;
  mk.color.a = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  markers.markers.push_back(mk);

  // getFrameTransform() returns a valid isometry by contract
  // sensor_pose_ is valid isometry (checked in configure())
  const Eigen::Isometry3d& sp =
      mobile_sensor_frame_ ? state.getFrameTransform(sensor_frame_id_) * sensor_pose_ : sensor_pose_;
  // target_pose_ is valid isometry (checked in configure())
  const Eigen::Isometry3d& tp =
      mobile_target_frame_ ? state.getFrameTransform(target_frame_id_) * target_pose_ : target_pose_;

  visualization_msgs::Marker mka;
  mka.type = visualization_msgs::Marker::ARROW;
  mka.action = visualization_msgs::Marker::ADD;
  mka.color = mk.color;
  mka.pose = mk.pose;

  mka.header = mk.header;
  mka.ns = mk.ns;
  mka.id = 2;
  mka.lifetime = mk.lifetime;
  mka.scale.x = 0.05;
  mka.scale.y = .15;
  mka.scale.z = 0.0;
  mka.points.resize(2);
  Eigen::Vector3d d = tp.translation() + tp.linear().col(2) * 0.5;
  mka.points[0].x = tp.translation().x();
  mka.points[0].y = tp.translation().y();
  mka.points[0].z = tp.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();
  markers.markers.push_back(mka);

  mka.id = 3;
  mka.color.b = 1.0;
  mka.color.r = 0.0;

  d = sp.translation() + sp.linear().col(2 - sensor_view_direction_) * 0.5;
  mka.points[0].x = sp.translation().x();
  mka.points[0].y = sp.translation().y();
  mka.points[0].z = sp.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();

  markers.markers.push_back(mka);
}

ConstraintEvaluationResult VisibilityConstraint::decide(const moveit::core::RobotState& state, bool verbose) const
{
  if (target_radius_ <= std::numeric_limits<double>::epsilon())
    return ConstraintEvaluationResult(true, 0.0);

  if (max_view_angle_ > 0.0 || max_range_angle_ > 0.0)
  {
    // getFrameTransform() returns a valid isometry by contract
    // sensor_pose_ is valid isometry (checked in configure())
    const Eigen::Isometry3d& sp =
        mobile_sensor_frame_ ? state.getFrameTransform(sensor_frame_id_) * sensor_pose_ : sensor_pose_;
    // target_pose_ is valid isometry (checked in configure())
    const Eigen::Isometry3d& tp =
        mobile_target_frame_ ? state.getFrameTransform(target_frame_id_) * target_pose_ : target_pose_;

    // necessary to do subtraction as SENSOR_Z is 0 and SENSOR_X is 2
    const Eigen::Vector3d& normal2 = sp.linear().col(2 - sensor_view_direction_);

    if (max_view_angle_ > 0.0)
    {
      const Eigen::Vector3d& normal1 = tp.linear().col(2) * -1.0;  // along Z axis and inverted
      double dp = normal2.dot(normal1);
      double ang = acos(dp);
      if (dp < 0.0)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint is violated because the sensor is looking at "
                                                  "the wrong side");
        return ConstraintEvaluationResult(false, 0.0);
      }
      if (max_view_angle_ < ang)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints",
                         "Visibility constraint is violated because the view angle is %lf "
                         "(above the maximum allowed of %lf)",
                         ang, max_view_angle_);
        return ConstraintEvaluationResult(false, 0.0);
      }
    }
    if (max_range_angle_ > 0.0)
    {
      const Eigen::Vector3d& dir = (tp.translation() - sp.translation()).normalized();
      double dp = normal2.dot(dir);
      if (dp < 0.0)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint is violated because the sensor is looking at "
                                                  "the wrong side");
        return ConstraintEvaluationResult(false, 0.0);
      }

      double ang = acos(dp);
      if (max_range_angle_ < ang)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints",
                         "Visibility constraint is violated because the range angle is %lf "
                         "(above the maximum allowed of %lf)",
                         ang, max_range_angle_);
        return ConstraintEvaluationResult(false, 0.0);
      }
    }
  }

  shapes::Mesh* m = getVisibilityCone(state);
  if (!m)
    return ConstraintEvaluationResult(false, 0.0);

  // add the visibility cone as an object
  collision_env_->getWorld()->addToObject("cone", shapes::ShapeConstPtr(m), Eigen::Isometry3d::Identity());

  // check for collisions between the robot and the cone
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm;
  acm.setDefaultEntry("cone", [this](collision_detection::Contact& contact) { return decideContact(contact); });
  req.contacts = true;
  req.verbose = verbose;
  req.max_contacts = 1;
  collision_env_->checkRobotCollision(req, res, state, acm);

  if (verbose)
  {
    std::stringstream ss;
    m->print(ss);
    ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint %ssatisfied. Visibility cone approximation:\n %s",
                   res.collision ? "not " : "", ss.str().c_str());
  }

  collision_env_->getWorld()->removeObject("cone");

  return ConstraintEvaluationResult(!res.collision, res.collision ? res.contacts.begin()->second.front().depth : 0.0);
}

bool VisibilityConstraint::decideContact(const collision_detection::Contact& contact) const
{
  if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_ATTACHED ||
      contact.body_type_2 == collision_detection::BodyTypes::ROBOT_ATTACHED)
    return true;
  if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK &&
      contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT &&
      (moveit::core::Transforms::sameFrame(contact.body_name_1, sensor_frame_id_) ||
       moveit::core::Transforms::sameFrame(contact.body_name_1, target_frame_id_)))
  {
    ROS_DEBUG_NAMED("kinematic_constraints", "Accepted collision with either sensor or target");
    return true;
  }
  if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_LINK &&
      contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT &&
      (moveit::core::Transforms::sameFrame(contact.body_name_2, sensor_frame_id_) ||
       moveit::core::Transforms::sameFrame(contact.body_name_2, target_frame_id_)))
  {
    ROS_DEBUG_NAMED("kinematic_constraints", "Accepted collision with either sensor or target");
    return true;
  }
  return false;
}

void VisibilityConstraint::print(std::ostream& out) const
{
  if (enabled())
  {
    out << "Visibility constraint for sensor in frame '" << sensor_frame_id_ << "' using target in frame '"
        << target_frame_id_ << "'" << std::endl;
    out << "Target radius: " << target_radius_ << ", using " << cone_sides_ << " sides." << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

void KinematicConstraintSet::clear()
{
  all_constraints_ = moveit_msgs::Constraints();
  kinematic_constraints_.clear();
  joint_constraints_.clear();
  position_constraints_.clear();
  orientation_constraints_.clear();
  visibility_constraints_.clear();
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::JointConstraint>& jc)
{
  bool result = true;
  for (const moveit_msgs::JointConstraint& joint_constraint : jc)
  {
    JointConstraint* ev = new JointConstraint(robot_model_);
    bool u = ev->configure(joint_constraint);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    joint_constraints_.push_back(joint_constraint);
    all_constraints_.joint_constraints.push_back(joint_constraint);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::PositionConstraint>& pc,
                                 const moveit::core::Transforms& tf)
{
  bool result = true;
  for (const moveit_msgs::PositionConstraint& position_constraint : pc)
  {
    PositionConstraint* ev = new PositionConstraint(robot_model_);
    bool u = ev->configure(position_constraint, tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    position_constraints_.push_back(position_constraint);
    all_constraints_.position_constraints.push_back(position_constraint);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::OrientationConstraint>& oc,
                                 const moveit::core::Transforms& tf)
{
  bool result = true;
  for (const moveit_msgs::OrientationConstraint& orientation_constraint : oc)
  {
    OrientationConstraint* ev = new OrientationConstraint(robot_model_);
    bool u = ev->configure(orientation_constraint, tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    orientation_constraints_.push_back(orientation_constraint);
    all_constraints_.orientation_constraints.push_back(orientation_constraint);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::VisibilityConstraint>& vc,
                                 const moveit::core::Transforms& tf)
{
  bool result = true;
  for (const moveit_msgs::VisibilityConstraint& visibility_constraint : vc)
  {
    VisibilityConstraint* ev = new VisibilityConstraint(robot_model_);
    bool u = ev->configure(visibility_constraint, tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    visibility_constraints_.push_back(visibility_constraint);
    all_constraints_.visibility_constraints.push_back(visibility_constraint);
  }
  return result;
}

bool KinematicConstraintSet::add(const moveit_msgs::Constraints& c, const moveit::core::Transforms& tf)
{
  bool j = add(c.joint_constraints);
  bool p = add(c.position_constraints, tf);
  bool o = add(c.orientation_constraints, tf);
  bool v = add(c.visibility_constraints, tf);
  return j && p && o && v;
}

ConstraintEvaluationResult KinematicConstraintSet::decide(const moveit::core::RobotState& state, bool verbose) const
{
  ConstraintEvaluationResult res(true, 0.0);
  for (const KinematicConstraintPtr& kinematic_constraint : kinematic_constraints_)
  {
    ConstraintEvaluationResult r = kinematic_constraint->decide(state, verbose);
    if (!r.satisfied)
      res.satisfied = false;
    res.distance += r.distance;
  }
  return res;
}

ConstraintEvaluationResult KinematicConstraintSet::decide(const moveit::core::RobotState& state,
                                                          std::vector<ConstraintEvaluationResult>& results,
                                                          bool verbose) const
{
  ConstraintEvaluationResult result(true, 0.0);
  results.resize(kinematic_constraints_.size());
  for (std::size_t i = 0; i < kinematic_constraints_.size(); ++i)
  {
    results[i] = kinematic_constraints_[i]->decide(state, verbose);
    result.satisfied = result.satisfied && results[i].satisfied;
    result.distance += results[i].distance;
  }

  return result;
}

void KinematicConstraintSet::print(std::ostream& out) const
{
  out << kinematic_constraints_.size() << " kinematic constraints" << std::endl;
  for (const KinematicConstraintPtr& kinematic_constraint : kinematic_constraints_)
    kinematic_constraint->print(out);
}

bool KinematicConstraintSet::equal(const KinematicConstraintSet& other, double margin) const
{
  // each constraint in this matches some in the other
  for (const KinematicConstraintPtr& kinematic_constraint : kinematic_constraints_)
  {
    bool found = false;
    for (unsigned int j = 0; !found && j < other.kinematic_constraints_.size(); ++j)
      found = kinematic_constraint->equal(*other.kinematic_constraints_[j], margin);
    if (!found)
      return false;
  }
  // each constraint in the other matches some constraint in this
  for (const KinematicConstraintPtr& kinematic_constraint : other.kinematic_constraints_)
  {
    bool found = false;
    for (unsigned int j = 0; !found && j < kinematic_constraints_.size(); ++j)
      found = kinematic_constraint->equal(*kinematic_constraints_[j], margin);
    if (!found)
      return false;
  }
  return true;
}

}  // end of namespace kinematic_constraints
