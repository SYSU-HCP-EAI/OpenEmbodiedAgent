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

#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/macros/class_forward.h>

#include <geometric_shapes/bodies.h>
#include <moveit_msgs/Constraints.h>

#include <iostream>
#include <vector>

/** \brief Representation and evaluation of kinematic constraints */
namespace kinematic_constraints
{
/// \brief Struct for containing the results of constraint evaluation
struct ConstraintEvaluationResult
{
  /**
   * \brief Constructor
   *
   * @param [in] result_satisfied True if the constraint evaluated to true, otherwise false
   * @param [in] dist The distance evaluated by the constraint
   *
   * @return
   */
  ConstraintEvaluationResult(bool result_satisfied = false, double dist = 0.0)
    : satisfied(result_satisfied), distance(dist)
  {
  }

  bool satisfied;  /**< \brief Whether or not the constraint or constraints were satisfied */
  double distance; /**< \brief The distance evaluation from the constraint or constraints */
};

MOVEIT_CLASS_FORWARD(KinematicConstraint);  // Defines KinematicConstraintPtr, ConstPtr, WeakPtr... etc

/// \brief Base class for representing a kinematic constraint
class KinematicConstraint
{
public:
  /// \brief Enum for representing a constraint
  enum ConstraintType
  {
    UNKNOWN_CONSTRAINT,
    JOINT_CONSTRAINT,
    POSITION_CONSTRAINT,
    ORIENTATION_CONSTRAINT,
    VISIBILITY_CONSTRAINT
  };

  /**
   * \brief Constructor
   *
   * @param [in] model The kinematic model used for constraint evaluation
   */
  KinematicConstraint(const moveit::core::RobotModelConstPtr& model);
  virtual ~KinematicConstraint();

  /** \brief Clear the stored constraint */
  virtual void clear() = 0;

  /**
   * \brief Decide whether the constraint is satisfied in the indicated state
   *
   * @param [in] state The kinematic state used for evaluation
   * @param [in] verbose Whether or not to print output
   *
   * @return
   */
  virtual ConstraintEvaluationResult decide(const moveit::core::RobotState& state, bool verbose = false) const = 0;

  /** \brief This function returns true if this constraint is
      configured and able to decide whether states do meet the
      constraint or not. If this function returns false it means
      that decide() will always return true -- there is no
      constraint to be checked. */
  virtual bool enabled() const = 0;

  /**
   * \brief Check if two constraints are the same.  This means that
   * the types are the same, the subject of the constraint is the
   * same, and all values associated with the constraint are within a
   * margin.  The other constraint must also be enabled.
   *
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   *
   * @return True if equal, otherwise false
   */
  virtual bool equal(const KinematicConstraint& other, double margin) const = 0;

  /**
   * \brief Get the type of constraint
   *
   *
   * @return The constraint type
   */
  ConstraintType getType() const
  {
    return type_;
  }

  /**
   * \brief Print the constraint data
   *
   * @param [in] out The file descriptor for printing
   */
  virtual void print(std::ostream& /*unused*/ = std::cout) const
  {
  }

  /**
   *
   * \brief The weight of a constraint is a multiplicative factor associated to the distance computed by the decide()
   *function
   *
   * @return The constraint weight
   */
  double getConstraintWeight() const
  {
    return constraint_weight_;
  }

  /**
   *
   *
   *
   * @return The kinematic model associated with this constraint
   */
  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

protected:
  ConstraintType type_;                          /**< \brief The type of the constraint */
  moveit::core::RobotModelConstPtr robot_model_; /**< \brief The kinematic model associated with this constraint */
  double constraint_weight_; /**< \brief The weight of a constraint is a multiplicative factor associated to the
                                distance computed by the decide() function  */
};

MOVEIT_CLASS_FORWARD(JointConstraint);  // Defines JointConstraintPtr, ConstPtr, WeakPtr... etc

/**
 * \brief Class for handling single DOF joint constraints.
 *
 * This class handles single DOF constraints expressed as a tolerance
 * above and below a target position.  Multi-DOF joints can be
 * accomodated by using local name formulations - i.e. for a planar
 * joint specifying a constraint in terms of "planar_joint_name"/x.
 *
 * Continuous revolute single DOF joints will be evaluated based on
 * wrapping around 3.14 and -3.14.  Tolerances above and below will be
 * evaluating over the wrap.  For instance, if the constraint value is
 * 3.14 and the tolerance above is .04, a value of -3.14 is in bounds,
 * as is a value of -3.12.  -3.1 is out of bounds.  Similarly, if the
 * value of the constraint is -3.14, the tolerance above is .04, and
 * the tolerance below is .02 then -3.1 is a valid value, as is 3.14;
 * 3.1 is out of bounds.
 *
 * Type will be JOINT_CONSTRAINT.
 */
class JointConstraint : public KinematicConstraint
{
public:
  /**
   * \brief Constructor
   *
   * @param [in] model The kinematic model used for constraint evaluation
   */
  JointConstraint(const moveit::core::RobotModelConstPtr& model)
    : KinematicConstraint(model), joint_model_(nullptr), joint_variable_index_(-1)
  {
    type_ = JOINT_CONSTRAINT;
  }

  /**
   * \brief Configure the constraint based on a
   * moveit_msgs::JointConstraint
   *
   * For the configure command to be successful, the joint must exist
   * in the kinematic model, the joint must not be a multi-DOF joint
   * (for these joints, local variables should be used), and the
   * tolerance values must be positive.
   *
   * @param [in] jc JointConstraint for configuration
   *
   * @return True if constraint can be configured from jc
   */
  bool configure(const moveit_msgs::JointConstraint& jc);

  /**
   * \brief Check if two joint constraints are the same.
   *
   * This means that the types are the same, the subject of the
   * constraint is the same, and all values associated with the
   * constraint are within a margin.  The other constraint must also
   * be enabled.  For this to be true of joint constraints, they must
   * act on the same joint, and the position and tolerance values must
   * be within the margins.
   *
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   *
   * @return True if equal, otherwise false
   */
  bool equal(const KinematicConstraint& other, double margin) const override;

  ConstraintEvaluationResult decide(const moveit::core::RobotState& state, bool verbose = false) const override;
  bool enabled() const override;
  void clear() override;
  void print(std::ostream& out = std::cout) const override;

  /**
   * \brief Get the joint model for which this constraint operates
   *
   * @return The relevant joint model if enabled, and otherwise NULL
   */
  const moveit::core::JointModel* getJointModel() const
  {
    return joint_model_;
  }
  /**
   * \brief Gets the local variable name if this constraint was
   * configured for a part of a multi-DOF joint
   *
   *
   * @return The component of the joint name after the slash, or the
   * empty string if there is no local variable name
   */
  const std::string& getLocalVariableName() const
  {
    return local_variable_name_;
  }

  /**
   *  \brief Gets the joint variable name, as known to the moveit::core::RobotModel
   *
   * This will include the local variable name if a variable of a multi-DOF joint is constrained.
   *
   * @return The joint variable name
   */
  const std::string& getJointVariableName() const
  {
    return joint_variable_name_;
  }

  int getJointVariableIndex() const
  {
    return joint_variable_index_;
  }

  /**
   * \brief Gets the desired position component of the constraint
   *
   *
   * @return The desired joint position
   */
  double getDesiredJointPosition() const
  {
    return joint_position_;
  }

  /**
   * \brief Gets the upper tolerance component of the joint constraint
   *
   *
   * @return The above joint tolerance
   */
  double getJointToleranceAbove() const
  {
    return joint_tolerance_above_;
  }

  /**
   * \brief Gets the lower tolerance component of the joint constraint
   *
   *
   * @return The below joint tolerance
   */
  double getJointToleranceBelow() const
  {
    return joint_tolerance_below_;
  }

protected:
  const moveit::core::JointModel* joint_model_; /**< \brief The joint from the kinematic model for this constraint */
  bool joint_is_continuous_;                    /**< \brief Whether or not the joint is continuous */
  std::string local_variable_name_;             /**< \brief The local variable name for a multi DOF joint, if any */
  std::string joint_variable_name_;             /**< \brief The joint variable name */
  int joint_variable_index_; /**< \brief The index of the joint variable name in the full robot state */
  double joint_position_, joint_tolerance_above_, joint_tolerance_below_; /**< \brief Position and tolerance values*/
};

MOVEIT_CLASS_FORWARD(OrientationConstraint);  // Defines OrientationConstraintPtr, ConstPtr, WeakPtr... etc

/**
 * \brief Class for constraints on the orientation of a link
 *
 * This class expresses an orientation constraint on a particular
 * link.  The constraint is specified in terms of a quaternion, with
 * tolerances on X,Y, and Z axes.  The rotation difference is computed
 * based on the XYZ Euler angle formulation (intrinsic rotations) or as a rotation vector. This depends on the
 * `Parameterization` type. The header on the quaternion can be specified in terms of either a fixed or a mobile
 * frame.  The type value will return ORIENTATION_CONSTRAINT.
 *
 */
class OrientationConstraint : public KinematicConstraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  /**
   * \brief Constructor
   *
   * @param [in] model The kinematic model used for constraint evaluation
   */
  OrientationConstraint(const moveit::core::RobotModelConstPtr& model)
    : KinematicConstraint(model), link_model_(nullptr)
  {
    type_ = ORIENTATION_CONSTRAINT;
  }

  /**
   * \brief Configure the constraint based on a
   * moveit_msgs::OrientationConstraint
   *
   * For the configure command to be successful, the link must exist
   * in the kinematic model. Note that if the absolute tolerance
   * values are left as 0.0 only values less than a very small epsilon
   * will evaluate to satisfied.
   *
   * @param [in] oc OrientationConstraint for configuration
   *
   * @return True if constraint can be configured from oc
   */
  bool configure(const moveit_msgs::OrientationConstraint& oc, const moveit::core::Transforms& tf);

  /**
   * \brief Check if two orientation constraints are the same.

   * This means that the types are the same, the subject of the
   * constraint is the same, and all values associated with the
   * constraint are within a margin.  The other constraint must also
   * be enabled.  For this to be true of orientation constraints:
   * \li The link must be the same
   * \li The rotations specified by the quaternions must be within the margin
   * \li The tolerances must all be within the margin
   *
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   *
   * @return True if equal, otherwise false
   */
  bool equal(const KinematicConstraint& other, double margin) const override;

  void clear() override;
  ConstraintEvaluationResult decide(const moveit::core::RobotState& state, bool verbose = false) const override;
  bool enabled() const override;
  void print(std::ostream& out = std::cout) const override;

  /**
   * \brief Gets the subject link model
   *
   *
   * @return Returns the current link model
   */
  const moveit::core::LinkModel* getLinkModel() const
  {
    return link_model_;
  }

  /**
   * \brief The target frame of the planning_models::Transforms class,
   * for interpreting the rotation frame.
   *
   * @return The reference frame.
   */
  const std::string& getReferenceFrame() const
  {
    return desired_rotation_frame_id_;
  }

  /**
   * \brief Whether or not a mobile reference frame is being employed.
   *
   * @return True if a mobile reference frame is being employed, and
   * otherwise false.
   */
  bool mobileReferenceFrame() const
  {
    return mobile_frame_;
  }

  /**
   * \brief The rotation target in the reference frame.
   *
   * @return The target rotation.
   *
   * The returned matrix is always a valid rotation matrix.
   */
  const Eigen::Matrix3d& getDesiredRotationMatrix() const
  {
    // validity of the rotation matrix is enforced in configure()
    return desired_rotation_matrix_;
  }

  /**
   * \brief Gets the X axis tolerance
   *
   *
   * @return The X axis tolerance
   */
  double getXAxisTolerance() const
  {
    return absolute_x_axis_tolerance_;
  }

  /**
   * \brief Gets the Y axis tolerance
   *
   *
   * @return The Y axis tolerance
   */
  double getYAxisTolerance() const
  {
    return absolute_y_axis_tolerance_;
  }

  /**
   * \brief Gets the Z axis tolerance
   *
   *
   * @return The Z axis tolerance
   */
  double getZAxisTolerance() const
  {
    return absolute_z_axis_tolerance_;
  }

  int getParameterizationType() const
  {
    return parameterization_type_;
  }

protected:
  const moveit::core::LinkModel* link_model_;   /**< \brief The target link model */
  Eigen::Matrix3d desired_rotation_matrix_;     /**< \brief The desired rotation matrix in the tf frame. Guaranteed to
                                                 * be valid rotation matrix. */
  Eigen::Matrix3d desired_rotation_matrix_inv_; /**< \brief The inverse of the desired rotation matrix, precomputed for
                                                 * efficiency. Guaranteed to be valid rotation matrix. */
  std::string desired_rotation_frame_id_;       /**< \brief The target frame of the transform tree */
  bool mobile_frame_;                           /**< \brief Whether or not the header frame is mobile or fixed */
  int parameterization_type_;                   /**< \brief Parameterization type for orientation tolerance. */
  double absolute_x_axis_tolerance_, absolute_y_axis_tolerance_,
      absolute_z_axis_tolerance_; /**< \brief Storage for the tolerances */
};

MOVEIT_CLASS_FORWARD(PositionConstraint);  // Defines PositionConstraintPtr, ConstPtr, WeakPtr... etc

/**
 * \brief Class for constraints on the XYZ position of a link
 *
 * This class expresses X,Y,Z position constraints of a link.  The
 * position area is specified as a bounding volume consisting of one
 * or more shapes - either solid primitives or meshes. The pose
 * information in the volumes will be interpreted by using the header
 * information.  The header may either specify a fixed frame or a
 * mobile frame.  Additionally, a target offset specified in the frame
 * of the link being constrained can be specified.  The type value
 * will return POSITION_CONSTRAINT.
 *
 */
class PositionConstraint : public KinematicConstraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  /**
   * \brief Constructor
   *
   * @param [in] model The kinematic model used for constraint evaluation
   */
  PositionConstraint(const moveit::core::RobotModelConstPtr& model) : KinematicConstraint(model), link_model_(nullptr)
  {
    type_ = POSITION_CONSTRAINT;
  }

  /**
   * \brief Configure the constraint based on a
   * moveit_msgs::PositionConstraint
   *
   * For the configure command to be successful, the link must be
   * specified in the model, and one or more constrained regions must
   * be correctly specified, which requires containing a valid shape
   * and a pose for that shape.  If the header frame on the constraint
   * is empty, the constraint will fail to configure.  If an invalid
   * quaternion is passed for a shape, the identity quaternion will be
   * substituted.
   *
   * @param [in] pc moveit_msgs::PositionConstraint for configuration
   *
   * @return True if constraint can be configured from pc
   */
  bool configure(const moveit_msgs::PositionConstraint& pc, const moveit::core::Transforms& tf);

  /**
   * \brief Check if two constraints are the same.  For position
   * constraints this means that:
   * \li The types are the same
   * \li The link model is the same
   * \li The frame of the constraints are the same
   * \li The target offsets are no more than the margin apart
   * \li Each entry in the constraint region of this constraint matches a region in the other constraint
   * \li Each entry in the other constraint region matches a region in the other constraint
   *
   * Two constraint regions matching each other means that:
   * \li The poses match within the margin
   * \li The types are the same
   * \li The shape volumes are within the margin
   *
   * Note that the two shapes can have different numbers of regions as
   * long as all regions are matched up to another.
   *
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   *
   * @return True if equal, otherwise false
   */
  bool equal(const KinematicConstraint& other, double margin) const override;

  void clear() override;
  ConstraintEvaluationResult decide(const moveit::core::RobotState& state, bool verbose = false) const override;
  bool enabled() const override;
  void print(std::ostream& out = std::cout) const override;

  /**
   * \brief Returns the associated link model, or NULL if not enabled
   *
   *
   * @return The link model
   */
  const moveit::core::LinkModel* getLinkModel() const
  {
    return link_model_;
  }

  /**
   * \brief Returns the target offset
   *
   *
   * @return The target offset
   */
  const Eigen::Vector3d& getLinkOffset() const
  {
    return offset_;
  }

  /**
   * \brief If the constraint is enabled and the link offset is
   * substantially different than zero
   *
   *
   * @return Whether or not there is a link offset
   */
  bool hasLinkOffset() const
  {
    if (!enabled())
      return false;
    return has_offset_;
  }

  /**
   * \brief Returns all the constraint regions
   *
   *
   * @return The constraint regions
   */
  const std::vector<bodies::BodyPtr>& getConstraintRegions() const
  {
    return constraint_region_;
  }

  /**
   * \brief Returns the reference frame
   *
   *
   * @return The reference frame
   */
  const std::string& getReferenceFrame() const
  {
    return constraint_frame_id_;
  }

  /**
   * \brief If enabled and the specified frame is a mobile frame,
   * return true.  Otherwise, returns false.
   *
   *
   * @return Whether a mobile reference frame is being employed
   */
  bool mobileReferenceFrame() const
  {
    if (!enabled())
      return false;
    return mobile_frame_;
  }

protected:
  Eigen::Vector3d offset_;                         /**< \brief The target offset */
  bool has_offset_;                                /**< \brief Whether the offset is substantially different than 0.0 */
  std::vector<bodies::BodyPtr> constraint_region_; /**< \brief The constraint region vector */
  /** \brief The constraint region pose vector. All isometries are guaranteed to be valid. */
  EigenSTL::vector_Isometry3d constraint_region_pose_;
  bool mobile_frame_;                         /**< \brief Whether or not a mobile frame is employed*/
  std::string constraint_frame_id_;           /**< \brief The constraint frame id */
  const moveit::core::LinkModel* link_model_; /**< \brief The link model constraint subject */
};

MOVEIT_CLASS_FORWARD(VisibilityConstraint);  // Defines VisibilityConstraintPtr, ConstPtr, WeakPtr... etc

/**
 * \brief Class for constraints on the visibility relationship between
 * a sensor and a target.
 *
 * Visibility constraints are the most complicated kinematic
 * constraint type.  They are designed to test whether a visibility
 * relationship is maintained in a given state.  For the visibility
 * relationship to be maintained, a sensor must have an unimpeded view
 * of a target - this is useful, for instance, if one wants to test
 * whether a robot can see its hand with a given sensor in a given
 * state.  The mechanism that is used to test the constraint is a
 * combination of geometric checks, and then a collision check that
 * tests whether a cone that connects the sensor and the target is
 * entirely unobstructed by the robot's links.
 *
 * The constraint consists of a sensor pose, a target pose, a few
 * parameters that govern the shape of the target, and a few
 * parameters that allow finer control over the geometry of the check.
 * There are two general ways that this constraint can be used.  The
 * first way to leave max_view_angle and max_range_angle in the
 * VisibilityConstraint message as 0.0.  In this case, the only check
 * that is performed is the collision check against the cone.  This
 * check assumes that the sensor can see in all directions - the
 * orientation of the sensor has no effect on the creation of the
 * cone.  It is important to note that the sensor and the target poses
 * must not result in collision - if they are associated with sensor
 * frames or robot links they must be specified to be outside the
 * robot's body or all states will violate the Visibility Constraint.
 * For an example of a visualization of a visibility constraint
 * (produced using the getMarkers member function), see this image:
 *
 * \image html fingertip_cone.png "Visibility constraint satisfied as cone volume is clear"
 *
 * The cone is shown in red, and the arrows show normals.  This
 * visibility constraint uses a sensor pose (the narrow end of the
 * cone ) of the PR2's narrow_stereo_optical_frame, except shifted
 * along the Z axis such that the pose is outside of the robot's head
 * and doesn't collide.  The cone is allowed to collide with the
 * actual sensor associated with the header frame, just not with any
 * other links.  The target pose is a 5 cm circle offset from the
 * l_gripper_r_finger_tip_link, again so as not to collide - again,
 * the cone can collide with the target link, but now with any other
 * links.  In the indicated state the cone is collision free and thus
 * satisfies the collision component of the visibility constraint.  In
 * this image, however, the right arm intersects the cone, violating
 * the visibility constraint:
 *
 * \image html fingertip_collision.png "Visibility constraint violated as right arm is within the cone"
 *
 * Note that both the target and the sensor frame can change with the
 * robot's state.
 *
 * The collision check essentially asks whether or not the volume
 * between the sensor pose and the target pose are clear, but that's
 * only one aspect of visibility we may care about. The visibility
 * constraint also allows for some geometric reasoning about the
 * relationship between the sensor or the target - this allows
 * information such as approximate field of view of the sensor to be
 * factored into the constraint.  To use this reasoning, the
 * sensor_view_direction of the field should first be set - this
 * specifies which axis of the sensor pose frame is actually pointing.
 * The system assumes that the the sensor pose has been set up such
 * that a single of the axes is pointing directly out of the sensor.
 * Once this value is set correctly, one or both of the max_view_angle
 * and the max_range_angle values can be set.  Setting a positive
 * max_view_angle will constrain the sensor along the specified axis
 * and the target along its Z axis to be pointing at each other.
 * Practically speaking, this ensures that the sensor has sufficient
 * visibility to the front of the target - if the target is pointing
 * the opposite direction, or is too steeply perpindicular to the
 * target, then the max_view_angle part of the constraint will be
 * violated.  The getMarkers function can again help explain this -
 * the view angle is the angular difference between the blue arrow
 * associated with the sensor and the red arrow associated with the
 * target.

 * \image html exact_opposites.png "Max view angle is evaluated at 0.0"
 * \image html fourty_five.png "Max view angle evaluates around pi/4"
 * \image html perpindicular.png "Max view angle evaluates at pi/2, the maximum"
 * \image html other_side.png "Sensor pointed at wrong side of target, will violate constraint as long as max_view_angle
 > 0.0"
 *
 * If constraining the target to be within the field of view of the
 * sensor is the goal, then the max_range_angle can be set.  The range
 * angle is calculated as the angle between the origins of the sensor
 * and the target frame - the orientations are unimportant.  In the
 * above images, the range angle is always 0, as the target's center
 * is directly lined up with the blue arrow.  In this image, however,
 * the view angle is evaluated at 0.0, while the range angle is
 * evaluated at .65.
 *
 * \image html range_angle.png "Range angle is high, so only sensors with wide fields of view would see the target"
 *
 * By limiting the max_range_angle, you can constrain the target to be
 * within the field of view of the sensor.  Max_view_angle and
 * max_range_angle can be used at once.
 */
class VisibilityConstraint : public KinematicConstraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  /**
   * \brief Constructor
   *
   * @param [in] model The kinematic model used for constraint evaluation
   */
  VisibilityConstraint(const moveit::core::RobotModelConstPtr& model);

  /**
   * \brief Configure the constraint based on a
   * moveit_msgs::VisibilityConstraint
   *
   * For the configure command to be successful, the target radius
   * must be non-zero (negative values will have the absolute value
   * taken).  If cone sides are less than 3, a value of 3 will be used.
   *
   * @param [in] vc moveit_msgs::VisibilityConstraint for configuration
   *
   * @return True if constraint can be configured from vc
   */
  bool configure(const moveit_msgs::VisibilityConstraint& vc, const moveit::core::Transforms& tf);

  /**
   * \brief Check if two constraints are the same.
   *
   * For visibility constraints this means that:
   * \li The types are the same,
   * \li The target frame ids are the same
   * \li The sensor frame ids are the same
   * \li The cone sides number is the same
   * \li The sensor view directions are the same
   * \li The max view angles and target radii are within the margin
   * \li The sensor and target poses are within the margin, as computed by taking the norm of the difference.
   *
   * @param [in] other The other constraint to test
   * @param [in] margin The margin to apply to all values associated with constraint
   *
   * @return True if equal, otherwise false
   */
  bool equal(const KinematicConstraint& other, double margin) const override;
  void clear() override;

  /**
   * \brief Gets a trimesh shape representing the visibility cone
   *
   * @param [in] state The state from which to produce the cone
   *
   * @return The shape associated with the cone
   */
  shapes::Mesh* getVisibilityCone(const moveit::core::RobotState& state) const;

  /**
   * \brief Adds markers associated with the visibility cone, sensor
   * and target to the visualization array
   *
   * The visibility cone and two arrows - a blue array that issues
   * from the sensor_view_direction of the sensor, and a red arrow the
   * issues along the Z axis of the the target frame.
   *
   * @param [in] state The state from which to produce the markers
   * @param [out] markers The marker array to which the markers will be added
   */
  void getMarkers(const moveit::core::RobotState& state, visualization_msgs::MarkerArray& markers) const;

  bool enabled() const override;
  ConstraintEvaluationResult decide(const moveit::core::RobotState& state, bool verbose = false) const override;
  void print(std::ostream& out = std::cout) const override;

protected:
  /**
   * \brief Function that gets passed into collision checking to allow some collisions.
   *
   * The cone object is allowed to touch either the sensor or the header frame, but not anything else
   *
   * @param [in] contact The contact in question
   *
   * @return True if the collision is allowed, otherwise false
   */
  bool decideContact(const collision_detection::Contact& contact) const;

  collision_detection::CollisionEnvPtr collision_env_; /**< \brief A copy of the collision robot maintained for
                                                              collision checking the cone against robot links */
  bool mobile_sensor_frame_;      /**< \brief True if the sensor is a non-fixed frame relative to the transform frame */
  bool mobile_target_frame_;      /**< \brief True if the target is a non-fixed frame relative to the transform frame */
  std::string target_frame_id_;   /**< \brief The target frame id */
  std::string sensor_frame_id_;   /**< \brief The sensor frame id */
  Eigen::Isometry3d sensor_pose_; /**< \brief The sensor pose transformed into the transform frame */
  int sensor_view_direction_;     /**< \brief Storage for the sensor view direction */
  Eigen::Isometry3d target_pose_; /**< \brief The target pose transformed into the transform frame */
  unsigned int cone_sides_;       /**< \brief Storage for the cone sides  */
  EigenSTL::vector_Vector3d points_; /**< \brief A set of points along the base of the circle */
  double target_radius_;             /**< \brief Storage for the target radius */
  double max_view_angle_;            /**< \brief Storage for the max view angle */
  double max_range_angle_;           /**< \brief Storage for the max range angle */
};

MOVEIT_CLASS_FORWARD(KinematicConstraintSet);  // Defines KinematicConstraintSetPtr, ConstPtr, WeakPtr... etc

/**
 * \brief A class that contains many different constraints, and can
 * check RobotState *versus the full set.  A set is satisfied if
 * and only if all constraints are satisfied.
 *
 * The set may contain any number of different kinds of constraints.
 * All constraints, including invalid ones, are stored internally.
 */
class KinematicConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  /**
   * \brief Constructor
   *
   * @param [in] model The kinematic model used for constraint evaluation
   */
  KinematicConstraintSet(const moveit::core::RobotModelConstPtr& model) : robot_model_(model)
  {
  }

  ~KinematicConstraintSet()
  {
    clear();
  }

  /** \brief Clear the stored constraints */
  void clear();

  /**
   * \brief Add all known constraints
   *
   * @param [in] c A message potentially contain vectors of constraints of add types
   *
   * @return Whether or not all constraints could be successfully
   * configured given the contents of the message.  The
   * KinematicConstraintSet can still be used even if the addition
   * returns false.
   */
  bool add(const moveit_msgs::Constraints& c, const moveit::core::Transforms& tf);

  /**
   * \brief Add a vector of joint constraints
   *
   * @param [in] jc A vector of joint constraints
   *
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::JointConstraint>& jc);

  /**
   * \brief Add a vector of position constraints
   *
   * @param [in] pc A vector of position constraints
   *
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::PositionConstraint>& pc, const moveit::core::Transforms& tf);

  /**
   * \brief Add a vector of orientation constraints
   *
   * @param [in] oc A vector of orientation constraints
   *
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::OrientationConstraint>& oc, const moveit::core::Transforms& tf);

  /**
   * \brief Add a vector of visibility constraints
   *
   * @param [in] vc A vector of visibility constraints
   *
   * @return Will return true only if all constraints are valid, and false otherwise
   */
  bool add(const std::vector<moveit_msgs::VisibilityConstraint>& vc, const moveit::core::Transforms& tf);

  /**
   * \brief Determines whether all constraints are satisfied by state,
   * returning a single evaluation result
   *
   * @param [in] state The state to test
   * @param [in] verbose Whether or not to make each constraint give debug output
   *
   * @return A single constraint evaluation result, where it will
   * report satisfied only if all constraints are satisfied, and with
   * a distance that is the sum of all individual distances.
   */
  ConstraintEvaluationResult decide(const moveit::core::RobotState& state, bool verbose = false) const;

  /**
   *
   * \brief Determines whether all constraints are satisfied by state,
   * returning a vector of results through a parameter in addition to
   * a summed result.
   *
   * @param [in] state The state to test
   *
   * @param [out] results The individual results from constraint
   * evaluation on each constraint contained in the set.
   *
   * @param [in] verbose Whether to print the results of each constraint
   * check.
   *
   * @return A single constraint evaluation result, where it will
   * report satisfied only if all constraints are satisfied, and with
   * a distance that is the sum of all individual distances.
   */
  ConstraintEvaluationResult decide(const moveit::core::RobotState& state,
                                    std::vector<ConstraintEvaluationResult>& results, bool verbose = false) const;

  /**
   * \brief Whether or not another KinematicConstraintSet is equal to
   * this one.
   *
   * Equality means that for each constraint in this set there is a
   * constraint in the other set for which equal() is true with the
   * given margin.  Multiple constraints in this set can be matched to
   * single constraints in the other set.  Some constraints in the
   * other set may not be matched to constraints in this set.
   *
   * @param [in] other The other set against which to test
   * @param [in] margin The margin to apply to all individual constraint equality tests
   *
   * @return True if all constraints are matched, false otherwise
   */
  bool equal(const KinematicConstraintSet& other, double margin) const;

  /**
   * \brief Print the constraint data
   *
   * @param [in] out The file stream for printing
   */
  void print(std::ostream& out = std::cout) const;

  /**
   * \brief Get all position constraints in the set
   *
   *
   * @return All position constraints
   */
  const std::vector<moveit_msgs::PositionConstraint>& getPositionConstraints() const
  {
    return position_constraints_;
  }

  /**
   * \brief Get all orientation constraints in the set
   *
   *
   * @return All orientation constraints
   */
  const std::vector<moveit_msgs::OrientationConstraint>& getOrientationConstraints() const
  {
    return orientation_constraints_;
  }

  /**
   * \brief Get all joint constraints in the set
   *
   *
   * @return All joint constraints
   */
  const std::vector<moveit_msgs::JointConstraint>& getJointConstraints() const
  {
    return joint_constraints_;
  }

  /**
   * \brief Get all visibility constraints in the set
   *
   *
   * @return All visibility constraints
   */
  const std::vector<moveit_msgs::VisibilityConstraint>& getVisibilityConstraints() const
  {
    return visibility_constraints_;
  }

  /**
   * \brief Get all constraints in the set
   *
   *
   * @return All constraints in a single message
   */
  const moveit_msgs::Constraints& getAllConstraints() const
  {
    return all_constraints_;
  }

  /**
   * \brief Returns whether or not there are any constraints in the set
   *
   *
   * @return True if there are no constraints, otherwise false.
   */
  bool empty() const
  {
    return kinematic_constraints_.empty();
  }

protected:
  moveit::core::RobotModelConstPtr robot_model_; /**< \brief The kinematic model used for by the Set */
  std::vector<KinematicConstraintPtr>
      kinematic_constraints_; /**<  \brief Shared pointers to all the member constraints */

  std::vector<moveit_msgs::JointConstraint> joint_constraints_; /**<  \brief Messages corresponding to all internal
                                                                   joint constraints */
  std::vector<moveit_msgs::PositionConstraint> position_constraints_;       /**<  \brief Messages corresponding to all
                                                                               internal position constraints */
  std::vector<moveit_msgs::OrientationConstraint> orientation_constraints_; /**<  \brief Messages corresponding to all
                                                                               internal orientation constraints */
  std::vector<moveit_msgs::VisibilityConstraint> visibility_constraints_;   /**<  \brief Messages corresponding to all
                                                                               internal visibility constraints */
  moveit_msgs::Constraints all_constraints_; /**<  \brief Messages corresponding to all internal constraints */
};
}  // namespace kinematic_constraints
