#pragma once
#include <trajopt/common.hpp>
#include <trajopt/json_marshal.hpp>
#include <trajopt_sco/optimizers.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

#include <memory>

namespace trajopt_interface
{
/**
@brief Used to apply cost/constraint to joint-space velocity

Term is applied to every step between first_step and last_step. It applies two limits, upper_limits/lower_limits,
to the joint velocity subject to the following cases.

* term_type = TT_COST
** upper_limit = lower_limit = 0 - Cost is applied with a SQUARED error scaled for each joint by coeffs
** upper_limit != lower_limit - 2 hinge costs are applied scaled for each joint by coeffs. If velocity < upper_limit and
velocity > lower_limit, no penalty.

* term_type = TT_CNT
** upper_limit = lower_limit = 0 - Equality constraint is applied
** upper_limit != lower_limit - 2 Inequality constraints are applied. These are both satisfied when velocity <
upper_limit and velocity > lower_limit

Note: coeffs, upper_limits, and lower_limits are optional. If a term is not given it will default to 0 for all joints.
If one value is given, this will be broadcast to all joints.

Note: Velocity is calculated numerically using forward finite difference

\f{align*}{
  cost = \sum_{t=0}^{T-2} \sum_j c_j (x_{t+1,j} - x_{t,j})^2
\f}
where j indexes over DOF, and \f$c_j\f$ are the coeffs.
*/

struct TermInfo;
MOVEIT_CLASS_FORWARD(TermInfo);  // Defines TermInfoPtr, ConstPtr, WeakPtr... etc

class TrajOptProblem;
MOVEIT_CLASS_FORWARD(TrajOptProblem);  // Defines TrajOptProblemPtr, ConstPtr, WeakPtr... etc

struct JointPoseTermInfo;
MOVEIT_CLASS_FORWARD(JointPoseTermInfo);  // Defines JointPoseTermInfoPtr, ConstPtr, WeakPtr... etc

struct CartPoseTermInfo;
MOVEIT_CLASS_FORWARD(CartPoseTermInfo);  // Defines CartPoseTermInfoPtr, ConstPtr, WeakPtr... etc

struct JointVelTermInfo;
MOVEIT_CLASS_FORWARD(JointVelTermInfo);  // Defines JointVelTermInfoPtr, ConstPtr, WeakPtr... etc

struct ProblemInfo;
TrajOptProblemPtr ConstructProblem(const ProblemInfo&);

enum TermType
{
  TT_COST = 0x1,      // 0000 0001
  TT_CNT = 0x2,       // 0000 0010
  TT_USE_TIME = 0x4,  // 0000 0100
};

struct BasicInfo
{
  /** @brief If true, first time step is fixed with a joint level constraint
      If this is false, the starting point of the trajectory will
      not be the current position of the robot. The use case example is: if we are trying to execute a process like
      sanding the critical part which is the actual process path not how we get to the start of the process path. So we
     plan the
      process path first leaving the start free to hopefully get the most optimal and then we plan from the current
     location with
      start fixed to the start of the process path. It depends on what we want the default behavior to be
   */
  bool start_fixed;
  /** @brief Number of time steps (rows) in the optimization matrix */
  int n_steps;
  sco::IntVec dofs_fixed;        // optional
  sco::ModelType convex_solver;  // which convex solver to use

  /** @brief If true, the last column in the optimization matrix will be 1/dt */
  bool use_time = false;
  /** @brief The upper limit of 1/dt values allowed in the optimization*/
  double dt_upper_lim = 1.0;
  /** @brief The lower limit of 1/dt values allowed in the optimization*/
  double dt_lower_lim = 1.0;
};

/**
Initialization info read from json
*/
struct InitInfo
{
  /** @brief Methods of initializing the optimization matrix. This defines how robot moves from the current
      state to the start state

    STATIONARY: Initializes all joint values to the initial value (the current value in the env)
    JOINT_INTERPOLATED: Linearly interpolates between initial value and the joint position specified in InitInfo.data
    GIVEN_TRAJ: Initializes the matrix to a given trajectory

    In all cases the dt column (if present) is appended the selected method is defined.
 */
  enum Type
  {
    STATIONARY,
    JOINT_INTERPOLATED,
    GIVEN_TRAJ,
  };
  /** @brief Specifies the type of initialization to use */
  Type type;
  /** @brief Data used during initialization. Use depends on the initialization selected. This data will be used
      to create initialization matrix. We need to give the goal information to this init info
   */
  trajopt::TrajArray data;
  //  Eigen::VectorXd data_vec;
  //  trajopt::TrajArray data_trajectory;
  /** @brief Default value the final column of the optimization is initialized too if time is being used */
  double dt = 1.0;
};

/**
When cost or constraint element of JSON doc is read, one of these guys gets
constructed to hold the parameters.
Then it later gets converted to a Cost object by the addObjectiveTerms method
*/
struct TermInfo
{
  std::string name;
  int term_type;
  int getSupportedTypes()
  {
    return supported_term_types_;
  }
  //  virtual void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) = 0;
  virtual void addObjectiveTerms(TrajOptProblem& prob) = 0;

  static TermInfoPtr fromName(const std::string& type);

  /**
   * Registers a user-defined TermInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  using MakerFunc = TermInfoPtr (*)(void);
  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~TermInfo() = default;

protected:
  TermInfo(int supported_term_types) : supported_term_types_(supported_term_types)
  {
  }

private:
  static std::map<std::string, MakerFunc> name_to_maker_;
  int supported_term_types_;
};

struct ProblemInfo
{
public:
  BasicInfo basic_info;
  sco::BasicTrustRegionSQPParameters opt_info;
  std::vector<TermInfoPtr> cost_infos;
  std::vector<TermInfoPtr> cnt_infos;
  InitInfo init_info;

  planning_scene::PlanningSceneConstPtr planning_scene;
  std::string planning_group_name;

  ProblemInfo(planning_scene::PlanningSceneConstPtr ps, const std::string& pg)
    : planning_scene(ps), planning_group_name(pg)
  {
  }
};

/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TrajOptProblem : public sco::OptProb
{
public:
  TrajOptProblem();
  TrajOptProblem(const ProblemInfo& problem_info);
  virtual ~TrajOptProblem() = default;
  /** @brief Returns the values of the specified joints (start_col to num_col) for the specified timestep i.*/
  sco::VarVector GetVarRow(int i, int start_col, int num_col)
  {
    return matrix_traj_vars.rblock(i, start_col, num_col);
  }
  /** @brief Returns the values of all joints for the specified timestep i.*/
  sco::VarVector GetVarRow(int i)
  {
    return matrix_traj_vars.row(i);
  }
  /** @brief Returns the value of the specified joint j for the specified timestep i.*/
  sco::Var& GetVar(int i, int j)
  {
    return matrix_traj_vars.at(i, j);
  }
  trajopt::VarArray& GetVars()
  {
    return matrix_traj_vars;
  }
  /** @brief Returns the number of steps in the problem. This is the number of rows in the optimization matrix.*/
  int GetNumSteps()
  {
    return matrix_traj_vars.rows();
  }
  /** @brief Returns the problem DOF. This is the number of columns in the optization matrix.
   * Note that this is not necessarily the same as the kinematic DOF.*/
  int GetNumDOF()
  {
    return matrix_traj_vars.cols();
  }
  /** @brief Returns the kinematic DOF of the active joint model group
   */
  int GetActiveGroupNumDOF()
  {
    return dof_;
  }
  planning_scene::PlanningSceneConstPtr GetPlanningScene()
  {
    return planning_scene_;
  }
  void SetInitTraj(const trajopt::TrajArray& x)
  {
    matrix_init_traj = x;
  }
  trajopt::TrajArray GetInitTraj()
  {
    return matrix_init_traj;
  }
  //  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);
  /** @brief Returns TrajOptProb.has_time */
  bool GetHasTime()
  {
    return has_time;
  }
  /** @brief Sets TrajOptProb.has_time  */
  void SetHasTime(bool tmp)
  {
    has_time = tmp;
  }

private:
  /** @brief If true, the last column in the optimization matrix will be 1/dt */
  bool has_time;
  /** @brief is the matrix holding the joint values of the trajectory for all timesteps */
  trajopt::VarArray matrix_traj_vars;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string planning_group_;
  /** @brief Kinematic DOF of the active joint model group  */
  int dof_;
  trajopt::TrajArray matrix_init_traj;
};

/** @brief This term is used when the goal frame is fixed in cartesian space

  Set term_type == TT_COST or TT_CNT for cost or constraint.
*/
struct CartPoseTermInfo : public TermInfo
{
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Timestep at which to apply term */
  int timestep;
  /** @brief  Cartesian position */
  Eigen::Vector3d xyz;
  /** @brief Rotation quaternion */
  Eigen::Vector4d wxyz;
  /** @brief coefficients for position and rotation */
  Eigen::Vector3d pos_coeffs, rot_coeffs;
  /** @brief Link which should reach desired pose */
  std::string link;
  /** @brief Static transform applied to the link */
  Eigen::Isometry3d tcp;

  CartPoseTermInfo();

  /** @brief Used to add term to pci from json */
  //  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void addObjectiveTerms(TrajOptProblem& prob) override;

  static TermInfoPtr create()
  {
    TermInfoPtr out(new CartPoseTermInfo());
    return out;
  }
};

/**
  \brief Joint space position cost
    Position operates on a single point (unlike velocity, etc). This is b/c the primary usecase is joint-space
    position waypoints

  \f{align*}{
  \sum_i c_i (x_i - xtarg_i)^2
  \f}
  where \f$i\f$ indexes over dof and \f$c_i\f$ are coeffs
 */
struct JointPoseTermInfo : public TermInfo
{
  /** @brief Vector of coefficients that scale the cost. Size should be the DOF of the system. Default: vector of 0's*/
  trajopt::DblVec coeffs;
  /** @brief Vector of position targets. This is a required value. Size should be the DOF of the system */
  trajopt::DblVec targets;
  /** @brief Vector of position upper limits. Size should be the DOF of the system. Default: vector of 0's*/
  trajopt::DblVec upper_tols;
  /** @brief Vector of position lower limits. Size should be the DOF of the system. Default: vector of 0's*/
  trajopt::DblVec lower_tols;
  /** @brief First time step to which the term is applied. Default: 0 */
  int first_step = 0;
  /** @brief Last time step to which the term is applied. Default: prob.GetNumSteps() - 1*/
  int last_step = -1;

  /** @brief Initialize term with it's supported types */
  JointPoseTermInfo() : TermInfo(TT_COST | TT_CNT | TT_USE_TIME)
  {
  }

  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void addObjectiveTerms(TrajOptProblem& prob) override;

  static TermInfoPtr create()
  {
    TermInfoPtr out(new JointPoseTermInfo());
    return out;
  }
};

struct JointVelTermInfo : public TermInfo
{
  /** @brief Vector of coefficients that scale the cost. Size should be the DOF of the system. Default: vector of 0's*/
  trajopt::DblVec coeffs;
  /** @brief Vector of velocity targets. This is a required value. Size should be the DOF of the system */
  trajopt::DblVec targets;
  /** @brief Vector of velocity upper limits. Size should be the DOF of the system. Default: vector of 0's*/
  trajopt::DblVec upper_tols;
  /** @brief Vector of velocity lower limits. Size should be the DOF of the system. Default: vector of 0's*/
  trajopt::DblVec lower_tols;
  /** @brief First time step to which the term is applied. Default: 0*/
  int first_step = 0;
  /** @brief Last time step to which the term is applied. Default: prob.GetNumSteps() - 1*/
  int last_step = -1;

  /** @brief Initialize term with it's supported types */
  JointVelTermInfo() : TermInfo(TT_COST | TT_CNT | TT_USE_TIME)
  {
  }

  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void addObjectiveTerms(TrajOptProblem& prob) override;

  static TermInfoPtr create()
  {
    TermInfoPtr out(new JointVelTermInfo());
    return out;
  }
};

void generateInitialTrajectory(const ProblemInfo& pci, const std::vector<double>& current_joint_values,
                               trajopt::TrajArray& init_traj);

}  // namespace trajopt_interface
