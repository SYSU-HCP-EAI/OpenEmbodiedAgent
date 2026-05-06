#pragma once

#include <moveit/planning_interface/planning_interface.h>

#include <trajopt_interface/problem_description.h>
#include <trajopt_interface/trajopt_interface.h>

namespace trajopt_interface
{
MOVEIT_CLASS_FORWARD(TrajOptPlanningContext);  // Defines TrajOptPlanningContextPtr, ConstPtr, WeakPtr... etc

class TrajOptPlanningContext : public planning_interface::PlanningContext
{
public:
  TrajOptPlanningContext(const std::string& name, const std::string& group,
                         const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh);
  ~TrajOptPlanningContext() override
  {
  }

  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;
  void clear() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;

  TrajOptInterfacePtr trajopt_interface_;
};
}  // namespace trajopt_interface
