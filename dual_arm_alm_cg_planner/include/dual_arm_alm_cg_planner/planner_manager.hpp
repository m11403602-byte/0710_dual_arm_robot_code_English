// =====================================================================
// planner_manager.hpp — Layer 3: MoveIt2 PlannerManager/PlanningContext
// =====================================================================
//   MoveIt2 plugin interface wrapper (a thin wrapper connecting AvoidanceSystem to MoveIt)
//   ⚠ the algorithm works entirely in degrees; the MoveIt interface is entirely in radians -> convert at the solve() boundary
//   ⚠ joint names: Arm A big_joint_1~6, Arm B small_joint_1~6 (adjust per your SRDF)
// =====================================================================
#ifndef DUAL_ARM_AVOIDANCE_PLANNER_PLANNER_MANAGER_HPP
#define DUAL_ARM_AVOIDANCE_PLANNER_PLANNER_MANAGER_HPP

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/rclcpp.hpp>

#include "dual_arm_alm_cg_planner/avoidance_system.hpp"
#include <string>

namespace dual_arm_alm_cg_planner
{

// =====================================================================
// PlanningContext: the execution unit for a single planning request (solve)
// =====================================================================
class DualArmPlanningContext : public planning_interface::PlanningContext
{
public:
  DualArmPlanningContext(const std::string& name, const std::string& group,
                         const rclcpp::Node::SharedPtr& node)
    : planning_interface::PlanningContext(name, group), node_(node) {}

  // main planning entry point
  bool solve(planning_interface::MotionPlanResponse& res) override;
  // DetailedResponse delegates to the above (legacy mode)
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override { return true; }   // this planner does not support mid-run termination
  void clear() override {}

  // parameters passed in from yaml
  double path_weight_         = 0.5;
  double danger_threshold_    = 0.4;
  double collision_tolerance_ = 0.1;
  double fix_tolerance_       = 0.1;
  int    max_refinement_iter_ = 15;
  double smooth_w_            = 0.3;
  double smooth_w_H_          = 1.0;
  double smooth_w_T_          = 1.0;
  double smooth_w_neighbor_   = 1.0;
  std::string joint_prefix_A_ = "big_joint_";
  std::string joint_prefix_B_ = "small_joint_";
  // time parameterization
  bool   time_optimal_      = true;
  double path_total_time_   = 20.0;
  double min_time_interval_ = 0.05;

  // [NEW] diagnostic output (yaml switches, all disabled by default)
  std::string export_csv_prefix_;       // non-empty → export CSV after each planning run
  int         export_level_   = 0;      // [NEW] 0=no export, 1=standard 6 files, 2=full 9 files
  // [NEW] ALM parameters (default = CG recommended values)
  double alm_mu0_ = 10.0; double alm_c0_ = 5.0; double alm_c_max_ = 2000.0;
  double alm_beta_c_ = 2.0; double alm_gamma_v_ = 0.5;
  int    alm_k_outer_ = 50;  int alm_k_inner_ = 200;

private:
  rclcpp::Node::SharedPtr node_;
};

// =====================================================================
// PlannerManager: the plugin entry point (this is what MoveIt loads)
// =====================================================================
class DualArmAlmCgPlannerManager : public planning_interface::PlannerManager
{
public:
  DualArmAlmCgPlannerManager() = default;
  ~DualArmAlmCgPlannerManager() override = default;

  // ⚠ parameter order: (model, node, ns) — validated in the legacy version
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  std::string getDescription() const override { return "Dual-Arm Avoidance Planner (CG)"; }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

private:
  // re-read all parameters from the parameter server (called both in initialize and on every getPlanningContext,
  //   so that yaml/rqt edits take effect on the next planning run). const because getPlanningContext is const.
  void load_parameters() const;

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string parameter_namespace_;
  // mutable: load_parameters() is const but must update these values
  mutable double path_weight_         = 0.5;
  mutable double danger_threshold_    = 0.4;
  mutable double collision_tolerance_ = 0.1;
  mutable double fix_tolerance_       = 0.1;
  mutable int    max_refinement_iter_ = 15;
  mutable double smooth_w_            = 0.3;
  mutable double smooth_w_H_          = 1.0;
  mutable double smooth_w_T_          = 1.0;
  mutable double smooth_w_neighbor_   = 1.0;
  mutable std::string joint_prefix_A_ = "big_joint_";
  mutable std::string joint_prefix_B_ = "small_joint_";
  mutable bool   time_optimal_      = true;
  mutable double path_total_time_   = 20.0;
  mutable double min_time_interval_ = 0.05;
  mutable std::string export_csv_prefix_;       // [NEW] non-empty → export CSV after planning
  mutable int         export_level_   = 0;      // [NEW] export level
  mutable double alm_mu0_ = 10.0; mutable double alm_c0_ = 5.0; mutable double alm_c_max_ = 2000.0;
  mutable double alm_beta_c_ = 2.0; mutable double alm_gamma_v_ = 0.5;
  mutable int    alm_k_outer_ = 50;  mutable int alm_k_inner_ = 200;
};

}  // namespace dual_arm_alm_cg_planner

#endif
