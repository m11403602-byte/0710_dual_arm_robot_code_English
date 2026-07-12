// =====================================================================
// planner_manager.cpp — MoveIt2 plugin implementation
// =====================================================================
#include "dual_arm_alm_cg_planner/planner_manager.hpp"
#include <exception>   // [REVISE] export try/catch

#include <pluginlib/class_list_macros.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <chrono>
#include <cmath>

namespace dual_arm_alm_cg_planner
{

// =====================================================================
// PlanningContext::solve — the main planning flow
// =====================================================================
bool DualArmPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // 1. timing
  auto start_time = std::chrono::steady_clock::now();

  // 2. check the validity of the goal constraints
  if (request_.goal_constraints.empty() ||
      request_.goal_constraints[0].joint_constraints.empty()) {
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // 3. extract start/goal from MoveIt
  moveit::core::RobotState start_state = planning_scene_->getCurrentState();
  moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
  moveit::core::RobotState goal_state(start_state);
  for (const auto& jc : request_.goal_constraints[0].joint_constraints)
    goal_state.setJointPositions(jc.joint_name, &jc.position);
  goal_state.update();

  // 4. convert to Eigen matrices (⚠ MoveIt uses radians, the algorithm uses degrees)
  //    the joint-name prefixes are determined by the yaml parameters joint_prefix_A/B (per SRDF)
  Eigen::MatrixXd A_wp(2, 6), B_wp(2, 6);
  for (int j = 0; j < 6; ++j) {
    const std::string jA = joint_prefix_A_ + std::to_string(j + 1);
    const std::string jB = joint_prefix_B_ + std::to_string(j + 1);
    A_wp(0, j) = start_state.getJointPositions(jA)[0] * 180.0 / M_PI;
    A_wp(1, j) = goal_state.getJointPositions(jA)[0]  * 180.0 / M_PI;
    B_wp(0, j) = start_state.getJointPositions(jB)[0] * 180.0 / M_PI;
    B_wp(1, j) = goal_state.getJointPositions(jB)[0]  * 180.0 / M_PI;
  }

  // 5. call the avoidance system (core library) — passing in all yaml-tunable parameters
  AvoidanceSystem optimizer(A_wp, B_wp, path_weight_, danger_threshold_,
                            collision_tolerance_, fix_tolerance_, max_refinement_iter_,
                            smooth_w_, smooth_w_H_, smooth_w_T_, smooth_w_neighbor_);
  optimizer.set_alm_params(alm_mu0_, alm_c0_, alm_c_max_,
                           alm_beta_c_, alm_gamma_v_);   // [NEW] yaml → ALM parameter injection
  optimizer.set_alm_iters(alm_k_outer_, alm_k_inner_);   // [NEW] yaml → iteration-limit injection
  optimizer.run_optimization();

  // [NEW] CSV export tool (acts only when export_csv_prefix is non-empty)
  //   ⚠ deliberately invoked outside the "pure path-planning time" timing region — disk I/O must not pollute the comparison data
  auto export_csv_if_enabled = [&]() {
    if (export_csv_prefix_.empty()) { return; }
    // [REVISE] wrap the consolidated export in try/catch: export is a diagnostic side effect; on failure (e.g., permissions/disk) it only warns and never brings down planning
    try {
      optimizer.export_unified(export_csv_prefix_, export_level_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(),
                  "[CSV] export failed, planning unaffected (check whether export_csv_prefix is writable): %s", e.what());
    }
  };

  if (optimizer.has_collision()) {
    export_csv_if_enabled();   // [NEW] export even on failure (debugging material; the failure path has no timing semantics)
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  // 6. convert the result back to a RobotTrajectory (⚠ degrees -> radians)
  //    first fill in equal-interval times (later re-parameterized per time_optimal)
  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
      start_state.getRobotModel(), getGroupName());
  const Trajectory& opt = optimizer.get_optimized_trajectory();
  for (int i = 0; i < opt.pos.rows(); ++i) {
    moveit::core::RobotState wp = start_state;
    for (int j = 0; j < 6; ++j) {
      double rad_A = opt.posA(i, j) * M_PI / 180.0;
      double rad_B = opt.posB(i, j) * M_PI / 180.0;
      const std::string jA = joint_prefix_A_ + std::to_string(j + 1);
      const std::string jB = joint_prefix_B_ + std::to_string(j + 1);
      wp.setJointPositions(jA, &rad_A);
      wp.setJointPositions(jB, &rad_B);
    }
    wp.update();
    trajectory->addSuffixWayPoint(wp, 0.1);   // temporary fill, reset below
  }

  // 7. record the pure path-planning time (only avoidance run_optimization + trajectory conversion, excluding time parameterization)
  auto plan_end = std::chrono::steady_clock::now();
  const double pure_plan_time = std::chrono::duration<double>(plan_end - start_time).count();
  std::cout << "\n[pure path-planning time] " << pure_plan_time << " s (avoidance only)\n";

  export_csv_if_enabled();   // [NEW] write to disk only after timing ends — keep the planning-time data clean

  // 8. time parameterization (handled inside the plugin; the TOTG adapter has been removed from the yaml to avoid duplication)
  const int n_wp = static_cast<int>(trajectory->getWayPointCount());
  if (time_optimal_) {
    // ===== TOTG: time-optimal parameterization (compute timestamps per velocity/acceleration limits) =====
    //   read the scaling (slider) from the RViz / MotionPlanRequest, default to 1.0 by default/on error
    double vel_scale = request_.max_velocity_scaling_factor;
    double acc_scale = request_.max_acceleration_scaling_factor;
    if (vel_scale <= 0.0 || vel_scale > 1.0) vel_scale = 1.0;
    if (acc_scale <= 0.0 || acc_scale > 1.0) acc_scale = 1.0;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool ok = totg.computeTimeStamps(*trajectory, vel_scale, acc_scale);
    if (!ok) {
      std::cout << "  [WARN] TOTG time parameterization failed, falling back to equal intervals\n";
      for (int i = 1; i < n_wp; ++i)
        trajectory->setWayPointDurationFromPrevious(i, min_time_interval_);
    } else {
      std::cout << "  [time parameterization] TOTG (time_optimal=true, vel_scale="
                << vel_scale << ", acc_scale=" << acc_scale << ")\n";
    }
  } else {
    // ===== custom equal-interval: dt = path_total_time/(n-1), but not smaller than min_time_interval =====
    double dt = (n_wp > 1) ? (path_total_time_ / (n_wp - 1)) : min_time_interval_;
    if (dt < min_time_interval_) dt = min_time_interval_;   // protected by the minimum interval
    for (int i = 1; i < n_wp; ++i)
      trajectory->setWayPointDurationFromPrevious(i, dt);
    std::cout << "  [time parameterization] equal interval dt=" << dt << " s (time_optimal=false, "
              << "target total time=" << path_total_time_ << ", minimum interval=" << min_time_interval_ << ")\n";
  }

  // 9. report the trajectory duration (the real execution duration after time parameterization, getDuration)
  const double traj_duration = trajectory->getDuration();
  std::cout << "[trajectory duration] " << traj_duration << " s (" << n_wp << " points, for robot execution)\n";

  // 10. trajectory planning duration = pure path planning + time-parameterization compute time (total compute time)
  auto end_time = std::chrono::steady_clock::now();
  const double total_plan_time = std::chrono::duration<double>(end_time - start_time).count();
  std::cout << "[trajectory planning duration] " << total_plan_time
            << " s (pure planning " << pure_plan_time
            << " + time parameterization " << (total_plan_time - pure_plan_time) << ")\n";

  res.trajectory_ = trajectory;
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  res.planning_time_ = total_plan_time;   // report the total planning time including time parameterization
  return true;
}

// DetailedResponse delegation (legacy mode)
bool DualArmPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  planning_interface::MotionPlanResponse normal_res;
  bool success = solve(normal_res);
  if (success) {
    res.trajectory_.push_back(normal_res.trajectory_);
    res.description_.push_back("DualArmAvoidancePlanner");
    res.processing_time_.push_back(normal_res.planning_time_);
  }
  res.error_code_ = normal_res.error_code_;
  return success;
}

// =====================================================================
// PlannerManager
// =====================================================================
bool DualArmAlmCgPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                       const rclcpp::Node::SharedPtr& node,
                                       const std::string& parameter_namespace)
{
  robot_model_         = model;
  node_                = node;
  parameter_namespace_ = parameter_namespace;

  load_parameters();   // read once at startup (re-read afterward on every getPlanningContext)

  RCLCPP_INFO(node_->get_logger(),
      "DualArmAvoidancePlanner initialized (path_weight=%.2f, danger_threshold=%.2f, "
      "collision_tol=%.2f, fix_tol=%.2f, max_iter=%d, smooth_w=%.2f, jointA='%s', jointB='%s')",
      path_weight_, danger_threshold_, collision_tolerance_, fix_tolerance_,
      max_refinement_iter_, smooth_w_, joint_prefix_A_.c_str(), joint_prefix_B_.c_str());
  return true;
}

// re-read all parameters from the parameter server (yaml/rqt edits take effect on the next planning run)
void DualArmAlmCgPlannerManager::load_parameters() const
{
  const std::string ns = parameter_namespace_.empty() ? "" : (parameter_namespace_ + ".");
  node_->get_parameter_or(ns + "path_weight",         path_weight_,         0.5);
  node_->get_parameter_or(ns + "danger_threshold",    danger_threshold_,    0.35);
  node_->get_parameter_or(ns + "collision_tolerance", collision_tolerance_, 0.15);
  node_->get_parameter_or(ns + "fix_tolerance",       fix_tolerance_,       0.1);
  node_->get_parameter_or(ns + "max_refinement_iter", max_refinement_iter_, 15);
  node_->get_parameter_or(ns + "smooth_w",            smooth_w_,            0.3);
  node_->get_parameter_or(ns + "smooth_w_H",          smooth_w_H_,          1.0);
  node_->get_parameter_or(ns + "smooth_w_T",          smooth_w_T_,          1.0);
  node_->get_parameter_or(ns + "smooth_w_neighbor",   smooth_w_neighbor_,   1.0);
  node_->get_parameter_or(ns + "joint_prefix_A",      joint_prefix_A_,      std::string("big_joint_"));
  node_->get_parameter_or(ns + "joint_prefix_B",      joint_prefix_B_,      std::string("small_joint_"));
  node_->get_parameter_or(ns + "time_optimal",        time_optimal_,        true);
  node_->get_parameter_or(ns + "path_total_time",     path_total_time_,     20.0);
  node_->get_parameter_or(ns + "min_time_interval",   min_time_interval_,   0.05);
  node_->get_parameter_or(ns + "export_csv_prefix",   export_csv_prefix_,   std::string("./alm_data"));   // [NEW]
  node_->get_parameter_or(ns + "export_level",        export_level_,        0);                 // [NEW]
  node_->get_parameter_or(ns + "alm_mu0",             alm_mu0_,             10.0);              // [NEW] the six ALM parameters
  node_->get_parameter_or(ns + "alm_c0",              alm_c0_,              5.0);
  node_->get_parameter_or(ns + "alm_c_max",           alm_c_max_,           2000.0);
  node_->get_parameter_or(ns + "alm_beta_c",          alm_beta_c_,          2.0);
  node_->get_parameter_or(ns + "alm_gamma_v",         alm_gamma_v_,         0.5);
  node_->get_parameter_or(ns + "alm_k_outer",         alm_k_outer_,         50);
  node_->get_parameter_or(ns + "alm_k_inner",         alm_k_inner_,         200);
}

void DualArmAlmCgPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.push_back("DualArmAvoidanceCG");
}

void DualArmAlmCgPlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& /*pcs*/)
{
  // this planner has no additional per-config settings; parameters are already read in initialize
}

planning_interface::PlanningContextPtr DualArmAlmCgPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!planning_scene) {
    RCLCPP_ERROR(node_->get_logger(), "planning_scene is empty");
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  // re-read parameters before every planning run (so yaml/rqt adjustments take effect without restarting move_group)
  load_parameters();

  auto context = std::make_shared<DualArmPlanningContext>(
      "dual_arm_avoidance_context", req.group_name, node_);
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  context->path_weight_         = path_weight_;
  context->danger_threshold_    = danger_threshold_;
  context->collision_tolerance_ = collision_tolerance_;
  context->fix_tolerance_       = fix_tolerance_;
  context->max_refinement_iter_ = max_refinement_iter_;
  context->smooth_w_            = smooth_w_;
  context->smooth_w_H_          = smooth_w_H_;
  context->smooth_w_T_          = smooth_w_T_;
  context->smooth_w_neighbor_   = smooth_w_neighbor_;
  context->joint_prefix_A_      = joint_prefix_A_;
  context->joint_prefix_B_      = joint_prefix_B_;
  context->time_optimal_        = time_optimal_;
  context->path_total_time_     = path_total_time_;
  context->min_time_interval_   = min_time_interval_;
  context->export_csv_prefix_   = export_csv_prefix_;   // [NEW]
  context->export_level_        = export_level_;        // [NEW]
  context->alm_mu0_  = alm_mu0_;  context->alm_c0_     = alm_c0_;      // [NEW] the six ALM parameters
  context->alm_c_max_ = alm_c_max_;
  context->alm_beta_c_ = alm_beta_c_; context->alm_gamma_v_ = alm_gamma_v_;
  context->alm_k_outer_ = alm_k_outer_; context->alm_k_inner_ = alm_k_inner_;

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return context;
}

bool DualArmAlmCgPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
  // only serve requests that have joint goal constraints
  return !req.goal_constraints.empty() &&
         !req.goal_constraints[0].joint_constraints.empty();
}

}  // namespace dual_arm_alm_cg_planner

// =====================================================================
// pluginlib registration (this is how MoveIt finds the plugin)
// =====================================================================
PLUGINLIB_EXPORT_CLASS(
    dual_arm_alm_cg_planner::DualArmAlmCgPlannerManager,
    planning_interface::PlannerManager)
