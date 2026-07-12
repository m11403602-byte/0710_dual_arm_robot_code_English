// =====================================================================
// avoidance_system.hpp — Layer 2: outer collision-repair system
// =====================================================================
//   outer collision-repair loop: Clamped Spline initial trajectory -> collision detection -> find targets ->
//   call the inner CG optimization -> Spline reconstruction -> re-check (up to max_refinement_iter rounds)
//   includes CSV export (all data export except plotting)
//
//   all in degrees; independent of MoveIt (pure math, usable standalone)
// =====================================================================
#ifndef DUAL_ARM_LAG_GD_PLANNER_AVOIDANCE_SYSTEM_HPP
#define DUAL_ARM_LAG_GD_PLANNER_AVOIDANCE_SYSTEM_HPP

#include "dual_arm_lag_gd_planner/gd_solver.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dual_arm_lag_gd_planner
{

// trajectory struct
struct Trajectory {
  Eigen::VectorXd time;   // (T)   step index
  Eigen::MatrixXd posA;   // (Tx6) Arm A joint angles
  Eigen::MatrixXd posB;   // (Tx6) Arm B joint angles
  Eigen::MatrixXd pos;    // (Tx12) [posA, posB]
};

// the indices returned by find_collision_targets
struct CollisionIndices {
  int minidx = 0;
  int maxidx = 0;
  std::vector<int> targets;   // 5 control points (0-indexed): [Head, q1, peak, q3, Tail]
};

// iter_log entry (snapshot of each outer repair round, for export)
struct IterLogEntry {
  Trajectory          traj_in;       // pre-repair trajectory
  Trajectory          traj_out;      // post-repair trajectory
  Eigen::VectorXd     path_D_max_in;  // per-step max_D before repair
  Eigen::VectorXd     path_D_max_out; // per-step max_D after repair
  std::vector<int>    targets_in;     // 5 points (pre-repair index)
  std::vector<int>    targets_out;    // 5 points (new-trajectory index)
  SolverLog           solver_log;     // this round's inner CG log
};

class AvoidanceSystem
{
public:
  // constructor (A_waypoints, B_waypoints, path_weight, DANGER_THRESHOLD=0.4)
  //   A_waypoints/B_waypoints: 2x6 (start row + goal row), degrees
  AvoidanceSystem(const Eigen::MatrixXd& A_waypoints,
                  const Eigen::MatrixXd& B_waypoints,
                  double path_weight,
                  double danger_threshold    = 0.35,
                  // the following are tunable parameters (with defaults used if not passed)
                  double collision_tolerance = 0.1,
                  double fix_tolerance       = 0.1,
                  int    max_refinement_iter = 15,
                  double smooth_w            = 0.3,
                  double smooth_w_H          = 1.0,
                  double smooth_w_T          = 1.0,
                  double smooth_w_neighbor   = 1.0);

  // run_optimization: the collision-repair main loop
  void run_optimization();

  // ===== Getters (only getters are exposed externally) =====
  const Trajectory& get_optimized_trajectory() const { return trajectory_opt_; }
  const Trajectory& get_original_trajectory()  const { return trajectory_ori_; }
  bool has_collision() const { return is_collision_; }


  // [NEW] pure Lagrangian parameter injection (yaml-tunable; if not called, the solver defaults are used)
  //   ⚠ λ/S are the initial values of decision variables (not the ALM outer multipliers); a different concept from ALM's set_alm_params
  void set_lag_params(double wd, double lam0, double s0,
                      double tol_phys_margin, double tol_stable, int max_iter)
  {
    lag_wd_ = wd; lag_lam0_ = lam0; lag_s0_ = s0;
    lag_tol_phys_ = tol_phys_margin; lag_tol_stable_ = tol_stable;
    lag_max_iter_ = max_iter;
  }
  bool is_optimized()  const { return is_optimized_; }

  // ===== CSV Export ([REVISE] consolidated into a single entry point; the old four exporters have been removed) =====
  // [NEW] export_unified: consolidated export (the sole public entry point)
  //   level 1 = the paper-standard 6 files, level 2 = +constraints_all/path_original/path_evolution
  void export_unified(const std::string& prefix, int level) const;

private:
  // ===== Robot / problem configuration =====
  Eigen::Matrix4d robotA_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d robotB_base_ = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd A_waypoints_;   // 2x6
  Eigen::MatrixXd B_waypoints_;   // 2x6

  double danger_threshold_    = 0.35;
  double path_weight_         = 0.5;
  double collision_tolerance_ = 0.15;
  double fix_tolerance_       = 0.1;
  double STEP_MAX_DEG_        = 0.5;
  int    max_refinement_iter_ = 15;
  // smoothing weights (forwarded to GdSolver)
  double smooth_w_            = 0.3;
  double smooth_w_H_          = 1.0;
  double smooth_w_T_          = 1.0;
  double smooth_w_neighbor_   = 1.0;

  // ===== State =====
  Trajectory      trajectory_ori_;
  Trajectory      trajectory_opt_;
  Eigen::VectorXd path_D_ori_;       // (T) original per-step max_D
  Eigen::VectorXd path_D_opt_;
  Eigen::MatrixXd path_D_all_ori_;   // (T x num_D) original full-constraint D (for export)
  bool   is_collision_ = true;
  bool   is_optimized_ = false;
  int    refinement_count_ = 0;
  // [NEW] pure Lagrangian parameters (default = Gradient_v2 values)
  double lag_wd_         = 1.0;     // dual strength (penalty coefficient)
  double lag_lam0_       = 30.0;    // λ_0 (⚠ the comment says 10, the actual code = 30)
  double lag_s0_         = 1.0;     // S_0 (S²=1)
  double lag_tol_phys_   = 0.01;    // max_D ≤ θ + margin
  double lag_tol_stable_ = 0.01;   // |Δmax_D| ≤ TOL_STABLE
  int    lag_max_iter_   = 500;
  std::vector<double> refinement_history_;
  std::vector<double> time_ms_;            // inner-loop time per round
  std::vector<IterLogEntry> iter_log_;

  // ===== Private methods =====
  void generate_initial_trajectory();
  void check_collision(const Trajectory& traj,
                       Eigen::VectorXd& path_D_max, bool& is_collision,
                       Eigen::MatrixXd* path_D_all = nullptr) const;
  CollisionIndices find_collision_targets(const Eigen::VectorXd& path_D) const;
  Trajectory regenerate_trajectory_global(const Trajectory& old_traj,
                                           const Eigen::MatrixXd& Xa_opt,
                                           const Eigen::MatrixXd& Xb_opt,
                                           const CollisionIndices& indices,
                                           std::vector<int>& targets_out) const;
  // call the inner CG: returns Xa_opt/Xb_opt (M x 6) + log
  void run_solver_global(const Trajectory& traj, const std::vector<int>& targets,
                         Eigen::MatrixXd& Xa_opt, Eigen::MatrixXd& Xb_opt,
                         SolverLog& solver_log);

  // ===== Clamped Cubic Spline utilities =====
  // given t_knots (n), values Y (dim x n), endpoint slopes v0/v1 (dim), query points t_query (q)
  //   -> returns (q x dim) interpolated result
  static Eigen::MatrixXd clamped_cubic_spline(const Eigen::VectorXd& t_knots,
                                              const Eigen::MatrixXd& Y,
                                              const Eigen::VectorXd& v0,
                                              const Eigen::VectorXd& v1,
                                              const Eigen::VectorXd& t_query);
};

}  // namespace dual_arm_lag_gd_planner

#endif
