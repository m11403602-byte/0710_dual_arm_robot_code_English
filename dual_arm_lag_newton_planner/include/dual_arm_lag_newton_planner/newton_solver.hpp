// =====================================================================
// newton_solver.hpp — Layer 1: pure Lagrangian inner optimizer (full-dimension Newton)
// =====================================================================
//   pure Lagrangian (S²-slack) + full-dimension Newton (1116×1116 KKT Hessian, LDLT) + α=1 pure step
//   ⚠ a "different mathematical object" from the ALM lineage:
//     decision variable V = [X; λ; S] ∈ ℝ^1116  (λ/S are decision variables, not outer-updated multipliers)
//     Lagrangian  L(V) = f(X) + w_d · λᵀ · (D(X) − θ·1 + S²)
//        (slacked:  h_i = D_i − θ + s_i² = 0,  Bertsekas §3.3.2)
//     gradient  G = [G_X; G_λ; G_S]  is the full first-order KKT residual:
//        G_X = ∇f + w_d Σ λ_i ∇D_i        (stationarity, num_X)
//        G_λ = w_d · (D − θ + S²)          (primal feasibility, num_C)
//        G_S = 2 w_d · S ⊙ λ              (complementarity, num_C)
//     direction  d = −H⁻¹G  (9-block KKT Hessian; ⚠ indefinite saddle point → LDLT may be unstable, the single point of change)
//     step length  α = 1  (pure Newton step, no line search)
//     step length  α from a 1D Newton line search (LS_DELTA=0.01, differing from ALM's 0.001)
//
//   convergence = phys_ok && stable_ok && stat_ok  (⚠ Newton enables stat_ok):
//     Newton solves the full KKT system → ‖G‖ can be driven near 0, so stat_ok = (‖G‖ ≤ TOL_STAT) is meaningful.
//     this is the key difference from GD/CG (for GD/CG, ‖G‖ does not converge at the saddle point, so stat_ok is disabled).
//
//   coordinates/units: all in degrees; distances in mm
//   the geometry (FK/spheres/calc_df/mask) is bit-identical to the ALM/Newton lineage (the same robot model)
// =====================================================================
#ifndef DUAL_ARM_LAG_NEWTON_PLANNER_GD_SOLVER_HPP
#define DUAL_ARM_LAG_NEWTON_PLANNER_GD_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dual_arm_lag_newton_planner
{

// bounding-sphere definition: {link_id (0-indexed), radius, local coordinates x/y/z}
struct BubbleDef {
  int    link_id;   // which link it is attached to (arm_frame, 0-indexed); base spheres use -1
  double radius;
  double cx, cy, cz;
};

// =====================================================================
// SolverLog — the full trace returned by run_lag
//   the outer avoidance_system's export functions read these fields
//   no penalty c/μ schedule here; instead carries the KKT residual + λ/S diagnostics
// =====================================================================
struct SolverLog {
  // --- scalar results ---
  int    total_iter    = 0;     // actual number of main-loop iterations
  int    converge_iter = -1;    // convergence iteration (-1 = not converged; phys_ok && stable_ok)
  int    diverge_iter  = -1;    // divergence iteration (-1 = not diverged; ‖G‖>1e9 or NaN/Inf)

  // --- initial/final summary (for export comparison) ---
  double max_D_init     = 0.0;
  double max_D_final    = 0.0;
  int    violation_init = 0;
  int    violation_final= 0;

  // --- per-iteration history (length = total_iter) ---
  // (the cost breakdown requested to be passed outward)
  std::vector<double> L_history;       // Lagrangian L
  std::vector<double> f_history;       // pure smoothing cost f = pw·fA + (1−pw)·fB
  std::vector<double> fa_history;      // fA  (Arm A smoothing cost)
  std::vector<double> fb_history;      // fB  (Arm B smoothing cost)
  std::vector<double> penalty_history; // w_d · λᵀ · h
  std::vector<double> maxD_history;    // per-iteration max(D)

  // step diagnostics
  std::vector<double> G_norm_history;  // ‖G‖ (full-V gradient norm)
  std::vector<double> d_norm_history;  // ‖d‖
  std::vector<double> alpha_history;   // line-search step length α

  // KKT residual components (specific to the pure Lagrangian)
  std::vector<double> r_stat_history;  // ‖G_X‖           stationarity
  std::vector<double> r_prim_history;  // ‖h‖ = ‖D−θ+S²‖  primal feasibility
  std::vector<double> r_comp_history;  // ‖λ ⊙ S‖         complementarity
  std::vector<double> r_dual_history;  // ‖min(λ,0)‖       dual feasibility (λ≥0)
  std::vector<double> lam_neg_history; // Σ min(λ,0)       cumulative λ drift negative
  std::vector<double> kkt_history;     // √(r_stat²+r_prim²+r_comp²+r_dual²)
  std::vector<double> G_lam_norm_history; // ‖G_λ‖ (= r_prim·w_d, recorded separately)
  std::vector<double> G_S_norm_history;   // ‖G_S‖

  // λ / S ranges + counts
  std::vector<double> lam_max_history;
  std::vector<double> lam_min_history;
  std::vector<double> S_max_history;
  std::vector<double> S_min_history;
  std::vector<int>    active_count_history;    // sum(λ > 1e-6)
  std::vector<int>    violation_count_history; // sum(D > θ)
  std::vector<int>    ls_inner_history;        // line-search inner step count
  std::vector<int>    ls_fallback_history;     // (Newton α=1 has no line search, always 0)
  std::vector<double> cond_H_history;          // [Newton] per-step cond(H) estimate (computed only when verbose, otherwise NaN)

  // --- final vectors ---
  Eigen::VectorXd final_D;    // (num_C) final full-constraint D
  Eigen::VectorXd lam_final;  // (num_C)
  Eigen::VectorXd S_final;    // (num_C)
};

class NewtonSolver
{
public:
  // constructor(X, robotA_base, robotB_base, danger_threshold, path_weight)
  //   X: (P+2) x 12 matrix (head + P interior points + tail), each row [A1..6, B1..6] (degrees)
  NewtonSolver(const Eigen::MatrixXd& X,
           const Eigen::Matrix4d& robotA_base,
           const Eigen::Matrix4d& robotB_base,
           double danger_threshold,
           double path_weight,
           // smoothing weights (default all 1)
           double smooth_w          = 0.3,
           double smooth_w_H        = 1.0,
           double smooth_w_T        = 1.0,
           double smooth_w_neighbor = 1.0);

  // run_lag: runs the Newton main loop; returns log, X_final is obtained via get_X_final()
  SolverLog run_lag();

  // ===== Getter =====
  // take the final X (num_X = 36 dims), i.e., the X segment of V_final (for avoidance_system to reshape)
  const Eigen::VectorXd& get_X_final() const { return X_final_; }
  // take the full V_final (num_X + 2·num_C = 1116 dims)
  const Eigen::VectorXd& get_V_final() const { return V_final_; }
  int get_M() const { return M_; }          // number of interior optimization points
  int get_num_D() const { return num_D_; }  // number of constraints per point (= K_AB)

  void set_verbose(bool v) { verbose_log_ = v; }

  // [NEW] pure Lagrangian parameter injection (yaml → manager → avoidance_system → here; call before run_lag)
  //   ⚠ λ here is the initial value of a decision variable, not an outer-updated penalty multiplier
  void set_lag_params(double wd, double lam0, double s0,
                      double tol_phys_margin, double tol_stable,
                      double tol_stat, int max_iter)
  {
    wd_              = wd;
    lam0_            = lam0;   s0_ = s0;
    TOL_PHYS_MARGIN_ = tol_phys_margin;
    TOL_STABLE_      = tol_stable;
    TOL_STAT_        = tol_stat;        // ⚠ Newton-specific: stat_ok enabled
    max_solver_iter_ = max_iter;
    rebuild_initial_V_();   // rebuild V_0 with the new λ0/S0 (overriding the constructor initialization)
  }

  // [NEW] iteration-limit injection (shared interface across the solver family)
  void set_iter_limits(int max_iter)
  {
    if (max_iter > 0) max_solver_iter_ = max_iter;
  }

  // ===== Static shared utilities =====

  // make_rotation: rotation matrix about axis by angle_deg
  static Eigen::Matrix4d make_rotation(char axis, double angle_deg);
  // make_translation: translation matrix along axis by dist_mm
  static Eigen::Matrix4d make_translation(char axis, double dist_mm);

  // calc_df: sphere-pair danger factor sj = exp(ln0.5/(Ri+Rj)^2 · d^2)
  static Eigen::MatrixXd calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
                                 const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2);

  // get_collision_masks(): 16x18 cross-arm mask (link-level cAB expanded to sphere level, K_AB=180)
  static Eigen::Array<bool, 16, 18> get_collision_masks();

  // robot_arm_bubble_RA610: RA610 FK → 16 spheres (4 base + 12 arm)
  static void robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);
  // robot_arm_bubble_RA605: RA605 FK → 18 spheres (8 base + 10 arm)
  static void robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);

private:
  // ===== Dimensions (determined at construction) =====
  int M_         = 0;   // number of interior optimization points (= P)
  int N_         = 6;   // axes per arm
  int num_D_     = 0;   // number of constraints per point (= K_AB, after mask filtering)
  int num_X_     = 0;   // = 2 · M · N
  int num_C_     = 0;   // = M · num_D
  int total_dim_ = 0;   // = num_X + 2·num_C  (= 1116)

  // ===== Robot / problem configuration =====
  Eigen::Matrix4d A_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d B_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 6> X_H_;   // head point [A; B]
  Eigen::Matrix<double, 2, 6> X_T_;   // tail point [A; B]
  Eigen::MatrixXd oriPos_;            // (M x 12) interior points' original joint angles (smoothing reference)
  Eigen::VectorXd Xm_initial_;        // V_0 = [X_0; λ_0; S_0]   (total_dim)
  Eigen::VectorXd V_final_;           // (total_dim) result V
  Eigen::VectorXd X_final_;           // (num_X) the X segment of the result V

  double danger_threshold_ = 0.35;
  double path_weight_      = 0.5;
  double delta_            = 0.01;   // finite-difference perturbation h (shared by FK and cost FD)

  // smoothing cost weights (default all 1)
  double smooth_w_          = 0.3;
  double smooth_w_H_        = 1.0;
  double smooth_w_T_        = 1.0;
  double smooth_w_neighbor_ = 1.0;

  // ===== Pure Lagrangian parameters (default values) =====
  double wd_               = 1.0;    // dual strength (penalty coefficient)
  double lam0_             = 30.0;   // λ_0  ⚠ default here is 30, not 10
  double s0_               = 1.0;    // S_0  (S²=1)
  double TOL_PHYS_MARGIN_  = 0.01;   // max_D ≤ θ + margin
  double TOL_STABLE_       = 0.01;   // |Δmax_D| ≤ TOL_STABLE
  double TOL_STAT_         = 0.1;    // ⚠ enabled for Newton: ‖G‖ ≤ TOL_STAT (disabled for GD/CG)
  int    max_solver_iter_  = 500;

  bool   verbose_log_      = false;

  // mask → linear indices (positions that are true in the 16x18, column-major)
  std::vector<int> lin_idx_AB_;
  int K_AB_ = 0;

  // ===== Index helpers (V = [X; λ; S], all 0-indexed) =====
  inline int idx_Xam(int m) const { return m * 2 * N_; }            // start of point m's Arm A 6 axes
  inline int idx_Xbm(int m) const { return m * 2 * N_ + N_; }       // start of point m's Arm B 6 axes
  inline int idx_Xm (int m) const { return m * 2 * N_; }            // start of point m's [Xa;Xb] 12 axes
  inline int idx_lam(int m) const { return num_X_ + m * num_D_; }   // start of point m's λ (global V)
  inline int idx_Sm (int m) const { return num_X_ + num_C_ + m * num_D_; } // start of point m's S (global V)
  inline int idx_lam_local(int m) const { return m * num_D_; }      // start of point m's λ (within the λ region)

  // ===== Inner numerical methods =====
  Eigen::VectorXd compute_Dm(const Eigen::VectorXd& X, int m) const;     // D at point m (num_D)
  Eigen::VectorXd compute_Dx_all(const Eigen::VectorXd& X) const;        // D over all points (num_C)
  void compute_D_cache(const Eigen::VectorXd& V,
                       Eigen::MatrixXd& D_base,                          // (num_D x M)
                       std::vector<Eigen::MatrixXd>& D_plus,             // M matrices (num_D x 12) forward
                       std::vector<Eigen::MatrixXd>& D_minus) const;     // M matrices (num_D x 12) backward (central difference)
  Eigen::VectorXd compute_G_smooth(const Eigen::VectorXd& V) const;      // ∇_X f (num_X)
  Eigen::VectorXd compute_G(const Eigen::VectorXd& V,
                            const Eigen::MatrixXd& D_base,
                            const std::vector<Eigen::MatrixXd>& D_plus) const;  // full KKT residual (total_dim)
  double cost_function_L(const Eigen::VectorXd& V, Eigen::VectorXd& Dx_all_out) const;
  double cost_function_F(const Eigen::VectorXd& V) const;
  void   cost_function_F_split(const Eigen::VectorXd& V,
                               double& f, double& fa, double& fb) const;  // [NEW] split out fa/fb to pass outward
  double cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                 const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const;
  // compute_H_smooth: smoothing-term Hessian ∇²_X f (full num_X×num_X dense FD; neighbor coupling is not block-diag)
  Eigen::MatrixXd compute_H_smooth(const Eigen::VectorXd& V) const;
  // weighted_D_sum: w_d·Σ λ⊙D(Xm_local) (used for the Hessian off-diagonal double perturbation)
  double weighted_D_sum(const Eigen::VectorXd& Xm_local, const Eigen::VectorXd& lam_m) const;
  // compute_H: the full 1116×1116 9-block KKT Hessian
  Eigen::MatrixXd compute_H(const Eigen::VectorXd& V,
                            const Eigen::MatrixXd& D_base,
                            const std::vector<Eigen::MatrixXd>& D_plus,
                            const std::vector<Eigen::MatrixXd>& D_minus) const;

  // rebuild V_0 from the current lam0_/s0_/X (shared by the constructor + set_lag_params)
  void rebuild_initial_V_();

  // ===== Bounding-sphere constants (compile-time, initialized with {...} in the .cpp) =====
  static const std::vector<BubbleDef> PEDESTAL_A;  // RA610 base 4 spheres
  static const std::vector<BubbleDef> BUBBLES_A;   // RA610 arm 12 spheres
  static const std::vector<BubbleDef> PEDESTAL_B;  // RA605 base 8 spheres
  static const std::vector<BubbleDef> BUBBLES_B;   // RA605 arm 10 spheres
};

}  // namespace dual_arm_lag_newton_planner

#endif  // DUAL_ARM_LAG_NEWTON_PLANNER_GD_SOLVER_HPP
