// =====================================================================
// gd_solver.hpp — Layer 1: inner GD optimizer (= MATLAB Dual_Arm_Inequality_ALM_GD_v6)
// =====================================================================
//   ALM (PHR) outer + GD (Fletcher-Reeves conjugate gradient) inner + 1D Newton line search
//   ⚠ GD version has "no Hessian, no LDLT, no direction memory": direction d = -G, step length from line search
//   the mathematical model is identical to CG_v7 / Newton_v6 (only the inner solution method is GD steepest descent)
//
//   [MATLAB] corresponding class: Dual_Arm_Inequality_ALM_GD_v6
//   coordinates/units: all in degrees (consistent with MATLAB); distances in mm
//   FK 4x4 uses Eigen::Matrix4d (fixed size); matrices involving num_D use dynamic size
// =====================================================================
#ifndef DUAL_ARM_ALM_GD_PLANNER_GD_SOLVER_HPP
#define DUAL_ARM_ALM_GD_PLANNER_GD_SOLVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace dual_arm_alm_gd_planner
{

// [MATLAB] bounding-sphere definition: {link_id (0-indexed), radius, local coordinates x/y/z}
struct BubbleDef {
  int    link_id;   // which link it is attached to (arm_frame, 0-indexed)
  double radius;
  double cx, cy, cz;
};

// [MATLAB] the log structure returned by run_alm (corresponds to the 33-column log; the verbose section is optional)
//   avoidance_system's export functions read these fields
struct SolverLog {
  // --- scalar results ---
  int    outer_iter       = 0;     // number of outer ALM rounds
  int    total_inner      = 0;     // total inner steps
  int    outer_break_iter = -1;    // convergence round (-1 = not converged)
  int    diverge_iter     = -1;    // divergence round (-1 = not diverged)
  double c_final          = 5.0;   // final penalty parameter c
  double final_G_norm     = 0.0;   // final ||G||
  bool   feasibility_mode = false; // c reached the upper bound (fallback)

  // --- outer history (length = outer_iter) ---
  std::vector<double> L_history;         // Lagrangian
  std::vector<double> f_history_f;       // smoothing cost f
  std::vector<double> f_history_fa;      // fA
  std::vector<double> f_history_fb;      // fB
  std::vector<double> penality_history;  // penalty term
  std::vector<double> maxD_history;      // per-round max_D
  std::vector<double> c_history;         // per-round c
  std::vector<double> mu_max_history;    // per-round max(mu)
  std::vector<double> vk_history;        // dual gradient V_k
  std::vector<double> v_pure_history;    // primal feasibility v_pure
  std::vector<double> compl_history;     // complementarity
  std::vector<double> G_last_history;    // last-step ||G|| per round
  std::vector<double> d_last_history;    // last-step ||d|| per round
  std::vector<int>    inner_iter;        // inner steps per round

  // --- inner history (length = total_inner) ---
  std::vector<double> G_norm_history;
  std::vector<double> d_norm_history;
  std::vector<double> alpha_history;     // line-search step length

  // --- final vectors ---
  Eigen::VectorXd mu_final;   // (num_C)
  Eigen::VectorXd final_D;    // (num_C) last round's full-constraint D
};

class GdSolver
{
public:
  // [MATLAB] constructor Dual_Arm_Inequality_ALM_GD_v6(X, robotA_base, robot_base, DANGER_THRESHOLD, path_weight)
  //   X: (P+2) x 12 matrix (head + P interior points + tail), each row [A1..6, B1..6] (degrees)
  GdSolver(const Eigen::MatrixXd& X,
           const Eigen::Matrix4d& robotA_base,
           const Eigen::Matrix4d& robotB_base,
           double danger_threshold,
           double path_weight,
           // smoothing weights (with defaults)
           double smooth_w          = 0.3,
           double smooth_w_H        = 1.0,
           double smooth_w_T        = 1.0,
           double smooth_w_neighbor = 1.0);

  // [MATLAB] run_alm: returns log; X_final is obtained via get_X_final()
  SolverLog run_alm();

  // getters (the outer layer retrieves results only through getters)
  const Eigen::VectorXd& get_X_final() const { return X_final_; }
  int get_M() const { return M_; }          // number of interior optimization points
  int get_num_D() const { return num_D_; }  // number of constraints per point (= K_AB)

  // [NEW] iteration-limit injection (yaml-tunable; K_inner is also applied to the first round)
  void set_iter_limits(int k_outer, int k_inner)
  {
    if (k_outer > 0) K_outer_ = k_outer;
    if (k_inner > 0) { K_inner_ = k_inner; K_inner_first_ = k_inner; }
  }

  // [NEW] ALM parameter injection (yaml → manager → avoidance_system → here; must be called before run_alm)
  //   the mu_ vector is rebuilt from mu0 (overriding the constructor's default initialization)
  //   multiplier update is classical PHR (Bertsekas 1982, eq.17), no upper-bound safeguarding (reset removed)
  //   ⚠ c0 must not exceed c_max — otherwise the penalty-parameter schedule behavior is undefined
  void set_alm_params(double mu0, double c0, double c_max,
                      double beta_c, double gamma_v)
  {
    mu_ = Eigen::VectorXd::Constant(num_C_, mu0);
    c_ = c0; c_max_ = c_max;
    beta_c_ = beta_c; gamma_v_ = gamma_v;
  }

  // ===== Static shared utilities (= MATLAB static methods) =====

  // [MATLAB] transmatrix(1, dir, deg): rotation matrix (angle deg)
  static Eigen::Matrix4d make_rotation(char axis, double angle_deg);
  // [MATLAB] transmatrix(2, dir, val): translation matrix (mm)
  static Eigen::Matrix4d make_translation(char axis, double dist_mm);

  // [MATLAB] calc_df(R1,R2,P1,P2): sphere-pair danger factor sj = exp(ln0.5/(Ri+Rj)^2 * d^2)
  //   P1:(n1x3), P2:(n2x3), R1:(n1), R2:(n2) -> sj:(n1 x n2)
  static Eigen::MatrixXd calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
                                 const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2);

  // [MATLAB] get_collision_masks(): 16x18 cross-arm mask (link-level cAB expanded to sphere level)
  static Eigen::Array<bool, 16, 18> get_collision_masks();

  // [MATLAB] robot_arm_bubble_RA610_1476: RA610 FK -> 16 spheres (4 base + 12 arm)
  //   returns bubble:(16x3) sphere centers, r:(16) radii, T_ee:4x4 end-effector transform
  static void robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);
  // [MATLAB] robot_arm_bubble_RA605_710: RA605 FK -> 18 spheres (8 base + 10 arm)
  static void robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
                                     Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                     Eigen::Matrix4d& T_ee);

private:
  // ===== Dimensions (determined at construction) =====
  int M_     = 0;   // number of interior optimization points (= P)
  int num_D_ = 0;   // number of constraints per point (= K_AB, after mask filtering)
  int num_X_ = 0;   // = 2 * M * 6
  int num_C_ = 0;   // = M * num_D

  // ===== Robot / problem configuration =====
  Eigen::Matrix4d A_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d B_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 2, 6> X_H_;   // head point [A; B]
  Eigen::Matrix<double, 2, 6> X_T_;   // tail point [A; B]
  Eigen::MatrixXd oriPos_;            // (M x 12) interior points' original joint angles (smoothing reference)
  Eigen::VectorXd Xm_initial_;        // (num_X) initial decision vector
  Eigen::VectorXd X_final_;           // (num_X) result

  double danger_threshold_ = 0.35;
  double path_weight_      = 0.5;
  double delta_            = 0.01;   // finite-difference step h

  // smoothing cost weights
  double smooth_w_          = 0.3;
  double smooth_w_H_        = 1.0;
  double smooth_w_T_        = 1.0;
  double smooth_w_neighbor_ = 1.0;

  // mask -> linear indices (positions that are true in the 16x18, column-major)
  std::vector<int> lin_idx_AB_;  // length = K_AB
  int K_AB_ = 0;

  // ===== ALM parameters (= GD_v6 defaults, same as CG_v7) =====
  Eigen::VectorXd mu_;          // (num_C) multipliers, initially all 10
  double c_                   = 5.0;
  double c_max_               = 2000.0; // unified across the three solvers (= yaml)
  double beta_c_              = 2.0;    // unified across the three solvers (= yaml)
  double gamma_v_             = 0.5;
  double epsilon_v_           = 0.01;
  double epsilon_g_           = 0.1;
  double epsilon_compl_       = 0.1;
  double epsilon_inner_       = 1.0;
  double epsilon_inner_min_   = 0.1;
  double epsilon_inner_decay_ = 0.5;
  int    K_outer_             = 50;
  int    K_inner_             = 200;
  int    K_inner_first_       = 200;


  // ===== Inner numerical methods (= MATLAB instance methods) =====
  Eigen::VectorXd compute_Dm(const Eigen::VectorXd& X, int m) const;          // D at point m (num_D)
  Eigen::VectorXd compute_Dx_all(const Eigen::VectorXd& X) const;             // D over all points (num_C)
  Eigen::VectorXd compute_G_smooth(const Eigen::VectorXd& X) const;           // smoothing gradient (num_X)
  Eigen::VectorXd compute_G_c(const Eigen::VectorXd& X,
                              const Eigen::MatrixXd& D_base, const std::vector<Eigen::MatrixXd>& D_plus,
                              const Eigen::VectorXd& mu_loc, double c_loc) const;   // full gradient
  void compute_D_cache(const Eigen::VectorXd& X,
                       Eigen::MatrixXd& D_base, std::vector<Eigen::MatrixXd>& D_plus) const;  // 2 return values
  double cost_function_F(const Eigen::VectorXd& X) const;
  void   cost_function_F_split(const Eigen::VectorXd& X, double& f, double& fa, double& fb) const;
  double cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                 const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const;
  double cost_L_loc(const Eigen::VectorXd& X, const Eigen::VectorXd& mu_loc, double c_loc) const;
  double line_search_newton_1d(const Eigen::VectorXd& X, const Eigen::VectorXd& d,
                               const Eigen::VectorXd& mu_loc, double c_loc) const;

  // index helper: the position of point m (0-indexed) within the 12-dim block of the X vector (returns the 0-indexed start)
  inline int base_x(int m) const { return m * 12; }

  // ===== Bounding-sphere constants (compile-time, initialized with {...} in the .cpp) =====
  static const std::vector<BubbleDef> PEDESTAL_A;  // RA610 base 4 spheres
  static const std::vector<BubbleDef> BUBBLES_A;   // RA610 arm 12 spheres
  static const std::vector<BubbleDef> PEDESTAL_B;  // RA605 base 8 spheres
  static const std::vector<BubbleDef> BUBBLES_B;   // RA605 arm 10 spheres
};

}  // namespace dual_arm_alm_gd_planner

#endif  // DUAL_ARM_ALM_GD_PLANNER_GD_SOLVER_HPP
