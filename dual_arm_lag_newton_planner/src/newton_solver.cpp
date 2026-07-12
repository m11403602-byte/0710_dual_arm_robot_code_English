// =====================================================================
// newton_solver.cpp — Layer 1 pure Lagrangian full-dimension Newton solver implementation
//   = MATLAB Dual_Arm_Lagrangian_Newton_v2
// =====================================================================
//   the geometry layer (sphere constants / transmatrix / calc_df / mask / FK) with ALM, Newton
//   is bit-identical across the lineage (the same robot model, verified item by item against the v2 MATLAB).
//   the solver math layer (L / G / compute_H / run_lag) is ported per Newton_v2.
// =====================================================================
#include "dual_arm_lag_newton_planner/newton_solver.hpp"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

namespace dual_arm_lag_newton_planner
{

// =====================================================================
// bounding-sphere constants (= arm_r/arm_p/arm_frame, ped_* inside MATLAB robot_arm_bubble)
//   ⚠ bit-identical to the ALM/Newton lineage — when changing the robot, all six packages must be modified in sync
// =====================================================================
const std::vector<BubbleDef> NewtonSolver::PEDESTAL_A = {
  { -1, 320.0,  220.0,  -65.0, 55.0 },
  { -1, 320.0,  220.0,  296.0, 55.0 },
  { -1, 320.0, -220.0,  296.0, 55.0 },
  { -1, 320.0, -220.0,  -64.0, 55.0 },
};
const std::vector<BubbleDef> NewtonSolver::BUBBLES_A = {
  { 0, 280.0,    0.0,   0.0, 145.0 },
  { 1, 300.0,  -15.0,   0.0, -70.0 },
  { 2, 115.0,  160.0,   0.0,  80.0 },
  { 2, 115.0,  160.0,   0.0, 210.0 },
  { 2, 115.0,  160.0,   0.0, 340.0 },
  { 2, 115.0,  160.0,   0.0, 460.0 },
  { 3, 250.0,   60.0,  60.0,  60.0 },
  { 4, 110.0,    0.0, 290.0,   0.0 },
  { 4, 110.0,    0.0, 430.0,   0.0 },
  { 4, 110.0,    0.0, 560.0,   0.0 },
  { 5, 105.0,    0.0,   0.0,   0.0 },
  { 6, 100.0,    0.0,   0.0,   0.0 },
};
const std::vector<BubbleDef> NewtonSolver::PEDESTAL_B = {
  { -1, 350.0,  170.0,  290.0, 661.0 },
  { -1, 350.0, -170.0,  290.0, 661.0 },
  { -1, 350.0, -170.0, -230.0, 661.0 },
  { -1, 350.0,  170.0, -230.0, 661.0 },
  { -1, 350.0,  185.0,  250.0, 231.0 },
  { -1, 350.0, -185.0,  250.0, 231.0 },
  { -1, 350.0, -185.0, -250.0, 231.0 },
  { -1, 350.0,  185.0, -250.0, 231.0 },
};
const std::vector<BubbleDef> NewtonSolver::BUBBLES_B = {
  { 0, 175.0,    0.0,    0.0,   85.0 },
  { 1, 135.0,    0.0,    0.0, -135.0 },
  { 2, 150.0,  -16.0,    0.0,   25.0 },
  { 2, 150.0,  -16.0,    0.0,  120.0 },
  { 2, 150.0,  -16.0,    0.0,  240.0 },
  { 3, 135.0,    0.0,   30.0,   10.0 },
  { 4,  95.0,    0.0,  130.0,    0.0 },
  { 4,  95.0,    0.0,  230.0,    0.0 },
  { 5,  95.0,    0.0,  -23.0,    0.0 },
  { 6,  75.0,    0.0,  -36.5,    0.0 },
};

// =====================================================================
// transmatrix → make_rotation / make_translation  (= MATLAB transmatrix)
// =====================================================================
Eigen::Matrix4d NewtonSolver::make_rotation(char axis, double angle_deg)
{
  const double cv = std::cos(angle_deg * M_PI / 180.0);
  const double sv = std::sin(angle_deg * M_PI / 180.0);
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  if (axis == 'x' || axis == 'X') {
    m(1,1)= cv; m(1,2)=-sv; m(2,1)= sv; m(2,2)= cv;
  } else if (axis == 'y' || axis == 'Y') {
    m(0,0)= cv; m(0,2)= sv; m(2,0)=-sv; m(2,2)= cv;
  } else if (axis == 'z' || axis == 'Z') {
    m(0,0)= cv; m(0,1)=-sv; m(1,0)= sv; m(1,1)= cv;
  }
  return m;
}

Eigen::Matrix4d NewtonSolver::make_translation(char axis, double dist_mm)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  if (axis == 'x' || axis == 'X')      m(0,3) = dist_mm;
  else if (axis == 'y' || axis == 'Y') m(1,3) = dist_mm;
  else if (axis == 'z' || axis == 'Z') m(2,3) = dist_mm;
  return m;
}

// =====================================================================
// calc_df: sphere-pair danger factor sj = exp( ln(0.5)/(R_i+R_j)^2 · d^2 )
// =====================================================================
Eigen::MatrixXd NewtonSolver::calc_df(const Eigen::VectorXd& R1, const Eigen::VectorXd& R2,
                                  const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2)
{
  const int n1 = static_cast<int>(P1.rows());
  const int n2 = static_cast<int>(P2.rows());
  const double log05 = std::log(0.5);
  Eigen::MatrixXd sj(n1, n2);
  for (int i = 0; i < n1; ++i) {
    for (int j = 0; j < n2; ++j) {
      const double dx = P1(i,0) - P2(j,0);
      const double dy = P1(i,1) - P2(j,1);
      const double dz = P1(i,2) - P2(j,2);
      const double d2  = dx*dx + dy*dy + dz*dz;
      const double Rij = R1(i) + R2(j);
      sj(i,j) = std::exp(log05 / (Rij * Rij) * d2);
    }
  }
  return sj;
}

// =====================================================================
// get_collision_masks: 16x18 cross-arm mask (link-level cAB expanded to sphere level, K_AB=180)
// =====================================================================
Eigen::Array<bool, 16, 18> NewtonSolver::get_collision_masks()
{
  // sphere → link ID (1-indexed, same as MATLAB sA/sB)
  static const int sA[16] = {1,1,1,1, 2, 3, 4,4,4,4, 5, 6,6,6, 7, 8};
  static const int sB[18] = {1,1,1,1,1,1,1,1, 2, 3, 4,4,4, 5, 6,6, 7, 8};
  // [MATLAB] cAB: links 1~3 (base/pedestal/L1) all 0; links 4~8 all 1
  auto cAB = [](int rowLink, int /*colLink*/) -> bool { return rowLink >= 4; };
  Eigen::Array<bool, 16, 18> mask;
  for (int i = 0; i < 16; ++i)
    for (int j = 0; j < 18; ++j)
      mask(i,j) = cAB(sA[i], sB[j]);
  return mask;
}

// =====================================================================
// RA610 FK: 16 spheres (4 base + 12 arm) + T_ee
// =====================================================================
void NewtonSolver::robot_arm_bubble_RA610(const Eigen::Matrix4d& T_base, const double J[6],
                                      Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                      Eigen::Matrix4d& T_ee)
{
  Eigen::Matrix4d T[7];
  T[0] = T_base * make_translation('z', 117.0);
  T[1] = make_translation('z', 448.5) * make_rotation('z', J[0]);
  T[2] = make_translation('y', 140.0) * make_rotation('x', J[1]);
  T[3] = make_translation('z', 640.0) * make_rotation('x', J[2]);
  T[4] = make_translation('z', 160.0) * make_rotation('y', J[3]);
  T[5] = make_translation('y', 678.0) * make_rotation('x', J[4]);
  T[6] = make_translation('y', 101.0) * make_rotation('y', J[5]);

  const int NUM_TOTAL = 16, NUM_PED = 4;
  bubble.resize(NUM_TOTAL, 3);
  r.resize(NUM_TOTAL);

  for (int k = 0; k < NUM_PED; ++k) {
    const BubbleDef& b = PEDESTAL_A[k];
    Eigen::Vector4d w = T_base * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
    bubble.row(k) = w.head<3>().transpose();
    r(k) = b.radius;
  }

  Eigen::Matrix4d T_cum = Eigen::Matrix4d::Identity();
  int arm_k = 0;
  for (int i = 0; i < 7; ++i) {
    T_cum = T_cum * T[i];
    for (const BubbleDef& b : BUBBLES_A) {
      if (b.link_id == i) {   // [MATLAB] arm_frame(j) == i-1 (0-indexed alignment)
        Eigen::Vector4d w = T_cum * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
        const int out_idx = NUM_PED + arm_k;
        bubble.row(out_idx) = w.head<3>().transpose();
        r(out_idx) = b.radius;
        ++arm_k;
      }
    }
  }
  T_ee = T_cum;
}

// =====================================================================
// RA605 FK: 18 spheres (8 base + 10 arm) + T_ee
// =====================================================================
void NewtonSolver::robot_arm_bubble_RA605(const Eigen::Matrix4d& T_base, const double J[6],
                                      Eigen::MatrixXd& bubble, Eigen::VectorXd& r,
                                      Eigen::Matrix4d& T_ee)
{
  Eigen::Matrix4d T[7];
  T[0] = T_base * make_translation('z', 802.0);
  T[1] = make_translation('z', 375.0) * make_rotation('z', J[0]);
  T[2] = make_translation('y',  30.0) * make_rotation('x', J[1]);
  T[3] = make_translation('z', 340.0) * make_rotation('x', J[2]);
  T[4] = make_translation('z',  40.0) * make_rotation('y', J[3]);
  T[5] = make_translation('y', 338.0) * make_rotation('x', J[4]);
  T[6] = make_translation('y',  86.5) * make_rotation('y', J[5]);

  const int NUM_TOTAL = 18, NUM_PED = 8;
  bubble.resize(NUM_TOTAL, 3);
  r.resize(NUM_TOTAL);

  for (int k = 0; k < NUM_PED; ++k) {
    const BubbleDef& b = PEDESTAL_B[k];
    Eigen::Vector4d w = T_base * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
    bubble.row(k) = w.head<3>().transpose();
    r(k) = b.radius;
  }

  Eigen::Matrix4d T_cum = Eigen::Matrix4d::Identity();
  int arm_k = 0;
  for (int i = 0; i < 7; ++i) {
    T_cum = T_cum * T[i];
    for (const BubbleDef& b : BUBBLES_B) {
      if (b.link_id == i) {
        Eigen::Vector4d w = T_cum * Eigen::Vector4d(b.cx, b.cy, b.cz, 1.0);
        const int out_idx = NUM_PED + arm_k;
        bubble.row(out_idx) = w.head<3>().transpose();
        r(out_idx) = b.radius;
        ++arm_k;
      }
    }
  }
  T_ee = T_cum;
}

// =====================================================================
// constructor (= MATLAB Dual_Arm_Lagrangian_Newton_v2 constructor)
// =====================================================================
NewtonSolver::NewtonSolver(const Eigen::MatrixXd& X,
                   const Eigen::Matrix4d& robotA_base,
                   const Eigen::Matrix4d& robotB_base,
                   double danger_threshold,
                   double path_weight,
                   double smooth_w,
                   double smooth_w_H,
                   double smooth_w_T,
                   double smooth_w_neighbor)
  : A_base_(robotA_base), B_base_(robotB_base),
    danger_threshold_(danger_threshold), path_weight_(path_weight),
    smooth_w_(smooth_w), smooth_w_H_(smooth_w_H),
    smooth_w_T_(smooth_w_T), smooth_w_neighbor_(smooth_w_neighbor)
{
  // [MATLAB] M = size(X,1) - 2 (excluding head and tail); N = 6
  M_ = static_cast<int>(X.rows()) - 2;
  N_ = 6;

  // [MATLAB] mask → linear indices (column-major, consistent with MATLAB logical indexing)
  Eigen::Array<bool, 16, 18> mask = get_collision_masks();
  lin_idx_AB_.clear();
  for (int col = 0; col < 18; ++col)
    for (int row = 0; row < 16; ++row)
      if (mask(row, col))
        lin_idx_AB_.push_back(col * 16 + row);
  K_AB_  = static_cast<int>(lin_idx_AB_.size());
  num_D_ = K_AB_;

  num_X_     = 2 * M_ * N_;
  num_C_     = M_ * num_D_;
  total_dim_ = num_X_ + 2 * num_C_;

  std::cout << "  [Mask] A×B: " << K_AB_ << " → num_D = " << num_D_ << "\n";

  // [MATLAB] head/tail points X_H/X_T = [A; B]
  X_H_.row(0) = X.row(0).segment(0, 6);
  X_H_.row(1) = X.row(0).segment(6, 6);
  X_T_.row(0) = X.row(X.rows()-1).segment(0, 6);
  X_T_.row(1) = X.row(X.rows()-1).segment(6, 6);

  // [MATLAB] interior points' original joint angles oriPos = [Xa_ori, Xb_ori] (M x 12)
  oriPos_.resize(M_, 12);
  for (int m = 0; m < M_; ++m)
    oriPos_.row(m) = X.row(m + 1);

  // [MATLAB] V_0 = [X_0; λ_0; S_0]; X_0 each point [A1..6, B1..6]
  rebuild_initial_V_();

  std::cout << "  [Init] Max_D=" << compute_Dx_all(Xm_initial_.head(num_X_)).maxCoeff()
            << " | Method=Newton(alpha=1, LDLT) (pure Lagrangian)\n";

  V_final_ = Xm_initial_;
  X_final_ = Xm_initial_.head(num_X_);
}

// =====================================================================
// rebuild_initial_V_: rebuild V_0 from the current oriPos / lam0_ / s0_
//   shared by the constructor + set_lag_params (must be rebuilt after yaml injection)
// =====================================================================
void NewtonSolver::rebuild_initial_V_()
{
  Xm_initial_.resize(total_dim_);

  // X segment (num_X): each point [A;B]
  for (int m = 0; m < M_; ++m) {
    const int b = idx_Xm(m);
    for (int j = 0; j < 6; ++j) {
      Xm_initial_(b + j)     = oriPos_(m, j);       // A
      Xm_initial_(b + 6 + j) = oriPos_(m, 6 + j);   // B
    }
  }

  // λ segment (num_C): all lam0_
  //   [MATLAB] lam_init(Dx_init>θ)=30 is an entry point for future ablation; since lam0=30 it is equivalent to filling all
  Xm_initial_.segment(num_X_, num_C_).setConstant(lam0_);
  // S segment (num_C): all s0_ (S² = s0_²)
  Xm_initial_.segment(num_X_ + num_C_, num_C_).setConstant(s0_);
}

// =====================================================================
// compute_Dm: the danger-factor vector at point m (0-indexed) (num_D)
//   X: the num_X-dim X vector (not the full V)
// =====================================================================
Eigen::VectorXd NewtonSolver::compute_Dm(const Eigen::VectorXd& X, int m) const
{
  double Ja[6], Jb[6];
  const int ba = idx_Xam(m), bb = idx_Xbm(m);
  for (int j = 0; j < 6; ++j) { Ja[j] = X(ba + j); Jb[j] = X(bb + j); }

  Eigen::MatrixXd bA, bB; Eigen::VectorXd rA, rB; Eigen::Matrix4d eeA, eeB;
  robot_arm_bubble_RA610(A_base_, Ja, bA, rA, eeA);
  robot_arm_bubble_RA605(B_base_, Jb, bB, rB, eeB);

  Eigen::MatrixXd sj = calc_df(rA, rB, bA, bB);   // 16x18

  // [MATLAB] D_m = sj(mask_AB) → column-major take lin_idx_AB
  Eigen::VectorXd D_m(K_AB_);
  for (int k = 0; k < K_AB_; ++k) {
    const int idx = lin_idx_AB_[k];
    D_m(k) = sj(idx % 16, idx / 16);
  }
  return D_m;
}

// [MATLAB] compute_Dx_all: concatenation over all M points (num_C)
Eigen::VectorXd NewtonSolver::compute_Dx_all(const Eigen::VectorXd& X) const
{
  Eigen::VectorXd Dx(num_C_);
  for (int m = 0; m < M_; ++m)
    Dx.segment(idx_lam_local(m), num_D_) = compute_Dm(X, m);
  return Dx;
}

// =====================================================================
// compute_D_cache: D_base (num_D x M) + D_plus + D_minus (central-difference cache)
//   the Newton Hessian diagonal uses central differences, so D_minus is needed; corresponds to MATLAB compute_D_cache
// =====================================================================
void NewtonSolver::compute_D_cache(const Eigen::VectorXd& V,
                               Eigen::MatrixXd& D_base,
                               std::vector<Eigen::MatrixXd>& D_plus,
                               std::vector<Eigen::MatrixXd>& D_minus) const
{
  const double h = delta_;
  const Eigen::VectorXd X = V.head(num_X_);
  D_base.resize(num_D_, M_);
  D_plus.assign(M_, Eigen::MatrixXd(num_D_, 12));
  D_minus.assign(M_, Eigen::MatrixXd(num_D_, 12));

  for (int m = 0; m < M_; ++m) {
    D_base.col(m) = compute_Dm(X, m);
    const int b = idx_Xm(m);
    for (int i = 0; i < 12; ++i) {
      Eigen::VectorXd Xp = X;  Xp(b + i) += h;
      D_plus[m].col(i) = compute_Dm(Xp, m);
      Eigen::VectorXd Xm = X;  Xm(b + i) -= h;
      D_minus[m].col(i) = compute_Dm(Xm, m);
    }
  }
}

// =====================================================================
// compute_G_smooth: ∇_X f (forward difference; f depends only on X)
// =====================================================================
Eigen::VectorXd NewtonSolver::compute_G_smooth(const Eigen::VectorXd& V) const
{
  const double h = delta_;
  const double f0 = cost_function_F(V);
  Eigen::VectorXd G(num_X_);
  Eigen::VectorXd Vp = V;
  for (int i = 0; i < num_X_; ++i) {
    const double orig = Vp(i);
    Vp(i) = orig + h;
    G(i) = (cost_function_F(Vp) - f0) / h;
    Vp(i) = orig;
  }
  return G;
}

// =====================================================================
// compute_G: full first-order KKT residual G = [G_X; G_λ; G_S]  (total_dim)
//   G_X = ∇f + w_d Σ λ_i ∇D_i        (stationarity)
//   G_λ = w_d · (D − θ + S²)          (primal feasibility)
//   G_S = 2 w_d · S ⊙ λ              (complementarity)
// =====================================================================
Eigen::VectorXd NewtonSolver::compute_G(const Eigen::VectorXd& V,
                                    const Eigen::MatrixXd& D_base,
                                    const std::vector<Eigen::MatrixXd>& D_plus) const
{
  const double h = delta_;
  const Eigen::VectorXd lambda = V.segment(num_X_, num_C_);
  const Eigen::VectorXd S      = V.segment(num_X_ + num_C_, num_C_);

  // G_X = G_smooth + G_collision
  Eigen::VectorXd G_X = compute_G_smooth(V);
  Eigen::VectorXd D_all(num_C_);
  for (int m = 0; m < M_; ++m) {
    const int i0 = idx_lam_local(m);
    const Eigen::VectorXd lam_m = lambda.segment(i0, num_D_);
    D_all.segment(i0, num_D_) = D_base.col(m);
    const double val_base = lam_m.dot(D_base.col(m)) * wd_;
    const int bx = idx_Xm(m);
    for (int j = 0; j < 12; ++j) {
      const double val_new = lam_m.dot(D_plus[m].col(j)) * wd_;
      G_X(bx + j) += (val_new - val_base) / h;
    }
  }

  // G_λ = w_d (D − θ + S²);  G_S = 2 w_d S⊙λ
  Eigen::VectorXd G_lam = (D_all.array() - danger_threshold_ + S.array().square()).matrix() * wd_;
  Eigen::VectorXd G_S   = 2.0 * wd_ * (S.array() * lambda.array()).matrix();

  Eigen::VectorXd G(total_dim_);
  G << G_X, G_lam, G_S;
  return G;
}

// =====================================================================
// cost_function_L: L(V) = f(X) + w_d λᵀ (D − θ + S²)
//   also returns Dx_all (to avoid a repeated FK)
// =====================================================================
double NewtonSolver::cost_function_L(const Eigen::VectorXd& V, Eigen::VectorXd& Dx_all_out) const
{
  const double f = cost_function_F(V);
  const Eigen::VectorXd lambda = V.segment(num_X_, num_C_);
  const Eigen::VectorXd S      = V.segment(num_X_ + num_C_, num_C_);
  Dx_all_out = compute_Dx_all(V.head(num_X_));
  const Eigen::VectorXd h_vec =
      (Dx_all_out.array() - danger_threshold_ + S.array().square()).matrix();
  const double penalty = lambda.dot(h_vec) * wd_;
  return f + penalty;
}

// =====================================================================
// cost_function_F: f = pw·fA + (1−pw)·fB
// =====================================================================
double NewtonSolver::cost_function_F(const Eigen::VectorXd& V) const
{
  double f, fa, fb;
  cost_function_F_split(V, f, fa, fb);
  return f;
}

// [NEW] split out fa/fb to pass outward (= the internal values of MATLAB cost_function_F, originally only debug-printed)
void NewtonSolver::cost_function_F_split(const Eigen::VectorXd& V,
                                     double& f, double& fa, double& fb) const
{
  // [MATLAB] X = reshape(V(1:num_X),12,[])'  → row m = [Xa_m, Xb_m]
  Eigen::MatrixXd Xmat(M_, 12);
  for (int m = 0; m < M_; ++m)
    Xmat.row(m) = V.segment(idx_Xm(m), 12).transpose();
  const Eigen::MatrixXd Xa = Xmat.leftCols(6);
  const Eigen::MatrixXd Xb = Xmat.rightCols(6);

  fa = cost_Xm(Xa, oriPos_.leftCols(6),  X_H_.row(0), X_T_.row(0));
  fb = cost_Xm(Xb, oriPos_.rightCols(6), X_H_.row(1), X_T_.row(1));
  f  = path_weight_ * fa + (1.0 - path_weight_) * fb;
}

// =====================================================================
// cost_Xm: single-arm smoothing cost (position prior + head/tail boundary + neighbor)
// =====================================================================
double NewtonSolver::cost_Xm(const Eigen::MatrixXd& Xa, const Eigen::MatrixXd& Xori,
                         const Eigen::RowVectorXd& XH, const Eigen::RowVectorXd& XT) const
{
  double c = 0.0;
  // (1) Position prior
  for (int m = 0; m < M_; ++m) {
    const Eigen::RowVectorXd dv = Xa.row(m) - Xori.row(m);
    c += smooth_w_ * dv.dot(dv);
  }
  // (2) Boundary(head) + (3) Neighbor
  for (int m = 0; m < M_; ++m) {
    if (m == 0) {
      const Eigen::RowVectorXd dH = Xa.row(0) - XH;
      c += smooth_w_H_ * dH.dot(dH);
    } else {
      const Eigen::RowVectorXd dv = Xa.row(m) - Xa.row(m - 1);
      c += smooth_w_neighbor_ * dv.dot(dv);
    }
  }
  // (2) Boundary(tail)
  const Eigen::RowVectorXd dT = Xa.row(M_ - 1) - XT;
  c += smooth_w_T_ * dT.dot(dT);
  return c;
}

// =====================================================================
// compute_H_smooth: smoothing-term Hessian ∇²_X f (full num_X×num_X dense FD)
//   ⚠ not block-diagonal: the smoothing cost includes a neighbor term ‖X_m−X_{m-1}‖², hence cross-point coupling
//   diagonal: (fp−2f0+fm)/h²;  off-diagonal: (f(V+h_i+h_j)−fp_i−fp_j+f0)/h²
// =====================================================================
Eigen::MatrixXd NewtonSolver::compute_H_smooth(const Eigen::VectorXd& V) const
{
  const double h = delta_;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_X_, num_X_);

  const double f0 = cost_function_F(V);
  Eigen::VectorXd fp(num_X_), fm(num_X_);
  Eigen::VectorXd Vt = V;
  for (int i = 0; i < num_X_; ++i) {
    const double orig = Vt(i);
    Vt(i) = orig + h; fp(i) = cost_function_F(Vt);
    Vt(i) = orig - h; fm(i) = cost_function_F(Vt);
    Vt(i) = orig;
  }
  for (int i = 0; i < num_X_; ++i) {
    for (int j = i; j < num_X_; ++j) {
      if (i == j) {
        H(i, i) = (fp(i) - 2.0 * f0 + fm(i)) / (h * h);
      } else {
        Eigen::VectorXd Vpp = V;
        Vpp(i) += h; Vpp(j) += h;
        const double val = (cost_function_F(Vpp) - fp(i) - fp(j) + f0) / (h * h);
        H(i, j) = val; H(j, i) = val;
      }
    }
  }
  return H;
}

// =====================================================================
// weighted_D_sum: w_d · Σ λ_m ⊙ D_m(Xm_local)  (used for the Hessian off-diagonal double perturbation)
//   Xm_local: this point's 12-dim local joint vector (first 6 = A, last 6 = B)
// =====================================================================
double NewtonSolver::weighted_D_sum(const Eigen::VectorXd& Xm_local,
                                    const Eigen::VectorXd& lam_m) const
{
  double Ja[6], Jb[6];
  for (int j = 0; j < 6; ++j) { Ja[j] = Xm_local(j); Jb[j] = Xm_local(6 + j); }
  Eigen::MatrixXd bA, bB; Eigen::VectorXd rA, rB; Eigen::Matrix4d eeA, eeB;
  robot_arm_bubble_RA610(A_base_, Ja, bA, rA, eeA);
  robot_arm_bubble_RA605(B_base_, Jb, bB, rB, eeB);
  Eigen::MatrixXd sj = calc_df(rA, rB, bA, bB);
  Eigen::VectorXd D_m(K_AB_);
  for (int k = 0; k < K_AB_; ++k) {
    const int idx = lin_idx_AB_[k];
    D_m(k) = sj(idx % 16, idx / 16);
  }
  return lam_m.dot(D_m) * wd_;
}

// =====================================================================
// compute_H: the full 1116×1116 9-block KKT Hessian
//        X(36)        λ(540)      S(540)
//      ┌──────────┬──────────┬──────────┐
//  X   │  H_XX    │  H_λX'   │    0     │
//  λ   │  H_λX    │    0     │  H_λS    │
//  S   │    0     │  H_λS    │  H_SS    │
//      └──────────┴──────────┴──────────┘
//   H_XX = H_smooth + H_collision (block-diag 12×12/point);  H_λS=diag(2S·wd); H_SS=diag(2λ·wd)
// =====================================================================
Eigen::MatrixXd NewtonSolver::compute_H(const Eigen::VectorXd& V,
                                        const Eigen::MatrixXd& D_base,
                                        const std::vector<Eigen::MatrixXd>& D_plus,
                                        const std::vector<Eigen::MatrixXd>& D_minus) const
{
  const double h = delta_;
  const Eigen::VectorXd X      = V.head(num_X_);
  const Eigen::VectorXd lambda = V.segment(num_X_, num_C_);
  const Eigen::VectorXd S      = V.segment(num_X_ + num_C_, num_C_);

  // --- Block 1: H_XX = H_smooth + H_collision ---
  Eigen::MatrixXd H_XX = compute_H_smooth(V);   // first place H_smooth, then accumulate H_collision in place
  for (int m = 0; m < M_; ++m) {
    const int bx = idx_Xm(m);
    const Eigen::VectorXd lam_m    = lambda.segment(idx_lam_local(m), num_D_);
    const Eigen::VectorXd Xm_local = X.segment(bx, 12);

    const double g_base = lam_m.dot(D_base.col(m)) * wd_;
    Eigen::VectorXd g_plus(12), g_minus(12);
    for (int i = 0; i < 12; ++i) {
      g_plus(i)  = lam_m.dot(D_plus[m].col(i))  * wd_;
      g_minus(i) = lam_m.dot(D_minus[m].col(i)) * wd_;
    }
    for (int i = 0; i < 12; ++i) {
      for (int j = i; j < 12; ++j) {
        double val;
        if (i == j) {
          val = (g_plus(i) - 2.0 * g_base + g_minus(i)) / (h * h);   // diagonal: central difference
        } else {
          Eigen::VectorXd Xpp = Xm_local;                            // off-diagonal: forward-forward (re-FK)
          Xpp(i) += h; Xpp(j) += h;
          const double g_pp = weighted_D_sum(Xpp, lam_m);
          val = (g_pp - g_plus(i) - g_plus(j) + g_base) / (h * h);
        }
        H_XX(bx + i, bx + j) += val;
        if (i != j) H_XX(bx + j, bx + i) += val;
      }
    }
  }

  // --- Block 2: H_λX (Jacobian, forward difference) ---
  Eigen::MatrixXd H_lam_X = Eigen::MatrixXd::Zero(num_C_, num_X_);
  for (int m = 0; m < M_; ++m) {
    const int bx    = idx_Xm(m);
    const int row_s = idx_lam_local(m);
    for (int i = 0; i < 12; ++i) {
      Eigen::VectorXd col_jac = (D_plus[m].col(i) - D_base.col(m)) / h * wd_;
      H_lam_X.block(row_s, bx + i, num_D_, 1) = col_jac;
    }
  }

  // --- Block 3 & 4: complementarity coupling diagonal (λ→0 or S→0 degenerate → H singular) ---
  Eigen::VectorXd H_lam_S_diag = 2.0 * S      * wd_;
  Eigen::VectorXd H_SS_diag    = 2.0 * lambda * wd_;

  // --- 9-block assembly ---
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_dim_, total_dim_);
  const int oX = 0, oL = num_X_, oS = num_X_ + num_C_;
  H.block(oX, oX, num_X_, num_X_) = H_XX;
  H.block(oL, oX, num_C_, num_X_) = H_lam_X;
  H.block(oX, oL, num_X_, num_C_) = H_lam_X.transpose();
  for (int i = 0; i < num_C_; ++i) {
    H(oL + i, oS + i) = H_lam_S_diag(i);   // H_λS
    H(oS + i, oL + i) = H_lam_S_diag(i);   // symmetric
    H(oS + i, oS + i) = H_SS_diag(i);      // H_SS
  }
  // symmetrization (= MATLAB triu(H)+triu(H,1)'): take the upper triangle as authoritative
  for (int i = 0; i < total_dim_; ++i)
    for (int j = 0; j < i; ++j) H(i, j) = H(j, i);

  return H;
}

// =====================================================================
// run_lag: the main loop (full-dimension Newton: d=-H\G LDLT, α=1 pure step)
//   = MATLAB Dual_Arm_Lagrangian_Newton_v2.run_newton (renamed run_lag in C++)
// =====================================================================
SolverLog NewtonSolver::run_lag()
{
  SolverLog log;

  Eigen::VectorXd Xn     = Xm_initial_;
  Eigen::VectorXd pre_Xn = Xn;

  // initial state
  Eigen::VectorXd Dx_init;
  double opt_L = cost_function_L(Xn, Dx_init);
  double max_D_curr = Dx_init.maxCoeff();
  double max_D_pre  = max_D_curr;

  log.max_D_init     = max_D_curr;
  log.violation_init = static_cast<int>((Dx_init.array() > danger_threshold_).count());

  Eigen::VectorXd Dx = Dx_init;
  int it = 0;

  for (it = 1; it <= max_solver_iter_; ++it) {
    // Step 1: FK perturbation cache (Newton needs D_minus for the Hessian central difference)
    Eigen::MatrixXd D_base;
    std::vector<Eigen::MatrixXd> D_plus, D_minus;
    compute_D_cache(Xn, D_base, D_plus, D_minus);

    // Step 1.5: assemble the 1116×1116 KKT Hessian
    const Eigen::MatrixXd H = compute_H(Xn, D_base, D_plus, D_minus);

    // Step 2: gradient G = ∇L  (KKT residual)
    Eigen::VectorXd G = compute_G(Xn, D_base, D_plus);
    const double G_norm = G.norm();

    // cond(H) diagnostic (computed only when verbose; does not affect the optimization result, purely informational)
    double cond_H = std::nan("");
    if (verbose_log_) {
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H, Eigen::EigenvaluesOnly);
      const Eigen::VectorXd ev = es.eigenvalues().cwiseAbs();
      const double mn = ev.minCoeff();
      cond_H = (mn > 0.0) ? (ev.maxCoeff() / mn) : std::numeric_limits<double>::infinity();
    }

    // Step 3: Newton direction d = -H⁻¹G
    //   ⚠ single point of change: the KKT system is an indefinite saddle point; MATLAB `\` uses Bunch-Kaufman LDLᵀ;
    //     here, per specification, Eigen LDLT is used (may be unstable for indefinite matrices — known/expected, documented).
    //     if a closer match to MATLAB `\` is needed, switch to H.fullPivLu().solve(G) (one line).
    Eigen::VectorXd d = -H.ldlt().solve(G);
    // auto solver = H.ldlt();
    // Eigen::VectorXd d; // 1. declare the variable externally

    // // check whether the matrix is invertible / the decomposition succeeded
    // if (solver.info() == Eigen::Success) 
    // {
    //     d = -solver.solve(G); // 2. assign directly, do not re-declare Eigen::VectorXd
    //     std::cout << "solve succeeded!" << std::endl;
    // } 
    // else 
    // {
    //     std::cout << "ldlt decomposition failed, the matrix may be near-singular! enabling the SVD fallback..." << std::endl;
    //     // 3. likewise assign directly
    //     d = -H.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(G); 
    // }
    //Eigen::VectorXd d = -H.fullPivLu().solve(G);
    //Eigen::VectorXd d = -H.bdcSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(G);
    const double d_norm = d.norm();

    // Step 4: divergence detection (⚠ Newton only checks NaN/Inf; MATLAB's d_norm≥1e30 is commented out)
    if (!d.allFinite()) {
      std::cout << "\n  ★ divergence! iter=" << it << " ||G||=" << G_norm
                << " ||d||=" << d_norm << " cond(H)≈" << cond_H << "\n";
      log.diverge_iter = it;
      log.G_norm_history.push_back(G_norm);
      log.d_norm_history.push_back(d_norm);
      break;
    }

    // Step 5: α = 1 pure Newton step (no line search)
    const double a = 1.0;
    const int  ls_inner = 1; const bool ls_fb = false;

    // Step 6: update V and evaluate
    Xn = pre_Xn + a * d;
    opt_L = cost_function_L(Xn, Dx);
    max_D_curr = Dx.maxCoeff();

    // Step 7: roll the history
    pre_Xn = Xn;

    // Step 8: split the V components
    const Eigen::VectorXd lam_part = Xn.segment(num_X_, num_C_);
    const Eigen::VectorXd S_part   = Xn.segment(num_X_ + num_C_, num_C_);

    // Step 9: KKT residual + cost breakdown (L/f/fa/fb passed outward)
    const Eigen::VectorXd G_X_part = G.head(num_X_);
    const Eigen::VectorXd h_vec =
        (Dx.array() - danger_threshold_ + S_part.array().square()).matrix();
    const double penalty = lam_part.dot(h_vec) * wd_;
    double f_now, fa_now, fb_now;
    cost_function_F_split(Xn, f_now, fa_now, fb_now);   // f_now == opt_L − penalty

    const double r_stat = G_X_part.norm();
    const double r_prim = h_vec.norm();
    const double r_comp = (lam_part.array() * S_part.array()).matrix().norm();
    const double r_dual = lam_part.cwiseMin(0.0).norm();
    const double lam_neg= lam_part.cwiseMin(0.0).sum();
    const double kkt_now= std::sqrt(r_stat*r_stat + r_prim*r_prim
                                    + r_comp*r_comp + r_dual*r_dual);
    const double G_lam_norm = G.segment(num_X_, num_C_).norm();
    const double G_S_norm   = G.segment(num_X_ + num_C_, num_C_).norm();
    const int    active_cnt = static_cast<int>((lam_part.array() > 1e-6).count());
    const int    viol_cnt   = static_cast<int>((Dx.array() > danger_threshold_).count());

    // Step 10: display
    std::cout << "  [" << it << "] L:" << opt_L << " (f:" << f_now
              << "+pen:" << penalty << ") | MaxD:" << max_D_curr
              << " | |G|=" << G_norm << " |d|=" << d_norm
              << " | KKT:" << kkt_now << " (X:" << r_stat << " h:" << r_prim
              << " λS:" << r_comp << " λ−:" << lam_neg << ")"
              << " | λ=[" << lam_part.maxCoeff() << "," << lam_part.minCoeff() << "]"
              << " S=[" << S_part.maxCoeff() << "," << S_part.minCoeff() << "]"
              << " α:" << a << "\n";

    // Step 11: write the log (always write the scalar history, for export)
    log.L_history.push_back(opt_L);
    log.f_history.push_back(f_now);
    log.fa_history.push_back(fa_now);
    log.fb_history.push_back(fb_now);
    log.penalty_history.push_back(penalty);
    log.maxD_history.push_back(max_D_curr);
    log.G_norm_history.push_back(G_norm);
    log.d_norm_history.push_back(d_norm);
    log.alpha_history.push_back(a);
    log.r_stat_history.push_back(r_stat);
    log.r_prim_history.push_back(r_prim);
    log.r_comp_history.push_back(r_comp);
    log.r_dual_history.push_back(r_dual);
    log.lam_neg_history.push_back(lam_neg);
    log.kkt_history.push_back(kkt_now);
    log.G_lam_norm_history.push_back(G_lam_norm);
    log.G_S_norm_history.push_back(G_S_norm);
    log.lam_max_history.push_back(lam_part.maxCoeff());
    log.lam_min_history.push_back(lam_part.minCoeff());
    log.S_max_history.push_back(S_part.maxCoeff());
    log.S_min_history.push_back(S_part.minCoeff());
    log.active_count_history.push_back(active_cnt);
    log.violation_count_history.push_back(viol_cnt);
    log.ls_inner_history.push_back(ls_inner);
    log.ls_fallback_history.push_back(ls_fb ? 1 : 0);
    log.cond_H_history.push_back(cond_H);

    // Step 12: convergence check (⚠ Newton: phys_ok && stable_ok && stat_ok)
    //   stat_ok uses the full ‖G‖ (not r_stat); Newton solves the KKT → ‖G‖ can be driven near 0
    const bool phys_ok   = max_D_curr <= danger_threshold_ + TOL_PHYS_MARGIN_;
    const bool stable_ok = std::abs(max_D_curr - max_D_pre) <= TOL_STABLE_;
    const bool stat_ok   = G_norm <= TOL_STAT_;
    if (phys_ok && stable_ok && stat_ok) {
      std::cout << "\n  ★ converged! iter=" << it << " max_D=" << max_D_curr
                << " |ΔmaxD|=" << std::abs(max_D_curr - max_D_pre) << "\n";
      log.converge_iter = it;
      break;
    }

    if (it == max_solver_iter_)
      std::cout << "\n  ⚠ reached the maximum iterations " << it << ", max_D=" << max_D_curr << " (not converged)\n";

    max_D_pre = max_D_curr;
  }

  // wrap-up
  const int actual_iter = std::min(it, max_solver_iter_);
  log.total_iter = actual_iter;

  V_final_ = Xn;
  X_final_ = Xn.head(num_X_);
  log.final_D    = compute_Dx_all(X_final_);
  log.lam_final  = Xn.segment(num_X_, num_C_);
  log.S_final    = Xn.segment(num_X_ + num_C_, num_C_);
  log.max_D_final     = log.final_D.maxCoeff();
  log.violation_final = static_cast<int>((log.final_D.array() > danger_threshold_).count());

  std::cout << "\n========== Newton v2 result summary ==========\n"
            << "  total iterations: " << actual_iter << " / " << max_solver_iter_ << "\n";
  if (log.converge_iter > 0)
    std::cout << "  status: ✅ converged at iter " << log.converge_iter << "\n";
  else if (log.diverge_iter > 0)
    std::cout << "  status: ❌ diverged at iter " << log.diverge_iter << "\n";
  else
    std::cout << "  status: ⚠ reached the limit without converging\n";
  std::cout << "  max_D: initial " << log.max_D_init << " → final " << log.max_D_final
            << " (θ=" << danger_threshold_ << ")\n"
            << "  Violations: initial " << log.violation_init << " → final "
            << log.violation_final << " / " << num_C_ << "\n"
            << "===========================================\n\n";

  return log;
}

}  // namespace dual_arm_lag_newton_planner
