# <p align="center">dual_arm_alm_cg_planner</p>

<p align="justify">
Dual-arm collision-avoidance path planner — a ROS 2 Humble / MoveIt2 plugin.
</p>

<p align="justify">
Implemented in C++17 + Eigen3. The inner loop employs <b> ALM (Augmented Lagrangian) + CG-FR (Fletcher-Reeves conjugate gradient) + 1D Newton line search </b> .
</p>

---

## Algorithm Overview

<p align="justify">
Given the start and goal joint angles of the two manipulators (bounding-sphere models: RA610 with 16 spheres / RA605 with 18 spheres), the planner solves for a joint-space trajectory in which the two arms do not collide.
</p>

<p align="justify">
It adopts a two-level (bilevel) optimization scheme:
</p>

```
Outer level (avoidance_system):  generate initial trajectory -> collision detection -> pick the 5 points of the dangerous segment ->
                                 call inner optimization -> Spline reconstruction -> re-check (up to max_refinement_iter rounds, tunable)
Inner level (cg_solver):         ALM outer loop (penalty parameter / multiplier update) + CG-FR inner loop + line search
```

<p align="justify">
The danger-factor threshold <code>danger_threshold = 0.35</code> and the collision-clearance margin <code>collision_tolerance = 0.15</code> (both are yaml defaults and must be positive). The collision boundary (0.5) = <code>danger_threshold + collision_tolerance</code>; <b> when configuring, the sum of the two must not exceed the collision boundary (0.5) </b> (0.5 = the two spheres are tangent, and exceeding it would fail to detect genuine collisions).
</p>

---

## Architecture: A Single .so

<p align="justify">
All source code (the core algorithm + the MoveIt plugin interface) is compiled into a <b> single shared library </b> <code>libdual_arm_alm_cg_planner.so</code> .
</p>

<table width="100%">
  <thead>
    <tr>
      <th align="center">Component</th>
      <th align="center">Role</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center"><code>cg_solver</code></td>
      <td align="center">Layer 1: inner CG solver + FK + bounding spheres + danger factor</td>
    </tr>
    <tr>
      <td align="center"><code>avoidance_system</code></td>
      <td align="center">Layer 2: outer collision-repair loop + Spline reconstruction</td>
    </tr>
    <tr>
      <td align="center"><code>data_io</code></td>
      <td align="center">CSV writing utility (for inspecting the joint path and the evolution of the danger factor, when needed)</td>
    </tr>
    <tr>
      <td align="center"><code>planner_manager</code></td>
      <td align="center">Layer 3: MoveIt2 PlannerManager / PlanningContext</td>
    </tr>
  </tbody>
</table>

---

## Optimization Workflow and Variable Definitions

### Outer Level: Collision-Repair Loop ( <code>AvoidanceSystem::run_optimization</code> )

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│ Clamped Cubic Spline generates the initial trajectory (Arm A/B: start -> goal)   │
│      |                                                                           │
│      v                                                                           │
│ Incrementally compute the danger factor  D_m = calc_df(...) -> max_D(t)          │
│      |                                                                           │
│      v                                                                           │
│ max_D(t) >= collision boundary (0.5)  (= danger_threshold + collision_tolerance) │
│ still colliding, keep optimizing?   --No-->  Done (has_collision=false)          │
│      | Yes                                                                       │
│      v                                                                           │
│ find_collision_targets(): take the 5 control points of the dangerous segment     │
│    [Head, q1, peak, q3, Tail]                                                    │
│      |                                                                           │
│      v                                                                           │
│ take the 3 interior points (q1, peak, q3) as decision variable X (36-dim)        │
│ call the inner optimizer  run_alm(X0) -> X*                                      │
│      |                                                                           │
│      v                                                                           │
│ regenerate_trajectory_global(): local Spline rebuild of the full path via X*     │
│      |                                                                           │
│      v                                                                           │
│ recompute max_D(t), back to "danger check" (up to max_refinement_iter rounds)    │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### Inner Level: ALM Conjugate-Gradient Solver ( <code>CgSolver::run_alm</code> )

> <p align="justify">In the diagram, the "parameter=value" pairs are illustrative yaml defaults; the actual values are governed by the config.</p>

```
┌───────────────────────────────────────────────────────────────────────────┐
│ initialize:  mu = alm_mu0(=10),  c = alm_c0(=5)                           │
│      |                                                                    │
│      v                                                                    │
│ ┌─ outer ALM loop (up to alm_k_outer=50 iters) ─────────────────────────┐ │
│ │ ┌─ inner CG-FR iteration (up to alm_k_inner=200 steps) ────────────┐  │ │
│ │ │ compute  G(X) = ∇f + Σ mu_i · max(0, g_i(X)) · ∇g_i              │  │ │
│ │ │ first iter:  d = -G                                              │  │ │
│ │ │ otherwise (Fletcher-Reeves):                                     │  │ │
│ │ │      beta = ‖G‖² / ‖G_prev‖²                                     │  │ │
│ │ │      d = -G + beta·d_prev                                        │  │ │
│ │ │ 1D Newton line search for the step length alpha                  │  │ │
│ │ │ X <- X + alpha·d                                                 │  │ │
│ │ │ ‖G‖ < inner tolerance (tightened each round)?  --No--> next step │  │ │
│ │ └─ Yes ────────────────────────────────────────────────────────────┘  │ │
│ │      v                                                                │ │
│ │ update multipliers  mu_i <- max(0, mu_i + c·g_i(X))                   │ │
│ │ ‖V_k‖ > alm_gamma_v(=0.5)·‖V_k-1‖ ?                                   │ │
│ │      --Yes--> c <- alm_beta_c(=2.0)·c  (cap alm_c_max=2000)           │ │
│ │      v                                                                │ │
│ │ convergence check: feasibility, gradient-norm, complementarity < tol. │ │
│ │      --No--> back to the inner CG-FR iteration                        │ │
│ └─ Yes --> return X*, SolverLog ────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────────┘
```

### Principal Variables

<table width="100%">
  <thead>
    <tr>
      <th align="center">Variable</th>
      <th align="center">Meaning</th>
      <th align="center">Dimension</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center"><code>X</code></td>
      <td align="center">Decision variable: the 3 interior control points of the dangerous segment × 6 axes per arm (degree)</td>
      <td align="center">36</td>
    </tr>
    <tr>
      <td align="center"><code>D_m</code></td>
      <td align="center">Danger-factor vector of the m-th control point</td>
      <td align="center"><code>num_D_</code></td>
    </tr>
    <tr>
      <td align="center"><code>g_i(X)</code></td>
      <td align="center">Inequality constraint: <code>D_i(X) − danger_threshold</code></td>
      <td align="center"><code>num_C_</code> in total</td>
    </tr>
    <tr>
      <td align="center"><code>G</code></td>
      <td align="center">Gradient of the Lagrangian with respect to X</td>
      <td align="center">36</td>
    </tr>
    <tr>
      <td align="center"><code>d</code></td>
      <td align="center">CG-FR search direction</td>
      <td align="center">36</td>
    </tr>
    <tr>
      <td align="center"><code>mu</code></td>
      <td align="center">ALM multipliers</td>
      <td align="center"><code>num_C_</code></td>
    </tr>
    <tr>
      <td align="center"><code>c</code></td>
      <td align="center">ALM penalty parameter</td>
      <td align="center">scalar</td>
    </tr>
  </tbody>
</table>

<p align="justify">
For the default values and descriptions of every parameter, see <a href="PARAMETERS.md">PARAMETERS.md</a> .
</p>

---

## Directory Structure

- <b> Root-directory files </b> ( <code>CMakeLists.txt</code> / <code>package.xml</code> / <code>*.xml</code> / <code>README.md</code> / <code>PARAMETERS.md</code> ) : <b> the build & description layer </b> — defines how the package is compiled and provides the external documentation.
- <b> <code>config/</code> </b> : <b> the runtime-parameter layer </b> — stores the yaml parameters loaded when move_group starts; must be copied to <code>hiwin_dual_arm/config/</code> .
- <b> <code>include/dual_arm_alm_cg_planner/</code> </b> : <b> the interface-declaration layer </b> .
- <b> <code>src/</code> </b> : <b> the algorithm-implementation layer </b> .

```
dual_arm_alm_cg_planner/
├── CMakeLists.txt
├── package.xml
├── dual_arm_alm_cg_planner.xml
├── README.md
├── PARAMETERS.md
├── config/
│   └── dual_arm_alm_cg_planning.yaml
├── include/dual_arm_alm_cg_planner/
│   ├── cg_solver.hpp
│   ├── avoidance_system.hpp
│   ├── data_io.hpp
│   └── planner_manager.hpp
└── src/
    ├── cg_solver.cpp
    ├── avoidance_system.cpp
    ├── data_io.cpp
    └── planner_manager.cpp
```

<table width="100%">
  <thead>
    <tr>
      <th align="center">File</th>
      <th align="center">Function</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center"><code>CMakeLists.txt</code></td>
      <td align="center">Build configuration: compiles the single .so</td>
    </tr>
    <tr>
      <td align="center"><code>package.xml</code></td>
      <td align="center">ROS 2 package description and dependency declarations</td>
    </tr>
    <tr>
      <td align="center"><code>dual_arm_alm_cg_planner.xml</code></td>
      <td align="center">pluginlib plugin description (enables dynamic loading by MoveIt2)</td>
    </tr>
    <tr>
      <td align="center"><code>README.md</code></td>
      <td align="center">This document</td>
    </tr>
    <tr>
      <td align="center"><code>PARAMETERS.md</code></td>
      <td align="center">Parameter reference table (tunable parameters)</td>
    </tr>
    <tr>
      <td align="center"><code>config/dual_arm_alm_cg_planning.yaml</code></td>
      <td align="center">Planner parameters (must be copied to <code>hiwin_dual_arm/config/</code> )</td>
    </tr>
    <tr>
      <td align="center"><code>cg_solver.hpp/.cpp</code></td>
      <td align="center">Layer 1: FK, bounding spheres, danger factor, ALM + CG-FR inner solve</td>
    </tr>
    <tr>
      <td align="center"><code>avoidance_system.hpp/.cpp</code></td>
      <td align="center">Layer 2: outer collision-repair loop, Spline reconstruction, CSV export</td>
    </tr>
    <tr>
      <td align="center"><code>data_io.hpp/.cpp</code></td>
      <td align="center">CSV writing utility</td>
    </tr>
    <tr>
      <td align="center"><code>planner_manager.hpp/.cpp</code></td>
      <td align="center">Layer 3: MoveIt2 plugin interface (radian ↔ degree, time parameterization)</td>
    </tr>
  </tbody>
</table>

---

## Building

<p align="justify">
After placing the package into the <code>src/</code> directory of a ROS 2 workspace:
</p>

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_alm_cg_planner
source install/setup.bash
```

<p align="justify">
Requirements: ROS 2 Humble, MoveIt2, Eigen3, and a C++17 compiler.
</p>

### ⚠️ Compilation Flags: Use <code>-O3</code>, <b> Do Not Use <code>-march=native</code> </b>

<p align="justify">
The CMake default Release flags are <code>-O3 -DNDEBUG</code> ( <b> deliberately excluding <code>-march=native</code> </b> , which would conflict with MoveIt's Eigen memory alignment and cause a crash).
</p>

---

## Usage (MoveIt2 Plugin)

> <p align="justify">Prerequisite: reside in the same workspace as <code>hiwin_dual_arm</code> (the robot description + MoveIt configuration).</p>

1. Copy this package's <code>config/dual_arm_alm_cg_planning.yaml</code> to <code>hiwin_dual_arm/config/</code> (the filename differs from planner to planner). The file already contains the complete <code>planning_plugin</code> and <code>request_adapters</code> settings, so copying it suffices — no further pipeline changes are required.

2. Launch (example):

```bash
ros2 launch hiwin_dual_arm brain.launch.py
```

<p align="justify">
The planning request must include joint goal constraints.
</p>

<p align="justify">
<b> Changing parameters </b> : edit the yaml → re-copy it to <code>hiwin_dual_arm/config/</code> → restart move_group for the changes to take effect ( <b> no recompilation needed </b> ).
</p>

---

## CSV Export

<p align="justify">
The single public entry point is <code>export_unified(prefix, level)</code> . Output is written to the <code>prefix/&lt;timestamp&gt;_CG/</code> directory (each export forms its own timestamped folder, so exports never overwrite one another).
</p>

<table width="100%">
  <thead>
    <tr>
      <th align="center">level</th>
      <th align="center">File</th>
      <th align="center">Content</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center"><b> 0 </b></td>
      <td align="center">(none)</td>
      <td align="center">Nothing is exported at all (master switch; the current yaml default)</td>
    </tr>
    <tr>
      <td align="center" rowspan="6"><b> 1 </b></td>
      <td align="center"><code>meta.csv</code></td>
      <td align="center">Parameter snapshot (key/value, including the five ALM parameters)</td>
    </tr>
    <tr>
      <td align="center"><code>summary.csv</code></td>
      <td align="center">Per-round repair scoreboard (round / iteration / L / f / maxD / c_final …)</td>
    </tr>
    <tr>
      <td align="center"><code>inner.csv</code></td>
      <td align="center">Concatenated inner-loop steps (round / inner_step / ‖G‖ / ‖d‖ / alpha)</td>
    </tr>
    <tr>
      <td align="center"><code>danger_final.csv</code></td>
      <td align="center">Initial vs. final per-step MaxD comparison</td>
    </tr>
    <tr>
      <td align="center"><code>danger_rounds.csv</code></td>
      <td align="center">Long table: round / step / MaxD_in / MaxD_out</td>
    </tr>
    <tr>
      <td align="center"><code>targets.csv</code></td>
      <td align="center">The 5 feature points per round (index + D value + joint angles before/after optimization)</td>
    </tr>
    <tr>
      <td align="center" rowspan="3"><b> 2 </b></td>
      <td align="center">The above 6 files + <code>constraints_all.csv</code></td>
      <td align="center">Full constraint D-value table</td>
    </tr>
    <tr>
      <td align="center"><code>path_original.csv</code></td>
      <td align="center">All points of the initial trajectory</td>
    </tr>
    <tr>
      <td align="center"><code>path_evolution.csv</code></td>
      <td align="center">Long table: round / step / A1..B6 (trajectory evolution per round)</td>
    </tr>
  </tbody>
</table>

<p align="justify">
On the MoveIt path, export is governed by the yaml keys <code>export_csv_prefix</code> (default <code>./alm_data</code> ) and <code>export_level</code> (default 0 = no export; to retain data, set it to 1 or 2 and restart move_group). All CSV files can be opened directly in Excel or with Python's <code>pandas.read_csv</code> .
</p>

---

## Items Requiring Adjustment

<p align="justify">
Before deployment, adjust the following according to your actual robot:
</p>

1. <b> Joint names </b> ( <code>solve()</code> in <code>src/planner_manager.cpp</code> ) : currently Arm A uses <code>big_joint_1~6</code> and Arm B uses <code>small_joint_1~6</code>; change these to the actual joint names in your SRDF.

2. <b> planning group </b> : the <code>group_name</code> used by solve; verify the group definition in your SRDF.

3. <b> Robot base </b> ( the constructor in <code>avoidance_system.cpp</code> ) : currently Arm A is <code>Ty(700)Rz(180)</code> and Arm B is <code>Ty(-700)</code> , with the two arms facing each other 1400 mm apart — adjust to your actual placement.

4. <b> Bounding-sphere parameters </b> ( the <code>BUBBLES_*</code> / <code>PEDESTAL_*</code> constants in <code>cg_solver.cpp</code> ) : the sphere coordinates and radii of RA610/RA605 — confirm them against your actual robot model.

---

## License

<p align="justify">
MIT
</p>
