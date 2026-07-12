# <p align="center">dual_arm_lag_gd_planner</p>

<p align="justify">
Dual-arm collision-avoidance path planner — a ROS 2 Humble / MoveIt2 plugin.
</p>

<p align="justify">
Implemented in C++17 + Eigen3. The inner loop employs <b> a pure Lagrangian formulation (decision variable V=[X;λ;S]) + GD (steepest descent) + 1D Newton line search </b> .
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
Inner level (gd_solver):         single-loop pure Lagrangian (V=[X;λ;S] ∈ ℝ^1116) + GD + line search
```

<p align="justify">
The danger-factor threshold <code>danger_threshold = 0.35</code> and the collision-clearance margin <code>collision_tolerance = 0.15</code> (both are yaml defaults and must be positive). The collision boundary (0.5) = <code>danger_threshold + collision_tolerance</code>; <b> when configuring, the sum of the two must not exceed the collision boundary (0.5) </b> (0.5 = the two spheres are tangent, and exceeding it would fail to detect genuine collisions).
</p>

---

## Architecture: A Single .so

<p align="justify">
All source code (the core algorithm + the MoveIt plugin interface) is compiled into a <b> single shared library </b> <code>libdual_arm_lag_gd_planner.so</code> .
</p>

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Component</th>
      <th align="justify">Role</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>gd_solver</code></td>
      <td align="justify">Layer 1: inner GD solver (V=[X;λ;S]) + FK + bounding spheres + danger factor</td>
    </tr>
    <tr>
      <td align="justify"><code>avoidance_system</code></td>
      <td align="justify">Layer 2: outer collision-repair loop + Spline reconstruction</td>
    </tr>
    <tr>
      <td align="justify"><code>data_io</code></td>
      <td align="justify">CSV writing utility (for inspecting the joint path and the evolution of the danger factor, when needed)</td>
    </tr>
    <tr>
      <td align="justify"><code>planner_manager</code></td>
      <td align="justify">Layer 3: MoveIt2 PlannerManager / PlanningContext</td>
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
│ take the 3 interior points (q1, peak, q3) as X (36-dim), with λ0/S0 forming V0   │
│ call the inner optimizer  run_lag(V0) -> V* = [X*; λ*; S*]                       │
│      |                                                                           │
│      v                                                                           │
│ regenerate_trajectory_global(): local Spline rebuild of the full path via X*     │
│      |                                                                           │
│      v                                                                           │
│ recompute max_D(t), back to "danger check" (up to max_refinement_iter rounds)    │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### Inner Level: Pure Lagrangian Gradient-Descent Solver ( <code>GdSolver::run_lag</code> )

> <div align="justify">In the diagram, the "parameter=value" pairs are illustrative yaml defaults; the actual values are governed by the config.</div>

```
┌───────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ initialize:  V = [X0; λ = lag_lam0(=30); S = lag_s0(=1)]                                              │
│      |                                                                                                │
│      v                                                                                                │
│ ┌─ main loop (up to lag_max_iter=500 steps) ────────────────────────────────────────────────────────┐ │
│ │ compute the KKT residual  G(V) = [G_X; G_λ; G_S]                                                  │ │
│ │ d = -G   (steepest-descent direction)                                                             │ │
│ │ 1D Newton line search for the step length alpha                                                   │ │
│ │ V <- V + alpha·d                                                                                  │ │
│ │      v                                                                                            │ │
│ │ convergence: max_D <= danger_threshold + lag_tol_phys_margin(=0.01)                               │ │
│ │              && |Δmax_D| <= lag_tol_stable(=0.01)                                                 │ │
│ │              (stationarity deliberately disabled — ‖G‖ does not converge at the KKT saddle point) │ │
│ │      --No--> next step                                                                            │ │
│ └─ Yes --> return V* = [X*; λ*; S*], SolverLog ─────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### Principal Variables

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Variable</th>
      <th align="justify">Meaning</th>
      <th align="justify">Dimension</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>V</code></td>
      <td align="justify">Decision variable <code>[X; λ; S]</code> (solved as one package; λ/S are not updated by an outer loop)</td>
      <td align="justify">1116</td>
    </tr>
    <tr>
      <td align="justify"><code>X</code></td>
      <td align="justify">The 3 interior control points of the dangerous segment × 6 axes per arm (degree)</td>
      <td align="justify">36</td>
    </tr>
    <tr>
      <td align="justify"><code>λ</code></td>
      <td align="justify">Inequality-constraint multipliers (one of the decision variables)</td>
      <td align="justify"><code>num_C_</code></td>
    </tr>
    <tr>
      <td align="justify"><code>S</code></td>
      <td align="justify">Slack variables (converting inequalities to equalities via S²)</td>
      <td align="justify"><code>num_C_</code></td>
    </tr>
    <tr>
      <td align="justify"><code>G</code></td>
      <td align="justify">Gradient of the Lagrangian with respect to V (the KKT residual <code>[G_X; G_λ; G_S]</code> )</td>
      <td align="justify">1116</td>
    </tr>
    <tr>
      <td align="justify"><code>d</code></td>
      <td align="justify">Steepest-descent direction <code>-G</code></td>
      <td align="justify">1116</td>
    </tr>
  </tbody>
</table>

<p align="justify">
For the default values and descriptions of every parameter, see <a href="PARAMETERS.md">PARAMETERS.md</a> .
</p>

---

## Directory Structure

<ul>
  <li align="justify" style="margin-bottom: 8px;"><b> Root-directory files </b> ( <code>CMakeLists.txt</code> / <code>package.xml</code> / <code>*.xml</code> / <code>README.md</code> / <code>PARAMETERS.md</code> ) : <b> the build & description layer </b> — defines how the package is compiled and provides the external documentation.</li>
  <li align="justify" style="margin-bottom: 8px;"><b> <code>config/</code> </b> : <b> the runtime-parameter layer </b> — stores the yaml parameters loaded when move_group starts; must be copied as <code>hiwin_dual_arm/config/dual_arm_lag_gd_planning.yaml</code> .</li>
  <li align="justify" style="margin-bottom: 8px;"><b> <code>include/dual_arm_lag_gd_planner/</code> </b> : <b> the interface-declaration layer </b> .</li>
  <li align="justify" style="margin-bottom: 8px;"><b> <code>src/</code> </b> : <b> the algorithm-implementation layer </b> .</li>
</ul>

```
dual_arm_lag_gd_planner/
├── CMakeLists.txt
├── package.xml
├── dual_arm_lag_gd_planner.xml
├── README.md
├── PARAMETERS.md
├── config/
│   └── dual_arm_lag_gd_planner.yaml
├── include/dual_arm_lag_gd_planner/
│   ├── gd_solver.hpp
│   ├── avoidance_system.hpp
│   ├── data_io.hpp
│   └── planner_manager.hpp
└── src/
    ├── gd_solver.cpp
    ├── avoidance_system.cpp
    ├── data_io.cpp
    └── planner_manager.cpp
```

<table width="100%">
  <thead>
    <tr>
      <th align="justify">File</th>
      <th align="justify">Function</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>CMakeLists.txt</code></td>
      <td align="justify">Build configuration: compiles the single .so</td>
    </tr>
    <tr>
      <td align="justify"><code>package.xml</code></td>
      <td align="justify">ROS 2 package description and dependency declarations</td>
    </tr>
    <tr>
      <td align="justify"><code>dual_arm_lag_gd_planner.xml</code></td>
      <td align="justify">pluginlib plugin description (enables dynamic loading by MoveIt2)</td>
    </tr>
    <tr>
      <td align="justify"><code>README.md</code></td>
      <td align="justify">This document</td>
    </tr>
    <tr>
      <td align="justify"><code>PARAMETERS.md</code></td>
      <td align="justify">Parameter reference table (tunable parameters)</td>
    </tr>
    <tr>
      <td align="justify"><code>config/dual_arm_lag_gd_planner.yaml</code></td>
      <td align="justify">Planner parameters (must be copied as <code>hiwin_dual_arm/config/dual_arm_lag_gd_planning.yaml</code> )</td>
    </tr>
    <tr>
      <td align="justify"><code>gd_solver.hpp/.cpp</code></td>
      <td align="justify">Layer 1: FK, bounding spheres, danger factor, pure Lagrangian GD solve</td>
    </tr>
    <tr>
      <td align="justify"><code>avoidance_system.hpp/.cpp</code></td>
      <td align="justify">Layer 2: outer collision-repair loop, Spline reconstruction, CSV export</td>
    </tr>
    <tr>
      <td align="justify"><code>data_io.hpp/.cpp</code></td>
      <td align="justify">CSV writing utility</td>
    </tr>
    <tr>
      <td align="justify"><code>planner_manager.hpp/.cpp</code></td>
      <td align="justify">Layer 3: MoveIt2 plugin interface (radian ↔ degree, time parameterization)</td>
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
colcon build --packages-select dual_arm_lag_gd_planner
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

> <div align="justify">Prerequisite: reside in the same workspace as <code>hiwin_dual_arm</code> (the robot description + MoveIt configuration).</div>

<ol>
  <li align="justify" style="margin-bottom: 8px;">Copy this package's <code>config/dual_arm_lag_gd_planner.yaml</code> as <code>hiwin_dual_arm/config/dual_arm_lag_gd_planning.yaml</code> (the filename differs from planner to planner). The file already contains the complete <code>planning_plugin</code> and <code>request_adapters</code> settings, so copying it suffices — no further pipeline changes are required.</li>
  <li align="justify" style="margin-bottom: 8px;">Launch (example):</li>
</ol>

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
The single public entry point is <code>export_unified(prefix, level)</code> . Output is written to the <code>prefix/&lt;timestamp&gt;_GD/</code> directory (each export forms its own timestamped folder, so exports never overwrite one another).
</p>

<table width="100%">
  <thead>
    <tr>
      <th align="justify">level</th>
      <th align="justify">File</th>
      <th align="justify">Content</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><b> 0 </b></td>
      <td align="justify">(none)</td>
      <td align="justify">Nothing is exported at all (master switch; the current yaml default)</td>
    </tr>
    <tr>
      <td align="justify" rowspan="6"><b> 1 </b></td>
      <td align="justify"><code>meta.csv</code></td>
      <td align="justify">Parameter snapshot (key/value)</td>
    </tr>
    <tr>
      <td align="justify"><code>summary.csv</code></td>
      <td align="justify">Per-round repair scoreboard</td>
    </tr>
    <tr>
      <td align="justify"><code>inner.csv</code></td>
      <td align="justify">Concatenated inner-loop steps (iter / ‖G‖ / ‖d‖ / alpha / λ / S)</td>
    </tr>
    <tr>
      <td align="justify"><code>danger_final.csv</code></td>
      <td align="justify">Initial vs. final per-step MaxD comparison</td>
    </tr>
    <tr>
      <td align="justify"><code>danger_rounds.csv</code></td>
      <td align="justify">Long table: round / step / MaxD_in / MaxD_out</td>
    </tr>
    <tr>
      <td align="justify"><code>targets.csv</code></td>
      <td align="justify">The 5 feature points per round (index + D value + joint angles before/after optimization)</td>
    </tr>
    <tr>
      <td align="justify" rowspan="3"><b> 2 </b></td>
      <td align="justify">The above 6 files + <code>constraints_all.csv</code></td>
      <td align="justify">Full constraint D-value table</td>
    </tr>
    <tr>
      <td align="justify"><code>path_original.csv</code></td>
      <td align="justify">All points of the initial trajectory</td>
    </tr>
    <tr>
      <td align="justify"><code>path_evolution.csv</code></td>
      <td align="justify">Long table: round / step / A1..B6</td>
    </tr>
  </tbody>
</table>

<p align="justify">
On the MoveIt path, export is governed by the yaml keys <code>export_csv_prefix</code> (default <code>./lag_data</code> ) and <code>export_level</code> (default 0 = no export; to retain data, set it to 1 or 2 and restart move_group). All CSV files can be opened directly in Excel or with Python's <code>pandas.read_csv</code> .
</p>

---

## Items Requiring Adjustment

<p align="justify">
Before deployment, adjust the following according to your actual robot:
</p>

<ol>
  <li align="justify" style="margin-bottom: 8px;"><b> Joint names </b> ( <code>solve()</code> in <code>src/planner_manager.cpp</code> ) : currently Arm A uses <code>big_joint_1~6</code> and Arm B uses <code>small_joint_1~6</code>; change these to the actual joint names in your SRDF.</li>
  <li align="justify" style="margin-bottom: 8px;"><b> planning group </b> : solve uses <code>req.group_name</code> ; verify the group definition in your SRDF.</li>
  <li align="justify" style="margin-bottom: 8px;"><b> Robot base </b> ( the constructor in <code>avoidance_system.cpp</code> ) : currently Arm A is <code>Ty(700)Rz(180)</code> and Arm B is <code>Ty(-700)</code> , with the two arms facing each other 1400 mm apart — adjust to your actual placement.</li>
  <li align="justify" style="margin-bottom: 8px;"><b> Bounding-sphere parameters </b> ( the <code>BUBBLES_*</code> / <code>PEDESTAL_*</code> constants in <code>gd_solver.cpp</code> ) : the sphere coordinates and radii of RA610/RA605 — confirm them against your actual robot model.</li>
</ol>

---

## License

<p align="justify">
MIT
</p>
