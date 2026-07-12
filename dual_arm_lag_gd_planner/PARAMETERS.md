# <p align="center">Parameter Reference — dual_arm_lag_gd_planner</p>

<p align="justify">
All tunable parameters of this planner reside in <code>config/dual_arm_lag_gd_planning.yaml</code> (this table = the yaml comments + supplementary notes). Modification workflow: edit the yaml → copy it to <code>hiwin_dual_arm/config/</code> → restart move_group for the changes to take effect ( <b> no recompilation needed </b> ).
</p>

---

## Commonly Tuned Parameters

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Parameter</th>
      <th align="justify">Default</th>
      <th align="justify">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>path_weight</code></td>
      <td align="justify"><code>0.5</code></td>
      <td align="justify">A/B arm cost weight <code>pw·fA + (1-pw)·fB</code> , range [0,1]. <b> Larger </b> → the avoidance amplitude of RA610-1476-GC2 (big_arm) becomes smaller and that of RA605-710-GC2 (small_arm) becomes larger; <b> smaller </b> → big_arm's avoidance amplitude becomes larger and small_arm's becomes smaller</td>
    </tr>
    <tr>
      <td align="justify"><code>danger_threshold</code></td>
      <td align="justify"><code>0.35</code></td>
      <td align="justify">Danger-factor threshold (the optimization objective); this value + <code>collision_tolerance</code> = collision boundary (0.5), and their sum must not exceed 0.5 (exceeding it would miss genuine collisions)</td>
    </tr>
    <tr>
      <td align="justify"><code>collision_tolerance</code></td>
      <td align="justify"><code>0.15</code></td>
      <td align="justify">Collision-detection buffer band (= collision boundary (0.5) − <code>danger_threshold</code> )</td>
    </tr>
    <tr>
      <td align="justify"><code>fix_tolerance</code></td>
      <td align="justify"><code>0.1</code></td>
      <td align="justify">The leading/trailing clearance (fix_gap) ratio used by find_targets</td>
    </tr>
    <tr>
      <td align="justify"><code>max_refinement_iter</code></td>
      <td align="justify"><code>15</code></td>
      <td align="justify">Maximum number of outer repair rounds</td>
    </tr>
  </tbody>
</table>

---

## Trajectory Smoothing Weights (the larger the smoothing weight, the smoother the path)

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Parameter</th>
      <th align="justify">Default</th>
      <th align="justify">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>smooth_w</code></td>
      <td align="justify"><code>0.3</code></td>
      <td align="justify">Primary smoothing weight: the weight on the distance from the original configuration ‖Xm−Xm^ori‖²</td>
    </tr>
    <tr>
      <td align="justify"><code>smooth_w_H</code></td>
      <td align="justify"><code>1.0</code></td>
      <td align="justify">Head-end weight: the distance between the first joint configuration and the Head end ‖X1−XH‖² (recommended to match <code>smooth_w_T</code> and <code>smooth_w_neighbor</code> )</td>
    </tr>
    <tr>
      <td align="justify"><code>smooth_w_T</code></td>
      <td align="justify"><code>1.0</code></td>
      <td align="justify">Tail-end weight: the distance between the third joint configuration and the Tail end ‖X3−XT‖² (recommended to match <code>smooth_w_H</code> and <code>smooth_w_neighbor</code> )</td>
    </tr>
    <tr>
      <td align="justify"><code>smooth_w_neighbor</code></td>
      <td align="justify"><code>1.0</code></td>
      <td align="justify">Neighbor weight: the distance between two adjacent configurations ‖X_(m+1)−X_m‖² (recommended to match <code>smooth_w_H</code> and <code>smooth_w_T</code> )</td>
    </tr>
  </tbody>
</table>

---

## Pure Lagrangian Parameters (corresponding to Dual_Arm_Lagrangian_Gradient_v2)

> <div align="justify"><b> Tuning guide </b> :</div>
> <ul>
>   <li align="justify" style="margin-bottom: 8px;"><b> Fails to converge in time </b> (all <code>lag_max_iter</code> steps are used up before the target is met) → increase <code>lag_max_iter</code> .</li>
> </ul>

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Parameter</th>
      <th align="justify">Default</th>
      <th align="justify">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>lag_lam0</code></td>
      <td align="justify"><code>30.0</code></td>
      <td align="justify">Initial value of λ₀ (the λ vector is rebuilt from it; original value 30)</td>
    </tr>
    <tr>
      <td align="justify"><code>lag_max_iter</code></td>
      <td align="justify"><code>500</code></td>
      <td align="justify">Maximum iterations of the main loop (a single-level structure, with no outer/inner distinction). <b> Larger </b> → grants a greater convergence budget</td>
    </tr>
  </tbody>
</table>

---

## Joint-Name Prefixes (adjust per SRDF)

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Parameter</th>
      <th align="justify">Default</th>
      <th align="justify">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>joint_prefix_A</code></td>
      <td align="justify"><code>"big_joint_"</code></td>
      <td align="justify">Arm A (RA610): big_joint_1 ~ big_joint_6</td>
    </tr>
    <tr>
      <td align="justify"><code>joint_prefix_B</code></td>
      <td align="justify"><code>"small_joint_"</code></td>
      <td align="justify">Arm B (RA605): small_joint_1 ~ small_joint_6</td>
    </tr>
  </tbody>
</table>

---

## Time Parameterization (handled inside the plugin)

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Parameter</th>
      <th align="justify">Default</th>
      <th align="justify">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>time_optimal</code></td>
      <td align="justify"><code>true</code></td>
      <td align="justify">true = TOTG time-optimal parameterization; false = custom equal-interval spacing</td>
    </tr>
    <tr>
      <td align="justify"><code>path_total_time</code></td>
      <td align="justify"><code>20.0</code></td>
      <td align="justify">(when time_optimal=false) the target total trajectory time (seconds)</td>
    </tr>
    <tr>
      <td align="justify"><code>min_time_interval</code></td>
      <td align="justify"><code>0.05</code></td>
      <td align="justify">(when time_optimal=false) the minimum time interval per point (seconds)</td>
    </tr>
  </tbody>
</table>

<ul>
  <li align="justify" style="margin-bottom: 8px;"><code>time_optimal: true</code> → uses TOTG to compute timestamps from the joint velocity/acceleration limits (the acceleration limits must be set in joint_limits.yaml; otherwise default values are used and a warning is printed).</li>
  <li align="justify" style="margin-bottom: 8px;"><code>time_optimal: false</code> → equal intervals <code>dt = path_total_time / (num_points − 1)</code> , where dt is never smaller than <code>min_time_interval</code> (when there are too many points, the actual total time will exceed <code>path_total_time</code> ).</li>
</ul>

---

## Diagnostic Output (all disabled by default)

<table width="100%">
  <thead>
    <tr>
      <th align="justify">Parameter</th>
      <th align="justify">Default</th>
      <th align="justify">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="justify"><code>export_csv_prefix</code></td>
      <td align="justify"><code>"./lag_data"</code></td>
      <td align="justify">Export root directory: <code>&lt;prefix&gt;/&lt;unix-seconds&gt;_GD/</code> (each export forms its own timestamped folder, so exports never overwrite one another; nothing is exported when export_level=0)</td>
    </tr>
    <tr>
      <td align="justify" rowspan="3"><code>export_level</code></td>
      <td align="justify" rowspan="3"><code>0</code></td>
      <td align="justify">0 = nothing exported at all (master switch)</td>
    </tr>
    <tr>
      <td align="justify">1 = the standard 6 files (meta / summary / inner / danger_final / danger_rounds / targets)</td>
    </tr>
    <tr>
      <td align="justify">2 = the full 9 files (+ constraints_all / path_original / path_evolution)</td>
    </tr>
  </tbody>
</table>
