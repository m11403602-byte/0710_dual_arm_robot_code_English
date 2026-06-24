# 參數對照表 — dual_arm_alm_gd_planner_1

本文件列出所有參數：外部可調 vs 寫死、預設值、所在位置、對應 MATLAB。

對應 MATLAB 原型：`Dual_Arm_avoidance_system_v3.m`（外層）+ `Dual_Arm_Inequality_ALM_GD_v6.m`（內層）。

---

## 一、外部輸入（不用改程式碼）

### 1-1. 規劃請求 / 命令列

| 參數 | 來源 | 預設 | 說明 |
|------|------|------|------|
| 起點關節角 | MoveIt `start_state` / standalone | — | A/B 臂各 6 軸（介面用 radian，內部 degree）|
| 終點關節角 | MoveIt `goal_constraints` / standalone | — | A/B 臂各 6 軸 |
| CSV filename | standalone 命令列 | `standalone_out` | 輸出資料夾/前綴 |

### 1-2. yaml 可調參數（共 11 個，改 yaml 重啟 move_group 生效）

| 參數 | 預設 | 對應成員 | 說明 |
|------|------|---------|------|
| `path_weight` | 0.5 | path_weight_ | A/B 臂成本權重 `pw·fA + (1-pw)·fB`，[0,1] |
| `danger_threshold` | 0.4 | danger_threshold_ | 危險因子閾值（判撞門檻 = 此值 + collision_tolerance）|
| `collision_tolerance` | 0.1 | collision_tolerance_ | 碰撞判定緩衝帶 |
| `fix_tolerance` | 0.1 | fix_tolerance_ | find_targets 的 fix_gap 比例 |
| `max_refinement_iter` | 15 | max_refinement_iter_ | 外層修復最多輪數 |
| `smooth_w` | 0.3 | smooth_w_ | 平滑項主權重 |
| `smooth_w_H` | 1.0 | smooth_w_H_ | Head 端權重 |
| `smooth_w_T` | 1.0 | smooth_w_T_ | Tail 端權重 |
| `smooth_w_neighbor` | 1.0 | smooth_w_neighbor_ | 鄰點權重 |
| `joint_prefix_A` | "big_joint_" | joint_prefix_A_ | A 臂關節名前綴（依 SRDF）|
| `joint_prefix_B` | "small_joint_" | joint_prefix_B_ | B 臂關節名前綴（依 SRDF）|
| `time_optimal` | true | time_optimal_ | true=TOTG 時間最佳化; false=自訂等間隔 |
| `path_total_time` | 5.0 | path_total_time_ | （time_optimal=false 時）目標軌跡總時間（秒）|
| `min_time_interval` | 0.05 | min_time_interval_ | （time_optimal=false 時）每點最小時間間隔（秒）|

> 傳遞路徑：yaml → planner_manager（initialize 讀入）→ AvoidanceSystem 建構函數 → GdSolver 建構函數（smooth_w 系列）。standalone/test 用 4 參數呼叫仍相容（新參數有預設值）。
> 其餘參數（ALM 內部、機器人幾何等）皆寫死，需改原始碼重編。

### 1-3. 時間參數化（在插件 solve() 內處理）

時間參數化把幾何路徑（一串關節角）加上時間軸，決定機器人執行時每點之間隔多久。

- `time_optimal: true` → 用 **TOTG**，依關節速度/加速度限制算時間戳。需在 joint_limits.yaml 設加速度限制，否則用預設 1 rad/s²（會印警告）。
- `time_optimal: false` → **自訂等間隔**：`dt = path_total_time / (點數-1)`，但 `dt` 不小於 `min_time_interval`。若點數多到算出的 dt < min_time_interval，則強制用 min_time_interval（此時實際總時間會超過 path_total_time）。

> ⚠️ **重要**：因時間參數化改在插件內做，必須從 MoveIt pipeline 的 `request_adapters` **移除** `default_planner_request_adapters/AddTimeOptimalParameterization`，否則 adapter 會再跑一次 TOTG，覆蓋 `time_optimal=false` 的等間隔設定。

### 1-4. 三個時間輸出（solve() 印出，意義不同）

| 輸出 | 量什麼 | 單位 | 用途 |
|------|--------|------|------|
| `[純路徑規劃時間]` | 避障 run_optimization + 軌跡轉換（不含時間參數化）| 秒 | 看避障演算法快慢 |
| `[軌跡規劃時長]` | 純路徑規劃 + 時間參數化**計算**耗時（= 回報給 MoveIt 的 planning_time）| 秒 | 看整個生成軌跡花多久（電腦計算）|
| `[軌跡時長]` | 機器人**執行**這條軌跡要花的時間（getDuration）| 秒 | 看機器人動作多久 |

> 注意「軌跡規劃時長」是電腦**計算**花的時間（通常零點零幾秒），「軌跡時長」是機器人**執行**花的時間（數秒），兩者完全不同。

---

## 二、外層常數（avoidance_system）

寫死於 `src/avoidance_system.cpp` 建構函數 / `include/.../avoidance_system.hpp` 成員。
標 ✅ 者已開放為 yaml 可調（見第一節），其餘寫死。

| 參數 | 值 | yaml? | 對應 MATLAB | 說明 |
|------|-----|-------|-------------|------|
| `collision_tolerance_` | 0.1 | ✅ | `collision_tolerance=0.1` | 碰撞判定緩衝（判撞門檻 = 0.4+0.1 = 0.5）|
| `fix_tolerance_` | 0.1 | ✅ | `fix_tolerance=0.1` | find_targets 的 fix_gap 比例 |
| `max_refinement_iter_` | 15 | ✅ | `max_refinement_iter=15` | 外層修復最多輪數 |
| `danger_threshold_` | 0.4 | ✅ | `DANGER_THRESHOLD=0.4` | 危險因子閾值 |
| `path_weight_` | 0.5 | ✅ | `path_weight=0.5` | A/B 臂成本權重 |
| `STEP_MAX_DEG_` | 0.5 | ✗ 寫死 | `STEP_MAX_DEG=0.5` | 軌跡切分最大角度差（度）|
| robotA_base | Ty(700)·Rz(180°) | ✗ 寫死 | A 臂 base | A 臂底座變換 |
| robotB_base | Ty(-700)·Rz(0°) | ✗ 寫死 | B 臂 base | B 臂底座變換（兩臂相距 1400mm 面對面）|

---

## 三、內層 ALM 參數（gd_solver）

寫死於 `include/.../gd_solver.hpp` 成員預設值。**全部與 MATLAB GD v6 一致**。

| 參數 | 值 | 位置 | 對應 MATLAB | 說明 |
|------|-----|------|-------------|------|
| `mu_`（mu_0）| 10.0 | cpp:290 | `mu_0=10` | 初始拉格朗日乘子（dual warm-start）|
| `c_`（c_0）| 5.0 | hpp:146 | `c_0=5` | 初始罰參數 |
| `c_max_` | 1e5 | hpp:147 | `c_max=1e5` | 罰參數上限 |
| `mu_max_safeguard_` | 1e8 | hpp:148 | `mu_max=1e8` | 乘子安全上限（超出則 reset 0）|
| `beta_c_` | 8.0 | hpp:149 | `beta_c=8` | 罰參數放大倍率 |
| `gamma_v_` | 0.5 | hpp:150 | `gamma_v=0.5` | violation 改善門檻 |
| `epsilon_v_` | 0.01 | hpp:151 | `eps_v=0.01` | 主可行性收斂閾值 |
| `epsilon_g_` | 0.1 | hpp:152 | `eps_g=0.1` | 一階最佳（梯度）收斂閾值 |
| `epsilon_compl_` | 0.1 | hpp:153 | `eps_compl=0.1` | 互補性收斂閾值 |
| `epsilon_inner_` | 1.0 | hpp:154 | `eps_inner=1.0` | 內層精度（起始）|
| `epsilon_inner_min_` | 0.1 | hpp:155 | → 0.1 | 內層精度下限 |
| `epsilon_inner_decay_` | 0.5 | hpp:156 | decay 0.5 | 內層精度遞減率 |
| `K_outer_` | 50 | hpp:157 | `K_outer=50` | 外層 ALM 最大輪數 |
| `K_inner_` | 200 | hpp:158 | `K_inner=200` | 內層 GD 最大迭代 |
| `K_inner_first_` | 200 | hpp:159 | `K_inner_first=200` | 首輪內層最大迭代 |

> KKT 三條件收斂（須同時）：v_pure ≤ eps_v、‖grad L‖ ≤ eps_g、compl ≤ eps_compl。

---

## 四、平滑項權重（gd_solver）

**已開放為 yaml 可調**（見第一節）。控制軌跡平滑度，預設與 MATLAB 一致。
傳遞：yaml → AvoidanceSystem → GdSolver 建構函數。

| 參數 | 值 | yaml? | 對應 MATLAB | 說明 |
|------|-----|-------|-------------|------|
| `smooth_w_` | 0.3 | ✅ | `smooth_w=0.3` | 平滑項主權重 |
| `smooth_w_H_` | 1.0 | ✅ | `smooth_w_H=1.0` | Head 端權重 |
| `smooth_w_T_` | 1.0 | ✅ | `smooth_w_T=1.0` | Tail 端權重 |
| `smooth_w_neighbor_` | 1.0 | hpp:138 | `smooth_w_neighbor=1.0` | 鄰點權重 |

---

## 五、線搜索與數值保護（gd_solver）

寫死於 `src/gd_solver.cpp` run_alm 內。

| 參數 | 值 | 位置 | 說明 |
|------|-----|------|------|
| `LS_DELTA` | 0.001 | cpp:464 | 1D Newton 線搜索差分步長 |
| `FALLBACK_ALPHA` | 0.0001 | cpp:468 | 線搜索失敗時退路步長 |
| `FAIL_CHECK_IT` | 10 | cpp:469 | 失敗檢查迭代數 |
| 發散門檻 | ‖G‖ > 1e10 | cpp:530 | 梯度爆炸視為發散 |
| `delta`（MATLAB）| 0.01 | — | MATLAB 對應 LS 步長（C++ 用 LS_DELTA）|

---

## 六、危險因子公式（gd_solver）

寫死於 `src/gd_solver.cpp` `calc_df()`。**核心避障數學**。

| 參數 | 值 | 位置 | 對應 MATLAB | 說明 |
|------|-----|------|-------------|------|
| 危險因子公式 | `sj = exp(ln(0.5)/(Ri+Rj)²·d²)` | cpp:116 | `calc_df` | 球對危險因子（d=球心距，R=半徑）|
| `ln(0.5)` | -0.6931 | cpp:107 | `log(0.5)` | 衰減係數（球心距 = Ri+Rj 時 sj=0.5）|

---

## 七、Spline 重建數值閾值（avoidance_system）

寫死於 `src/avoidance_system.cpp`。

| 參數 | 值 | 位置 | 說明 |
|------|-----|------|------|
| 距離防除零 | 1e-4 → 1e-6 | cpp:138 | 初始 spline 弦長過小時的下限 |
| 重合過濾閾值 | 1e-4 | cpp:354 | regenerate 的 valid_mask（段距離 > 1e-4 才有效）|
| 退化 t_knots | 1e-6 | cpp:366 | anchor 全重合時的退化 spline 參數 |

---

## 八、機器人幾何（gd_solver）

寫死於 `src/gd_solver.cpp`。換機器人需整批修改。

| 項目 | 內容 | 位置 |
|------|------|------|
| RA610 (A 臂) 底盤球 | 4 球，R=320 | cpp:19 `PEDESTAL_A` |
| RA610 手臂球 | 12 球 | cpp:27 `BUBBLES_A` |
| RA605 (B 臂) 底盤球 | 8 球，R=350 | cpp:43 `PEDESTAL_B` |
| RA605 手臂球 | 10 球 | `BUBBLES_B` |
| 球→連桿映射 | sA[16]、sB[18] | cpp:129-130 |
| RA610 FK chain | 7 連桿 DH 變換 | `robot_arm_bubble_RA610` |
| RA605 FK chain | 7 連桿 DH 變換 | `robot_arm_bubble_RA605` |

> 球總數：A 臂 16（4 底盤 + 12 手臂）、B 臂 18（8 底盤 + 10 手臂）。

---

## 九、決策變數維度（gd_solver，執行期決定）

| 參數 | 值（本案）| 說明 |
|------|----------|------|
| M（中間優化點）| 3 | 不含 Head/Tail |
| num_X | 36 = 2×3×6 | 2 臂 × 3 點 × 6 軸 |
| K_AB（跨臂約束）| 180 | 16×18 經 mask 過濾後 |
| num_C | 540 = 3 × 180 | M × num_D |
| self_collision | false | 本版不含自我碰撞（num_D = K_AB）|

---

## 十、關節名（planner_manager）

**已開放為 yaml 可調**（`joint_prefix_A` / `joint_prefix_B`，見第一節）。
程式用 `prefix + 數字` 組成關節名，改 yaml 即可適配不同 SRDF，不用重編。

| 參數 | yaml 值 | 組成的關節名 | 說明 |
|------|---------|------------|------|
| `joint_prefix_A` | "big_joint_" | `big_joint_1` ~ `big_joint_6` | A 臂，須與 SRDF 一致 |
| `joint_prefix_B` | "small_joint_" | `small_joint_1` ~ `small_joint_6` | B 臂，須與 SRDF 一致 |

---

## 摘要：真正常需調整的

| 場景 | 要調的參數 | 在哪 |
|------|-----------|------|
| 改避障鬆緊 | `danger_threshold` | ✅ yaml（不用重編）|
| 改 A/B 臂優先 | `path_weight` | ✅ yaml |
| 改軌跡平滑度 | `smooth_w` 系列 | ✅ yaml |
| 改修復輪數 | `max_refinement_iter` | ✅ yaml |
| 改碰撞緩衝 | `collision_tolerance` | ✅ yaml |
| 換機器人關節名 | `joint_prefix_A/B` | ✅ yaml |
| 切換時間參數化方式 | `time_optimal` | ✅ yaml |
| 自訂軌跡總時間 | `path_total_time` / `min_time_interval` | ✅ yaml |
| 改軌跡密度 | `STEP_MAX_DEG_` | ✗ avoidance_system.hpp（重編）|
| 改收斂嚴格度 | `epsilon_v/g/compl` | ✗ gd_solver.hpp（重編）|
| 改 ALM 收斂行為 | `c_0/mu_0/beta_c` | ✗ gd_solver.hpp（重編，進階）|
| 換機器人幾何 | 包覆球 + FK + base | ✗ gd_solver.cpp + planner_manager.cpp（重編）|

> ✅ = yaml 可調（改 yaml 重啟 move_group 生效）；✗ = 寫死，需改原始碼重編。
> 目前共 **14 個參數**開放為 yaml 可調。



## 診斷輸出參數（yaml, 運行期動態, [NEW]）

| 參數 | 預設 | 說明 |
|------|------|------|
| `export_csv_prefix` | `""`（關） | 非空 → 每次規劃後匯出 CSV 三組（`<prefix>_<unix秒>_danger.csv`、ALM 診斷、軌跡繪圖）；規劃失敗也匯出 |
| `solver_verbose` | `false` | 內層逐迭代記錄 G/d 全向量（耗記憶體；深掘/論文用） |

### [NEW] export_unified 整合匯出（取代插件路徑的舊三組匯出）

| 參數 | 預設 | 說明 |
|------|------|------|
| `export_level` | 1 | **0=完全不匯出（總開關）**；1=論文標配 6 檔；2=+constraints_all / path_original / path_evolution 共 9 檔 |

輸出目錄：`<export_csv_prefix>/<unix秒>_<solver>/`，檔案數固定不隨修復輪數膨脹（長表 + round 欄設計）。舊四匯出器已刪除（standalone 亦改走 export_unified level 2）。

## ALM 參數（yaml, 運行期動態, [NEW] — 原等級 3 硬編碼開放為參數）

| 參數 | GD 預設 | 說明 |
|------|------|------|
| `alm_mu0` | 10 | 乘子 warm-start 初值（mu_ 向量以此重建）；⚠ 勿超過 1e8（程式內固定的 mu_max_safeguard）|
| `alm_c0` | 5 | 罰參數初值；⚠ 勿超過 `alm_c_max` |
| `alm_c_max` | 100000 | 罰參數上限 |
| `alm_beta_c` | 8 | 罰參數放大係數 |
| `alm_gamma_v` | 0.5 | 放大觸發比（程式內箝位至 (0,1)，非法值回落 0.5）|

五參數全數寫入每次規劃的 meta.csv（可追溯性）。`mu_max_safeguard` 為 Birgin 理論保護值，固定 1e8 於 solver 內，不開放調整。發散急救順序：先降 `alm_beta_c`，再降 `alm_c_max`，再動 `alm_c0`。
