# dual_arm_alm_gd_planner_1

雙臂避障路徑規劃器 — ROS 2 Humble / MoveIt2 插件。

由 MATLAB 原型（`Dual_Arm_avoidance_system_v3` + `Dual_Arm_Inequality_ALM_GD_v6`）轉寫為 C++17，使用 Eigen3。內層採用 **ALM (PHR 增廣拉格朗日) + GD (最陡下降梯度法) + 1D Newton 線搜索**，無 Hessian、無 LDLT。

---

## 演算法概觀

兩隻機械臂（RA610 16 球 / RA605 18 球包覆模型），給定起點與終點關節角，求一條兩臂互不碰撞的關節空間軌跡。

採雙層最佳化：

```
外層 (avoidance_system):  生成初始軌跡 -> 碰撞偵測 -> 找危險段 5 點 ->
                          呼叫內層優化 -> Spline 重建 -> 重新檢查 (最多 15 輪)
內層 (gd_solver):         ALM 外層 (罰參數/乘子更新) + GD 內層 + 線搜索,
                          以平滑危險因子 sj = exp(ln0.5/(Ri+Rj)^2 * d^2) 為約束
```

危險因子閾值 `danger_threshold = 0.4`；碰撞判定用 `0.4 + 0.1 = 0.5`（保留緩衝帶）。

---

## 架構：單一 .so

所有原始碼（核心演算法 + MoveIt 插件介面）編進**單一共享庫** `libdual_arm_alm_gd_planner_1.so`。

| 組成 | 角色 |
|------|------|
| `gd_solver` | 第 1 層: 內層 GD 求解器 + FK + 包覆球 + 危險因子 |
| `avoidance_system` | 第 2 層: 外層碰撞修復迴圈 + Spline 重建 |
| `data_io` | CSV 寫入工具 |
| `planner_manager` | 第 3 層: MoveIt2 PlannerManager / PlanningContext |

另編出一個執行檔（連同一個 .so）：`run_standalone`（讀 waypoints 跑避障匯出 CSV）。

> **為何單一 .so（而非拆成 core.so + plugin.so）**
> 早期版本曾把核心演算法拆成獨立的 `dual_arm_avoidance_core.so`。但拆成兩個 .so 時，含 Eigen 矩陣的 `Trajectory` 物件需跨 .so 邊界傳遞，與 MoveIt 的 .so 之間易發生記憶體對齊不一致而崩潰。合併為單一 .so 後，Eigen 物件不跨自己的 .so 邊界，較穩定。代價是失去「純 Eigen、零 MoveIt 依賴」的獨立核心庫（standalone/test 改連這個含 MoveIt 依賴的主 .so）。

---

## 目錄結構

```
dual_arm_alm_gd_planner_1/
├── CMakeLists.txt
├── package.xml
├── dual_arm_alm_gd_planner_1.xml        # plugin 描述
├── README.md
├── config/
│   └── dual_arm_alm_gd_planner_1.yaml   # 規劃器參數
├── include/dual_arm_alm_gd_planner_1/
│   ├── gd_solver.hpp                       # 第 1 層: 內層 GD 求解器
│   ├── avoidance_system.hpp                # 第 2 層: 外層碰撞修復
│   ├── data_io.hpp                         # CSV 寫入工具
│   └── planner_manager.hpp                 # 第 3 層: MoveIt2 介面
├── src/
│   ├── gd_solver.cpp
│   ├── avoidance_system.cpp
│   ├── data_io.cpp
│   └── planner_manager.cpp
└── standalone/
    ├── run_standalone.cpp                  # 獨立執行 (跑避障匯出 CSV)
```

---

## 編譯

放進 ROS 2 workspace 的 `src/` 後：

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_alm_gd_planner_1
source install/setup.bash
```

需求：ROS 2 Humble、MoveIt2、Eigen3、C++17 編譯器。

### ⚠️ 編譯選項：使用 `-O3`，**不要用 `-march=native`**

CMake 預設 Release `-O3 -DNDEBUG`（**刻意不含 `-march=native`**）。

> **為何不用 -march=native**
> `-march=native` 會讓 Eigen 啟用 AVX，將矩陣對齊到 32-byte；而 MoveIt 的 .so 是用標準選項編的（16-byte 對齊）。當含 Eigen 矩陣的 `Trajectory` 在本插件與 MoveIt 之間傳遞/析構時，兩邊對齊假設不一致，會在碰撞路徑 `regenerate_trajectory_global` 回傳大矩陣後於 `__libc_free` 崩潰（Segmentation fault）。
> 實測：去掉 `-march=native`（僅 `-O3`）即穩定，且規劃僅需約 0.02 秒，速度差異可忽略。
> 部署前可確認：`grep add_compile_options CMakeLists.txt` 應只見 `-O3 -DNDEBUG`，無 `march=native`。

---

## 使用方式

### A. 獨立執行（用核心演算法，仍需在含 MoveIt 的環境編譯）

最快上手，用來開發、除錯、產實驗資料：

```bash
ros2 run dual_arm_alm_gd_planner_1 run_standalone myout
# 或直接執行: ./install/.../lib/dual_arm_alm_gd_planner_1/run_standalone myout
```

waypoints 寫在 `standalone/run_standalone.cpp` 的 `main()` 內（全 degree），要換 case 改數值重編即可。輸出一系列 `myout_*.csv`。

回傳碼：`0` = 成功避障，`1` = 仍碰撞。

### B. 在自己的 C++ 程式呼叫 AvoidanceSystem

```cpp
#include "dual_arm_alm_gd_planner_1/avoidance_system.hpp"
using namespace dual_arm_alm_gd_planner_1;

Eigen::MatrixXd A(2,6), B(2,6);   // 每臂 2 列: 起點 / 終點 (degree)
A << -30,-30.8,38.6,0,-7.8,0,  30,-30.8,38.6,0,-7.8,0;
B << -30,-19.8,-29.8,0,49.6,0,  30,-19.8,-29.8,0,49.6,0;

AvoidanceSystem sys(A, B, /*path_weight=*/0.5, /*danger_threshold=*/0.4);
sys.run_optimization();

if (!sys.has_collision()) {
    auto traj = sys.get_optimized_trajectory();   // traj.pos: (T x 12)
    sys.export_full_log("result");
}
```

> 註：核心類別 `AvoidanceSystem` / `GdSolver` 本身只依賴 Eigen（程式碼層面無 ROS/MoveIt 呼叫）。但因合併為單一 .so，連結時會帶到 MoveIt 依賴，故需在含 MoveIt 的環境編譯。

### C. MoveIt2 插件

在 move_group 設定規劃插件：

```yaml
planning_plugin: dual_arm_alm_gd_planner_1/DualArmAlmGdPlannerManager
```

並載入 `config/dual_arm_alm_gd_planner_1.yaml` 的參數。規劃請求需含 joint goal constraints。

---

## CSV 匯出（除畫圖外的全部資料）

| 函式 | 輸出檔 | 內容 |
|------|--------|------|
| `export_danger_factor(p)` | `p_danger.csv` 之類 | 原始 vs 優化每步 max_D 對照 |
| `export_full_log(p)` | `p_Summary` / `p_Path_*` / `p_D_*` / `p_Targets_*` | 每輪修復完整追蹤 |
| `export_trajectory_data(p)` | `p_Meta` / `p_Original_*` / `p_Iter_*` / `p_Targets` | 繪圖工具用 |
| `export_diagnostics(p)` | `p_diag_summary` / `p_diag_inner` | ALM 收斂診斷 |

MATLAB 的單一 xlsx 多 sheet → 此處改為多個 CSV 檔（純 std，零依賴）。所有 CSV 可用 Excel、MATLAB `readtable`、Python `pandas.read_csv` 直接開啟。

---

## 需要調整的地方

部署前請依實際機器人調整：

1. **關節名**（`src/planner_manager.cpp` 的 `solve()`）：目前用 A 臂 `big_joint_1~6`、B 臂 `small_joint_1~6`，改成你 SRDF 的實際關節名。
2. **planning group**：solve 用 `req.group_name`，確認 SRDF 的 group 設定。
3. **機器人底座**（`avoidance_system.cpp` 建構函數）：目前 A 臂 `Ty(700)Rz(180)`、B 臂 `Ty(-700)`，兩臂相距 1400mm 面對面 — 依實際擺位調整。
4. **包覆球參數**（`gd_solver.cpp` 的 `BUBBLES_*` / `PEDESTAL_*` 常數）：RA610/RA605 的球座標與半徑，依實際機型確認。
5. **package.xml maintainer**：改成你的 email。

---

## 與 MATLAB 原型的對應與差異

數值行為已對照驗證一致（同一 case 下 ALM 每圈 f / MaxD / v_pure / compl / c 吻合至小數點後 5 位，收斂圈數、最終 max_D 相同）。

主要實作差異：

- **索引**：MATLAB 1-indexed → C++ 0-indexed（邊界保護對應調整）。同一物理點在兩邊編號差 1。
- **Spline**：MATLAB 內建 `spline`（clamped 模式）→ C++ 手刻 clamped cubic spline（解三對角系統求二階導，再分段三次插值）；演算法相同。
- **線性求解**：用 Eigen `colPivHouseholderQr`（spline 三對角）；內層 GD 本身不解線性系統。
- **匯出**：MATLAB xlsx 多 sheet / .mat → C++ 多 CSV 檔。
- **不轉換**：MATLAB 的繪圖函式（`draw_df_bubble` / `animate_*` / `plot_*`）未轉寫；改以 CSV 匯出供外部工具繪圖。

### regenerate_trajectory_global 對齊重點

此函式（局部 Spline 重建）忠實對照 MATLAB，幾個關鍵：

- `num_patch = Patch_Pos.rows() - 2`（用 spline **實際**回傳行數，對應 MATLAB `size(patch_core,1)`）— **不可**用預期值 `T_patch - 2`，否則 spline 實際行數與預期不符時會讀越界崩潰。
- `num_head = targets.front() + 1`：MATLAB `targets(1)`（1-indexed）保留含 Head 的前段；C++ 0-indexed 下「前 N 列」的 N = 位置 + 1。
- 退化分支 `if (valid_rows.size() < 2)`：anchor 幾乎全重合時，比照 MATLAB 用頭尾兩點 + `t_knots=[0;1e-6]` 建極短 spline。
- 拼接用 Eigen 區塊 `topRows / middleRows / bottomRows`（對應 MATLAB 整段賦值），尺寸不符會丟例外而非默默越界。

---

## 已知踩雷筆記

- **不要用 `-march=native`**（見上「編譯選項」），會與 MoveIt 的 Eigen 對齊衝突而崩潰。
- **核心庫的 `std::cout`**：建構函數開頭設 `std::cout << std::unitbuf` 關閉緩衝，確保 `[Init]/[Outer]` 等訊息在 MoveIt（非終端機）環境下即時輸出，不被緩衝累積成「有時不印」。
- **MoveIt 用多執行緒執行器**：核心庫的 static 成員皆為 `const`（唯讀），多執行緒讀取安全。

---

## 授權

MIT
