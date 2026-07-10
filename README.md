# HIWIN 雙機械手臂 ROS 2（Humble）避障路徑規劃使用手冊

本工作區（`~/hiwin_ws/src/`）共有 **9 個 ROS 2 package**：機器人描述包
`hiwin_dual_arm_description`、MoveIt2 設定包 `hiwin_dual_arm`、
實機跨域橋接包 `dual_arm_domain_bridge`，以及 6 個可選擇的雙臂避障規劃器。
各規劃器的演算法推導與參數對照表，請見其各自的 `README.md` 與 `PARAMETERS.md`。

---

## 1. 這個 workspace 在做什麼

HIWIN 雙機械手臂（A 臂 RA610-1476 與 B 臂 RA605-710，面對面對裝、相距 1400mm）
共用同一工作空間，兩臂的關節軌跡可能互撞。本 workspace 的功能是：給定兩臂各自的
起點與終點關節角，自動規劃出一條兩臂互不碰撞的關節空間軌跡。避障演算法包成
**MoveIt2 規劃器插件**，在 `move_group` 中作為 `planning_plugin` 使用；實機運行時，再由跨域橋接層將軌跡送往兩臂各自的驅動端執行。

---

## 2. Package 總覽

```
~/hiwin_ws/src/
├── hiwin_dual_arm/                  ← MoveIt2 設定包 + 啟動（非演算法）
├── hiwin_dual_arm_description/      ← 雙臂 URDF/mesh 描述包（RA610-1476 + RA605-710 + 底盤）
├── dual_arm_domain_bridge/          ← 實機跨域橋接（joint_states 合併 / 軌跡 action 轉送）
├── dual_arm_alm_newton_planner/     ┐
├── dual_arm_alm_cg_planner/         │  ALM 系列（增廣拉格朗日）
├── dual_arm_alm_gd_planner/         ┘
├── dual_arm_lag_newton_planner/     ┐
├── dual_arm_lag_cg_planner/         │  Lagrangian 系列
└── dual_arm_lag_gd_planner/         ┘
```

6 個規劃器解的是同一個雙臂避障問題，差別只在內層最佳化採用的數學模型與求解方法：

| | Newton | CG（共軛梯度） | GD（最陡下降） |
|---|---|---|---|
| **ALM 模型** | `dual_arm_alm_newton_planner` | `dual_arm_alm_cg_planner` | `dual_arm_alm_gd_planner` |
| **Lagrangian 模型** | `dual_arm_lag_newton_planner` | `dual_arm_lag_cg_planner` | `dual_arm_lag_gd_planner` |

⚠️ ALM 與 Lagrangian 是不同數學模型，不可混用參數。

`hiwin_dual_arm/config/` 底下的 6 份 `dual_arm_*_planning.yaml`，各自將
`planning_plugin` 指向對應的 `DualArmXxxPlannerManager` 並帶入該規劃器的參數；
MoveIt 會依檔名慣例（`*_planning.yaml`）自動掃描並註冊成 planning pipeline，
不需要另外維護清單。

---

## 3. 環境建置（ROS 2 Humble，新機器只做一次）

> 測試環境：Ubuntu 22.04、ROS 2 Humble。

```bash
# ① 建立工作空間，並將本 repo 的 9 個 package 放入 src/
mkdir -p ~/hiwin_ws/src
cd ~/hiwin_ws/src
git clone https://github.com/dalletter2019-tech-Git/Double-arm_obstacle_avoidance.git .    # 或直接把 9 個 package 資料夾複製進來

# ② 安裝依賴（rosdep 依各 package.xml 自動裝 moveit_*、Eigen3 等）
cd ~/hiwin_ws
sudo rosdep init            # 之前 init 過會報錯，忽略即可
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# ③ 編譯整個 workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 4. 實機控制指令

實機採 domain 隔離：主機（規劃端）與兩臂驅動端各在不同的 `ROS_DOMAIN_ID`，
由 `dual_arm_domain_bridge` 負責跨域連接，將兩臂的 `joint_states` 合併後交給主機，
並把 `FollowJointTrajectory` action 的軌跡與執行回報在 domain 之間轉送：

```
D0   主機:  brain.launch.py（六規劃器 + move_group + RViz）
D10  A 臂:  RA610-1476 驅動（hiwin_driver）
D20  B 臂:  RA605-710  驅動（hiwin_driver）
橋層（跑在主機）: joint_states 合併 + FollowJointTrajectory action 透明轉送
```

> 每個終端機開頭，都先完成該小節開頭的**前置**步驟（以下不再重複）。
> 其中 `RMW_IMPLEMENTATION` 三台機器必須一致（跨機通訊的前提）；
> `ROS_DOMAIN_ID` 先確認目前值，再 export 成該終端所需的編號。

### 4.1 主機端（規劃 + 橋接，Domain 0）

**前置**（每個終端機開頭）：

```bash
source ~/hiwin_ws/install/setup.bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
```

**終端 A — 規劃端**：

```bash
ros2 launch hiwin_dual_arm brain.launch.py
```

**終端 B — 橋接層**（軌跡 action 轉送 + joint_states 合併）：

```bash
ros2 launch dual_arm_domain_bridge bridge_relay.launch.py \
     host_domain:=0 arm_a_domain:=10 arm_b_domain:=20
```

橋接建立後，RViz 的 **Plan & Execute / Stop** 即可直接對實機（或虛擬硬體）使用。

### 4.2 手臂端

手臂控制器須事先依上銀官方方式，將 hiwin_ros2（`hiwin_driver`）安裝於 `~/ws_ros2`（見 §7）。

**前置**（每個終端機開頭）：

```bash
# 第一件事：啟動 EtherCAT
sudo /etc/init.d/ethercat start

# 第二件事：source 工作空間與環境檢查
source /opt/ros/humble/setup.bash
cd ~/ws_ros2
source install/setup.bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
```

**A 臂（RA610-1476，Domain 10）**：

```bash
export ROS_DOMAIN_ID=10
taskset -c 2 ros2 launch hiwin_driver ra6_control.launch.py ra_type:=ra610_1476 cabinet:=ecat launch_rviz:=false
```

**B 臂（RA605-710，Domain 20）**：指令同上，改為 `export ROS_DOMAIN_ID=20`、
`ra_type:=ra605_710`。

**解除安全繼電器**（driver 啟動後，於對應臂的 domain 下各執行一次）：

```bash
# A 臂（D10）／B 臂（D20）各做一次
ros2 service call /gpio_controller/set_io hiwin_msgs/srv/SetIO \
     "{io_group: system, interface_name: reset_safety_rly, value: 1.0}"
```

### 4.3 建議啟動順序

```
① A 臂 driver（D10）→ ② B 臂 driver（D20）→（各解除安全繼電器）
→ ③ 橋接層（D0）→ ④ brain.launch.py（D0）→ RViz 規劃/執行
```

---

## 5. 切換規劃器

在 RViz 左側 **MotionPlanning** 面板 → **Context** 分頁 → **Planning Library**
下拉選單中，選擇 6 個 `dual_arm_*` 規劃器之一。

<p align="center"><img src="docs/images/rviz_context.png" alt="RViz 規劃畫面 Context" width="600"></p>

---

## 6. 設定起點 / 終點關節角（跑規劃前必做）

規劃器需要「起點」與「終點」才能計算避障軌跡；RViz 啟動後不會自動設定，需手動指定：

1. **起點**：在 **Planning** 分頁的 **Start State** 下拉選單選 `<current state>`
   ，使起點就是現在位置。同時將 **Displays → Planning Request** 下的
   **Query Start State** 取消勾選，僅保留 **Query Goal State**（如下方截圖）。
2. **終點**：在 3D 畫面直接拖曳 marker 到想要的終點姿態，或在 **Joints** 頁面
   以滑桿調整成精確角度（12 個關節：`big_joint_1~6`、`small_joint_1~6`）。
3. 起點與終點都設定好後按 **Plan**；規劃成功會在 3D 畫面播放軌跡動畫，並於左下角 log 印出耗時。

**內建命名姿態**（`dual_hiwin.srdf` 的 `group_state`，可於 Start/Goal State 下拉選單直接選用）：

| 名稱 | 對應 group | 內容 |
|---|---|---|
| `Dual_ori` | `Dual_arm`（雙臂） | 12 軸全 0（Home 姿態） |

Planning 分頁：Start State 選 `<current state>`，**Query Start State** 取消勾選

<p align="center"><img src="docs/images/rviz_planning.png" alt="RViz 規劃畫面 Planning" width="800"></p>

Joints 頁面：以滑桿逐軸調整目標關節角，與拖曳 marker 兩種方式可互相搭配。

<p align="center"><img src="docs/images/rviz_Joints.png" alt="RViz 規劃畫面 Joints" width="800"></p>

---

## 7. 上銀（HIWIN）官方 ROS 2 Humble 程式庫

本專案結合上銀官方驅動程式庫搭配使用：

| 程式庫 | 說明 |
|---|---|
| [hiwin_ros2](https://github.com/HIWINCorporation/hiwin_ros2) | 官方 ROS 2 Humble 主力庫，提供 `ros2_control` 硬體介面與 MoveIt2 整合，支援 RA6/RS4 系列，可切模擬/實機，本專案兩支手臂是由此套件驅動（本手冊 §4 的 `hiwin_driver` 出自此庫）。 |
| [hiwin_robot_client_library](https://github.com/HIWINCorporation/hiwin_robot_client_library) | 封裝與機器人控制器底層通訊的介面庫，供 `hiwin_ros2` 呼叫。 |
| [hiwin_ros](https://github.com/HIWINCorporation/hiwin_ros) | 舊版 ROS 1 程式庫，非 ROS 2。 |

本 workspace 的 `hiwin_dual_arm_description` 使用了上列官方程式庫中的
`hiwin_description` 套件；其餘套件則參考上銀 hiwin_ros2 的手臂控制套件開發而成。

---

## 8. 授權

各 package 授權於其 `package.xml` 標註；`hiwin_dual_arm_description` 之衍生內容出處見該包 `NOTICE`。
