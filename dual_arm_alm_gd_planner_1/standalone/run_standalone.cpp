// =====================================================================
// run_standalone.cpp — 獨立執行檔 (不依賴 MoveIt / ROS)
// =====================================================================
//   直接呼叫核心庫 (AvoidanceSystem), 跑避障 + 匯出 CSV
//   用途: 開發/除錯/產實驗資料, 不用啟動 ROS/MoveIt
//
//   用法:
//     ./run_standalone                        # 用內建測試 waypoints
//     ./run_standalone <out_prefix>           # 指定輸出檔名前綴
//   waypoints 在程式內設定 (全 degree); 要換 case 改 main 裡的數值即可
// =====================================================================
#include "dual_arm_alm_gd_planner_1/avoidance_system.hpp"

#include <iostream>
#include <string>

using namespace dual_arm_alm_gd_planner_1;

int main(int argc, char** argv)
{
  // 輸出前綴 (預設 "standalone_out")
  std::string prefix = (argc >= 2) ? argv[1] : "standalone_out";

  // ===== 測試 waypoints (degree) =====
  //   A/B 各 2 列: 第 0 列 = 起點, 第 1 列 = 終點
  //   要換 case 直接改這裡的數值
  Eigen::MatrixXd A(2, 6), B(2, 6);
  A << -30.000000, -30.823862,  38.612173,  0.000000, -7.788311, 0.000000,
        30.000000, -30.823862,  38.612173,  0.000000, -7.788311, 0.000000;
  B << -30.000000, -19.764124, -29.835040, -0.000000, 49.599164, 0.000000,
        30.000000, -19.764124, -29.835040, -0.000000, 49.599164, 0.000000;

  const double path_weight      = 0.5;
  const double danger_threshold = 0.4;

  std::cout << "==========================================\n";
  std::cout << " Dual-Arm Avoidance — Standalone (GD)\n";
  std::cout << "==========================================\n";

  // ===== 建構 + 跑 =====
  AvoidanceSystem sys(A, B, path_weight, danger_threshold);
  sys.run_optimization();

  // ===== 結果 =====
  std::cout << "\n=== 結果 ===\n";
  std::cout << "is_optimized = " << (sys.is_optimized() ? "true" : "false") << "\n";
  std::cout << "has_collision = " << (sys.has_collision() ? "true" : "false") << "\n";
  std::cout << "原始軌跡點數: " << sys.get_original_trajectory().time.size() << "\n";
  std::cout << "優化軌跡點數: " << sys.get_optimized_trajectory().time.size() << "\n";

  // ===== 匯出 CSV (除畫圖外的全部 export) =====
  // [REVISE] 改用整合匯出: 建子目錄 <prefix>/<unix秒>_<solver>/, level 2 = 完整 9 檔
  sys.export_unified(prefix, 2);

  std::cout << "\n全部 CSV 已輸出至資料夾: " << prefix << "/ (整合匯出 level 2, 共 9 檔)\n";
  return sys.has_collision() ? 1 : 0;   // 回傳碼: 0=成功避障, 1=仍碰撞
}
