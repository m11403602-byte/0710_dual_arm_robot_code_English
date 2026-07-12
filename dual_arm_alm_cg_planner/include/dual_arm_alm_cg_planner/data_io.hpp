// =====================================================================
// data_io.hpp — CSV writing utility (pure std, zero dependencies)
// =====================================================================
//   multiple logical sheets map to multiple separate .csv files
//   write only, no read (waypoints are provided directly by the caller)
// =====================================================================
#ifndef DUAL_ARM_AVOIDANCE_PLANNER_DATA_IO_HPP
#define DUAL_ARM_AVOIDANCE_PLANNER_DATA_IO_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dual_arm_alm_cg_planner
{

// write a matrix to CSV (with header)
//   header: column names (length must = mat.cols(); if empty, no header is written)
//   mat:    data (one row per record)
void write_csv(const std::string& path,
               const std::vector<std::string>& header,
               const Eigen::MatrixXd& mat);

// write a CSV with "mixed header + string first column + numeric values" (for tables like Targets whose first column is a Pt name)
//   row_labels: the first-column string of each row (length = mat.rows())
//   header:     column names (including the first column, length = mat.cols()+1)
void write_csv_labeled(const std::string& path,
                       const std::vector<std::string>& header,
                       const std::vector<std::string>& row_labels,
                       const Eigen::MatrixXd& mat);

}  // namespace dual_arm_alm_cg_planner

#endif
