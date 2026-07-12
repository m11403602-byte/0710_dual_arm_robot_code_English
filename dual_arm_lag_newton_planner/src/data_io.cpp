// =====================================================================
// data_io.cpp — CSV writing utility implementation
// =====================================================================
#include "dual_arm_lag_newton_planner/data_io.hpp"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>   // C++17: automatically create the parent directory

namespace dual_arm_lag_newton_planner
{

// ensure the parent directory exists before writing (auto mkdir -p when path contains a directory)
static void ensure_parent_dir(const std::string& path)
{
  try {
    std::filesystem::path p(path);
    if (p.has_parent_path() && !p.parent_path().empty())
      std::filesystem::create_directories(p.parent_path());
  } catch (const std::exception& e) {
    std::cerr << "ensure_parent_dir: failed to create directory " << path << ": " << e.what() << "\n";
  }
}

static void write_header(std::ofstream& f, const std::vector<std::string>& header)
{
  for (size_t j = 0; j < header.size(); ++j) {
    if (j) f << ",";
    f << header[j];
  }
  f << "\n";
}

void write_csv(const std::string& path,
               const std::vector<std::string>& header,
               const Eigen::MatrixXd& mat)
{
  ensure_parent_dir(path);
  std::ofstream f(path);
  if (!f.is_open()) {
    std::cerr << "write_csv: cannot open " << path << "\n";
    return;
  }
  f << std::setprecision(10);
  if (!header.empty()) write_header(f, header);
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      if (j) f << ",";
      f << mat(i, j);
    }
    f << "\n";
  }
}

void write_csv_labeled(const std::string& path,
                       const std::vector<std::string>& header,
                       const std::vector<std::string>& row_labels,
                       const Eigen::MatrixXd& mat)
{
  ensure_parent_dir(path);
  std::ofstream f(path);
  if (!f.is_open()) {
    std::cerr << "write_csv_labeled: cannot open " << path << "\n";
    return;
  }
  f << std::setprecision(10);
  if (!header.empty()) write_header(f, header);
  for (int i = 0; i < mat.rows(); ++i) {
    f << (i < static_cast<int>(row_labels.size()) ? row_labels[i] : "");
    for (int j = 0; j < mat.cols(); ++j) f << "," << mat(i, j);
    f << "\n";
  }
}

}  // namespace dual_arm_lag_newton_planner
