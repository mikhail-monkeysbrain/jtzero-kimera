#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

constexpr double kMaxOrthogonalityError = 1e-6;
constexpr double kMaxDetError = 1e-6;
constexpr double kMinOpticalDown = 0.98;
constexpr double kMaxHorizontalTranslationM = 0.010;
constexpr double kExpectedZMinM = 0.052;
constexpr double kExpectedZMaxM = 0.058;

struct Check {
  std::string name;
  bool pass;
  std::string detail;
};

bool loadMatrix(const cv::FileStorage& fs, const std::string& key,
                int rows, int cols, Eigen::MatrixXd* out) {
  const cv::FileNode node = fs[key];
  if (node.empty()) return false;
  int r = 0, c = 0;
  node["rows"] >> r;
  node["cols"] >> c;
  if (r != rows || c != cols) return false;
  std::vector<double> data;
  node["data"] >> data;
  if (static_cast<int>(data.size()) != rows * cols) return false;
  out->resize(rows, cols);
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      (*out)(i, j) = data[i * cols + j];
  return true;
}

bool loadVec3(const cv::FileStorage& fs, const std::string& key, Vec3* out) {
  const cv::FileNode node = fs[key];
  if (node.empty() || node.size() != 3) return false;
  std::vector<double> v;
  node >> v;
  if (v.size() != 3) return false;
  *out = Vec3(v[0], v[1], v[2]);
  return true;
}

std::string vecText(const Vec3& v) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6)
     << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return ss.str();
}

void printCheck(const Check& c) {
  std::cout << (c.pass ? "[PASS] " : "[FAIL] ") << c.name;
  if (!c.detail.empty()) std::cout << " -- " << c.detail;
  std::cout << '\n';
}
}

int main(int argc, char** argv) {
  const std::string yaml = argc > 1 ? argv[1] : "calibration/ov9281_extrinsics_candidate.yaml";
  cv::FileStorage fs(yaml, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "ERROR: cannot open " << yaml << '\n';
    return 2;
  }

  Eigen::MatrixXd Tdyn, RcbDyn;
  Vec3 tListed;
  if (!loadMatrix(fs, "T_BS", 4, 4, &Tdyn)) {
    std::cerr << "ERROR: invalid T_BS\n";
    return 2;
  }
  if (!loadMatrix(fs, "R_CB", 3, 3, &RcbDyn)) {
    std::cerr << "ERROR: invalid R_CB\n";
    return 2;
  }
  if (!loadVec3(fs, "t_BC", &tListed)) {
    std::cerr << "ERROR: invalid t_BC\n";
    return 2;
  }

  const Eigen::Matrix4d T = Tdyn;
  const Mat3 Rbc = T.block<3,3>(0,0);
  const Vec3 tbc = T.block<3,1>(0,3);
  const Mat3 Rcb = RcbDyn;

  std::vector<Check> checks;
  const double ortho = (Rbc.transpose() * Rbc - Mat3::Identity()).norm();
  const double det = Rbc.determinant();
  const double inverseErr = (Rcb - Rbc.transpose()).norm();
  const double bottomErr = (T.row(3) - Eigen::RowVector4d(0,0,0,1)).norm();
  const double tDuplicateErr = (tbc - tListed).norm();

  checks.push_back({"R_BC orthonormal", ortho <= kMaxOrthogonalityError,
                    "error=" + std::to_string(ortho)});
  checks.push_back({"det(R_BC)=+1", std::abs(det - 1.0) <= kMaxDetError,
                    "det=" + std::to_string(det)});
  checks.push_back({"R_CB == R_BC^T", inverseErr <= 1e-6,
                    "error=" + std::to_string(inverseErr)});
  checks.push_back({"T_BS homogeneous bottom row", bottomErr <= 1e-12,
                    "error=" + std::to_string(bottomErr)});
  checks.push_back({"t_BC duplicate matches T_BS", tDuplicateErr <= 1e-12,
                    "error=" + std::to_string(tDuplicateErr)});

  const Vec3 cxB = Rbc * Vec3::UnitX();
  const Vec3 cyB = Rbc * Vec3::UnitY();
  const Vec3 czB = Rbc * Vec3::UnitZ();

  checks.push_back({"camera optical +C_Z points body-down", czB.z() >= kMinOpticalDown,
                    "R_BC*C_Z=" + vecText(czB)});
  checks.push_back({"camera +C_X is approximately body -Y",
                    cxB.y() < -0.98,
                    "R_BC*C_X=" + vecText(cxB)});
  checks.push_back({"camera +C_Y is approximately body +X",
                    cyB.x() > 0.98,
                    "R_BC*C_Y=" + vecText(cyB)});

  const double horizontal = std::hypot(tbc.x(), tbc.y());
  checks.push_back({"mechanical XY translation within 10 mm", horizontal <= kMaxHorizontalTranslationM,
                    "XY=" + std::to_string(horizontal * 1000.0) + " mm"});
  checks.push_back({"mechanical Z translation within 55+/-3 mm",
                    tbc.z() >= kExpectedZMinM && tbc.z() <= kExpectedZMaxM,
                    "Z=" + std::to_string(tbc.z() * 1000.0) + " mm"});

  std::cout << "\n============================================================\n";
  std::cout << "CAMERA-IMU FULL EXTRINSICS STATIC VALIDATION\n";
  std::cout << "============================================================\n";
  std::cout << "file: " << yaml << "\n\n";
  std::cout << std::fixed << std::setprecision(9);
  std::cout << "R_BC:\n" << Rbc << "\n\n";
  std::cout << "t_BC [m]: " << tbc.transpose() << "\n\n";

  bool all = true;
  for (const auto& c : checks) {
    printCheck(c);
    all = all && c.pass;
  }

  std::cout << "\nIMPORTANT: translation checks above validate the YAML against the\n"
               "measured mechanical geometry. They do NOT independently observe\n"
               "the 55 mm lever arm from visual/IMU motion.\n";
  std::cout << "Rotation experimental status is established separately by the two\n"
               "post-rebuild ChArUco + HIGHRES_IMU calibration runs.\n\n";
  std::cout << "STATIC RESULT: " << (all ? "PASS" : "FAIL") << '\n';
  std::cout << "FULL 6-DOF EXPERIMENTAL RESULT: NOT YET CLAIMED\n";
  return all ? 0 : 1;
}
