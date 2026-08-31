#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kDetectionStride = 4;          // 120 FPS -> ~30 FPS pose stream.
constexpr int kMinCharucoCorners = 12;
constexpr double kMaxReprojectionRmsePx = 1.5;
constexpr double kMinAngularSpeedRadS = 0.15;
constexpr double kMaxAngularSpeedRadS = 3.0;
constexpr double kMinPairDtSec = 0.020;
constexpr double kMaxPairDtSec = 0.050;
constexpr int64_t kMaxRawCameraDtNs = 20'000'000LL;
constexpr double kRobustFloorRadS = 0.08;
constexpr double kRobustMadScale = 3.5;

struct CameraFrame {
    size_t raw_index = 0;
    uint32_t sequence = 0;
    int64_t timestamp_ns = 0;
    uint64_t mjpeg_offset = 0;
    uint32_t bytes_used = 0;
    bool continuous_from_prev = false;
};

struct ImuSample {
    int64_t timestamp_ns = 0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct PoseSample {
    size_t raw_index = 0;
    uint32_t sequence = 0;
    int64_t timestamp_ns = 0;
    Eigen::Matrix3d R_CW = Eigen::Matrix3d::Identity();
    double reprojection_rmse_px = 0.0;
    int charuco_corners = 0;
};

struct PairSample {
    int64_t t0_ns = 0;
    int64_t t1_ns = 0;
    Eigen::Vector3d omega_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_C = Eigen::Vector3d::Zero();
    double reprojection_rmse_px = 0.0;
};

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, ',')) out.push_back(field);
    if (!line.empty() && line.back() == ',') out.emplace_back();
    return out;
}

double median(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    const size_t n = v.size();
    std::nth_element(v.begin(), v.begin() + n / 2, v.end());
    const double hi = v[n / 2];
    if (n & 1U) return hi;
    std::nth_element(v.begin(), v.begin() + n / 2 - 1, v.end());
    return 0.5 * (v[n / 2 - 1] + hi);
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(v.begin(), v.end());
    const double p = q * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(p));
    const size_t hi = std::min(lo + 1, v.size() - 1);
    const double f = p - static_cast<double>(lo);
    return v[lo] * (1.0 - f) + v[hi] * f;
}

std::vector<CameraFrame> loadCameraIndex(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open camera index: " + path);

    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error("Empty camera index");

    std::vector<CameraFrame> frames;
    size_t raw_index = 0;
    uint32_t prev_seq = 0;
    int64_t prev_ts = 0;
    bool have_prev = false;

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto c = splitCsv(line);
        if (c.size() < 7) continue;

        CameraFrame f;
        f.raw_index = raw_index++;
        f.sequence = static_cast<uint32_t>(std::stoul(c[0]));
        f.timestamp_ns = std::stoll(c[2]);
        f.mjpeg_offset = std::stoull(c[5]);
        f.bytes_used = static_cast<uint32_t>(std::stoul(c[6]));

        if (have_prev) {
            const int64_t dt = f.timestamp_ns - prev_ts;
            f.continuous_from_prev =
                f.sequence == prev_seq + 1U && dt > 0 && dt <= kMaxRawCameraDtNs;
        }

        prev_seq = f.sequence;
        prev_ts = f.timestamp_ns;
        have_prev = true;
        frames.push_back(f);
    }

    return frames;
}

std::vector<ImuSample> loadImu(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open combined CSV: " + path);

    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error("Empty combined CSV");

    std::vector<ImuSample> imu;
    while (std::getline(in, line)) {
        if (line.rfind("IMU,", 0) != 0) continue;
        const auto c = splitCsv(line);
        if (c.size() <= 13 || c[3].empty() || c[11].empty()) continue;

        ImuSample s;
        s.timestamp_ns = std::stoll(c[3]);
        s.gyro = Eigen::Vector3d(
            std::stod(c[11]), std::stod(c[12]), std::stod(c[13]));
        imu.push_back(s);
    }

    std::sort(imu.begin(), imu.end(),
              [](const ImuSample& a, const ImuSample& b) {
                  return a.timestamp_ns < b.timestamp_ns;
              });
    return imu;
}

Eigen::Vector3d interpolateGyro(
    const std::vector<ImuSample>& imu,
    int64_t t_ns,
    size_t hi) {
    if (hi == 0 || hi >= imu.size())
        throw std::runtime_error("IMU interpolation outside data range");
    const auto& a = imu[hi - 1];
    const auto& b = imu[hi];
    const double den = static_cast<double>(b.timestamp_ns - a.timestamp_ns);
    if (den <= 0.0) return a.gyro;
    const double u = static_cast<double>(t_ns - a.timestamp_ns) / den;
    return a.gyro * (1.0 - u) + b.gyro * u;
}

bool averageGyroInterval(
    const std::vector<ImuSample>& imu,
    int64_t t0_ns,
    int64_t t1_ns,
    Eigen::Vector3d* avg) {
    if (!avg || imu.size() < 2 || t1_ns <= t0_ns) return false;
    if (t0_ns < imu.front().timestamp_ns || t1_ns > imu.back().timestamp_ns)
        return false;

    auto it0 = std::lower_bound(
        imu.begin(), imu.end(), t0_ns,
        [](const ImuSample& s, int64_t t) { return s.timestamp_ns < t; });
    auto it1 = std::lower_bound(
        imu.begin(), imu.end(), t1_ns,
        [](const ImuSample& s, int64_t t) { return s.timestamp_ns < t; });

    size_t hi0 = static_cast<size_t>(std::distance(imu.begin(), it0));
    size_t hi1 = static_cast<size_t>(std::distance(imu.begin(), it1));
    if (hi0 == 0) hi0 = 1;
    if (hi1 == 0) hi1 = 1;
    if (hi0 >= imu.size() || hi1 >= imu.size()) return false;

    int64_t prev_t = t0_ns;
    Eigen::Vector3d prev_g = interpolateGyro(imu, t0_ns, hi0);
    Eigen::Vector3d integral = Eigen::Vector3d::Zero();

    size_t i = hi0;
    while (i < imu.size() && imu[i].timestamp_ns < t1_ns) {
        const int64_t t = imu[i].timestamp_ns;
        const double dt = static_cast<double>(t - prev_t) * 1e-9;
        integral += 0.5 * (prev_g + imu[i].gyro) * dt;
        prev_t = t;
        prev_g = imu[i].gyro;
        ++i;
    }

    const Eigen::Vector3d end_g = interpolateGyro(imu, t1_ns, hi1);
    const double dt_last = static_cast<double>(t1_ns - prev_t) * 1e-9;
    integral += 0.5 * (prev_g + end_g) * dt_last;

    const double total_dt = static_cast<double>(t1_ns - t0_ns) * 1e-9;
    *avg = integral / total_dt;
    return avg->allFinite();
}

Eigen::Matrix3d cvMat33ToEigen(const cv::Mat& R) {
    Eigen::Matrix3d out;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out(r, c) = R.at<double>(r, c);
    return out;
}

bool readMjpegFrame(
    std::ifstream& mjpeg,
    const CameraFrame& f,
    cv::Mat* image) {
    if (!image || f.bytes_used == 0) return false;
    std::vector<unsigned char> bytes(f.bytes_used);
    mjpeg.clear();
    mjpeg.seekg(static_cast<std::streamoff>(f.mjpeg_offset), std::ios::beg);
    if (!mjpeg) return false;
    mjpeg.read(reinterpret_cast<char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    if (mjpeg.gcount() != static_cast<std::streamsize>(bytes.size())) return false;
    *image = cv::imdecode(bytes, cv::IMREAD_GRAYSCALE);
    return !image->empty();
}

double reprojectionRmse(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const cv::Mat& K,
    const cv::Mat& D) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, K, D, projected);
    double sum2 = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const cv::Point2f d = projected[i] - image_points[i];
        sum2 += static_cast<double>(d.x * d.x + d.y * d.y);
    }
    return projected.empty() ? 1e9 : std::sqrt(sum2 / projected.size());
}

Eigen::Matrix3d solveWahba(
    const std::vector<PairSample>& pairs,
    const std::vector<size_t>& indices) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (const size_t i : indices) {
        const auto& p = pairs[i];
        const double speed = 0.5 * (p.omega_B.norm() + p.omega_C.norm());
        const double w = std::min(std::max(speed, 0.2), 1.5);
        H += w * p.omega_C * p.omega_B.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if ((U * V.transpose()).determinant() < 0.0) S(2, 2) = -1.0;
    return U * S * V.transpose();
}

std::vector<double> vectorResiduals(
    const std::vector<PairSample>& pairs,
    const std::vector<size_t>& indices,
    const Eigen::Matrix3d& R_CB) {
    std::vector<double> out;
    out.reserve(indices.size());
    for (const size_t i : indices)
        out.push_back((pairs[i].omega_C - R_CB * pairs[i].omega_B).norm());
    return out;
}

std::vector<double> directionErrorsDeg(
    const std::vector<PairSample>& pairs,
    const std::vector<size_t>& indices,
    const Eigen::Matrix3d& R_CB) {
    std::vector<double> out;
    out.reserve(indices.size());
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    for (const size_t i : indices) {
        const Eigen::Vector3d a = pairs[i].omega_C.normalized();
        const Eigen::Vector3d b = (R_CB * pairs[i].omega_B).normalized();
        const double d = std::clamp(a.dot(b), -1.0, 1.0);
        out.push_back(std::acos(d) * kRadToDeg);
    }
    return out;
}

Eigen::Vector3d matrixToRpyDeg(const Eigen::Matrix3d& R) {
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    const double roll = std::atan2(R(2, 1), R(2, 2));
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw) * kRadToDeg;
}

bool rawIntervalContinuous(
    const std::vector<CameraFrame>& frames,
    size_t a,
    size_t b) {
    if (b <= a || b >= frames.size()) return false;
    for (size_t i = a + 1; i <= b; ++i)
        if (!frames[i].continuous_from_prev) return false;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string combined_csv = argc > 1
            ? argv[1] : "/home/vio/camera_imu_rotation_clean_60s.csv";
        const std::string camera_csv = argc > 2
            ? argv[2] : "/home/vio/camera_imu_rotation_clean_60s_camera.csv";
        const std::string mjpeg_path = argc > 3
            ? argv[3] : "/home/vio/camera_imu_rotation_clean_60s.mjpg";
        const std::string intrinsics_path = argc > 4
            ? argv[4] : "calibration/ov9281_intrinsics.yaml";

        cv::FileStorage fs(intrinsics_path, cv::FileStorage::READ);
        if (!fs.isOpened()) throw std::runtime_error("Cannot open intrinsics YAML");

        int squares_x = 0;
        int squares_y = 0;
        double square_mm = 0.0;
        double marker_mm = 0.0;
        cv::Mat K, D;
        fs["board_squares_x"] >> squares_x;
        fs["board_squares_y"] >> squares_y;
        fs["board_square_length_mm"] >> square_mm;
        fs["board_marker_length_mm"] >> marker_mm;
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        fs.release();

        if (squares_x <= 1 || squares_y <= 1 || square_mm <= 0.0 || marker_mm <= 0.0 ||
            K.empty() || D.empty()) {
            throw std::runtime_error("Invalid calibration YAML");
        }
        K.convertTo(K, CV_64F);
        D.convertTo(D, CV_64F);

        const auto frames = loadCameraIndex(camera_csv);
        const auto imu = loadImu(combined_csv);
        if (frames.size() < 20 || imu.size() < 20)
            throw std::runtime_error("Not enough camera/IMU samples");

        size_t raw_discontinuities = 0;
        for (size_t i = 1; i < frames.size(); ++i)
            if (!frames[i].continuous_from_prev) ++raw_discontinuities;

        auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::CharucoBoard board(
            cv::Size(squares_x, squares_y),
            static_cast<float>(square_mm * 1e-3),
            static_cast<float>(marker_mm * 1e-3),
            dictionary);
        cv::aruco::CharucoDetector detector(board);

        const std::vector<cv::Point3f>& chessboard_corners = board.getChessboardCorners();
        std::ifstream mjpeg(mjpeg_path, std::ios::binary);
        if (!mjpeg) throw std::runtime_error("Cannot open MJPEG: " + mjpeg_path);

        std::vector<PoseSample> poses;
        size_t decode_failures = 0;
        size_t detection_failures = 0;
        size_t reprojection_rejects = 0;

        for (size_t i = 0; i < frames.size(); i += kDetectionStride) {
            cv::Mat image;
            if (!readMjpegFrame(mjpeg, frames[i], &image)) {
                ++decode_failures;
                continue;
            }

            std::vector<cv::Point2f> charuco_corners;
            std::vector<int> charuco_ids;
            detector.detectBoard(image, charuco_corners, charuco_ids);
            if (charuco_ids.size() < static_cast<size_t>(kMinCharucoCorners)) {
                ++detection_failures;
                continue;
            }

            std::vector<cv::Point3f> object_points;
            std::vector<cv::Point2f> image_points;
            object_points.reserve(charuco_ids.size());
            image_points.reserve(charuco_ids.size());
            for (size_t k = 0; k < charuco_ids.size(); ++k) {
                const int id = charuco_ids[k];
                if (id < 0 || id >= static_cast<int>(chessboard_corners.size())) continue;
                object_points.push_back(chessboard_corners[static_cast<size_t>(id)]);
                image_points.push_back(charuco_corners[k]);
            }
            if (object_points.size() < static_cast<size_t>(kMinCharucoCorners)) {
                ++detection_failures;
                continue;
            }

            cv::Mat rvec, tvec;
            if (!cv::solvePnP(object_points, image_points, K, D, rvec, tvec,
                              false, cv::SOLVEPNP_ITERATIVE)) {
                ++detection_failures;
                continue;
            }
            cv::solvePnPRefineLM(object_points, image_points, K, D, rvec, tvec);
            const double rmse = reprojectionRmse(
                object_points, image_points, rvec, tvec, K, D);
            if (!std::isfinite(rmse) || rmse > kMaxReprojectionRmsePx) {
                ++reprojection_rejects;
                continue;
            }

            cv::Mat Rcv;
            cv::Rodrigues(rvec, Rcv);
            Rcv.convertTo(Rcv, CV_64F);

            PoseSample p;
            p.raw_index = i;
            p.sequence = frames[i].sequence;
            p.timestamp_ns = frames[i].timestamp_ns;
            p.R_CW = cvMat33ToEigen(Rcv);
            p.reprojection_rmse_px = rmse;
            p.charuco_corners = static_cast<int>(object_points.size());
            poses.push_back(p);
        }

        std::vector<PairSample> pairs;
        size_t continuity_rejects = 0;
        size_t dt_rejects = 0;
        size_t imu_rejects = 0;
        size_t speed_rejects = 0;

        for (size_t i = 1; i < poses.size(); ++i) {
            const PoseSample& a = poses[i - 1];
            const PoseSample& b = poses[i];
            if (b.raw_index != a.raw_index + kDetectionStride ||
                !rawIntervalContinuous(frames, a.raw_index, b.raw_index)) {
                ++continuity_rejects;
                continue;
            }

            const double dt = static_cast<double>(b.timestamp_ns - a.timestamp_ns) * 1e-9;
            if (dt < kMinPairDtSec || dt > kMaxPairDtSec) {
                ++dt_rejects;
                continue;
            }

            // solvePnP returns R_CW (fixed board/world -> camera). For a rigid camera
            // rotating with angular velocity omega_C expressed in camera coordinates:
            //   R_CW(t+dt) * R_CW(t)^T ~= Exp(-[omega_C]x dt)
            // therefore omega_C = -Log(R_delta)/dt.
            const Eigen::Matrix3d R_delta = b.R_CW * a.R_CW.transpose();
            Eigen::AngleAxisd aa(R_delta);
            Eigen::Vector3d omega_C = -aa.axis() * aa.angle() / dt;

            Eigen::Vector3d omega_B;
            if (!averageGyroInterval(imu, a.timestamp_ns, b.timestamp_ns, &omega_B)) {
                ++imu_rejects;
                continue;
            }

            const double sB = omega_B.norm();
            const double sC = omega_C.norm();
            if (!omega_B.allFinite() || !omega_C.allFinite() ||
                sB < kMinAngularSpeedRadS || sC < kMinAngularSpeedRadS ||
                sB > kMaxAngularSpeedRadS || sC > kMaxAngularSpeedRadS) {
                ++speed_rejects;
                continue;
            }

            PairSample p;
            p.t0_ns = a.timestamp_ns;
            p.t1_ns = b.timestamp_ns;
            p.omega_B = omega_B;
            p.omega_C = omega_C;
            p.reprojection_rmse_px = 0.5 *
                (a.reprojection_rmse_px + b.reprojection_rmse_px);
            pairs.push_back(p);
        }

        if (pairs.size() < 30)
            throw std::runtime_error("Too few valid angular-velocity pairs for calibration");

        std::vector<size_t> inliers(pairs.size());
        std::iota(inliers.begin(), inliers.end(), 0);
        Eigen::Matrix3d R_CB = Eigen::Matrix3d::Identity();

        for (int iter = 0; iter < 4; ++iter) {
            R_CB = solveWahba(pairs, inliers);
            const auto residuals = vectorResiduals(pairs, inliers, R_CB);
            const double med = median(residuals);
            std::vector<double> abs_dev;
            abs_dev.reserve(residuals.size());
            for (double r : residuals) abs_dev.push_back(std::abs(r - med));
            const double mad = median(abs_dev);
            const double threshold = std::max(kRobustFloorRadS, med + kRobustMadScale * 1.4826 * mad);

            std::vector<size_t> next;
            next.reserve(inliers.size());
            for (size_t k = 0; k < inliers.size(); ++k)
                if (residuals[k] <= threshold) next.push_back(inliers[k]);

            if (next.size() < 30 || next.size() == inliers.size()) break;
            inliers.swap(next);
        }

        R_CB = solveWahba(pairs, inliers);
        const Eigen::Matrix3d R_BC = R_CB.transpose();
        const auto residuals = vectorResiduals(pairs, inliers, R_CB);
        const auto direction_deg = directionErrorsDeg(pairs, inliers, R_CB);

        std::vector<double> pose_rmse;
        pose_rmse.reserve(poses.size());
        for (const auto& p : poses) pose_rmse.push_back(p.reprojection_rmse_px);

        const Eigen::Vector3d rpy_CB = matrixToRpyDeg(R_CB);
        const Eigen::Vector3d rpy_BC = matrixToRpyDeg(R_BC);

        std::cout << std::fixed << std::setprecision(9);
        std::cout << "\n============================================================\n";
        std::cout << "CAMERA-IMU ROTATION CALIBRATION\n";
        std::cout << "============================================================\n";
        std::cout << "raw camera frames:        " << frames.size() << '\n';
        std::cout << "raw discontinuities:      " << raw_discontinuities << '\n';
        std::cout << "IMU samples:              " << imu.size() << '\n';
        std::cout << "pose attempts (~30 FPS):  " << ((frames.size() + kDetectionStride - 1) / kDetectionStride) << '\n';
        std::cout << "valid ChArUco poses:      " << poses.size() << '\n';
        std::cout << "decode failures:          " << decode_failures << '\n';
        std::cout << "detection failures:       " << detection_failures << '\n';
        std::cout << "reprojection rejects:     " << reprojection_rejects << '\n';
        std::cout << "pose reproj median:       " << median(pose_rmse) << " px\n";
        std::cout << "pose reproj p95:          " << percentile(pose_rmse, 0.95) << " px\n";
        std::cout << "candidate pairs:          " << pairs.size() << '\n';
        std::cout << "robust inliers:           " << inliers.size() << '\n';
        std::cout << "continuity rejects:       " << continuity_rejects << '\n';
        std::cout << "dt rejects:               " << dt_rejects << '\n';
        std::cout << "IMU coverage rejects:     " << imu_rejects << '\n';
        std::cout << "speed rejects:            " << speed_rejects << '\n';
        std::cout << "gyro-vector residual med: " << median(residuals) << " rad/s\n";
        std::cout << "gyro-vector residual p95: " << percentile(residuals, 0.95) << " rad/s\n";
        std::cout << "direction error median:   " << median(direction_deg) << " deg\n";
        std::cout << "direction error p95:      " << percentile(direction_deg, 0.95) << " deg\n";

        std::cout << "\nR_CB  (omega_C = R_CB * omega_B):\n" << R_CB << "\n";
        std::cout << "RPY_CB deg: " << rpy_CB.transpose() << "\n";
        std::cout << "det(R_CB):  " << R_CB.determinant() << "\n";
        std::cout << "orthogonality error: "
                  << (R_CB.transpose() * R_CB - Eigen::Matrix3d::Identity()).norm() << "\n";

        std::cout << "\nR_BC = R_CB^T:\n" << R_BC << "\n";
        std::cout << "RPY_BC deg: " << rpy_BC.transpose() << "\n";

        std::cout << "\nNOTE: R_BC above is the inverse rotation of the measured gyro mapping.\n";
        std::cout << "Kimera T_BS serialization direction must still be checked before writing final extrinsics YAML.\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
