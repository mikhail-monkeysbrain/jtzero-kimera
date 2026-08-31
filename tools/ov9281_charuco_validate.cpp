#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kSquaresX = 7;
constexpr int kSquaresY = 5;
constexpr float kSquareLengthMm = 27.324F;
constexpr float kMarkerLengthMm = 20.043F;
constexpr int kMinCharucoCorners = 12;

struct ViewResult {
    std::string name;
    int corners = 0;
    double error_px = 0.0;
};

std::vector<fs::path> findImages(const fs::path& dir) {
    std::vector<fs::path> images;
    if (!fs::exists(dir)) return images;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if (name.rfind("frame_", 0) == 0 && entry.path().extension() == ".png") {
            images.push_back(entry.path());
        }
    }
    std::sort(images.begin(), images.end());
    return images;
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n & 1U) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double reprojectionRmse(const std::vector<cv::Point3f>& object_points,
                        const std::vector<cv::Point2f>& image_points,
                        const cv::Mat& rvec,
                        const cv::Mat& tvec,
                        const cv::Mat& K,
                        const cv::Mat& D) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, K, D, projected);
    double sum_sq = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        const double dx = image_points[i].x - projected[i].x;
        const double dy = image_points[i].y - projected[i].y;
        sum_sq += dx * dx + dy * dy;
    }
    return std::sqrt(sum_sq / std::max<size_t>(1, image_points.size()));
}
}

int main(int argc, char** argv) {
    const fs::path input_dir = argc > 1 ? argv[1] : "/home/vio/charuco_validation";
    const fs::path yaml_path = argc > 2 ? argv[2] : "/home/vio/charuco_calibration/ov9281_intrinsics.yaml";

    cv::FileStorage fs_yaml(yaml_path.string(), cv::FileStorage::READ);
    if (!fs_yaml.isOpened()) {
        std::cerr << "ERROR: cannot open calibration YAML: " << yaml_path << '\n';
        return 1;
    }

    int image_width = 0, image_height = 0;
    cv::Mat K, D;
    fs_yaml["image_width"] >> image_width;
    fs_yaml["image_height"] >> image_height;
    fs_yaml["camera_matrix"] >> K;
    fs_yaml["distortion_coefficients"] >> D;
    fs_yaml.release();

    if (image_width != kWidth || image_height != kHeight || K.empty() || D.empty()) {
        std::cerr << "ERROR: calibration YAML does not contain expected 640x480 K/D\n";
        return 2;
    }

    const auto images = findImages(input_dir);
    if (images.empty()) {
        std::cerr << "ERROR: no frame_*.png images found in " << input_dir << '\n';
        return 3;
    }

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(kSquaresX, kSquaresY),
                                  kSquareLengthMm,
                                  kMarkerLengthMm,
                                  dictionary);
    cv::aruco::CharucoDetector detector(board);
    const auto& board_points = board.getChessboardCorners();

    std::cout << "=== OV9281 CHARUCO INDEPENDENT VALIDATION ===\n"
              << "input: " << input_dir << '\n'
              << "images: " << images.size() << '\n'
              << "calibration: " << yaml_path << '\n'
              << std::fixed << std::setprecision(6)
              << "fx=" << K.at<double>(0,0) << " fy=" << K.at<double>(1,1)
              << " cx=" << K.at<double>(0,2) << " cy=" << K.at<double>(1,2) << "\n\n";

    std::vector<ViewResult> results;
    int rejected_detection = 0;
    int rejected_pnp = 0;

    for (const auto& path : images) {
        cv::Mat gray = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        if (gray.empty() || gray.cols != kWidth || gray.rows != kHeight) {
            std::cout << "REJECT " << path.filename().string() << " invalid image/mode\n";
            ++rejected_detection;
            continue;
        }

        cv::Mat charuco_corners, charuco_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;
        detector.detectBoard(gray, charuco_corners, charuco_ids, marker_corners, marker_ids);

        const int count = charuco_ids.empty() ? 0 : static_cast<int>(charuco_ids.total());
        if (count < kMinCharucoCorners) {
            std::cout << "REJECT " << path.filename().string() << " corners=" << count << '\n';
            ++rejected_detection;
            continue;
        }

        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;
        object_points.reserve(count);
        image_points.reserve(count);
        for (int i = 0; i < count; ++i) {
            const int id = charuco_ids.at<int>(i);
            if (id < 0 || id >= static_cast<int>(board_points.size())) continue;
            object_points.push_back(board_points[id]);
            image_points.push_back(charuco_corners.at<cv::Point2f>(i));
        }

        cv::Mat rvec, tvec;
        const bool ok = cv::solvePnP(object_points, image_points, K, D, rvec, tvec,
                                     false, cv::SOLVEPNP_ITERATIVE);
        if (!ok) {
            std::cout << "REJECT " << path.filename().string() << " solvePnP failed\n";
            ++rejected_pnp;
            continue;
        }

        const double err = reprojectionRmse(object_points, image_points, rvec, tvec, K, D);
        results.push_back({path.filename().string(), static_cast<int>(image_points.size()), err});
        std::cout << "VIEW   " << path.filename().string()
                  << " corners=" << image_points.size()
                  << " error=" << std::setprecision(4) << err << " px\n";
    }

    if (results.size() < 8) {
        std::cerr << "ERROR: only " << results.size() << " usable validation views; need at least 8\n";
        return 4;
    }

    std::vector<double> errors;
    errors.reserve(results.size());
    double sum_sq = 0.0;
    double max_error = 0.0;
    int above_050 = 0;
    int above_075 = 0;
    for (const auto& r : results) {
        errors.push_back(r.error_px);
        sum_sq += r.error_px * r.error_px;
        max_error = std::max(max_error, r.error_px);
        if (r.error_px > 0.50) ++above_050;
        if (r.error_px > 0.75) ++above_075;
    }

    const double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
    const double med = median(errors);
    const double aggregate_rmse = std::sqrt(sum_sq / errors.size());
    const bool pass = med <= 0.50 && aggregate_rmse <= 0.55 && max_error <= 0.90 && above_075 == 0;

    std::cout << std::fixed << std::setprecision(6)
              << "\n=== VALIDATION RESULT ===\n"
              << "usable_views=" << results.size() << '\n'
              << "rejected_detection=" << rejected_detection << '\n'
              << "rejected_pnp=" << rejected_pnp << '\n'
              << "mean_error=" << mean << " px\n"
              << "median_error=" << med << " px\n"
              << "aggregate_view_rmse=" << aggregate_rmse << " px\n"
              << "max_error=" << max_error << " px\n"
              << "views_above_0.50px=" << above_050 << '\n'
              << "views_above_0.75px=" << above_075 << '\n'
              << "RESULT: " << (pass ? "PASS" : "FAIL") << '\n';

    return pass ? 0 : 5;
}
