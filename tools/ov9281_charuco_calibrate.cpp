#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
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
constexpr int kCoverageCols = 4;
constexpr int kCoverageRows = 3;

struct ViewData {
    fs::path path;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    double reprojection_error = 0.0;
};

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    if (n % 2 == 1) return values[n / 2];
    return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

double mad(const std::vector<double>& values, double med) {
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (double v : values) deviations.push_back(std::abs(v - med));
    return median(std::move(deviations));
}

double perViewError(const std::vector<cv::Point3f>& object_points,
                    const std::vector<cv::Point2f>& image_points,
                    const cv::Mat& rvec,
                    const cv::Mat& tvec,
                    const cv::Mat& camera_matrix,
                    const cv::Mat& dist_coeffs) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);
    double sum_sq = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        const cv::Point2f d = image_points[i] - projected[i];
        sum_sq += static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y;
    }
    return image_points.empty() ? 0.0 : std::sqrt(sum_sq / image_points.size());
}

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

void updateCoverage(const std::vector<cv::Point2f>& pts,
                    int coverage[kCoverageRows][kCoverageCols]) {
    for (const auto& p : pts) {
        int cx = std::clamp(static_cast<int>(p.x * kCoverageCols / kWidth), 0, kCoverageCols - 1);
        int cy = std::clamp(static_cast<int>(p.y * kCoverageRows / kHeight), 0, kCoverageRows - 1);
        coverage[cy][cx]++;
    }
}

void printCoverage(const int coverage[kCoverageRows][kCoverageCols]) {
    std::cout << "\nCoverage grid (detected ChArUco corners per cell):\n";
    for (int y = 0; y < kCoverageRows; ++y) {
        for (int x = 0; x < kCoverageCols; ++x) {
            std::cout << std::setw(6) << coverage[y][x];
        }
        std::cout << '\n';
    }
}
}

int main(int argc, char** argv) {
    const fs::path input_dir = argc > 1 ? argv[1] : "/home/vio/charuco_calibration";
    const fs::path yaml_path = argc > 2 ? argv[2] : "/home/vio/charuco_calibration/ov9281_intrinsics.yaml";
    const fs::path report_path = argc > 3 ? argv[3] : "/home/vio/charuco_calibration/ov9281_intrinsics_report.txt";

    const auto images = findImages(input_dir);
    if (images.empty()) {
        std::cerr << "ERROR: no frame_*.png images found in " << input_dir << '\n';
        return 1;
    }

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(kSquaresX, kSquaresY),
                                  kSquareLengthMm,
                                  kMarkerLengthMm,
                                  dictionary);
    cv::aruco::CharucoDetector detector(board);
    const auto& board_points = board.getChessboardCorners();

    std::vector<ViewData> views;
    int coverage[kCoverageRows][kCoverageCols] = {};
    int rejected_detection = 0;

    std::cout << "=== OV9281 CHARUCO CALIBRATION ===\n"
              << "input: " << input_dir << '\n'
              << "images: " << images.size() << '\n'
              << "mode: 640x480\n"
              << "board: 7x5 DICT_4X4_50\n"
              << "squareLength: " << kSquareLengthMm << " mm\n"
              << "markerLength: " << kMarkerLengthMm << " mm\n\n";

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

        ViewData view;
        view.path = path;
        view.object_points.reserve(count);
        view.image_points.reserve(count);

        for (int i = 0; i < count; ++i) {
            const int id = charuco_ids.at<int>(i);
            if (id < 0 || id >= static_cast<int>(board_points.size())) continue;
            view.object_points.push_back(board_points[id]);
            view.image_points.push_back(charuco_corners.at<cv::Point2f>(i));
        }

        if (view.image_points.size() < static_cast<size_t>(kMinCharucoCorners)) {
            ++rejected_detection;
            continue;
        }

        updateCoverage(view.image_points, coverage);
        std::cout << "ACCEPT " << path.filename().string()
                  << " corners=" << view.image_points.size() << '\n';
        views.push_back(std::move(view));
    }

    if (views.size() < 10) {
        std::cerr << "ERROR: only " << views.size() << " usable views; need at least 10\n";
        return 2;
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    object_points.reserve(views.size());
    image_points.reserve(views.size());
    for (const auto& v : views) {
        object_points.push_back(v.object_points);
        image_points.push_back(v.image_points);
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 2) = (kWidth - 1) * 0.5;
    camera_matrix.at<double>(1, 2) = (kHeight - 1) * 0.5;
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    const int flags = 0;
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                    100, 1e-12);
    const double rms = cv::calibrateCamera(object_points, image_points,
                                           cv::Size(kWidth, kHeight),
                                           camera_matrix, dist_coeffs,
                                           rvecs, tvecs, flags, criteria);

    std::vector<double> errors;
    errors.reserve(views.size());
    for (size_t i = 0; i < views.size(); ++i) {
        views[i].reprojection_error = perViewError(views[i].object_points,
                                                   views[i].image_points,
                                                   rvecs[i], tvecs[i],
                                                   camera_matrix, dist_coeffs);
        errors.push_back(views[i].reprojection_error);
    }

    const double med = median(errors);
    const double error_mad = mad(errors, med);
    const double warn_threshold = std::max(0.50, med + 2.5 * std::max(error_mad, 0.01));

    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    const double cx = camera_matrix.at<double>(0, 2);
    const double cy = camera_matrix.at<double>(1, 2);
    const double k1 = dist_coeffs.at<double>(0, 0);
    const double k2 = dist_coeffs.at<double>(1, 0);
    const double p1 = dist_coeffs.at<double>(2, 0);
    const double p2 = dist_coeffs.at<double>(3, 0);
    const double k3 = dist_coeffs.at<double>(4, 0);

    std::cout << std::fixed << std::setprecision(6)
              << "\n=== RESULT ===\n"
              << "usable_views=" << views.size() << '\n'
              << "rejected_detection=" << rejected_detection << '\n'
              << "RMS=" << rms << " px\n"
              << "fx=" << fx << '\n'
              << "fy=" << fy << '\n'
              << "cx=" << cx << '\n'
              << "cy=" << cy << '\n'
              << "k1=" << k1 << '\n'
              << "k2=" << k2 << '\n'
              << "p1=" << p1 << '\n'
              << "p2=" << p2 << '\n'
              << "k3=" << k3 << '\n'
              << "per_view_median=" << med << " px\n"
              << "per_view_MAD=" << error_mad << " px\n"
              << "outlier_warning_threshold=" << warn_threshold << " px\n";

    printCoverage(coverage);

    std::cout << "\nPer-view reprojection error:\n";
    int warned = 0;
    for (const auto& v : views) {
        const bool bad = v.reprojection_error > warn_threshold;
        if (bad) ++warned;
        std::cout << (bad ? "WARN   " : "OK     ")
                  << v.path.filename().string()
                  << " error=" << std::fixed << std::setprecision(4)
                  << v.reprojection_error << " px\n";
    }

    cv::FileStorage fs_yaml(yaml_path.string(), cv::FileStorage::WRITE);
    if (!fs_yaml.isOpened()) {
        std::cerr << "ERROR: cannot write " << yaml_path << '\n';
        return 3;
    }
    fs_yaml << "camera_name" << "OV9281_USB_UVC";
    fs_yaml << "image_width" << kWidth;
    fs_yaml << "image_height" << kHeight;
    fs_yaml << "camera_model" << "pinhole";
    fs_yaml << "distortion_model" << "radtan_5";
    fs_yaml << "board_squares_x" << kSquaresX;
    fs_yaml << "board_squares_y" << kSquaresY;
    fs_yaml << "board_square_length_mm" << kSquareLengthMm;
    fs_yaml << "board_marker_length_mm" << kMarkerLengthMm;
    fs_yaml << "camera_matrix" << camera_matrix;
    fs_yaml << "distortion_coefficients" << dist_coeffs;
    fs_yaml << "rms_reprojection_error_px" << rms;
    fs_yaml << "usable_views" << static_cast<int>(views.size());
    fs_yaml.release();

    std::ofstream report(report_path);
    if (!report) {
        std::cerr << "ERROR: cannot write " << report_path << '\n';
        return 4;
    }
    report << std::fixed << std::setprecision(8)
           << "OV9281 ChArUco intrinsic calibration\n"
           << "image_size=640x480\n"
           << "board=7x5 DICT_4X4_50\n"
           << "square_length_mm=" << kSquareLengthMm << '\n'
           << "marker_length_mm=" << kMarkerLengthMm << '\n'
           << "input_images=" << images.size() << '\n'
           << "usable_views=" << views.size() << '\n'
           << "rejected_detection=" << rejected_detection << '\n'
           << "RMS_px=" << rms << '\n'
           << "fx=" << fx << '\n'
           << "fy=" << fy << '\n'
           << "cx=" << cx << '\n'
           << "cy=" << cy << '\n'
           << "k1=" << k1 << '\n'
           << "k2=" << k2 << '\n'
           << "p1=" << p1 << '\n'
           << "p2=" << p2 << '\n'
           << "k3=" << k3 << '\n'
           << "per_view_median_px=" << med << '\n'
           << "per_view_MAD_px=" << error_mad << '\n'
           << "outlier_warning_threshold_px=" << warn_threshold << '\n'
           << "warned_views=" << warned << "\n\n"
           << "coverage_grid:\n";
    for (int y = 0; y < kCoverageRows; ++y) {
        for (int x = 0; x < kCoverageCols; ++x) {
            if (x) report << ',';
            report << coverage[y][x];
        }
        report << '\n';
    }
    report << "\nper_view_errors:\n";
    for (const auto& v : views) {
        report << v.path.filename().string() << ','
               << v.image_points.size() << ','
               << v.reprojection_error << ','
               << (v.reprojection_error > warn_threshold ? "WARN" : "OK") << '\n';
    }

    std::cout << "\nSaved YAML: " << yaml_path << '\n'
              << "Saved report: " << report_path << '\n'
              << "Warned views: " << warned << '\n';

    return 0;
}
