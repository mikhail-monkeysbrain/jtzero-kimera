#include <opencv2/opencv.hpp>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct PairFiles {
    fs::path left;
    fs::path right;
    std::string stem;
};

static std::vector<PairFiles> findPairs(const fs::path &dir)
{
    std::vector<PairFiles> pairs;

    if (!fs::exists(dir))
        return pairs;

    for (const auto &entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file())
            continue;

        const fs::path p = entry.path();
        const std::string name = p.filename().string();

        const std::string suffix = "_ov9281.png";
        if (name.size() <= suffix.size() ||
            name.substr(name.size() - suffix.size()) != suffix)
            continue;

        const std::string stem =
            name.substr(0, name.size() - suffix.size());

        fs::path right = dir / (stem + "_ov5647.png");
        if (fs::exists(right))
            pairs.push_back({p, right, stem});
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const PairFiles &a, const PairFiles &b) {
                  return a.stem < b.stem;
              });

    return pairs;
}

static std::vector<cv::Point3f> makeObjectPoints(
    int cols, int rows, double square_m)
{
    std::vector<cv::Point3f> pts;
    pts.reserve(cols * rows);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            pts.emplace_back(
                static_cast<float>(x * square_m),
                static_cast<float>(y * square_m),
                0.0f);
        }
    }

    return pts;
}

static bool detectBoard(
    const cv::Mat &gray,
    const cv::Size &board,
    std::vector<cv::Point2f> &corners)
{
    corners.clear();

#if CV_VERSION_MAJOR >= 4
    const int sb_flags =
        cv::CALIB_CB_EXHAUSTIVE |
        cv::CALIB_CB_ACCURACY |
        cv::CALIB_CB_NORMALIZE_IMAGE;

    if (cv::findChessboardCornersSB(gray, board, corners, sb_flags))
        return true;
#endif

    const int flags =
        cv::CALIB_CB_ADAPTIVE_THRESH |
        cv::CALIB_CB_NORMALIZE_IMAGE |
        cv::CALIB_CB_FAST_CHECK;

    if (!cv::findChessboardCorners(gray, board, corners, flags))
        return false;

    cv::cornerSubPix(
        gray,
        corners,
        cv::Size(11, 11),
        cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
            50,
            1e-4));

    return true;
}

static double computeStereoEpipolarError(
    const std::vector<std::vector<cv::Point2f>> &left_pts,
    const std::vector<std::vector<cv::Point2f>> &right_pts,
    const cv::Mat &K1,
    const cv::Mat &D1,
    const cv::Mat &K2,
    const cv::Mat &D2,
    const cv::Mat &R,
    const cv::Mat &T)
{
    cv::Mat E, F;
    cv::Mat Tx = (cv::Mat_<double>(3, 3) <<
        0.0, -T.at<double>(2), T.at<double>(1),
        T.at<double>(2), 0.0, -T.at<double>(0),
        -T.at<double>(1), T.at<double>(0), 0.0);

    E = Tx * R;
    F = K2.inv().t() * E * K1.inv();

    double total = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < left_pts.size(); ++i) {
        std::vector<cv::Vec3f> lines2;
        cv::computeCorrespondEpilines(left_pts[i], 1, F, lines2);

        for (size_t j = 0; j < lines2.size(); ++j) {
            const auto &l = lines2[j];
            const auto &p = right_pts[i][j];
            const double denom =
                std::sqrt(double(l[0]) * l[0] + double(l[1]) * l[1]);

            if (denom > 1e-12) {
                total += std::abs(l[0] * p.x + l[1] * p.y + l[2]) / denom;
                ++count;
            }
        }
    }

    return count ? total / static_cast<double>(count) : 0.0;
}

int main(int argc, char **argv)
{
    if (argc < 5) {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " <capture_dir> <inner_cols> <inner_rows> <square_mm> [output_yaml]\n\n"
            << "Example for a board with 9x6 inner corners and 25 mm squares:\n"
            << "  " << argv[0]
            << " stereo_captures 9 6 25 calibration/stereo_ov9281_ov5647.yaml\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const int cols = std::stoi(argv[2]);
    const int rows = std::stoi(argv[3]);
    const double square_mm = std::stod(argv[4]);
    const double square_m = square_mm / 1000.0;

    const fs::path output =
        argc > 5
            ? fs::path(argv[5])
            : fs::path("calibration/stereo_ov9281_ov5647.yaml");

    if (cols < 3 || rows < 3 || square_mm <= 0.0) {
        std::cerr << "Invalid chessboard parameters\n";
        return 1;
    }

    const cv::Size board(cols, rows);
    const auto pairs = findPairs(capture_dir);

    if (pairs.empty()) {
        std::cerr << "No synchronized image pairs found in "
                  << capture_dir << "\n";
        return 1;
    }

    std::cout << "========================================\n";
    std::cout << " JT-Zero stereo calibration\n";
    std::cout << "========================================\n";
    std::cout << "Capture dir       : " << capture_dir << "\n";
    std::cout << "Pairs found       : " << pairs.size() << "\n";
    std::cout << "Board inner       : " << cols << " x " << rows << "\n";
    std::cout << "Square            : " << square_mm << " mm\n\n";

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> left_points;
    std::vector<std::vector<cv::Point2f>> right_points;
    std::vector<std::string> accepted_stems;

    cv::Size image_size;

    const auto object_template =
        makeObjectPoints(cols, rows, square_m);

    for (const auto &pair : pairs) {
        cv::Mat left = cv::imread(pair.left.string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread(pair.right.string(), cv::IMREAD_GRAYSCALE);

        if (left.empty() || right.empty()) {
            std::cout << "[SKIP] " << pair.stem << " read failed\n";
            continue;
        }

        if (left.size() != right.size()) {
            std::cout << "[SKIP] " << pair.stem
                      << " size mismatch left=" << left.cols << "x" << left.rows
                      << " right=" << right.cols << "x" << right.rows << "\n";
            continue;
        }

        if (image_size.empty())
            image_size = left.size();

        if (left.size() != image_size) {
            std::cout << "[SKIP] " << pair.stem
                      << " inconsistent image size\n";
            continue;
        }

        std::vector<cv::Point2f> c1, c2;
        const bool ok1 = detectBoard(left, board, c1);
        const bool ok2 = detectBoard(right, board, c2);

        if (!ok1 || !ok2) {
            std::cout << "[SKIP] " << pair.stem
                      << " chessboard left=" << (ok1 ? "OK" : "MISS")
                      << " right=" << (ok2 ? "OK" : "MISS")
                      << "\n";
            continue;
        }

        if (c1.size() != object_template.size() ||
            c2.size() != object_template.size()) {
            std::cout << "[SKIP] " << pair.stem
                      << " unexpected corner count\n";
            continue;
        }

        object_points.push_back(object_template);
        left_points.push_back(std::move(c1));
        right_points.push_back(std::move(c2));
        accepted_stems.push_back(pair.stem);

        std::cout << "[OK]   " << pair.stem << "\n";
    }

    std::cout << "\nAccepted pairs    : " << object_points.size()
              << " / " << pairs.size() << "\n";

    if (object_points.size() < 10) {
        std::cerr
            << "Need at least 10 good stereo pairs; 20-30 diverse pairs are recommended.\n";
        return 2;
    }

    cv::Mat K1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1 = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat K2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D2 = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Mat> rvecs1, tvecs1, rvecs2, tvecs2;

    const int mono_flags =
        cv::CALIB_RATIONAL_MODEL;

    const cv::TermCriteria criteria(
        cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
        100,
        1e-8);

    const double rms1 = cv::calibrateCamera(
        object_points,
        left_points,
        image_size,
        K1,
        D1,
        rvecs1,
        tvecs1,
        mono_flags,
        criteria);

    const double rms2 = cv::calibrateCamera(
        object_points,
        right_points,
        image_size,
        K2,
        D2,
        rvecs2,
        tvecs2,
        mono_flags,
        criteria);

    cv::Mat R, T, E, F;

    const int stereo_flags =
        cv::CALIB_FIX_INTRINSIC;

    const double stereo_rms = cv::stereoCalibrate(
        object_points,
        left_points,
        right_points,
        K1,
        D1,
        K2,
        D2,
        image_size,
        R,
        T,
        E,
        F,
        stereo_flags,
        criteria);

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect roi1, roi2;

    cv::stereoRectify(
        K1, D1,
        K2, D2,
        image_size,
        R, T,
        R1, R2,
        P1, P2,
        Q,
        cv::CALIB_ZERO_DISPARITY,
        0.0,
        image_size,
        &roi1,
        &roi2);

    cv::Mat map1x, map1y, map2x, map2y;

    cv::initUndistortRectifyMap(
        K1, D1, R1, P1, image_size,
        CV_32FC1, map1x, map1y);

    cv::initUndistortRectifyMap(
        K2, D2, R2, P2, image_size,
        CV_32FC1, map2x, map2y);

    const double baseline_m = cv::norm(T);
    const double epi_error = computeStereoEpipolarError(
        left_points, right_points, K1, D1, K2, D2, R, T);

    fs::create_directories(output.parent_path());

    cv::FileStorage fsout(output.string(), cv::FileStorage::WRITE);
    if (!fsout.isOpened()) {
        std::cerr << "Cannot open output " << output << "\n";
        return 3;
    }

    fsout << "camera_left" << "OV9281_USB";
    fsout << "camera_right" << "OV5647_CSI";
    fsout << "image_width" << image_size.width;
    fsout << "image_height" << image_size.height;
    fsout << "board_inner_cols" << cols;
    fsout << "board_inner_rows" << rows;
    fsout << "square_mm" << square_mm;
    fsout << "accepted_pairs" << static_cast<int>(object_points.size());

    fsout << "rms_left" << rms1;
    fsout << "rms_right" << rms2;
    fsout << "rms_stereo" << stereo_rms;
    fsout << "mean_epipolar_error_px" << epi_error;
    fsout << "baseline_m" << baseline_m;

    fsout << "K1" << K1;
    fsout << "D1" << D1;
    fsout << "K2" << K2;
    fsout << "D2" << D2;
    fsout << "R" << R;
    fsout << "T" << T;
    fsout << "E" << E;
    fsout << "F" << F;
    fsout << "R1" << R1;
    fsout << "R2" << R2;
    fsout << "P1" << P1;
    fsout << "P2" << P2;
    fsout << "Q" << Q;

    fsout << "roi1" << "[" << roi1.x << roi1.y << roi1.width << roi1.height << "]";
    fsout << "roi2" << "[" << roi2.x << roi2.y << roi2.width << roi2.height << "]";

    fsout << "accepted_pair_stems" << "[";
    for (const auto &s : accepted_stems)
        fsout << s;
    fsout << "]";

    fsout.release();

    const fs::path rect_dir = capture_dir / "rectified_preview";
    fs::create_directories(rect_dir);

    const size_t preview_count = std::min<size_t>(accepted_stems.size(), 5);

    for (size_t i = 0; i < preview_count; ++i) {
        const fs::path left_path =
            capture_dir / (accepted_stems[i] + "_ov9281.png");
        const fs::path right_path =
            capture_dir / (accepted_stems[i] + "_ov5647.png");

        cv::Mat left = cv::imread(left_path.string(), cv::IMREAD_COLOR);
        cv::Mat right = cv::imread(right_path.string(), cv::IMREAD_COLOR);

        if (left.empty() || right.empty())
            continue;

        cv::Mat rl, rr;
        cv::remap(left, rl, map1x, map1y, cv::INTER_LINEAR);
        cv::remap(right, rr, map2x, map2y, cv::INTER_LINEAR);

        cv::Mat joined;
        cv::hconcat(rl, rr, joined);

        for (int y = 40; y < joined.rows; y += 40)
            cv::line(joined,
                     cv::Point(0, y),
                     cv::Point(joined.cols - 1, y),
                     cv::Scalar(0, 255, 0),
                     1);

        const fs::path out =
            rect_dir / (accepted_stems[i] + "_rectified.png");

        cv::imwrite(out.string(), joined);
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n========================================\n";
    std::cout << " RESULT\n";
    std::cout << "========================================\n";
    std::cout << "Left RMS          : " << rms1 << " px\n";
    std::cout << "Right RMS         : " << rms2 << " px\n";
    std::cout << "Stereo RMS        : " << stereo_rms << " px\n";
    std::cout << "Epipolar error    : " << epi_error << " px\n";
    std::cout << "Baseline          : " << baseline_m * 1000.0 << " mm\n";
    std::cout << "Calibration file  : " << output << "\n";
    std::cout << "Rectified previews: " << rect_dir << "\n";
    std::cout << "========================================\n";

    return 0;
}
