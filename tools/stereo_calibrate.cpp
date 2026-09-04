#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static constexpr int BOARD_SQUARES_X = 7;
static constexpr int BOARD_SQUARES_Y = 5;
static constexpr int MIN_SHARED_CORNERS = 8;
static constexpr int MIN_GOOD_PAIRS = 12;

struct PairFiles {
    fs::path left;
    fs::path right;
    std::string stem;
};

struct Detection {
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
};

static std::vector<PairFiles> findPairs(const fs::path &dir)
{
    std::vector<PairFiles> pairs;

    if (!fs::exists(dir))
        return pairs;

    const std::string suffix = "_ov9281.png";

    for (const auto &entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file())
            continue;

        const std::string name = entry.path().filename().string();
        if (name.size() <= suffix.size() ||
            name.substr(name.size() - suffix.size()) != suffix)
            continue;

        const std::string stem =
            name.substr(0, name.size() - suffix.size());

        const fs::path right = dir / (stem + "_ov5647.png");
        if (fs::exists(right))
            pairs.push_back({entry.path(), right, stem});
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const PairFiles &a, const PairFiles &b) {
                  return a.stem < b.stem;
              });

    return pairs;
}

static Detection detectCharuco(
    const cv::Mat &gray,
    const cv::Ptr<cv::aruco::Dictionary> &dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard> &board)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;

    cv::aruco::DetectorParameters params;
    cv::aruco::detectMarkers(
        gray,
        dictionary,
        marker_corners,
        marker_ids,
        params);

    Detection out;

    if (marker_ids.empty())
        return out;

    cv::Mat charuco_corners;
    cv::Mat charuco_ids;

    const int n = cv::aruco::interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
        charuco_corners,
        charuco_ids);

    if (n <= 0 || charuco_ids.empty())
        return out;

    out.corners.reserve(static_cast<size_t>(n));
    out.ids.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
        out.corners.push_back(charuco_corners.at<cv::Point2f>(i));
        out.ids.push_back(charuco_ids.at<int>(i));
    }

    return out;
}

static void buildSharedCorrespondences(
    const Detection &left,
    const Detection &right,
    const std::vector<cv::Point3f> &board_corners,
    std::vector<cv::Point3f> &object,
    std::vector<cv::Point2f> &left_img,
    std::vector<cv::Point2f> &right_img)
{
    std::map<int, cv::Point2f> left_by_id;
    std::map<int, cv::Point2f> right_by_id;

    for (size_t i = 0; i < left.ids.size(); ++i)
        left_by_id[left.ids[i]] = left.corners[i];

    for (size_t i = 0; i < right.ids.size(); ++i)
        right_by_id[right.ids[i]] = right.corners[i];

    for (const auto &[id, lp] : left_by_id) {
        const auto it = right_by_id.find(id);
        if (it == right_by_id.end())
            continue;

        if (id < 0 || static_cast<size_t>(id) >= board_corners.size())
            continue;

        object.push_back(board_corners[static_cast<size_t>(id)]);
        left_img.push_back(lp);
        right_img.push_back(it->second);
    }
}

static double meanReprojectionError(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const cv::Mat &K,
    const cv::Mat &D)
{
    double sum = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            object_points[i], rvecs[i], tvecs[i],
            K, D, projected);

        for (size_t j = 0; j < projected.size(); ++j) {
            sum += cv::norm(projected[j] - image_points[i][j]);
            ++count;
        }
    }

    return count ? sum / static_cast<double>(count) : 0.0;
}

static double meanEpipolarError(
    const std::vector<std::vector<cv::Point2f>> &left_points,
    const std::vector<std::vector<cv::Point2f>> &right_points,
    const cv::Mat &F)
{
    double total = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < left_points.size(); ++i) {
        std::vector<cv::Vec3f> lines_right;
        cv::computeCorrespondEpilines(
            left_points[i], 1, F, lines_right);

        for (size_t j = 0; j < lines_right.size(); ++j) {
            const cv::Vec3f &l = lines_right[j];
            const cv::Point2f &p = right_points[i][j];
            const double denom =
                std::sqrt(double(l[0]) * l[0] +
                          double(l[1]) * l[1]);

            if (denom > 1e-12) {
                total += std::abs(
                    l[0] * p.x + l[1] * p.y + l[2]) / denom;
                ++count;
            }
        }
    }

    return count ? total / static_cast<double>(count) : 0.0;
}

static void saveDetectionPreview(
    const fs::path &path,
    const cv::Mat &left_gray,
    const cv::Mat &right_gray,
    const Detection &left_det,
    const Detection &right_det,
    size_t shared)
{
    cv::Mat left, right;
    cv::cvtColor(left_gray, left, cv::COLOR_GRAY2BGR);
    cv::cvtColor(right_gray, right, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < left_det.corners.size(); ++i) {
        cv::circle(left, left_det.corners[i], 4, cv::Scalar(0,255,0), 1);
        cv::putText(left, std::to_string(left_det.ids[i]),
                    left_det.corners[i] + cv::Point2f(4,-4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0,255,0), 1);
    }

    for (size_t i = 0; i < right_det.corners.size(); ++i) {
        cv::circle(right, right_det.corners[i], 4, cv::Scalar(0,255,0), 1);
        cv::putText(right, std::to_string(right_det.ids[i]),
                    right_det.corners[i] + cv::Point2f(4,-4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0,255,0), 1);
    }

    cv::Mat joined;
    cv::hconcat(left, right, joined);

    cv::putText(
        joined,
        "shared=" + std::to_string(shared),
        cv::Point(15, joined.rows - 15),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0,255,255),
        2);

    cv::imwrite(path.string(), joined);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " <capture_dir> <square_mm> <marker_mm> [output_yaml]\n\n"
            << "Board is fixed to ChArUco 7x5, DICT_4X4_50.\n"
            << "Nominal PDF dimensions: square=30 mm marker=22 mm.\n"
            << "Use MEASURED printed dimensions for metric baseline.\n\n"
            << "Example:\n"
            << "  " << argv[0]
            << " stereo_captures 30 22 calibration/stereo_ov9281_ov5647.yaml\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const double square_mm = std::stod(argv[2]);
    const double marker_mm = std::stod(argv[3]);

    if (square_mm <= 0.0 ||
        marker_mm <= 0.0 ||
        marker_mm >= square_mm) {
        std::cerr << "Invalid ChArUco dimensions\n";
        return 1;
    }

    const fs::path output =
        argc > 4
            ? fs::path(argv[4])
            : fs::path("calibration/stereo_ov9281_ov5647.yaml");

    const float square_m =
        static_cast<float>(square_mm / 1000.0);
    const float marker_m =
        static_cast<float>(marker_mm / 1000.0);

    const auto dictionary =
        cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_4X4_50);

    const auto board =
        cv::aruco::CharucoBoard::create(
            BOARD_SQUARES_X,
            BOARD_SQUARES_Y,
            square_m,
            marker_m,
            dictionary);

    const std::vector<cv::Point3f> board_corners =
        board->getChessboardCorners();

    const auto pairs = findPairs(capture_dir);

    if (pairs.empty()) {
        std::cerr << "No synchronized pairs found in "
                  << capture_dir << "\n";
        return 1;
    }

    std::cout << "========================================\n";
    std::cout << " JT-Zero ChArUco stereo calibration\n";
    std::cout << "========================================\n";
    std::cout << "Board             : 7x5 ChArUco\n";
    std::cout << "Dictionary        : DICT_4X4_50\n";
    std::cout << "Square            : " << square_mm << " mm\n";
    std::cout << "Marker            : " << marker_mm << " mm\n";
    std::cout << "Pairs on disk     : " << pairs.size() << "\n\n";

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> left_points;
    std::vector<std::vector<cv::Point2f>> right_points;
    std::vector<std::string> accepted_stems;

    cv::Size image_size;

    const fs::path detection_dir =
        capture_dir / "charuco_detection";
    fs::create_directories(detection_dir);

    for (const PairFiles &pair : pairs) {
        cv::Mat left =
            cv::imread(pair.left.string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right =
            cv::imread(pair.right.string(), cv::IMREAD_GRAYSCALE);

        if (left.empty() || right.empty()) {
            std::cout << "[SKIP] " << pair.stem
                      << " read failed\n";
            continue;
        }

        if (left.size() != right.size()) {
            std::cout << "[SKIP] " << pair.stem
                      << " size mismatch\n";
            continue;
        }

        if (image_size.empty())
            image_size = left.size();

        if (left.size() != image_size) {
            std::cout << "[SKIP] " << pair.stem
                      << " inconsistent image size\n";
            continue;
        }

        const Detection dl =
            detectCharuco(left, dictionary, board);
        const Detection dr =
            detectCharuco(right, dictionary, board);

        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> lp;
        std::vector<cv::Point2f> rp;

        buildSharedCorrespondences(
            dl, dr, board_corners, obj, lp, rp);

        saveDetectionPreview(
            detection_dir / (pair.stem + "_detected.png"),
            left, right, dl, dr, obj.size());

        if (static_cast<int>(obj.size()) < MIN_SHARED_CORNERS) {
            std::cout
                << "[SKIP] " << pair.stem
                << " left=" << dl.ids.size()
                << " right=" << dr.ids.size()
                << " shared=" << obj.size()
                << " (<" << MIN_SHARED_CORNERS << ")\n";
            continue;
        }

        object_points.push_back(std::move(obj));
        left_points.push_back(std::move(lp));
        right_points.push_back(std::move(rp));
        accepted_stems.push_back(pair.stem);

        std::cout
            << "[OK]   " << pair.stem
            << " left=" << dl.ids.size()
            << " right=" << dr.ids.size()
            << " shared=" << object_points.back().size()
            << "\n";
    }

    std::cout << "\nUsable stereo pairs: "
              << object_points.size()
              << " / " << pairs.size() << "\n";

    if (static_cast<int>(object_points.size()) < MIN_GOOD_PAIRS) {
        std::cerr
            << "Need at least " << MIN_GOOD_PAIRS
            << " usable pairs. 20-30 diverse pairs are recommended.\n"
            << "Check previews in " << detection_dir << "\n";
        return 2;
    }

    cv::Mat K1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1 = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat K2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D2 = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs1, tvecs1;
    std::vector<cv::Mat> rvecs2, tvecs2;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        100,
        1e-9);

    // Keep the same pinhole + radtan_5 model already used by JT-Zero.
    const int mono_flags = 0;

    const double rms1 =
        cv::calibrateCamera(
            object_points,
            left_points,
            image_size,
            K1, D1,
            rvecs1, tvecs1,
            mono_flags,
            criteria);

    const double rms2 =
        cv::calibrateCamera(
            object_points,
            right_points,
            image_size,
            K2, D2,
            rvecs2, tvecs2,
            mono_flags,
            criteria);

    const double reproj1 =
        meanReprojectionError(
            object_points, left_points,
            rvecs1, tvecs1, K1, D1);

    const double reproj2 =
        meanReprojectionError(
            object_points, right_points,
            rvecs2, tvecs2, K2, D2);

    cv::Mat R, T, E, F;

    const double stereo_rms =
        cv::stereoCalibrate(
            object_points,
            left_points,
            right_points,
            K1, D1,
            K2, D2,
            image_size,
            R, T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            criteria);

    const double epi_error =
        meanEpipolarError(
            left_points,
            right_points,
            F);

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
        K1, D1, R1, P1,
        image_size,
        CV_32FC1,
        map1x, map1y);

    cv::initUndistortRectifyMap(
        K2, D2, R2, P2,
        image_size,
        CV_32FC1,
        map2x, map2y);

    const double baseline_m = cv::norm(T);

    if (!output.parent_path().empty())
        fs::create_directories(output.parent_path());

    cv::FileStorage out(
        output.string(),
        cv::FileStorage::WRITE);

    if (!out.isOpened()) {
        std::cerr << "Cannot write " << output << "\n";
        return 3;
    }

    out << "camera_left" << "OV9281_USB";
    out << "camera_right" << "OV5647_CSI";
    out << "image_width" << image_size.width;
    out << "image_height" << image_size.height;

    out << "board_type" << "charuco";
    out << "board_squares_x" << BOARD_SQUARES_X;
    out << "board_squares_y" << BOARD_SQUARES_Y;
    out << "dictionary" << "DICT_4X4_50";
    out << "square_length_mm" << square_mm;
    out << "marker_length_mm" << marker_mm;

    out << "usable_pairs"
        << static_cast<int>(object_points.size());

    out << "rms_left" << rms1;
    out << "rms_right" << rms2;
    out << "mean_reprojection_left_px" << reproj1;
    out << "mean_reprojection_right_px" << reproj2;
    out << "rms_stereo" << stereo_rms;
    out << "mean_epipolar_error_px" << epi_error;
    out << "baseline_m" << baseline_m;

    out << "K1" << K1;
    out << "D1" << D1;
    out << "K2" << K2;
    out << "D2" << D2;

    out << "R_ov9281_to_ov5647" << R;
    out << "T_ov9281_to_ov5647_m" << T;
    out << "E" << E;
    out << "F" << F;

    out << "R1" << R1;
    out << "R2" << R2;
    out << "P1" << P1;
    out << "P2" << P2;
    out << "Q" << Q;

    out << "roi1_x" << roi1.x;
    out << "roi1_y" << roi1.y;
    out << "roi1_width" << roi1.width;
    out << "roi1_height" << roi1.height;

    out << "roi2_x" << roi2.x;
    out << "roi2_y" << roi2.y;
    out << "roi2_width" << roi2.width;
    out << "roi2_height" << roi2.height;

    out << "accepted_pair_stems" << "[";
    for (const std::string &stem : accepted_stems)
        out << stem;
    out << "]";

    out.release();

    const fs::path rect_dir =
        capture_dir / "rectified_preview";
    fs::create_directories(rect_dir);

    const size_t preview_count =
        std::min<size_t>(accepted_stems.size(), 8);

    for (size_t i = 0; i < preview_count; ++i) {
        cv::Mat left = cv::imread(
            (capture_dir /
             (accepted_stems[i] + "_ov9281.png")).string(),
            cv::IMREAD_COLOR);

        cv::Mat right = cv::imread(
            (capture_dir /
             (accepted_stems[i] + "_ov5647.png")).string(),
            cv::IMREAD_COLOR);

        if (left.empty() || right.empty())
            continue;

        cv::Mat rl, rr;

        cv::remap(
            left, rl,
            map1x, map1y,
            cv::INTER_LINEAR);

        cv::remap(
            right, rr,
            map2x, map2y,
            cv::INTER_LINEAR);

        cv::Mat joined;
        cv::hconcat(rl, rr, joined);

        for (int y = 30; y < joined.rows; y += 30) {
            cv::line(
                joined,
                cv::Point(0, y),
                cv::Point(joined.cols - 1, y),
                cv::Scalar(0, 255, 0),
                1);
        }

        cv::imwrite(
            (rect_dir /
             (accepted_stems[i] + "_rectified.png")).string(),
            joined);
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n========================================\n";
    std::cout << " RESULT\n";
    std::cout << "========================================\n";
    std::cout << "Usable pairs      : " << object_points.size() << "\n";
    std::cout << "Left RMS          : " << rms1 << " px\n";
    std::cout << "Right RMS         : " << rms2 << " px\n";
    std::cout << "Left mean reproj  : " << reproj1 << " px\n";
    std::cout << "Right mean reproj : " << reproj2 << " px\n";
    std::cout << "Stereo RMS        : " << stereo_rms << " px\n";
    std::cout << "Epipolar error    : " << epi_error << " px\n";
    std::cout << "Baseline          : "
              << baseline_m * 1000.0 << " mm\n";
    std::cout << "Calibration file  : " << output << "\n";
    std::cout << "Detection previews: " << detection_dir << "\n";
    std::cout << "Rectified previews: " << rect_dir << "\n";
    std::cout << "========================================\n";

    return 0;
}
